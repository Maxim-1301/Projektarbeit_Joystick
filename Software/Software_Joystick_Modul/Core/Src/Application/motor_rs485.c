/*
 * motor_rs485.c
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#include "Application/motor_rs485.h"

#include "main.h"
#include "usart.h"

#define UART_TIMEOUT_MS 10U

/* Empfangspuffer */
volatile uint8_t rec_buffer_A[MOTOR_FRAME_SIZE] = {0U};
volatile uint8_t rec_buffer_B[MOTOR_FRAME_SIZE] = {0U};

/* Einzelne Empfangsbytes */
static volatile uint8_t receive_byte_A = 0U;
static volatile uint8_t receive_byte_B = 0U;

/* Motor States */
volatile uint8_t motor_state_A = MOTOR_STATE_WAIT_VOLTAGE;
volatile uint8_t motor_state_B = MOTOR_STATE_WAIT_VOLTAGE;

volatile uint16_t motor_fault_A = 0U;
volatile uint16_t motor_fault_B = 0U;

/* Position innerhalb des Empfangsframes */
volatile uint8_t UARTA_i = 0U;
volatile uint8_t UARTB_i = 0U;

/* Kennzeichnung eines vollständigen Frames */
volatile uint8_t frame_ready_A = 0U;
volatile uint8_t frame_ready_B = 0U;

/* Statuswerte von Motor A */
uint16_t motor_speed_A = 0U;
uint16_t motor_power_A = 0U;
uint16_t battery_voltage_A = 0U;
uint8_t battery_capacity_A = 0U;
uint16_t gps_speed_A = 0U;
uint16_t motor_temperature_A = 0U;

/* Statuswerte von Motor B */
uint16_t motor_speed_B = 0U;
uint16_t motor_power_B = 0U;
uint16_t battery_voltage_B = 0U;
uint8_t battery_capacity_B = 0U;
uint16_t gps_speed_B = 0U;
uint16_t motor_temperature_B = 0U;

/* Empfangsstatistik */
uint32_t valid_frames_A = 0U;
uint32_t valid_frames_B = 0U;

uint32_t checksum_errors_A = 0U;
uint32_t checksum_errors_B = 0U;

static uint8_t calculate_checksum(
    uint8_t byte_0,
    uint8_t byte_1,
    uint8_t byte_2,
    uint8_t byte_3)
{
    uint8_t checksum;

    checksum = byte_0;
    checksum ^= byte_1;
    checksum ^= byte_2;
    checksum ^= byte_3;

    return checksum & 0x7FU;
}

void send_en_A(uint8_t enable)
{
    if (enable != 0U)
    {
        /*
         * DE = High: Sender aktiv
         * /RE = High: Empfänger deaktiviert
         */
        HAL_GPIO_WritePin(
            MotorA_DE_GPIO_Port,
            MotorA_DE_Pin,
            GPIO_PIN_SET);

        HAL_GPIO_WritePin(
            MotorA_NRE_GPIO_Port,
            MotorA_NRE_Pin,
            GPIO_PIN_SET);
    }
    else
    {
        /*
         * DE = Low: Sender deaktiviert
         * /RE = Low: Empfänger aktiv
         */
        HAL_GPIO_WritePin(
            MotorA_DE_GPIO_Port,
            MotorA_DE_Pin,
            GPIO_PIN_RESET);

        HAL_GPIO_WritePin(
            MotorA_NRE_GPIO_Port,
            MotorA_NRE_Pin,
            GPIO_PIN_RESET);
    }
}

void send_en_B(uint8_t enable)
{
    if (enable != 0U)
    {
        HAL_GPIO_WritePin(
            MotorB_DE_GPIO_Port,
            MotorB_DE_Pin,
            GPIO_PIN_SET);

        HAL_GPIO_WritePin(
            MotorB_NRE_GPIO_Port,
            MotorB_NRE_Pin,
            GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(
            MotorB_DE_GPIO_Port,
            MotorB_DE_Pin,
            GPIO_PIN_RESET);

        HAL_GPIO_WritePin(
            MotorB_NRE_GPIO_Port,
            MotorB_NRE_Pin,
            GPIO_PIN_RESET);
    }
}

static void receive_next_byte_A(void)
{
    (void)HAL_UART_Receive_IT(
        &huart1,
        (uint8_t *)&receive_byte_A,
        1U);
}

static void receive_next_byte_B(void)
{
    (void)HAL_UART_Receive_IT(
        &huart3,
        (uint8_t *)&receive_byte_B,
        1U);
}

void motor_rs485_init(void)
{
	motor_state_A = MOTOR_STATE_WAIT_VOLTAGE;
	motor_state_B = MOTOR_STATE_WAIT_VOLTAGE;

	motor_fault_A = 0U;
	motor_fault_B = 0U;

    UARTA_i = 0U;
    UARTB_i = 0U;

    frame_ready_A = 0U;
    frame_ready_B = 0U;

    send_en_A(0U);
    send_en_B(0U);

    receive_next_byte_A();
    receive_next_byte_B();
}

static void send_frame_A(
    uint8_t command,
    uint8_t direction,
    uint8_t power)
{
    uint8_t frame[MOTOR_FRAME_SIZE];

    frame[0] = ADDR_MAIN_SYSTEM;
    frame[1] = command;
    frame[2] = direction;
    frame[3] = power;

    frame[4] = calculate_checksum(
        frame[0],
        frame[1],
        frame[2],
        frame[3]);

    send_en_A(1U);

    (void)HAL_UART_Transmit(
        &huart1,
        frame,
        MOTOR_FRAME_SIZE,
        UART_TIMEOUT_MS);

    send_en_A(0U);
}

static void send_frame_B(
    uint8_t command,
    uint8_t direction,
    uint8_t power)
{
    uint8_t frame[MOTOR_FRAME_SIZE];

    frame[0] = ADDR_MAIN_SYSTEM;
    frame[1] = command;
    frame[2] = direction;
    frame[3] = power;

    frame[4] = calculate_checksum(
        frame[0],
        frame[1],
        frame[2],
        frame[3]);

    send_en_B(1U);

    (void)HAL_UART_Transmit(
        &huart3,
        frame,
        MOTOR_FRAME_SIZE,
        UART_TIMEOUT_MS);

    send_en_B(0U);
}

void motor_send_A(int16_t power)
{
    uint8_t direction;
    uint8_t magnitude;

    if (motor_state_A == MOTOR_STATE_FAULT)
    {
        send_frame_A(
            MOTOR_STOP,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (motor_state_A == MOTOR_STATE_WAIT_VOLTAGE)
    {
        send_frame_A(
            POWER_THROTTLE,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (motor_state_A == MOTOR_STATE_STARTING)
    {
        send_frame_A(
            MOTOR_START,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (power == 0)
    {
        send_frame_A(
            POWER_THROTTLE,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }


    if (power > 0)
    {
        direction = 1U;
        magnitude = (uint8_t)power;
    }
    else
    {
        direction = 0U;
        magnitude = (uint8_t)(-power);
    }

    if (magnitude > MOTOR_MAX_POWER)
    {
        magnitude = MOTOR_MAX_POWER;
    }

    send_frame_A(
        POWER_THROTTLE,
        direction,
        magnitude);
}

void motor_send_B(int16_t power)
{
    uint8_t direction;
    uint8_t magnitude;

    if (motor_state_B == MOTOR_STATE_FAULT)
    {
        send_frame_B(
            MOTOR_STOP,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (motor_state_B == MOTOR_STATE_WAIT_VOLTAGE)
    {
        send_frame_B(
            POWER_THROTTLE,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (motor_state_B == MOTOR_STATE_STARTING)
    {
        send_frame_B(
            MOTOR_START,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }

    if (power == 0)
    {
        send_frame_B(
            POWER_THROTTLE,
            EMPTY_DATABYTE,
            EMPTY_DATABYTE);

        return;
    }


    if (power > 0)
    {
        direction = 1U;
        magnitude = (uint8_t)power;
    }
    else
    {
        direction = 0U;
        magnitude = (uint8_t)(-power);
    }

    if (magnitude > MOTOR_MAX_POWER)
    {
        magnitude = MOTOR_MAX_POWER;
    }

    send_frame_B(
        POWER_THROTTLE,
        direction,
        magnitude);
}

static void receive_A(void)
{
    /*
     * Ein neuer Frame muss mit der Adresse 0xC5 beginnen.
     */
    if ((UARTA_i == 0U) &&
        (receive_byte_A != ADDR_MAIN_SYSTEM))
    {
        receive_next_byte_A();
        return;
    }

    rec_buffer_A[UARTA_i] = receive_byte_A;
    UARTA_i++;

    if (UARTA_i >= MOTOR_FRAME_SIZE)
    {
        UARTA_i = 0U;
        frame_ready_A = 1U;

        /*
         * Der Empfang wird erst nach der Frameauswertung
         * wieder gestartet. Dadurch wird der Puffer nicht
         * während der Auswertung überschrieben.
         */
    }
    else
    {
        receive_next_byte_A();
    }
}

static void receive_B(void)
{
    if ((UARTB_i == 0U) &&
        (receive_byte_B != ADDR_MAIN_SYSTEM))
    {
        receive_next_byte_B();
        return;
    }

    rec_buffer_B[UARTB_i] = receive_byte_B;
    UARTB_i++;

    if (UARTB_i >= MOTOR_FRAME_SIZE)
    {
        UARTB_i = 0U;
        frame_ready_B = 1U;
    }
    else
    {
        receive_next_byte_B();
    }
}

void motor_uart_rx_callback(
    UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        receive_A();
    }
    else if (huart == &huart3)
    {
        receive_B();
    }
}

static void evaluate_frame_A(void)
{
    uint16_t received_value;

    /*
     * Normaler 14-Bit-Wert:
     * DataH enthält die oberen sieben Bit.
     * DataL enthält die unteren sieben Bit.
     */
    received_value =
        ((uint16_t)(rec_buffer_A[2] & 0x7FU) << 7U) |
        rec_buffer_A[3];

    if ((rec_buffer_A[1] == VOLTAGE_STATUS) &&
        (motor_state_A != MOTOR_STATE_READY) &&
        (motor_state_A != MOTOR_STATE_FAULT))
    {
        if (rec_buffer_A[3] == VOLTAGE_NOT_READY)
        {
            motor_state_A = MOTOR_STATE_WAIT_VOLTAGE;
        }
        else if (rec_buffer_A[3] == VOLTAGE_READY)
        {
            if (rec_buffer_A[2] == EMPTY_DATABYTE)
            {
                motor_state_A = MOTOR_STATE_STARTING;
            }
            else
            {
                motor_fault_A = rec_buffer_A[2];
                motor_state_A = MOTOR_STATE_FAULT;
            }
        }
    }
    else if ((motor_state_A == MOTOR_STATE_STARTING) &&
             (rec_buffer_A[1] != VOLTAGE_STATUS))
    {
        /*
         * Das erste normale Statusframe nach dem
         * START-Befehl bestätigt den Betriebszustand.
         */
        motor_state_A = MOTOR_STATE_READY;
    }

    switch (rec_buffer_A[1])
    {
		case VOLTAGE_STATUS:
			/*
			 * Wurde bereits für die Startsequenz ausgewertet.
			 * Der Wert wird nicht auf dem Display angezeigt.
			 */
			break;

		case MOTOR_STATUS:
			if (received_value != 0U)
			{
				motor_fault_A = received_value;
				motor_state_A = MOTOR_STATE_FAULT;
			}
			break;

        case MOTOR_SPEED:
            /*
             * Einheit: 1 rpm
             */
            motor_speed_A = received_value;
            break;

        case MOTOR_POWER:
            /*
             * Einheit: 1 W
             */
            motor_power_A = received_value;
            break;

        case BATTERY_VOLTAGE:
            /*
             * Einheit: 0,1 V
             */
            battery_voltage_A = received_value;
            break;

        case BATTERY_CAPACITY:
            /*
             * Kapazität steht direkt in DataL.
             * Wertebereich: 0 bis 100 Prozent.
             */
            battery_capacity_A = rec_buffer_A[3];

            if (battery_capacity_A > 100U)
            {
                battery_capacity_A = 100U;
            }
            break;

        case GPS_SPEED:
            /*
             * Einheit: 0,1 m/s
             */
            gps_speed_A = received_value;
            break;

        case MOTOR_TEMP:
            /*
             * Bei Temperaturwerten werden laut Bachelorarbeit
             * nur die unteren vier Bit von DataH verwendet.
             * Einheit: 0,1 Grad Celsius.
             */
            motor_temperature_A =
                ((uint16_t)(rec_buffer_A[2] & 0x0FU) << 7U) |
                rec_buffer_A[3];
            break;

        default:
            /*
             * Andere Statusframes werden momentan ignoriert.
             */
            break;
    }
}

static void evaluate_frame_B(void)
{
    uint16_t received_value;

    received_value =
        ((uint16_t)(rec_buffer_B[2] & 0x7FU) << 7U) |
        rec_buffer_B[3];

    if ((rec_buffer_B[1] == VOLTAGE_STATUS) &&
        (motor_state_B != MOTOR_STATE_READY) &&
        (motor_state_B != MOTOR_STATE_FAULT))
    {
        if (rec_buffer_B[3] == VOLTAGE_NOT_READY)
        {
            motor_state_B = MOTOR_STATE_WAIT_VOLTAGE;
        }
        else if (rec_buffer_B[3] == VOLTAGE_READY)
        {
            if (rec_buffer_B[2] == EMPTY_DATABYTE)
            {
                motor_state_B = MOTOR_STATE_STARTING;
            }
            else
            {
                motor_fault_B = rec_buffer_B[2];
                motor_state_B = MOTOR_STATE_FAULT;
            }
        }
    }
    else if ((motor_state_B == MOTOR_STATE_STARTING) &&
             (rec_buffer_B[1] != VOLTAGE_STATUS))
    {
        motor_state_B = MOTOR_STATE_READY;
    }

    switch (rec_buffer_B[1])
    {
		case VOLTAGE_STATUS:
			break;

		case MOTOR_STATUS:
			if (received_value != 0U)
			{
				motor_fault_B = received_value;
				motor_state_B = MOTOR_STATE_FAULT;
			}
			break;

        case MOTOR_SPEED:
            motor_speed_B = received_value;
            break;

        case MOTOR_POWER:
            motor_power_B = received_value;
            break;

        case BATTERY_VOLTAGE:
            battery_voltage_B = received_value;
            break;

        case BATTERY_CAPACITY:
            battery_capacity_B = rec_buffer_B[3];

            if (battery_capacity_B > 100U)
            {
                battery_capacity_B = 100U;
            }
            break;

        case GPS_SPEED:
            gps_speed_B = received_value;
            break;

        case MOTOR_TEMP:
            motor_temperature_B =
                ((uint16_t)(rec_buffer_B[2] & 0x0FU) << 7U) |
                rec_buffer_B[3];
            break;

        default:
            break;
    }
}

static void process_frame_A(void)
{
    uint8_t expected_checksum;
    uint8_t received_checksum;

    if (frame_ready_A == 0U)
    {
        return;
    }

    expected_checksum = calculate_checksum(
        rec_buffer_A[0],
        rec_buffer_A[1],
        rec_buffer_A[2],
        rec_buffer_A[3]);

    received_checksum = rec_buffer_A[4];

    if (received_checksum == expected_checksum)
    {
        valid_frames_A++;

        evaluate_frame_A();
    }
    else
    {
        checksum_errors_A++;
    }

    frame_ready_A = 0U;

    receive_next_byte_A();
}

static void process_frame_B(void)
{
    uint8_t expected_checksum;
    uint8_t received_checksum;

    if (frame_ready_B == 0U)
    {
        return;
    }

    expected_checksum = calculate_checksum(
        rec_buffer_B[0],
        rec_buffer_B[1],
        rec_buffer_B[2],
        rec_buffer_B[3]);

    received_checksum = rec_buffer_B[4];

    if (received_checksum == expected_checksum)
    {
        valid_frames_B++;

        evaluate_frame_B();
    }
    else
    {
        checksum_errors_B++;
    }

    frame_ready_B = 0U;

    receive_next_byte_B();
}

void motor_process_received_frames(void)
{
    process_frame_A();
    process_frame_B();
}

void motor_uart_error_callback(
    UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        UARTA_i = 0U;
        frame_ready_A = 0U;

        receive_next_byte_A();
    }
    else if (huart == &huart3)
    {
        UARTB_i = 0U;
        frame_ready_B = 0U;

        receive_next_byte_B();
    }
}
