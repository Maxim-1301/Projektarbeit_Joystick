#include "uart_rs485.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"

#include <stdio.h>

#define RS485_START_BYTE       0xC5U
#define RS485_QUEUE_SIZE       16U
#define RS485_RAW_QUEUE_SIZE   2048U
#define RS485_TX_PERIOD_MS     30U
#define RS485_BUS_IDLE_MS      2U
#define RS485_TX_TIMEOUT_MS    10U
#define RS485_TELEMETRY_COUNT  6U

/*
 * Test values sent by the simulated motor.
 *
 * Units match the evaluation in Software_Joystick_Modul:
 * - speed:       1 rpm
 * - power:       1 W
 * - voltage:     0.1 V
 * - capacity:    1 %
 * - GPS speed:   0.1 m/s
 * - temperature: 0.1 degrees Celsius
 *
 * All values can be edited through Live Expressions.
 */
volatile uint16_t rs485_test_motor_speed_rpm = 2450U;
volatile uint16_t rs485_test_motor_power_w = 2350U;
volatile uint16_t rs485_test_battery_voltage_01v = 512U;
volatile uint8_t rs485_test_battery_capacity_percent = 87U;
volatile uint16_t rs485_test_gps_speed_01ms = 84U;
volatile uint16_t rs485_test_motor_temperature_01c = 485U;

/* Frame-Puffer für erkannte 5-Byte-Frames */
static volatile rs485_frame_t frame_queue[RS485_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;

/* Rohdaten-Puffer für alle empfangenen Bytes */
static volatile uint8_t raw_queue[RS485_RAW_QUEUE_SIZE];
static volatile uint16_t raw_head = 0;
static volatile uint16_t raw_tail = 0;

/* Diagnosezähler */
static volatile uint32_t lost_frame_count = 0;
static volatile uint32_t usart_error_count = 0;
static volatile uint32_t rx_byte_count = 0;
static volatile uint32_t raw_lost_count = 0;

static volatile uint8_t last_rx_byte = 0;

/*
 * HAL-UART-Fehlercode und echtes USART_ISR-Register.
 * Das USART_ISR ist hier besonders wichtig, weil wir damit sehen,
 * ob FE, NE, ORE usw. gesetzt waren.
 */
static volatile uint32_t last_uart_error = 0;
static volatile uint32_t last_usart_isr = 0;

/* Einzelne Fehlerzähler */
static volatile uint32_t framing_error_count = 0;
static volatile uint32_t noise_error_count = 0;
static volatile uint32_t overrun_error_count = 0;
static volatile uint32_t parity_error_count = 0;

/* Transmit diagnostics and scheduler state. */
static volatile uint32_t tx_frame_count = 0U;
static volatile uint32_t tx_error_count = 0U;
static volatile uint8_t last_tx_command = 0U;
static volatile uint16_t last_tx_value = 0U;
static volatile uint8_t last_tx_frame[RS485_FRAME_LENGTH] = {0U};

static volatile uint32_t last_rx_byte_time = 0U;
static uint32_t last_tx_time = 0U;
static uint8_t telemetry_index = 0U;

/* Parser-Zustand für 5-Byte-Frames */
static uint8_t rx_frame[RS485_FRAME_LENGTH];
static uint8_t rx_index = 0;

/* Empfangsbyte für HAL_UART_Receive_IT */
static volatile uint8_t rs485_rx_byte = 0;

static uint8_t rs485_checksum4(const uint8_t *data)
{
    return (uint8_t)((data[0] ^ data[1] ^ data[2] ^ data[3]) & 0x7FU);
}

static uint16_t rs485_limit_value(uint16_t value, uint16_t maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

static void rs485_build_telemetry_frame(
    uint8_t command,
    uint16_t value,
    uint8_t frame[RS485_FRAME_LENGTH])
{
    uint16_t limited_value;

    frame[0] = RS485_START_BYTE;
    frame[1] = command;

    if (command == RS485_COMMAND_BATTERY_CAPACITY)
    {
        limited_value = rs485_limit_value(value, 100U);
        frame[2] = 0U;
        frame[3] = (uint8_t)limited_value;
    }
    else if (command == RS485_COMMAND_MOTOR_TEMP)
    {
        /*
         * The receiver uses only the lower four bits of DataH for
         * motor temperature. The representable range is 0..2047.
         */
        limited_value = rs485_limit_value(value, 0x07FFU);
        frame[2] = (uint8_t)((limited_value >> 7U) & 0x0FU);
        frame[3] = (uint8_t)(limited_value & 0x7FU);
    }
    else
    {
        limited_value = rs485_limit_value(value, 0x3FFFU);
        frame[2] = (uint8_t)((limited_value >> 7U) & 0x7FU);
        frame[3] = (uint8_t)(limited_value & 0x7FU);
    }

    frame[4] = rs485_checksum4(frame);
}

static HAL_StatusTypeDef rs485_transmit_frame(
    const uint8_t frame[RS485_FRAME_LENGTH])
{
    HAL_StatusTypeDef status;

    /*
     * RTS high selects transmit mode on the external RS485
     * transceiver.
     */
    HAL_GPIO_WritePin(
        RS485_RTS_GPIO_Port,
        RS485_RTS_Pin,
        GPIO_PIN_SET);

    status = HAL_UART_Transmit(
        &huart1,
        (uint8_t *)frame,
        RS485_FRAME_LENGTH,
        RS485_TX_TIMEOUT_MS);

    /*
     * Return to receive mode immediately after the complete frame.
     */
    HAL_GPIO_WritePin(
        RS485_RTS_GPIO_Port,
        RS485_RTS_Pin,
        GPIO_PIN_RESET);

    return status;
}

static void rs485_send_next_telemetry(void)
{
    uint8_t command;
    uint16_t value;
    uint8_t frame[RS485_FRAME_LENGTH];
    HAL_StatusTypeDef status;

    switch (telemetry_index)
    {
        case 0U:
            command = RS485_COMMAND_MOTOR_SPEED;
            value = rs485_test_motor_speed_rpm;
            break;

        case 1U:
            command = RS485_COMMAND_MOTOR_POWER;
            value = rs485_test_motor_power_w;
            break;

        case 2U:
            command = RS485_COMMAND_BATTERY_VOLTAGE;
            value = rs485_test_battery_voltage_01v;
            break;

        case 3U:
            command = RS485_COMMAND_BATTERY_CAPACITY;
            value = rs485_test_battery_capacity_percent;
            break;

        case 4U:
            command = RS485_COMMAND_GPS_SPEED;
            value = rs485_test_gps_speed_01ms;
            break;

        default:
            command = RS485_COMMAND_MOTOR_TEMP;
            value = rs485_test_motor_temperature_01c;
            break;
    }

    rs485_build_telemetry_frame(command, value, frame);

    for (uint8_t i = 0U; i < RS485_FRAME_LENGTH; i++)
    {
        last_tx_frame[i] = frame[i];
    }

    last_tx_command = command;
    last_tx_value = value;

    status = rs485_transmit_frame(frame);

    if (status == HAL_OK)
    {
        tx_frame_count++;
    }
    else
    {
        tx_error_count++;
    }

    telemetry_index++;
    if (telemetry_index >= RS485_TELEMETRY_COUNT)
    {
        telemetry_index = 0U;
    }
}

static void rs485_raw_queue_push(uint8_t byte)
{
    uint16_t next_head = (uint16_t)((raw_head + 1U) % RS485_RAW_QUEUE_SIZE);

    if (next_head == raw_tail)
    {
        raw_lost_count++;
        return;
    }

    raw_queue[raw_head] = byte;
    raw_head = next_head;
}

static void rs485_queue_push(const uint8_t *data, bool checksum_ok, uint32_t error_flags)
{
    uint8_t next_head = (uint8_t)((queue_head + 1U) % RS485_QUEUE_SIZE);

    if (next_head == queue_tail)
    {
        lost_frame_count++;
        return;
    }

    for (uint8_t i = 0; i < RS485_FRAME_LENGTH; i++)
    {
        frame_queue[queue_head].data[i] = data[i];
    }

    frame_queue[queue_head].checksum_ok = checksum_ok;
    frame_queue[queue_head].error_flags = error_flags;

    queue_head = next_head;
}

static void rs485_process_byte(uint8_t byte, uint32_t error_flags)
{
    if (rx_index == 0U)
    {
        if (byte == RS485_START_BYTE)
        {
            rx_frame[0] = byte;
            rx_index = 1U;
        }

        return;
    }

    rx_frame[rx_index] = byte;
    rx_index++;

    if (rx_index >= RS485_FRAME_LENGTH)
    {
        bool checksum_ok = (rx_frame[4] == rs485_checksum4(rx_frame));

        rs485_queue_push(rx_frame, checksum_ok, error_flags);

        rx_index = 0U;
    }
}

void uart_rs485_start(void)
{
    HAL_StatusTypeDef status;
    uint32_t current_time;

    /*
     * RTS Low:
     * Empfänger aktiv, Sender deaktiviert.
     *
     * Falls dein CubeMX-Label nur RTS heißt, dann diese Zeile ersetzen durch:
     *
     * HAL_GPIO_WritePin(RTS_GPIO_Port, RTS_Pin, GPIO_PIN_RESET);
     */
    HAL_GPIO_WritePin(RS485_RTS_GPIO_Port, RS485_RTS_Pin, GPIO_PIN_RESET);

    current_time = HAL_GetTick();
    last_tx_time = current_time;
    last_rx_byte_time = current_time;
    telemetry_index = 0U;

    status = HAL_UART_Receive_IT(&huart1, (uint8_t *)&rs485_rx_byte, 1);

    printf("USART1 Receive_IT status = %d\n", status);
}

void uart_rs485_process(void)
{
    uint32_t current_time;

    current_time = HAL_GetTick();

    if ((uint32_t)(current_time - last_tx_time) <
        RS485_TX_PERIOD_MS)
    {
        return;
    }

    /*
     * Do not start a transmission in the middle of an incoming
     * frame. Two quiet milliseconds provide a safe frame boundary
     * at 38400 baud.
     */
    if ((uint32_t)(current_time - last_rx_byte_time) <
        RS485_BUS_IDLE_MS)
    {
        return;
    }

    last_tx_time = current_time;
    rs485_send_next_telemetry();
}

bool uart_rs485_get_raw_byte(uint8_t *byte)
{
    if (byte == 0)
    {
        return false;
    }

    if (raw_head == raw_tail)
    {
        return false;
    }

    __disable_irq();

    *byte = raw_queue[raw_tail];
    raw_tail = (uint16_t)((raw_tail + 1U) % RS485_RAW_QUEUE_SIZE);

    __enable_irq();

    return true;
}

bool uart_rs485_get_frame(rs485_frame_t *frame)
{
    if (frame == 0)
    {
        return false;
    }

    if (queue_head == queue_tail)
    {
        return false;
    }

    __disable_irq();

    for (uint8_t i = 0; i < RS485_FRAME_LENGTH; i++)
    {
        frame->data[i] = frame_queue[queue_tail].data[i];
    }

    frame->checksum_ok = frame_queue[queue_tail].checksum_ok;
    frame->error_flags = frame_queue[queue_tail].error_flags;

    queue_tail = (uint8_t)((queue_tail + 1U) % RS485_QUEUE_SIZE);

    __enable_irq();

    return true;
}

uint32_t uart_rs485_get_lost_frame_count(void)
{
    return lost_frame_count;
}

uint32_t uart_rs485_get_error_count(void)
{
    return usart_error_count;
}

uint32_t uart_rs485_get_rx_byte_count(void)
{
    return rx_byte_count;
}

uint32_t uart_rs485_get_raw_lost_count(void)
{
    return raw_lost_count;
}

uint8_t uart_rs485_get_last_rx_byte(void)
{
    return last_rx_byte;
}

uint32_t uart_rs485_get_last_uart_error(void)
{
    return last_uart_error;
}

uint32_t uart_rs485_get_last_usart_isr(void)
{
    return last_usart_isr;
}

uint32_t uart_rs485_get_framing_error_count(void)
{
    return framing_error_count;
}

uint32_t uart_rs485_get_noise_error_count(void)
{
    return noise_error_count;
}

uint32_t uart_rs485_get_overrun_error_count(void)
{
    return overrun_error_count;
}

uint32_t uart_rs485_get_parity_error_count(void)
{
    return parity_error_count;
}

uint32_t uart_rs485_get_tx_frame_count(void)
{
    return tx_frame_count;
}

uint32_t uart_rs485_get_tx_error_count(void)
{
    return tx_error_count;
}

uint8_t uart_rs485_get_last_tx_command(void)
{
    return last_tx_command;
}

uint16_t uart_rs485_get_last_tx_value(void)
{
    return last_tx_value;
}

void uart_rs485_get_last_tx_frame(uint8_t frame[RS485_FRAME_LENGTH])
{
    if (frame == 0)
    {
        return;
    }

    for (uint8_t i = 0U; i < RS485_FRAME_LENGTH; i++)
    {
        frame[i] = last_tx_frame[i];
    }
}

/*
 * RX-Callback:
 * So kurz wie möglich halten.
 * Kein printf() hier.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        rx_byte_count++;
        last_rx_byte = rs485_rx_byte;
        last_rx_byte_time = HAL_GetTick();

        /*
         * Alles roh speichern.
         * Dadurch sehen wir später wirklich jedes empfangene Byte.
         */
        rs485_raw_queue_push(rs485_rx_byte);

        /*
         * Optional läuft der alte 5-Byte-Parser parallel weiter.
         * Wenn du maximale Entlastung willst, kannst du diese Zeile testweise auskommentieren.
         */
        rs485_process_byte(rs485_rx_byte, 0U);

        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rs485_rx_byte, 1);
    }
}

/*
 * Error-Callback:
 * Wichtig: Erst Fehlerstatus sichern, dann Empfang neu starten.
 * Kein printf() hier.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        usart_error_count++;

        /*
         * USART_ISR sofort sichern.
         * Dieses Register zeigt FE, NE, ORE, PE usw.
         */
        last_usart_isr = USART1->ISR;

        /*
         * HAL-Fehlercode sichern.
         */
        last_uart_error = HAL_UART_GetError(huart);

        /*
         * Einzelne Fehlerarten zählen.
         * Die HAL-Fehlerbits sind in stm32f3xx_hal_uart.h definiert.
         */
        if ((last_uart_error & HAL_UART_ERROR_FE) != 0U)
        {
            framing_error_count++;
        }

        if ((last_uart_error & HAL_UART_ERROR_NE) != 0U)
        {
            noise_error_count++;
        }

        if ((last_uart_error & HAL_UART_ERROR_ORE) != 0U)
        {
            overrun_error_count++;
        }

        if ((last_uart_error & HAL_UART_ERROR_PE) != 0U)
        {
            parity_error_count++;
        }

        /*
         * Parser neu synchronisieren.
         * Wenn ein UART-Fehler auftrat, kann ein Byte verloren oder falsch sein.
         * Danach lieber wieder auf neues Startbyte 0xC5 warten.
         */
        rx_index = 0U;

        /*
         * Empfang wieder starten.
         */
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&rs485_rx_byte, 1);
    }
}

/*
 * Softwaretest ohne echte RS485-Hardware.
 * Für echte Messungen in main.c nicht aufrufen.
 */
void uart_rs485_test_inject_frame(void)
{
    uint8_t test_frame_ok[5]  = {0xC5, 0x26, 0x00, 0x00, 0x63};
    uint8_t test_frame_bad[5] = {0xC5, 0x26, 0x00, 0x00, 0x00};

    for (uint8_t i = 0; i < 5; i++)
    {
        rs485_process_byte(test_frame_ok[i], 0U);
    }

    for (uint8_t i = 0; i < 5; i++)
    {
        rs485_process_byte(test_frame_bad[i], 0U);
    }
}
