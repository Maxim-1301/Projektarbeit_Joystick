/*
 * communication_module.c
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#include "Application/communication_module.h"

#include "Application/display_i2c.h"
#include "Application/joystick.h"
#include "Application/motor_rs485.h"

#include "i2c.h"

#include <stdio.h>

#define CONTROL_PERIOD_MS  30U
#define DISPLAY_PERIOD_MS  250U

/* Aktuelle Motorleistung von -127 bis +127 */
int16_t power_val_MA = 0;
int16_t power_val_MB = 0;

/* Zeitsteuerung */
static uint32_t last_control_time = 0U;
static uint32_t last_display_time = 0U;

/* Displaystatus */
static uint8_t display_available = 0U;

/*
 * 0 = Motor A anzeigen
 * 1 = Motor B anzeigen
 */
static uint8_t selected_motor = 0U;

/*
 * Wird für die Erkennung einer neuen Tastenbetätigung benötigt.
 */
static uint8_t previous_button_1 = 0U;

static void calculate_power_values(void)
{
    int16_t reduced_power;
    int16_t absolute_x;

    if (con_x < 0)
    {
        absolute_x = -con_x;
    }
    else
    {
        absolute_x = con_x;
    }

    if (absolute_x > MOTOR_MAX_POWER)
    {
        absolute_x = MOTOR_MAX_POWER;
    }

    /*
     * con_y liegt bereits direkt zwischen -127 und +127.
     *
     * Die Lenkung erhöht den Schub nicht.
     * Der kurveninnere Motor wird lediglich reduziert:
     *
     * reduzierter Motor =
     * Schub * (127 - |Lenkung|) / 127
     */
    reduced_power =
        (int16_t)(
            ((int32_t)con_y *
             (MOTOR_MAX_POWER - absolute_x)) /
            MOTOR_MAX_POWER);

    if (con_y >= 0)
    {
        /*
         * Vorwärtsfahrt.
         */
        if (con_x >= 0)
        {
            /*
             * Rechtskurve:
             * Motor A links bleibt unverändert.
             * Motor B rechts wird reduziert.
             */
            power_val_MA = con_y;
            power_val_MB = reduced_power;
        }
        else
        {
            /*
             * Linkskurve.
             */
            power_val_MA = reduced_power;
            power_val_MB = con_y;
        }
    }
    else
    {
        /*
         * Rückwärtsfahrt.
         */
        if (con_x >= 0)
        {
            /*
             * Rückwärts nach rechts.
             */
            power_val_MA = reduced_power;
            power_val_MB = con_y;
        }
        else
        {
            /*
             * Rückwärts nach links.
             */
            power_val_MA = con_y;
            power_val_MB = reduced_power;
        }
    }
}

static void select_display_motor(void)
{
    /*
     * Nur bei der neuen Betätigung umschalten.
     * Dadurch wird nicht bei jedem Programmdurchlauf gewechselt,
     * solange der Taster gehalten wird.
     */
    if ((joystick_button_1 != 0U) &&
        (previous_button_1 == 0U))
    {
        if (selected_motor == 0U)
        {
            selected_motor = 1U;
        }
        else
        {
            selected_motor = 0U;
        }
    }

    previous_button_1 = joystick_button_1;
}

static void display_motor_A(void)
{
    char line[32];

    /*
     * Zeile 1:
     * Motorname und Drehzahl.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "Motor A     %5urpm",
        (unsigned int)motor_speed_A);

    (void)Display_WriteLine(0U, line);

    /*
     * Zeile 2:
     * Leistung wird von Watt in Kilowatt umgerechnet.
     *
     * Beispiel:
     * 2350 W wird als 2.35 kW angezeigt.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "Leistung: %2u.%02u kW",
        (unsigned int)(motor_power_A / 1000U),
        (unsigned int)((motor_power_A % 1000U) / 10U));

    (void)Display_WriteLine(1U, line);

    /*
     * Zeile 3:
     * Batteriespannung und Batteriekapazität.
     *
     * battery_voltage_A = 486 bedeutet 48,6 V.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "Akku:%3u.%1uV    %3u%%",
        (unsigned int)(battery_voltage_A / 10U),
        (unsigned int)(battery_voltage_A % 10U),
        (unsigned int)battery_capacity_A);

    (void)Display_WriteLine(2U, line);

    /*
     * Zeile 4:
     * Geschwindigkeit und Motortemperatur.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "v:%2u.%1um/s T:%2u.%1uC",
        (unsigned int)(gps_speed_A / 10U),
        (unsigned int)(gps_speed_A % 10U),
        (unsigned int)(motor_temperature_A / 10U),
        (unsigned int)(motor_temperature_A % 10U));

    (void)Display_WriteLine(3U, line);
}

static void display_motor_B(void)
{
    char line[32];

    (void)snprintf(
        line,
        sizeof(line),
        "Motor B     %5urpm",
        (unsigned int)motor_speed_B);

    (void)Display_WriteLine(0U, line);

    (void)snprintf(
        line,
        sizeof(line),
        "Leistung: %2u.%02u kW",
        (unsigned int)(motor_power_B / 1000U),
        (unsigned int)((motor_power_B % 1000U) / 10U));

    (void)Display_WriteLine(1U, line);

    (void)snprintf(
        line,
        sizeof(line),
        "Akku:%3u.%1uV    %3u%%",
        (unsigned int)(battery_voltage_B / 10U),
        (unsigned int)(battery_voltage_B % 10U),
        (unsigned int)battery_capacity_B);

    (void)Display_WriteLine(2U, line);

    (void)snprintf(
        line,
        sizeof(line),
        "v:%2u.%1um/s T:%2u.%1uC",
        (unsigned int)(gps_speed_B / 10U),
        (unsigned int)(gps_speed_B % 10U),
        (unsigned int)(motor_temperature_B / 10U),
        (unsigned int)(motor_temperature_B % 10U));

    (void)Display_WriteLine(3U, line);
}

static void update_display(void)
{
    if (selected_motor == 0U)
    {
        display_motor_A();
    }
    else
    {
        display_motor_B();
    }
}

HAL_StatusTypeDef communication_module_init(void)
{
    motor_rs485_init();

    if (joystick_init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (Display_Init(&hi2c1) == HAL_OK)
    {
        display_available = 1U;
    }
    else
    {
        /*
         * Die Motorsteuerung darf auch ohne Display weiterlaufen.
         */
        display_available = 0U;
    }

    last_control_time = HAL_GetTick();
    last_display_time = last_control_time;

    return HAL_OK;
}

void communication_module_process(void)
{
    uint32_t current_time;

    current_time = HAL_GetTick();

    /*
     * Empfangene Motorframes möglichst schnell auswerten.
     */
    motor_process_received_frames();

    /*
     * Motorregelung nur alle 30 ms ausführen.
     */
    if ((uint32_t)(current_time - last_control_time) <
        CONTROL_PERIOD_MS)
    {
        return;
    }

    last_control_time = current_time;

    /*
     * Joystick aktualisieren.
     */
    joystick_update();

    /*
     * Zwischen Motor A und B auf dem Display umschalten.
     */
    select_display_motor();

    /*
     * Leistungswerte berechnen.
     */
    calculate_power_values();

    /*
     * Leistungsbefehle an beide Motoren übertragen.
     */
    motor_send_A(power_val_MA);
    motor_send_B(power_val_MB);

    /*
     * Display ungefähr viermal pro Sekunde aktualisieren.
     */
    if ((display_available != 0U) &&
        ((uint32_t)(current_time - last_display_time) >=
         DISPLAY_PERIOD_MS))
    {
        last_display_time = current_time;

        update_display();
    }
}

void communication_uart_rx_callback(
    UART_HandleTypeDef *huart)
{
    motor_uart_rx_callback(huart);
}

void communication_uart_error_callback(
    UART_HandleTypeDef *huart)
{
    motor_uart_error_callback(huart);
}
