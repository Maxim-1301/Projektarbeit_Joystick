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

/* Zeitintervalle */
#define CONTROL_PERIOD_MS             30U
#define DISPLAY_PERIOD_MS            250U
#define MODE_MESSAGE_TIME_MS        2000U
#define MODE_BUTTON_DEBOUNCE_MS      150U

/* Displayseiten */
#define DISPLAY_PAGE_MAIN              0U
#define DISPLAY_PAGE_DETAILS           1U

/* Betriebsarten */
#define DRIVE_MODE_NORMAL              0U
#define DRIVE_MODE_ROTATION            1U

/* Aktuelle Motorbefehle von -127 bis +127 */
int16_t power_val_MA = 0;
int16_t power_val_MB = 0;

/* Aktuelle Displayseite */
static uint8_t display_page = DISPLAY_PAGE_MAIN;

/* Aktuelle Betriebsart */
static uint8_t driving_mode = DRIVE_MODE_NORMAL;

/* Vorherige Tasterzustände zur Flankenerkennung */
static uint8_t previous_button_1 = 0U;
static uint8_t previous_button_2 = 0U;

/* Status der kurzen Modusmeldung */
static uint8_t mode_message_active = 0U;
static uint8_t display_force_update = 1U;

/* Display verfügbar */
static uint8_t display_available = 0U;

/* Zeitsteuerung */
static uint32_t last_control_time = 0U;
static uint32_t last_display_time = 0U;
static uint32_t mode_message_start_time = 0U;
static uint32_t last_mode_button_time = 0U;
static uint8_t rotation_button_ready = 0U;


/*
 * Motorwerte aus den Joystickwerten berechnen.
 */
static void calculate_power_values(void)
{
    int16_t absolute_x;
    int16_t reduced_power;

    /*
     * Im Drehmodus werden X und Y ignoriert.
     *
     * Z positiv:
     * Motor A vorwärts, Motor B rückwärts.
     *
     * Z negativ:
     * Motor A rückwärts, Motor B vorwärts.
     */
    if (driving_mode == DRIVE_MODE_ROTATION)
    {
        power_val_MA = con_z;
        power_val_MB = -con_z;
        return;
    }

    /*
     * Im Normalmodus wird die Z-Achse vollständig ignoriert.
     * Die Fahrt erfolgt nur mit X und Y.
     */
    if (con_x < 0)
    {
        absolute_x = -con_x;
    }
    else
    {
        absolute_x = con_x;
    }

    /*
     * Die Lenkung reduziert ausschließlich den
     * kurveninneren Motor.
     */
    reduced_power =
        (int16_t)(((int32_t)con_y *
                  (JOYSTICK_MAX_VALUE - absolute_x)) /
                  JOYSTICK_MAX_VALUE);

    if (con_y >= 0)
    {
        if (con_x >= 0)
        {
            /* Vorwärts und rechts */
            power_val_MA = con_y;
            power_val_MB = reduced_power;
        }
        else
        {
            /* Vorwärts und links */
            power_val_MA = reduced_power;
            power_val_MB = con_y;
        }
    }
    else
    {
        if (con_x >= 0)
        {
            /* Rückwärts und rechts */
            power_val_MA = reduced_power;
            power_val_MB = con_y;
        }
        else
        {
            /* Rückwärts und links */
            power_val_MA = con_y;
            power_val_MB = reduced_power;
        }
    }
}


/*
 * Taster 1 schaltet zwischen Haupt- und Detailseite um.
 */
static void update_display_button(void)
{
    /*
     * Während der Modusmeldung wird die normale
     * Displayumschaltung ignoriert.
     */
    if ((mode_message_active == 0U) &&
        (joystick_button_1 != 0U) &&
        (previous_button_1 == 0U))
    {
        if (display_page == DISPLAY_PAGE_MAIN)
        {
            display_page = DISPLAY_PAGE_DETAILS;
        }
        else
        {
            display_page = DISPLAY_PAGE_MAIN;
        }

        /*
         * Neue Seite sofort ausgeben.
         */
        display_force_update = 1U;
    }

    previous_button_1 = joystick_button_1;
}


/*
 * Taster 2 schaltet dauerhaft zwischen Normal- und Drehmodus um.
 */
static void update_rotation_mode_button(uint32_t current_time)
{
    /*
     * Nach dem Einschalten muss Taster 2 zunächst
     * einmal losgelassen worden sein.
     */
    if (rotation_button_ready == 0U)
    {
        if (joystick_button_2 == 0U)
        {
            rotation_button_ready = 1U;
        }

        previous_button_2 = joystick_button_2;
        return;
    }

    /*
     * Neue Betätigung von Taster 2 erkennen.
     */
    if ((joystick_button_2 != 0U) &&
        (previous_button_2 == 0U))
    {
        if ((uint32_t)(current_time -
                       last_mode_button_time) >=
            MODE_BUTTON_DEBOUNCE_MS)
        {
            last_mode_button_time = current_time;

            if (driving_mode == DRIVE_MODE_NORMAL)
            {
                driving_mode = DRIVE_MODE_ROTATION;
            }
            else
            {
                driving_mode = DRIVE_MODE_NORMAL;
            }

            mode_message_active = 1U;
            mode_message_start_time = current_time;

            display_page = DISPLAY_PAGE_MAIN;
            display_force_update = 1U;
        }
    }

    previous_button_2 = joystick_button_2;
}


/*
 * Hauptseite:
 * Akkustand, Leistung und Geschwindigkeit.
 */
static void display_main_page(void)
{
    char line[32];

    /*
     *         11111111112
     * 12345678901234567890
     *      Motor A Motor B
     */
    (void)Display_WriteLine(
        0U,
        "     Motor A Motor B");

    /*
     * Akkustand in Prozent.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "Akku:%6u%% %6u%%",
        (unsigned int)battery_capacity_A,
        (unsigned int)battery_capacity_B);

    (void)Display_WriteLine(1U, line);

    /*
     * Motorleistung in kW.
     * motor_power ist in Watt gespeichert.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "P[kW]:  %1u.%02u   %2u.%02u",
        (unsigned int)(motor_power_A / 1000U),
        (unsigned int)((motor_power_A % 1000U) / 10U),
        (unsigned int)(motor_power_B / 1000U),
        (unsigned int)((motor_power_B % 1000U) / 10U));

    (void)Display_WriteLine(2U, line);

    /*
     * Geschwindigkeit in m/s.
     * gps_speed ist in 0,1 m/s gespeichert.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "v[m/s]: %2u.%1u   %3u.%1u",
        (unsigned int)(gps_speed_A / 10U),
        (unsigned int)(gps_speed_A % 10U),
        (unsigned int)(gps_speed_B / 10U),
        (unsigned int)(gps_speed_B % 10U));

    (void)Display_WriteLine(3U, line);
}


/*
 * Detailseite:
 * Drehzahl, Batteriespannung und Motortemperatur.
 */
static void display_details_page(void)
{
    char line[32];

    (void)Display_WriteLine(
        0U,
        "     Motor A Motor B");

    /*
     * Drehzahl in Umdrehungen pro Minute.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "rpm: %7u %7u",
        (unsigned int)motor_speed_A,
        (unsigned int)motor_speed_B);

    (void)Display_WriteLine(1U, line);

    /*
     * Batteriespannung in Volt.
     * battery_voltage ist in 0,1 V gespeichert.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "U[V]:%5u.%1u %5u.%1u",
        (unsigned int)(battery_voltage_A / 10U),
        (unsigned int)(battery_voltage_A % 10U),
        (unsigned int)(battery_voltage_B / 10U),
        (unsigned int)(battery_voltage_B % 10U));

    (void)Display_WriteLine(2U, line);

    /*
     * Motortemperatur in Grad Celsius.
     * motor_temperature ist in 0,1 Grad gespeichert.
     */
    (void)snprintf(
        line,
        sizeof(line),
        "T[C]:%5u.%1u %5u.%1u",
        (unsigned int)(motor_temperature_A / 10U),
        (unsigned int)(motor_temperature_A % 10U),
        (unsigned int)(motor_temperature_B / 10U),
        (unsigned int)(motor_temperature_B % 10U));

    (void)Display_WriteLine(3U, line);
}


/*
 * Kurze Meldung beim Umschalten der Betriebsart.
 */
static void display_mode_message(void)
{
    (void)Display_WriteLine(0U, "--------------------");

    if (driving_mode == DRIVE_MODE_ROTATION)
    {
        (void)Display_WriteLine(1U, "  DREHMODUS AKTIV");
        (void)Display_WriteLine(2U, "  Z-Achse steuert");
    }
    else
    {
        (void)Display_WriteLine(1U, " NORMALMODUS AKTIV");
        (void)Display_WriteLine(2U, "  X/Y-Fahrt aktiv");
    }

    (void)Display_WriteLine(3U, "--------------------");
}


/*
 * Aktuell benötigte Displayseite ausgeben.
 */
static void update_display(uint32_t current_time)
{
    /*
     * Modusmeldung für eine Sekunde anzeigen.
     */
    if (mode_message_active != 0U)
    {
        if ((uint32_t)(current_time -
                       mode_message_start_time) <
            MODE_MESSAGE_TIME_MS)
        {
            display_mode_message();
            return;
        }

        /*
         * Meldungszeit ist abgelaufen.
         */
        mode_message_active = 0U;
        display_page = DISPLAY_PAGE_MAIN;
    }

    /*
     * Normale Displayseiten anzeigen.
     */
    if (display_page == DISPLAY_PAGE_MAIN)
    {
        display_main_page();
    }
    else
    {
        display_details_page();
    }
}


HAL_StatusTypeDef communication_module_init(void)
{
    /*
     * Definierter Startzustand:
     * normale Fahrt über X und Y.
     */
    driving_mode = DRIVE_MODE_NORMAL;
    display_page = DISPLAY_PAGE_MAIN;
    mode_message_active = 0U;

    power_val_MA = 0;
    power_val_MB = 0;

    previous_button_1 = 0U;
    previous_button_2 = 0U;
    rotation_button_ready = 0U;

    motor_rs485_init();

    if (joystick_init() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Taster einmal einlesen, damit beim Start keine
     * falsche Tastenflanke erkannt wird.
     */
    joystick_update();

    previous_button_1 = joystick_button_1;
    previous_button_2 = joystick_button_2;

    /*
     * Taster 2 muss beim Start losgelassen sein.
     */
    if (joystick_button_2 == 0U)
    {
        rotation_button_ready = 1U;
    }
    else
    {
        rotation_button_ready = 0U;
    }

    /*
     * Display initialisieren und sofort die Hauptseite
     * anzeigen. Dadurch wird der alte Displayinhalt
     * schnellstmöglich überschrieben.
     */
    if (Display_Init(&hi2c1) == HAL_OK)
    {
        display_available = 1U;

        (void)Display_Clear();
        display_main_page();

        display_force_update = 0U;
    }
    else
    {
        /*
         * Motorsteuerung läuft auch ohne Display weiter.
         */
        display_available = 0U;
    }

    last_control_time = HAL_GetTick();
    last_display_time = last_control_time;

    last_mode_button_time =
        last_control_time - MODE_BUTTON_DEBOUNCE_MS;

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
     * Steuerung nur alle 30 ms ausführen.
     */
    if ((uint32_t)(current_time - last_control_time) <
        CONTROL_PERIOD_MS)
    {
        return;
    }

    last_control_time = current_time;

    /*
     * Joystickachsen und Taster aktualisieren.
     */
    joystick_update();

    /*
     * Taster 1: Displayseite.
     */
    update_display_button();

    /*
     * Taster 2: Normal-/Drehmodus.
     */
    update_rotation_mode_button(current_time);

    /*
     * Motorwerte aus dem aktuellen Modus berechnen.
     */
    calculate_power_values();

    /*
     * Motorbefehle alle 30 ms übertragen.
     */
    motor_send_A(power_val_MA);
    motor_send_B(power_val_MB);

    /*
     * Display regelmäßig oder nach einer Änderung sofort
     * aktualisieren.
     */
    if ((display_available != 0U) &&
        ((display_force_update != 0U) ||
         ((uint32_t)(current_time -
                     last_display_time) >=
          DISPLAY_PERIOD_MS)))
    {
        last_display_time = current_time;
        display_force_update = 0U;

        update_display(current_time);
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
