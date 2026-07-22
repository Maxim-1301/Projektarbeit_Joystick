/*
 * motor_rs485.h
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#ifndef INC_APPLICATION_MOTOR_RS485_H_
#define INC_APPLICATION_MOTOR_RS485_H_


#include "stm32f3xx_hal.h"


/* Allgemeine Frame-Konfiguration */
#define MOTOR_FRAME_SIZE         5U
#define ADDR_MAIN_SYSTEM         0xC5U

/* Befehle an den Motor */
#define POWER_THROTTLE           0x25U
#define MOTOR_STOP               0x26U
#define EMPTY_DATABYTE           0x00U

#define MOTOR_MAX_POWER          127

/* Empfangene Motorstatus-Kommandos */
#define BATTERY_VOLTAGE          0x1DU
#define BATTERY_CAPACITY         0x1EU
#define MOTOR_SPEED              0x20U
#define MOTOR_POWER              0x21U
#define MOTOR_TEMP               0x23U
#define GPS_SPEED                0x26U

/* Empfangspuffer */
extern volatile uint8_t rec_buffer_A[MOTOR_FRAME_SIZE];
extern volatile uint8_t rec_buffer_B[MOTOR_FRAME_SIZE];

extern volatile uint8_t UARTA_i;
extern volatile uint8_t UARTB_i;

extern volatile uint8_t frame_ready_A;
extern volatile uint8_t frame_ready_B;

/* Statuswerte von Motor A */
extern uint16_t motor_speed_A;
extern uint16_t motor_power_A;
extern uint16_t battery_voltage_A;
extern uint8_t battery_capacity_A;
extern uint16_t gps_speed_A;
extern uint16_t motor_temperature_A;

/* Statuswerte von Motor B */
extern uint16_t motor_speed_B;
extern uint16_t motor_power_B;
extern uint16_t battery_voltage_B;
extern uint8_t battery_capacity_B;
extern uint16_t gps_speed_B;
extern uint16_t motor_temperature_B;

/* Statistik zur Überwachung */
extern uint32_t valid_frames_A;
extern uint32_t valid_frames_B;

extern uint32_t checksum_errors_A;
extern uint32_t checksum_errors_B;


/* Initialisierung */
void motor_rs485_init(void);

/* RS485-Senderichtung */
void send_en_A(uint8_t enable);
void send_en_B(uint8_t enable);

/* Motorbefehle senden */
void motor_send_A(int16_t power);
void motor_send_B(int16_t power);

/* Empfangene Frames auswerten */
void motor_process_received_frames(void);

/* HAL-UART-Callbacks */
void motor_uart_rx_callback(
    UART_HandleTypeDef *huart);

void motor_uart_error_callback(
    UART_HandleTypeDef *huart);


#endif
