#ifndef UART_RS485_H
#define UART_RS485_H

#include <stdint.h>
#include <stdbool.h>

#define RS485_FRAME_LENGTH  5U

/* Commands used by the communication module for motor telemetry. */
#define RS485_COMMAND_BATTERY_VOLTAGE   0x1DU
#define RS485_COMMAND_BATTERY_CAPACITY  0x1EU
#define RS485_COMMAND_MOTOR_SPEED       0x20U
#define RS485_COMMAND_MOTOR_POWER       0x21U
#define RS485_COMMAND_MOTOR_TEMP        0x23U
#define RS485_COMMAND_GPS_SPEED         0x26U

typedef struct
{
    uint8_t data[RS485_FRAME_LENGTH];
    bool checksum_ok;
    uint32_t error_flags;
} rs485_frame_t;

/*
 * Telemetry test values.
 *
 * These variables are volatile so they can be changed through
 * Live Expressions while the test application is running.
 */
extern volatile uint16_t rs485_test_motor_speed_rpm;
extern volatile uint16_t rs485_test_motor_power_w;
extern volatile uint16_t rs485_test_battery_voltage_01v;
extern volatile uint8_t rs485_test_battery_capacity_percent;
extern volatile uint16_t rs485_test_gps_speed_01ms;
extern volatile uint16_t rs485_test_motor_temperature_01c;

void uart_rs485_start(void);
void uart_rs485_process(void);

bool uart_rs485_get_frame(rs485_frame_t *frame);
bool uart_rs485_get_raw_byte(uint8_t *byte);

uint32_t uart_rs485_get_lost_frame_count(void);
uint32_t uart_rs485_get_error_count(void);
uint32_t uart_rs485_get_rx_byte_count(void);
uint32_t uart_rs485_get_raw_lost_count(void);

uint8_t uart_rs485_get_last_rx_byte(void);
uint32_t uart_rs485_get_last_uart_error(void);
uint32_t uart_rs485_get_last_usart_isr(void);

/* Einzelne Fehlerzähler */
uint32_t uart_rs485_get_framing_error_count(void);
uint32_t uart_rs485_get_noise_error_count(void);
uint32_t uart_rs485_get_overrun_error_count(void);
uint32_t uart_rs485_get_parity_error_count(void);

/* Transmit diagnostics. */
uint32_t uart_rs485_get_tx_frame_count(void);
uint32_t uart_rs485_get_tx_error_count(void);
uint8_t uart_rs485_get_last_tx_command(void);
uint16_t uart_rs485_get_last_tx_value(void);
void uart_rs485_get_last_tx_frame(uint8_t frame[RS485_FRAME_LENGTH]);

/* Nur für Test ohne echte RS485-Hardware */
void uart_rs485_test_inject_frame(void);

#endif
