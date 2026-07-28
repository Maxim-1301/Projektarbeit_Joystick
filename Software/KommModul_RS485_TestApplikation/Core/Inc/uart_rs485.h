#ifndef UART_RS485_H
#define UART_RS485_H

#include <stdint.h>
#include <stdbool.h>

#define RS485_FRAME_LENGTH  5U

typedef struct
{
    uint8_t data[RS485_FRAME_LENGTH];
    bool checksum_ok;
    uint32_t error_flags;
} rs485_frame_t;

void uart_rs485_start(void);

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

/* Nur für Test ohne echte RS485-Hardware */
void uart_rs485_test_inject_frame(void);

#endif
