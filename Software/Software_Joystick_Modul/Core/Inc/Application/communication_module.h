/*
 * communication_module.h
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#ifndef INC_APPLICATION_COMMUNICATION_MODULE_H_
#define INC_APPLICATION_COMMUNICATION_MODULE_H_


#include "stm32f3xx_hal.h"

/* Aktuelle Leistungswerte der beiden Motoren */
extern int16_t power_val_MA;
extern int16_t power_val_MB;

/* Initialisierung und zyklische Verarbeitung */
HAL_StatusTypeDef communication_module_init(void);
void communication_module_process(void);

/* UART-Callbacks */
void communication_uart_rx_callback(
    UART_HandleTypeDef *huart);

void communication_uart_error_callback(
    UART_HandleTypeDef *huart);


#endif
