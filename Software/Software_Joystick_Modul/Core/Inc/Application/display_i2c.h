/*
 * display_i2c.h
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#ifndef INC_APPLICATION_DISPLAY_I2C_H_
#define INC_APPLICATION_DISPLAY_I2C_H_


#include "stm32f3xx_hal.h"

#define DISPLAY_COLUMNS  20U
#define DISPLAY_ROWS     4U

/*
 * I2C-Adresse des PCF8574-Backpacks.
 *
 * Viele Backpacks verwenden 0x27.
 * Manche verwenden 0x3F.
 *
 * Die STM32-HAL erwartet die Adresse um ein Bit
 * nach links verschoben.
 */
#define DISPLAY_I2C_ADDRESS  (0x27U << 1U)

/*
 * Display initialisieren.
 */
HAL_StatusTypeDef Display_Init(
    I2C_HandleTypeDef *hi2c);

/*
 * Gesamtes Display löschen.
 */
HAL_StatusTypeDef Display_Clear(void);

/*
 * Cursor positionieren.
 *
 * row:    0 bis 3
 * column: 0 bis 19
 */
HAL_StatusTypeDef Display_SetCursor(
    uint8_t row,
    uint8_t column);

/*
 * Text ab aktueller Cursorposition ausgeben.
 */
HAL_StatusTypeDef Display_Write(
    const char *text);

/*
 * Eine vollständige Displayzeile ausgeben.
 *
 * Kürzere Texte werden automatisch mit Leerzeichen aufgefüllt.
 * Längere Texte werden nach 20 Zeichen abgeschnitten.
 */
HAL_StatusTypeDef Display_WriteLine(
    uint8_t row,
    const char *text);

/*
 * Hintergrundbeleuchtung ein- oder ausschalten.
 *
 * enabled = 0: aus
 * enabled = 1: an
 */
HAL_StatusTypeDef Display_SetBacklight(
    uint8_t enabled);


#endif
