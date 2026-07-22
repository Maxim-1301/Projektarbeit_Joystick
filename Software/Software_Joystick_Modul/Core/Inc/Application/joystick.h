/*
 * joystick.h
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#ifndef INC_APPLICATION_JOYSTICK_H_
#define INC_APPLICATION_JOYSTICK_H_

#include "stm32f3xx_hal.h"

/*
 * Normierter Joystickbereich entspricht direkt dem Motorbereich.
 */
#define JOYSTICK_MIN_VALUE       (-127)
#define JOYSTICK_MAX_VALUE       127
#define JOYSTICK_DEAD_ZONE       5

/*
 * ADC-Kalibrierwerte.
 *
 * Falls der Joystick maximal ungefähr 3,1 V erreicht,
 * kann JOYSTICK_ADC_MAX später beispielsweise auf etwa
 * 3850 eingestellt werden.
 */
#define JOYSTICK_ADC_MIN         0U
#define JOYSTICK_ADC_CENTER      2048U
#define JOYSTICK_ADC_MAX         4095U

/*
 * Globale Werte, ähnlich zum Aufbau der Bachelorarbeit.
 */
extern volatile uint16_t joystick_adc_values[3];

extern int16_t con_x;
extern int16_t con_y;
extern int16_t con_z;

extern uint8_t joystick_button_1;
extern uint8_t joystick_button_2;

HAL_StatusTypeDef joystick_init(void);
void joystick_update(void);

#endif
