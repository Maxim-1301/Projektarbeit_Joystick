/*
 * joystick.c
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#include "Application/joystick.h"

#include "adc.h"
#include "main.h"

/*
 * Reihenfolge entsprechend der ADC-Konfiguration:
 *
 * Index 0: ADC4_IN3, X-Achse
 * Index 1: ADC4_IN4, Y-Achse
 * Index 2: ADC4_IN5, Z-Achse
 */
volatile uint16_t joystick_adc_values[3] =
{
    JOYSTICK_ADC_CENTER,
    JOYSTICK_ADC_CENTER,
    JOYSTICK_ADC_CENTER
};

int16_t con_x = 0;
int16_t con_y = 0;
int16_t con_z = 0;

uint8_t joystick_button_1 = 0U;
uint8_t joystick_button_2 = 0U;

static int16_t scale_axis(uint16_t adc_value)
{
    int32_t result;

    if (adc_value >= JOYSTICK_ADC_CENTER)
    {
        /*
         * Positive Hälfte:
         * ADC-Mitte bis ADC-Maximum wird auf 0 bis 127 normiert.
         */
        result =
            ((int32_t)(adc_value - JOYSTICK_ADC_CENTER) *
             JOYSTICK_MAX_VALUE) /
            (JOYSTICK_ADC_MAX - JOYSTICK_ADC_CENTER);
    }
    else
    {
        /*
         * Negative Hälfte:
         * ADC-Mitte bis ADC-Minimum wird auf 0 bis -127 normiert.
         */
        result =
            -((int32_t)(JOYSTICK_ADC_CENTER - adc_value) *
              JOYSTICK_MAX_VALUE) /
            (JOYSTICK_ADC_CENTER - JOYSTICK_ADC_MIN);
    }

    if (result > JOYSTICK_MAX_VALUE)
    {
        result = JOYSTICK_MAX_VALUE;
    }

    if (result < JOYSTICK_MIN_VALUE)
    {
        result = JOYSTICK_MIN_VALUE;
    }

    /*
     * Kleine Abweichungen um die mechanische Mittelstellung
     * werden ignoriert.
     */
    if ((result > -JOYSTICK_DEAD_ZONE) &&
        (result < JOYSTICK_DEAD_ZONE))
    {
        result = 0;
    }

    return (int16_t)result;
}

HAL_StatusTypeDef joystick_init(void)
{
    /*
     * ADC kalibrieren.
     */
    if (HAL_ADCEx_Calibration_Start(
            &hadc4,
            ADC_SINGLE_ENDED) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * ADC mit zirkulärem DMA starten.
     *
     * Der Pointer ist hier notwendig, weil die HAL wissen muss,
     * an welche Speicheradresse der DMA schreiben soll.
     */
    if (HAL_ADC_Start_DMA(
            &hadc4,
            (uint32_t *)(uintptr_t)joystick_adc_values,
            3U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void joystick_update(void)
{
    /*
     * scale_axis() wird hier für jede Achse aufgerufen.
     */
    con_x = scale_axis(joystick_adc_values[0]);
    con_y = scale_axis(joystick_adc_values[1]);
    con_z = scale_axis(joystick_adc_values[2]);

    /*
     * Taster sind durch Pull-up aktiv-low:
     * Gedrückt bedeutet GPIO_PIN_RESET.
     */
    if (HAL_GPIO_ReadPin(
            Joystick_Taster1_GPIO_Port,
            Joystick_Taster1_Pin) == GPIO_PIN_RESET)
    {
        joystick_button_1 = 1U;
    }
    else
    {
        joystick_button_1 = 0U;
    }

    if (HAL_GPIO_ReadPin(
            Joystick_Taster2_GPIO_Port,
            Joystick_Taster2_Pin) == GPIO_PIN_RESET)
    {
        joystick_button_2 = 1U;
    }
    else
    {
        joystick_button_2 = 0U;
    }
}
