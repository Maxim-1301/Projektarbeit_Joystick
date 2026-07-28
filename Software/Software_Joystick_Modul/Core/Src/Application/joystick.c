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
    int32_t adc;
    int32_t adc_min;
    int32_t adc_center;
    int32_t adc_max;
    int32_t result;

    /*
     * Alle Werte ausdrücklich vorzeichenbehaftet behandeln.
     */
    adc        = (int32_t)adc_value;
    adc_min    = (int32_t)JOYSTICK_ADC_MIN;
    adc_center = (int32_t)JOYSTICK_ADC_CENTER;
    adc_max    = (int32_t)JOYSTICK_ADC_MAX;

    /*
     * Negative Hälfte:
     * ADC-Minimum bis ADC-Mitte wird auf -127 bis 0 skaliert.
     */
    if (adc < adc_center)
    {
        if (adc <= adc_min)
        {
            result = JOYSTICK_MIN_VALUE;
        }
        else
        {
            result =
                -((adc_center - adc) * JOYSTICK_MAX_VALUE) /
                 (adc_center - adc_min);
        }
    }

    /*
     * Positive Hälfte:
     * ADC-Mitte bis ADC-Maximum wird auf 0 bis +127 skaliert.
     */
    else
    {
        if (adc >= adc_max)
        {
            result = JOYSTICK_MAX_VALUE;
        }
        else
        {
            result =
                ((adc - adc_center) * JOYSTICK_MAX_VALUE) /
                (adc_max - adc_center);
        }
    }

    /*
     * Totzone einschließlich der Grenzwerte.
     */
    if ((result >= -JOYSTICK_DEAD_ZONE) &&
        (result <= JOYSTICK_DEAD_ZONE))
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
     * CubeMX aktiviert den DMA2-Channel2-Interrupt automatisch.
     * Für das direkte Lesen des zirkulären DMA-Puffers wird er
     * nicht benötigt.
     */
    HAL_NVIC_DisableIRQ(DMA2_Channel2_IRQn);

    /*
     * ADC mit zirkulärem DMA starten.
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
