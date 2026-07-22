/*
 * display_i2c.c
 *
 *  Created on: 21.07.2026
 *      Author: jonat
 */


#include "Application/display_i2c.h"

#include <string.h>

/*
 * PCF8574-Pin-Belegung:
 *
 * P0 = RS
 * P1 = RW
 * P2 = Enable
 * P3 = Hintergrundbeleuchtung
 * P4 = D4
 * P5 = D5
 * P6 = D6
 * P7 = D7
 */
#define LCD_RS_BIT           0x01U
#define LCD_RW_BIT           0x02U
#define LCD_ENABLE_BIT       0x04U
#define LCD_BACKLIGHT_BIT    0x08U

#define LCD_I2C_TIMEOUT_MS   10U

/*
 * Verwendete I2C-Schnittstelle.
 */
static I2C_HandleTypeDef *display_i2c;

/*
 * Zustand der Hintergrundbeleuchtung.
 *
 * Beim Programmstart ist die Beleuchtung eingeschaltet.
 */
static uint8_t backlight_state = LCD_BACKLIGHT_BIT;

static HAL_StatusTypeDef expander_write(
    uint8_t value)
{
    /*
     * Hintergrundbeleuchtung zum Ausgangswert hinzufügen.
     */
    value |= backlight_state;

    return HAL_I2C_Master_Transmit(
        display_i2c,
        DISPLAY_I2C_ADDRESS,
        &value,
        1U,
        LCD_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef write_nibble(
    uint8_t nibble,
    uint8_t register_select)
{
    uint8_t output;
    HAL_StatusTypeDef status;

    /*
     * Das Nibble wird auf P4 bis P7 gelegt.
     *
     * register_select ist entweder:
     *
     * 0: Kommando
     * LCD_RS_BIT: Zeichen
     */
    output =
        (uint8_t)((nibble & 0x0FU) << 4U) |
        register_select;

    /*
     * Enable auf High setzen.
     */
    status = expander_write(
        output | LCD_ENABLE_BIT);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Enable wieder auf Low setzen.
     * Durch diese fallende Flanke übernimmt das Display die Daten.
     */
    status = expander_write(
        output & (uint8_t)~LCD_ENABLE_BIT);

    return status;
}

static HAL_StatusTypeDef write_byte(
    uint8_t value,
    uint8_t register_select)
{
    HAL_StatusTypeDef status;

    /*
     * Im 4-Bit-Modus wird zuerst das obere Nibble übertragen.
     */
    status = write_nibble(
        (uint8_t)(value >> 4U),
        register_select);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Danach wird das untere Nibble übertragen.
     */
    status = write_nibble(
        value & 0x0FU,
        register_select);

    return status;
}

static HAL_StatusTypeDef send_command(
    uint8_t command)
{
    HAL_StatusTypeDef status;

    /*
     * RS = 0 bedeutet Kommando.
     */
    status = write_byte(command, 0U);

    /*
     * Clear Display und Return Home benötigen laut
     * HD44780-Datenblatt eine längere Ausführungszeit.
     */
    if ((command == 0x01U) ||
        (command == 0x02U))
    {
        HAL_Delay(2U);
    }

    return status;
}

static HAL_StatusTypeDef send_character(
    char character)
{
    /*
     * RS = 1 bedeutet darzustellendes Zeichen.
     */
    return write_byte(
        (uint8_t)character,
        LCD_RS_BIT);
}

HAL_StatusTypeDef Display_Init(
    I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    display_i2c = hi2c;

    /*
     * Prüfen, ob das Backpack unter der eingestellten
     * I2C-Adresse erreichbar ist.
     */
    status = HAL_I2C_IsDeviceReady(
        display_i2c,
        DISPLAY_I2C_ADDRESS,
        3U,
        LCD_I2C_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Das Display benötigt nach dem Einschalten mindestens
     * etwa 40 ms Wartezeit.
     */
    HAL_Delay(50U);

    /*
     * Initialisierungssequenz nach HD44780-Datenblatt.
     *
     * Das Display startet möglicherweise im 8-Bit-Modus.
     * Durch dreimal 0x03 und anschließend 0x02 wird
     * zuverlässig in den 4-Bit-Modus gewechselt.
     */
    status = write_nibble(0x03U, 0U);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(5U);

    status = write_nibble(0x03U, 0U);

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(1U);

    status = write_nibble(0x03U, 0U);

    if (status != HAL_OK)
    {
        return status;
    }

    status = write_nibble(0x02U, 0U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Function Set:
     *
     * 4-Bit-Modus
     * zweizeilige Controllerbetriebsart
     * Zeichensatz mit 5x8 Pixeln
     *
     * Auch ein 4x20-Display verwendet beim Controller
     * diese zweizeilige Betriebsart.
     */
    status = send_command(0x28U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Display ein
     * Cursor aus
     * Blinken aus
     */
    status = send_command(0x0CU);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Cursor nach jedem Zeichen nach rechts bewegen.
     */
    status = send_command(0x06U);

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Display löschen.
     */
    status = Display_Clear();

    return status;
}

HAL_StatusTypeDef Display_Clear(void)
{
    /*
     * HD44780-Kommando 0x01:
     * Display löschen und Cursor auf Anfang setzen.
     */
    return send_command(0x01U);
}

HAL_StatusTypeDef Display_SetCursor(
    uint8_t row,
    uint8_t column)
{
    /*
     * Speicheradressen eines 4x20-Displays.
     *
     * Zeile 1 beginnt bei 0x00.
     * Zeile 2 beginnt bei 0x40.
     * Zeile 3 beginnt bei 0x14.
     * Zeile 4 beginnt bei 0x54.
     */
    static const uint8_t row_addresses[DISPLAY_ROWS] =
    {
        0x00U,
        0x40U,
        0x14U,
        0x54U
    };

    uint8_t address;

    if (row >= DISPLAY_ROWS)
    {
        return HAL_ERROR;
    }

    if (column >= DISPLAY_COLUMNS)
    {
        return HAL_ERROR;
    }

    address = row_addresses[row] + column;

    /*
     * Bit 7 setzt das HD44780-Kommando
     * "Set DDRAM Address".
     */
    return send_command(
        0x80U | address);
}

HAL_StatusTypeDef Display_Write(
    const char *text)
{
    HAL_StatusTypeDef status;

    if (text == NULL)
    {
        return HAL_ERROR;
    }

    status = HAL_OK;

    while (*text != '\0')
    {
        status = send_character(*text);

        if (status != HAL_OK)
        {
            return status;
        }

        text++;
    }

    return status;
}

HAL_StatusTypeDef Display_WriteLine(
    uint8_t row,
    const char *text)
{
    char line[DISPLAY_COLUMNS + 1U];

    uint8_t index;

    if (row >= DISPLAY_ROWS)
    {
        return HAL_ERROR;
    }

    if (text == NULL)
    {
        return HAL_ERROR;
    }

    /*
     * Die gesamte Zeile zunächst mit Leerzeichen füllen.
     * Dadurch werden alte, längere Texte vollständig gelöscht.
     */
    for (index = 0U;
         index < DISPLAY_COLUMNS;
         index++)
    {
        line[index] = ' ';
    }

    /*
     * Text maximal bis zur zwanzigsten Stelle kopieren.
     */
    index = 0U;

    while ((text[index] != '\0') &&
           (index < DISPLAY_COLUMNS))
    {
        line[index] = text[index];
        index++;
    }

    /*
     * String abschließen.
     */
    line[DISPLAY_COLUMNS] = '\0';

    /*
     * Cursor an den Anfang der gewünschten Zeile setzen.
     */
    if (Display_SetCursor(row, 0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * Immer genau 20 Zeichen schreiben.
     */
    return Display_Write(line);
}

HAL_StatusTypeDef Display_SetBacklight(
    uint8_t enabled)
{
    if (enabled != 0U)
    {
        backlight_state = LCD_BACKLIGHT_BIT;
    }
    else
    {
        backlight_state = 0U;
    }

    /*
     * Neuen Zustand direkt an den Portexpander übertragen.
     */
    return expander_write(0U);
}
