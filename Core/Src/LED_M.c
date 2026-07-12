/*
 * LED_M.c
 *
 *  Created on: 2026. 7. 9.
 *      Author: 한국전파진흥협회
 */

#include "LED_M.h"

const uint8_t LED_M_PATTERN_WARNING_4[8] =
{
    0b00011000,
    0b00111100,
    0b00111100,
    0b00011000,
    0b00011000,
    0b00000000,
    0b00011000,
    0b00011000
};

const uint8_t LED_M_PATTERN_WARNING_5[8] =
{
    0b10000001,
    0b01000010,
    0b00100100,
    0b00011000,
    0b00011000,
    0b00100100,
    0b01000010,
    0b10000001
};

const uint8_t LED_M_PATTERN_ALL_ON[8] =
{
    0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF
};

const uint8_t LED_M_PATTERN_ALL_OFF[8] =
{
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};

/*
#define SPI_SR_RXNE   (1U << 0)
#define SPI_SR_TXE    (1U << 1)
#define SPI_SR_BSY    (1U << 7)
*/

static void LED_M_CS_Low(LED_M_Handle_t *led)
{
    led->cs_port->BSRR = ((uint32_t)led->cs_pin << 16U);
}

static void LED_M_CS_High(LED_M_Handle_t *led)
{
    led->cs_port->BSRR = led->cs_pin;
}

static void SPI_Transmit8(SPI_TypeDef *spi, uint8_t data)
{
    while ((spi->SR & SPI_SR_TXE) == 0U)
    {
    }

    *(volatile uint8_t *)&spi->DR = data;

    while ((spi->SR & SPI_SR_RXNE) == 0U)
    {
    }

    (void)spi->DR;

    while ((spi->SR & SPI_SR_BSY) != 0U)
    {
    }
}

void LED_M_WriteReg(LED_M_Handle_t *led, uint8_t reg, uint8_t data)
{
    LED_M_CS_Low(led);

    SPI_Transmit8(led->spi, reg);
    SPI_Transmit8(led->spi, data);

    LED_M_CS_High(led);
}

void LED_M_Init(LED_M_Handle_t *led)
{
    LED_M_CS_High(led);

    LED_M_WriteReg(led, LED_M_REG_DISPLAYTEST, 0x00);
    LED_M_WriteReg(led, LED_M_REG_DECODEMODE, 0x00);
    LED_M_WriteReg(led, LED_M_REG_SCANLIMIT, 0x07);
    LED_M_WriteReg(led, LED_M_REG_INTENSITY, 0x08);
    LED_M_WriteReg(led, LED_M_REG_SHUTDOWN, 0x01);

    LED_M_ClearDisplay(led);
}

void LED_M_ClearDisplay(LED_M_Handle_t *led)
{
    LED_M_DisplayPattern(led, LED_M_PATTERN_ALL_OFF);
}

void LED_M_DisplayPattern(LED_M_Handle_t *led, const uint8_t *pattern8)
{
    uint8_t row;

    if ((led == 0) || (pattern8 == 0))
    {
        return;
    }

    for (row = 0U; row < 8U; row++)
    {
        LED_M_WriteReg(led, (uint8_t)(LED_M_REG_DIGIT0 + row), pattern8[row]);
    }
}

void LED_M_SetIntensity(LED_M_Handle_t *led, uint8_t intensity)
{
    if (intensity > 0x0F)
    {
        intensity = 0x0F;
    }

    LED_M_WriteReg(led, LED_M_REG_INTENSITY, intensity);
}

void LED_M_ShutdownMode(LED_M_Handle_t *led, uint8_t on)
{
    LED_M_WriteReg(led, LED_M_REG_SHUTDOWN, on ? 0x01 : 0x00);
}
