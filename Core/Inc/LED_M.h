/*
 * LED_M.h
 *
 *  Created on: 2026. 7. 9.
 *      Author: 한국전파진흥협회
 */

#ifndef LED_M_H
#define LED_M_H

#include "stm32f4xx.h"
#include <stdint.h>

#define LED_M_REG_NOOP        0x00
#define LED_M_REG_DECODEMODE  0x09
#define LED_M_REG_INTENSITY   0x0A
#define LED_M_REG_SCANLIMIT   0x0B
#define LED_M_REG_SHUTDOWN    0x0C
#define LED_M_REG_DISPLAYTEST 0x0F
#define LED_M_REG_DIGIT0      0x01

typedef struct
{
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;
} LED_M_Handle_t;

void LED_M_Init(LED_M_Handle_t *led);
void LED_M_WriteReg(LED_M_Handle_t *led, uint8_t reg, uint8_t data);
void LED_M_ClearDisplay(LED_M_Handle_t *led);
void LED_M_DisplayPattern(LED_M_Handle_t *led, const uint8_t *pattern8);
void LED_M_SetIntensity(LED_M_Handle_t *led, uint8_t intensity);
void LED_M_ShutdownMode(LED_M_Handle_t *led, uint8_t on);

extern const uint8_t LED_M_PATTERN_WARNING_4[8];
extern const uint8_t LED_M_PATTERN_WARNING_5[8];
extern const uint8_t LED_M_PATTERN_ALL_ON[8];
extern const uint8_t LED_M_PATTERN_ALL_OFF[8];

#endif /* LED_M_H */
