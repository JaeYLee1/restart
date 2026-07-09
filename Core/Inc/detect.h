/*
 * detect.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_DETECT_H_
#define INC_DETECT_H_

#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * 실제 연결한 Detect Pin에 맞게 수정
 *
 * 예시:
 * PA0 사용 시
 * GPIOA / bit 0 / EXTI0_IRQn
 */

/* Detect GPIO */
#define DETECT_GPIO              GPIOE
#define DETECT_PIN_NUMBER        0
#define DETECT_PIN_MASK          (1 << DETECT_PIN_NUMBER)

/* RCC_AHB1ENR GPIOA Enable bit */
#define DETECT_GPIO_CLK_BIT      4

/* SYSCFG EXTICR Port Code
 * GPIOA = 0
 * GPIOB = 1
 * GPIOC = 2
 * GPIOD = 3
 * GPIOE = 4
 */
#define DETECT_PORT_CODE         4

/* EXTI IRQ */
#define DETECT_IRQn              EXTI0_IRQn

/* Detect Active Level
 * LOW면 모듈 결합으로 판단
 */
#define DETECT_ACTIVE_LEVEL      0

void Detect_Init(void);
void Detect_RtosInit(void);

/* stm32f4xx_it.c EXTI Handler에서 호출 */
void Detect_IrqHandler(void);

#endif /* INC_DETECT_H_ */
