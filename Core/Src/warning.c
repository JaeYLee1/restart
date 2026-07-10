/*
 * warning.c
 *
 *  Created on: 2026. 7. 9.
 *      Author: 한국전파진흥협회
 */


/*
 * warning.c
 *
 * AI drowsiness warning manager
 * - Level1: Buzzer
 * - Level2: Buzzer + LED Matrix + Motor slowdown hook
 * - GUI reset: buzzer/LED off + motor recovery hook
 */

#include "warning.h"

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "LED_M.h"
#include "motor.h"

/* 필요하면 system_data에 warning_level 반영할 때 사용 */
#include "system_data.h"

/* ================= Pin / HW 정의 ================= */

/*
 * Buzzer: PF14
 * Active High 기준
 */
#define WARNING_BUZZ_GPIO          GPIOF
#define WARNING_BUZZ_PIN_MASK      (1U << 14)

/*
 * LED Matrix 1: SPI1
 * PA5 = SCK
 * PA7 = MOSI
 * CS  = PD14
 */
#define LED_M1_SPI                 SPI1
#define LED_M1_CS_GPIO             GPIOD
#define LED_M1_CS_PIN_MASK         (1U << 14)

/*
 * LED Matrix 2: SPI4
 * PE12 = SCK
 * PE14 = MOSI
 * CS   = PE9
 */
#define LED_M2_SPI                 SPI4
#define LED_M2_CS_GPIO             GPIOE
#define LED_M2_CS_PIN_MASK         (1U << 9)

/* ================= 내부 상태 ================= */

#define WARNING_TASK_STACK_SIZE    256U
#define WARNING_TASK_PRIORITY      3U

static TaskHandle_t warning_task_handle = NULL;

static volatile WarningState_t warning_state = WARNING_STATE_NORMAL;
static volatile uint8_t warning_level = 0U;

/* LED Matrix instance */
static LED_M_Handle_t led_matrix1 =
{
    .spi = LED_M1_SPI,
    .cs_port = LED_M1_CS_GPIO,
    .cs_pin = LED_M1_CS_PIN_MASK
};

static LED_M_Handle_t led_matrix2 =
{
    .spi = LED_M2_SPI,
    .cs_port = LED_M2_CS_GPIO,
    .cs_pin = LED_M2_CS_PIN_MASK
};

/* 내부 함수 */
static void Warning_HwInit(void);
static void WarningTask(void *argument);

static void Warning_BuzzerOn(void);
static void Warning_BuzzerOff(void);

static void Warning_LedMatrixOff(void);
static void Warning_LedMatrixLevel1(void);
static void Warning_LedMatrixLevel2(void);

void Warning_Motor_OnLevel2(void)
{
    Motor_WarningSlowdown();
}


void Warning_Motor_OnRecovery(void)
{
    Motor_WarningRecover();
}
/* ================= Public API ================= */

void Warning_Init(void)
{
    warning_state = WARNING_STATE_NORMAL;
    warning_level = 0U;

    Warning_HwInit();

    LED_M_Init(&led_matrix1);
    LED_M_Init(&led_matrix2);

    Warning_BuzzerOff();
    Warning_LedMatrixOff();

    /*
     * system_data에 필드가 있다면 반영.
     * 없으면 이 부분은 주석 처리해도 됨.
     */
#if 0
    g_system_data.sleep_flag = 0U;
    g_system_data.warning_flag = 0U;
#endif
}


void Warning_RtosInit(void)
{
    if (xTaskCreate(WarningTask,
                    "WARNING",
                    WARNING_TASK_STACK_SIZE,
                    NULL,
                    WARNING_TASK_PRIORITY,
                    &warning_task_handle) != pdPASS)
    {
        Error_Handler();
    }
}


void Warning_OnAiLevel(uint8_t level)
{
    if (level > 2U)
    {
        level = 2U;
    }

    /*
     * Level2는 안전상 latch.
     * GUI reset(K) 전까지 유지.
     */
    taskENTER_CRITICAL();

    if (level == 0U)
    {
        /*
         * AI 정상 신호.
         * 단, 이미 Level1/Level2 경고가 들어간 상태라면
         * GUI reset 명령으로만 해제하는 구조로 둔다.
         */
        if (warning_state == WARNING_STATE_NORMAL)
        {
            warning_level = 0U;
        }
    }
    else if (level == 1U)
    {
        if ((warning_state == WARNING_STATE_NORMAL) ||
            (warning_state == WARNING_STATE_RECOVERY))
        {
            warning_level = 1U;
            warning_state = WARNING_STATE_LEVEL1;
        }
    }
    else
    {
        warning_level = 2U;
        warning_state = WARNING_STATE_LEVEL2;
    }

    taskEXIT_CRITICAL();

#if 0
    g_system_data.sleep_flag = warning_level;
    g_system_data.warning_flag = warning_level;
#endif

    if (warning_task_handle != NULL)
    {
        xTaskNotifyGive(warning_task_handle);
    }
}


void Warning_OnGuiReset(void)
{
    taskENTER_CRITICAL();

    warning_level = 0U;
    warning_state = WARNING_STATE_RECOVERY;

    taskEXIT_CRITICAL();

#if 0
    g_system_data.sleep_flag = 0U;
    g_system_data.warning_flag = 0U;
#endif

    if (warning_task_handle != NULL)
    {
        xTaskNotifyGive(warning_task_handle);
    }
}


uint8_t Warning_GetLevel(void)
{
    return warning_level;
}


WarningState_t Warning_GetState(void)
{
    return warning_state;
}

/* ================= Hardware Init ================= */

static void Warning_HwInit(void)
{
    /*
     * GPIOA, GPIOD, GPIOE, GPIOF clock enable
     * A: SPI1 pins
     * D: LED Matrix1 CS
     * E: SPI4 pins + LED Matrix2 CS
     * F: Buzzer
     */
    RCC->AHB1ENR |= (1U << 0);   /* GPIOA */
    RCC->AHB1ENR |= (1U << 3);   /* GPIOD */
    RCC->AHB1ENR |= (1U << 4);   /* GPIOE */
    RCC->AHB1ENR |= (1U << 5);   /* GPIOF */

    /*
     * SPI1: PA5=SCK, PA7=MOSI, AF5
     */
    GPIOA->MODER &= ~((3U << (5U * 2U)) |
                      (3U << (7U * 2U)));
    GPIOA->MODER |=  ((2U << (5U * 2U)) |
                      (2U << (7U * 2U)));

    GPIOA->AFR[0] &= ~((0xFU << (5U * 4U)) |
                       (0xFU << (7U * 4U)));
    GPIOA->AFR[0] |=  ((5U << (5U * 4U)) |
                       (5U << (7U * 4U)));

    GPIOA->OSPEEDR |= ((3U << (5U * 2U)) |
                       (3U << (7U * 2U)));

    /*
     * SPI4: PE12=SCK, PE14=MOSI, AF5
     */
    GPIOE->MODER &= ~((3U << (12U * 2U)) |
                      (3U << (14U * 2U)));
    GPIOE->MODER |=  ((2U << (12U * 2U)) |
                      (2U << (14U * 2U)));

    GPIOE->AFR[1] &= ~((0xFU << ((12U - 8U) * 4U)) |
                       (0xFU << ((14U - 8U) * 4U)));
    GPIOE->AFR[1] |=  ((5U << ((12U - 8U) * 4U)) |
                       (5U << ((14U - 8U) * 4U)));

    GPIOE->OSPEEDR |= ((3U << (12U * 2U)) |
                       (3U << (14U * 2U)));

    /*
     * LED Matrix 1 CS: PD14 output
     */
    GPIOD->MODER &= ~(3U << (14U * 2U));
    GPIOD->MODER |=  (1U << (14U * 2U));

    /*
     * LED Matrix 2 CS: PE9 output
     */
    GPIOE->MODER &= ~(3U << (9U * 2U));
    GPIOE->MODER |=  (1U << (9U * 2U));

    /*
     * Buzzer: PF14 output
     */
    GPIOF->MODER &= ~(3U << (14U * 2U));
    GPIOF->MODER |=  (1U << (14U * 2U));

    /*
     * CS idle high
     */
    LED_M1_CS_GPIO->BSRR = LED_M1_CS_PIN_MASK;
    LED_M2_CS_GPIO->BSRR = LED_M2_CS_PIN_MASK;

    /*
     * Buzzer off
     */
    WARNING_BUZZ_GPIO->BSRR = (WARNING_BUZZ_PIN_MASK << 16U);

    /*
     * SPI1 clock enable
     */
    RCC->APB2ENR |= (1U << 12U);

    SPI1->CR1 = 0U;
    SPI1->CR1 |= (1U << 2U);               /* MSTR */
    SPI1->CR1 |= (1U << 9U) | (1U << 8U);  /* SSM, SSI */
    SPI1->CR1 |= (4U << 3U);               /* BR = /32 */
    SPI1->CR1 |= (1U << 6U);               /* SPE */

    /*
     * SPI4 clock enable
     */
    RCC->APB2ENR |= (1U << 13U);

    SPI4->CR1 = 0U;
    SPI4->CR1 |= (1U << 2U);               /* MSTR */
    SPI4->CR1 |= (1U << 9U) | (1U << 8U);  /* SSM, SSI */
    SPI4->CR1 |= (4U << 3U);               /* BR = /32 */
    SPI4->CR1 |= (1U << 6U);               /* SPE */
}

/* ================= Buzzer / LED Control ================= */

static void Warning_BuzzerOn(void)
{
    WARNING_BUZZ_GPIO->BSRR = WARNING_BUZZ_PIN_MASK;
}


static void Warning_BuzzerOff(void)
{
    WARNING_BUZZ_GPIO->BSRR = (WARNING_BUZZ_PIN_MASK << 16U);
}


static void Warning_LedMatrixOff(void)
{
    LED_M_ClearDisplay(&led_matrix1);
    LED_M_ClearDisplay(&led_matrix2);
}


static void Warning_LedMatrixLevel1(void)
{
    LED_M_DisplayPattern(&led_matrix1, LED_M_PATTERN_WARNING_4);
    LED_M_DisplayPattern(&led_matrix2, LED_M_PATTERN_WARNING_4);
}


static void Warning_LedMatrixLevel2(void)
{
    LED_M_DisplayPattern(&led_matrix1, LED_M_PATTERN_WARNING_5);
    LED_M_DisplayPattern(&led_matrix2, LED_M_PATTERN_WARNING_5);
}

/* ================= Warning Task ================= */

static void WarningTask(void *argument)
{
    WarningState_t prev_state = WARNING_STATE_NORMAL;
    uint8_t blink_on = 0U;

    (void)argument;

    for (;;)
    {
        WarningState_t current_state;
        uint8_t current_level;

        current_state = warning_state;
        current_level = warning_level;

        /*
         * State transition hook
         */
        if (current_state != prev_state)
        {
            if (current_state == WARNING_STATE_LEVEL2)
            {
                Warning_Motor_OnLevel2();
            }
            else if (current_state == WARNING_STATE_RECOVERY)
            {
                Warning_Motor_OnRecovery();
            }

            prev_state = current_state;
        }

        switch (current_state)
        {
            case WARNING_STATE_NORMAL:
            {
                Warning_BuzzerOff();
                Warning_LedMatrixOff();
                blink_on = 0U;
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }

            case WARNING_STATE_LEVEL1:
            {
                /*
                 * 1단계: 부저만 짧게 반복
                 * 도트 매트릭스는 OFF
                 */
                Warning_LedMatrixOff();

                Warning_BuzzerOn();
                vTaskDelay(pdMS_TO_TICKS(80));

                Warning_BuzzerOff();
                vTaskDelay(pdMS_TO_TICKS(120));

                break;
            }

            case WARNING_STATE_LEVEL2:
            {
                /*
                 * 2단계: 부저 빠르게 반복 + 도트 매트릭스 점멸
                 */
                if (blink_on == 0U)
                {
                    if (current_level >= 2U)
                    {
                        Warning_LedMatrixLevel2();
                    }
                    else
                    {
                        Warning_LedMatrixLevel1();
                    }

                    Warning_BuzzerOn();
                    blink_on = 1U;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
                else
                {
                    Warning_LedMatrixOff();
                    Warning_BuzzerOff();
                    blink_on = 0U;
                    vTaskDelay(pdMS_TO_TICKS(100));
                }

                break;
            }

            case WARNING_STATE_RECOVERY:
            default:
            {
                /*
                 * GUI가 K를 누른 뒤 복구 상태.
                 * 부저/LED 즉시 끄고, motor hook은 위 transition에서 1회 호출.
                 */
                Warning_BuzzerOff();
                Warning_LedMatrixOff();
                blink_on = 0U;

                taskENTER_CRITICAL();
                warning_state = WARNING_STATE_NORMAL;
                warning_level = 0U;
                taskEXIT_CRITICAL();

                vTaskDelay(pdMS_TO_TICKS(50));
                break;
            }
        }
    }
}
