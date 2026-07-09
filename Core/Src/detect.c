/*
 * detect.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


/*
 * detect.c
 *
 * Detect GPIO + EXTI + FreeRTOS Task
 */

#include "detect.h"
#include "module_manager.h"
#include "system_data.h"

/* 디바운싱 시간 */
#define DETECT_DEBOUNCE_MS       30U

/* Detect Task Handle */
static TaskHandle_t detect_task_handle;

/* 현재 안정된 Detect 상태 */
static uint8_t detect_active_state = 0U;


/* 내부 함수 선언 */
static uint8_t Detect_ReadPin(void);
static void DetectTask(void *argument);
static void Detect_ProcessState(void);


/* ================= Detect 초기화 ================= */

// GPIOE0 EXTI 설정 초기화
void Detect_Init(void)
{
    uint32_t exti_index;
    uint32_t exti_shift;

    /* GPIO Clock Enable */
    RCC->AHB1ENR |= (1U << DETECT_GPIO_CLK_BIT);

    /* SYSCFG Clock Enable */
    RCC->APB2ENR |= (1U << 14);

    /* Detect Pin = Input Mode */
    DETECT_GPIO->MODER &= ~(3U << (DETECT_PIN_NUMBER * 2U));

    /* Pull-up */
    DETECT_GPIO->PUPDR &= ~(3U << (DETECT_PIN_NUMBER * 2U));
    DETECT_GPIO->PUPDR |=  (1U << (DETECT_PIN_NUMBER * 2U));

    /*
     * EXTI Port 연결
     * EXTI0~3   : EXTICR[0]
     * EXTI4~7   : EXTICR[1]
     * EXTI8~11  : EXTICR[2]
     * EXTI12~15 : EXTICR[3]
     */
    exti_index = DETECT_PIN_NUMBER / 4U;
    exti_shift = (DETECT_PIN_NUMBER % 4U) * 4U;

    SYSCFG->EXTICR[exti_index] &= ~(0xFU << exti_shift);
    SYSCFG->EXTICR[exti_index] |= (DETECT_PORT_CODE << exti_shift);

    /* Rising / Falling Edge 모두 감지 */
    EXTI->RTSR |= DETECT_PIN_MASK;
    EXTI->FTSR |= DETECT_PIN_MASK;

    /* Pending Clear */
    EXTI->PR = DETECT_PIN_MASK;

    /* EXTI Interrupt Enable */
    EXTI->IMR |= DETECT_PIN_MASK;
}


void Detect_RtosInit(void)
{
    /* Detect Task 생성 */
    if (xTaskCreate(DetectTask,
                    "DETECT",
                    256,
                    NULL,
                    3,
                    &detect_task_handle) != pdPASS)
    {
        Error_Handler();
    }

    /*
     * vTaskNotifyGiveFromISR() 사용
     * FreeRTOSConfig.h의
     * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 값과 맞춤
     */
    NVIC_SetPriority(DETECT_IRQn, 5);
    NVIC_EnableIRQ(DETECT_IRQn);
}


/* ================= EXTI ISR 처리 ================= */

/*
 * stm32f4xx_it.c EXTI Handler에서 호출
 * ISR에서는 Task만 깨움
 */
void Detect_IrqHandler(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* EXTI Pending 확인 */
    if (EXTI->PR & DETECT_PIN_MASK)
    {
        /* Pending Clear */
        EXTI->PR = DETECT_PIN_MASK;

        /* DetectTask 깨움 */
        if (detect_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(detect_task_handle,
                                   &higher_priority_task_woken);
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


/* ================= Detect Task ================= */

static uint8_t Detect_ReadPin(void)
{
    uint8_t pin_level;

    pin_level = (DETECT_GPIO->IDR & DETECT_PIN_MASK) ? 1U : 0U;

    if (pin_level == DETECT_ACTIVE_LEVEL)
    {
        return 1;
    }

    return 0;
}


/*
 * GPIO 상태가 바뀌었을 때만
 * ModuleManager에 결합 / 분리 전달
 */
static void Detect_ProcessState(void)
{
    uint8_t current_detect_state;

    current_detect_state = Detect_ReadPin();

    /* 이전 상태와 같으면 무시 */
    if (current_detect_state == detect_active_state)
    {
        return;
    }

    detect_active_state = current_detect_state;

    /* GUI 상태 반영 */
    g_system_data.detect_state = detect_active_state;

    /* 모듈 결합 */
    if (detect_active_state == 1U)
    {
        g_system_data.detect_state = 1U;
        g_system_data.relay_state = 0U;
        g_system_data.fsm_state = FSM_DETECTED;
    }
    /* 모듈 분리 */
    else
    {
        g_system_data.detect_state = 0U;
        g_system_data.relay_state = 0U;
        g_system_data.fsm_state = FSM_IDLE;

        ModuleManager_OnDetached();
    }
}


static void DetectTask(void *argument)
{
    (void)argument;

    /* 부팅 시 현재 핀 상태 한번 확인 */
    detect_active_state = Detect_ReadPin();

    g_system_data.detect_state = detect_active_state;

    if (detect_active_state == 1U)
    {
        g_system_data.relay_state = 0U;
        g_system_data.fsm_state = FSM_DETECTED;
    }
    else
    {
        g_system_data.relay_state = 0U;
        g_system_data.fsm_state = FSM_IDLE;
    }

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(DETECT_DEBOUNCE_MS));

        Detect_ProcessState();
    }
}
