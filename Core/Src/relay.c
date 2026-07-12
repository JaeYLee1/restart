/*
 * relay.c
 *
 *  Created on: 2026. 7. 10.
 *      Author: 한국전파진흥협회
 */

#include "relay.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system_data.h"
#include "module_manager.h"

/* ================= Pin Define ================= */

/*
 * Relay Active Low
 * PE5 : Precharge Relay
 * PE6 : Bypass Relay
 *
 * ON  = LOW
 * OFF = HIGH
 */
#define PRECHARGE_PIN_NUMBER      5U
#define BYPASS_PIN_NUMBER         6U

#define PRECHARGE_PIN_MASK        (1U << PRECHARGE_PIN_NUMBER)
#define BYPASS_PIN_MASK           (1U << BYPASS_PIN_NUMBER)

/* Sequence Timing */
#define RELAY_PRECHARGE_TIME_MS   1000U
#define RELAY_BYPASS_SETTLE_MS    200U

/* ================= Internal State ================= */

typedef enum
{
    RELAY_CMD_NONE = 0,
    RELAY_CMD_CONNECT,
    RELAY_CMD_DISCONNECT
} RelayCommand_t;

static TaskHandle_t relay_task_handle = NULL;

static volatile RelayState_t relay_state = RELAY_STATE_OFF;
static volatile RelayCommand_t relay_cmd = RELAY_CMD_NONE;

/* ================= Internal Function ================= */

static void Relay_GPIO_Init(void);
static void RelayTask(void *argument);

static void Relay_ProcessConnect(void);
static void Relay_ProcessDisconnect(void);

/* ================= Init ================= */

void Relay_Init(void)
{
    Relay_GPIO_Init();

    Precharge_Off();
    Bypass_Off();

    relay_state = RELAY_STATE_OFF;
    relay_cmd = RELAY_CMD_NONE;

    g_system_data.relay_state = 0U;
}


void Relay_RtosInit(void)
{
    if (xTaskCreate(RelayTask,
                    "RELAY",
                    256,
                    NULL,
                    3,
                    &relay_task_handle) != pdPASS)
    {
        Error_Handler();
    }
}


static void Relay_GPIO_Init(void)
{
    /* GPIOE Clock Enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    /*
     * 출력 모드로 바꾸기 전에 HIGH를 먼저 걸어서
     * Active Low 릴레이가 순간적으로 켜지는 것을 방지
     */
    GPIOE->BSRR = PRECHARGE_PIN_MASK | BYPASS_PIN_MASK;

    /* PE5, PE6 Output Mode */
    GPIOE->MODER &= ~((3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (3U << (BYPASS_PIN_NUMBER * 2U)));

    GPIOE->MODER |=  ((1U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (1U << (BYPASS_PIN_NUMBER * 2U)));

    /* Push-Pull */
    GPIOE->OTYPER &= ~(PRECHARGE_PIN_MASK | BYPASS_PIN_MASK);

    /* Low Speed */
    GPIOE->OSPEEDR &= ~((3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                        (3U << (BYPASS_PIN_NUMBER * 2U)));

    /* No Pull-up / Pull-down */
    GPIOE->PUPDR &= ~((3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (3U << (BYPASS_PIN_NUMBER * 2U)));
}

/* ================= Request API ================= */

uint8_t Relay_RequestConnect(void)
{
    if (relay_task_handle == NULL)
    {
        return 0U;
    }

    /*
     * 이미 연결 상태면 재인증만 수행
     */
    if (relay_state == RELAY_STATE_CONNECTED)
    {
        g_system_data.relay_state = 1U;
        g_system_data.fsm_state = FSM_AUTHENTICATING;

        ModuleManager_OnAttached();

        return 1U;
    }

    /*
     * 시퀀스 진행 중이면 중복 요청 무시
     */
    if ((relay_state == RELAY_STATE_PRECHARGE) ||
        (relay_state == RELAY_STATE_BYPASS))
    {
        return 0U;
    }

    relay_cmd = RELAY_CMD_CONNECT;
    xTaskNotifyGive(relay_task_handle);

    return 1U;
}


uint8_t Relay_RequestDisconnect(void)
{
    if (relay_task_handle == NULL)
    {
        Relay_ForceOff();
        return 0U;
    }

    relay_cmd = RELAY_CMD_DISCONNECT;
    xTaskNotifyGive(relay_task_handle);

    return 1U;
}


void Relay_ForceOff(void)
{
    Precharge_Off();
    Bypass_Off();

    relay_state = RELAY_STATE_OFF;

    g_system_data.relay_state = 0U;

    if (g_system_data.detect_state == 1U)
    {
        g_system_data.fsm_state = FSM_DETECTED;
    }
    else
    {
        g_system_data.fsm_state = FSM_IDLE;
    }

    ModuleManager_OnDetached();
}


RelayState_t Relay_GetState(void)
{
    return relay_state;
}


uint8_t Relay_IsConnected(void)
{
    return (relay_state == RELAY_STATE_CONNECTED) ? 1U : 0U;
}

/* ================= Relay Low-level Control ================= */

void Precharge_On(void)
{
    /* Active Low: LOW = ON */
    GPIOE->BSRR = (PRECHARGE_PIN_MASK << 16U);
}


void Precharge_Off(void)
{
    /* Active Low: HIGH = OFF */
    GPIOE->BSRR = PRECHARGE_PIN_MASK;
}


void Bypass_On(void)
{
    /* Active Low: LOW = ON */
    GPIOE->BSRR = (BYPASS_PIN_MASK << 16U);
}


void Bypass_Off(void)
{
    /* Active Low: HIGH = OFF */
    GPIOE->BSRR = BYPASS_PIN_MASK;
}


uint8_t Precharge_GetPinState(void)
{
    return ((GPIOE->ODR & PRECHARGE_PIN_MASK) == 0U) ? 1U : 0U;
}


uint8_t Bypass_GetPinState(void)
{
    return ((GPIOE->ODR & BYPASS_PIN_MASK) == 0U) ? 1U : 0U;
}

/* ================= Relay Sequence ================= */

static void Relay_ProcessConnect(void)
{
    /*
     * 안전 상태에서 시작
     */
    Bypass_Off();
    Precharge_Off();

    g_system_data.relay_state = 0U;
    g_system_data.fsm_state = FSM_DETECTED;

    /*
     * 1. Precharge ON
     */
    relay_state = RELAY_STATE_PRECHARGE;
    Precharge_On();

    vTaskDelay(pdMS_TO_TICKS(RELAY_PRECHARGE_TIME_MS));

    /*
     * 2. Bypass ON
     */
    relay_state = RELAY_STATE_BYPASS;
    Bypass_On();

    vTaskDelay(pdMS_TO_TICKS(RELAY_BYPASS_SETTLE_MS));

    /*
     * 3. Precharge OFF
     * 최종적으로 Bypass만 ON 유지
     */
    Precharge_Off();

    relay_state = RELAY_STATE_CONNECTED;

#if 0
    vTaskDelay(pdMS_TO_TICKS(1500));
#endif
    /*
     * 4. 이제 모듈 전원 연결 완료.
     * 이 시점부터 인증 대기 시작.
     */
    g_system_data.relay_state = 1U;
    g_system_data.fsm_state = FSM_AUTHENTICATING;

    ModuleManager_OnAttached();
}


static void Relay_ProcessDisconnect(void)
{
    relay_state = RELAY_STATE_DISCONNECTING;

    /*
     * 안전 차단.
     * Bypass 먼저 OFF, Precharge도 OFF.
     */
    Bypass_Off();
    Precharge_Off();

    relay_state = RELAY_STATE_OFF;

    g_system_data.relay_state = 0U;

    /*
     * 물리적으로 모듈이 꽂혀있으면 DETECTED 상태로 둬야
     * 다시 CONNECT 가능.
     */
    if (g_system_data.detect_state == 1U)
    {
        g_system_data.fsm_state = FSM_DETECTED;
    }
    else
    {
        g_system_data.fsm_state = FSM_IDLE;
    }

    ModuleManager_OnDetached();
}


static void RelayTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        RelayCommand_t cmd;

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        cmd = relay_cmd;
        relay_cmd = RELAY_CMD_NONE;

        if (cmd == RELAY_CMD_CONNECT)
        {
            Relay_ProcessConnect();
        }
        else if (cmd == RELAY_CMD_DISCONNECT)
        {
            Relay_ProcessDisconnect();
        }
    }
}
