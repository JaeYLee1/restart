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
#define MODULEB_PELTIER_PIN_NUMBER  3U
#define PRECHARGE_PIN_NUMBER        5U
#define BYPASS_PIN_NUMBER           6U

#define MODULEB_PELTIER_PIN_MASK    (1U << MODULEB_PELTIER_PIN_NUMBER)
#define PRECHARGE_PIN_MASK          (1U << PRECHARGE_PIN_NUMBER)
#define BYPASS_PIN_MASK             (1U << BYPASS_PIN_NUMBER)

#define RELAY_ALL_PIN_MASK          \
    (MODULEB_PELTIER_PIN_MASK | PRECHARGE_PIN_MASK | BYPASS_PIN_MASK)

/* Sequence Timing */
#define RELAY_PRECHARGE_TIME_MS   1000U
#define RELAY_BYPASS_SETTLE_MS    1000U

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

static void RelayTask(void *argument);

static void Relay_ProcessConnect(void);
static void Relay_ProcessDisconnect(void);
static void Relay_UpdateModuleBPeltier(void);

/* ================= Init ================= */

void Relay_Init(void)
{
    Relay_GPIO_Init();

    ModuleB_Peltier_Off();
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


void Relay_GPIO_Init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

    /*
     * Active-Low 릴레이이므로 출력 모드 전환 전에 HIGH = OFF 상태로 먼저 세팅
     */
    GPIOE->BSRR = RELAY_ALL_PIN_MASK;

    GPIOE->MODER &= ~((3U << (MODULEB_PELTIER_PIN_NUMBER * 2U)) |
                      (3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (3U << (BYPASS_PIN_NUMBER * 2U)));

    GPIOE->MODER |=  ((1U << (MODULEB_PELTIER_PIN_NUMBER * 2U)) |
                      (1U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (1U << (BYPASS_PIN_NUMBER * 2U)));

    GPIOE->OTYPER &= ~RELAY_ALL_PIN_MASK;

    GPIOE->OSPEEDR &= ~((3U << (MODULEB_PELTIER_PIN_NUMBER * 2U)) |
                        (3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                        (3U << (BYPASS_PIN_NUMBER * 2U)));

    GPIOE->PUPDR &= ~((3U << (MODULEB_PELTIER_PIN_NUMBER * 2U)) |
                      (3U << (PRECHARGE_PIN_NUMBER * 2U)) |
                      (3U << (BYPASS_PIN_NUMBER * 2U)));

    Precharge_Off();
    Bypass_Off();
    ModuleB_Peltier_Off();
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
    ModuleB_Peltier_Off();
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

void ModuleB_Peltier_On(void)
{
    GPIOE->BSRR = (MODULEB_PELTIER_PIN_MASK << 16U);  /* LOW = ON */
}

void ModuleB_Peltier_Off(void)
{
    GPIOE->BSRR = MODULEB_PELTIER_PIN_MASK;           /* HIGH = OFF */
}

uint8_t ModuleB_Peltier_GetPinState(void)
{
    return ((GPIOE->ODR & MODULEB_PELTIER_PIN_MASK) == 0U) ? 1U : 0U;
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
    ModuleB_Peltier_Off();

    Bypass_Off();
    Precharge_Off();

    if (g_system_data.detect_state == 0U)
    {
        Relay_ForceOff();
        return;
    }

    g_system_data.relay_state = 0U;
    g_system_data.fsm_state = FSM_DETECTED;

    relay_state = RELAY_STATE_PRECHARGE;
    Precharge_On();
    vTaskDelay(pdMS_TO_TICKS(RELAY_PRECHARGE_TIME_MS));

    if ((g_system_data.detect_state == 0U) ||
        (relay_state == RELAY_STATE_OFF))
    {
        Relay_ForceOff();
        return;
    }

    relay_state = RELAY_STATE_BYPASS;
    Bypass_On();
    vTaskDelay(pdMS_TO_TICKS(RELAY_BYPASS_SETTLE_MS));

    if ((g_system_data.detect_state == 0U) ||
        (relay_state == RELAY_STATE_OFF))
    {
        Relay_ForceOff();
        return;
    }

    Precharge_Off();

    relay_state = RELAY_STATE_CONNECTED;
    g_system_data.relay_state = 1U;
    g_system_data.fsm_state = FSM_AUTHENTICATING;

    ModuleManager_OnAttached();
}

static void Relay_ProcessDisconnect(void)
{
    relay_state = RELAY_STATE_DISCONNECTING;

    ModuleB_Peltier_Off();
    Bypass_Off();
    Precharge_Off();

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

static void RelayTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        RelayCommand_t cmd;

        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50U)) > 0U)
        {
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

        Relay_UpdateModuleBPeltier();
    }
}

static void Relay_UpdateModuleBPeltier(void)
{
    if ((ModuleManager_GetModuleType() == 2U) &&
        (ModuleManager_IsAccepted() == 1U) &&
        (g_system_data.fsm_state == FSM_ACTIVE) &&
        (Relay_IsConnected() == 1U))
    {
        ModuleB_Peltier_On();
    }
    else
    {
        ModuleB_Peltier_Off();
    }
}
