#include "uart_task.h"

#include <stdio.h>
#include <string.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system_data.h"
#include "module_manager.h"
#include "motor.h"
#include "module_b.h"
#include "warning.h"
#include "relay.h"
#include "queue.h"
#include "semphr.h"

#define UART_RX_QUEUE_LENGTH    64U

static QueueHandle_t uart_rx_queue = NULL;
static uint8_t uart_rx_byte;

#define UART_TELEMETRY_PERIOD_MS     100U
//#define UART_TELEMETRY_PERIOD_MS     2500U

typedef void (*UART_CommandHandler_t)(uint8_t command);

typedef struct
{
    uint8_t command;
    UART_CommandHandler_t handler;
} UART_CommandEntry_t;

/* main.c에 생성된 USART3 핸들 */
extern UART_HandleTypeDef huart3;

/* 내부 함수 선언 */
static void UARTTelemetryTask(void *argument);
static void UARTCommandTask(void *argument);

/* 명령어 처리 */
static void UART_HandleStart(uint8_t command);
static void UART_HandleSlow(uint8_t command);
static void UART_HandleMid(uint8_t command);
static void UART_HandleFast(uint8_t command);
static void UART_HandleStop(uint8_t command);

static void UART_HandleConnect(uint8_t command);
static void UART_HandleDisconnect(uint8_t command);
static void UART_HandleRetryAuth(uint8_t command);

static void UART_HandleTargetTemp(uint8_t command);

static void UART_HandleWarningL1(uint8_t command);
static void UART_HandleWarningL2(uint8_t command);
static void UART_HandleAiNormal(uint8_t command);
static void UART_HandleWarningReset(uint8_t command);

static void UART_ProcessCommandChar(uint8_t command);

static const UART_CommandEntry_t uart_command_table[] =
{
    { '1', UART_HandleStart },
    { '2', UART_HandleSlow },
    { '3', UART_HandleMid },
    { '4', UART_HandleFast },
    { '5', UART_HandleStop },

    { 'C', UART_HandleConnect },
    { 'D', UART_HandleDisconnect },
    { 'R', UART_HandleRetryAuth },

    { 'T', UART_HandleTargetTemp },

    { 'A', UART_HandleWarningL1 },
    { 'B', UART_HandleWarningL2 },
    { 'N', UART_HandleAiNormal },
    { 'K', UART_HandleWarningReset },
};

#define UART_COMMAND_TABLE_COUNT \
    (sizeof(uart_command_table) / sizeof(uart_command_table[0]))

/* 명령 문자 normalize 함수 */
static uint8_t UART_NormalizeCommand(uint8_t command)
{
    if ((command >= 'a') && (command <= 'z'))
    {
        command = (uint8_t)(command - 'a' + 'A');
    }

    return command;
}

/* TX mutex */
static SemaphoreHandle_t uart_tx_mutex = NULL;

static void UART_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    if (uart_tx_mutex != NULL)
    {
        if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100U)) == pdPASS)
        {
            HAL_UART_Transmit(&huart3,
                              (uint8_t *)str,
                              (uint16_t)strlen(str),
                              100U);

            xSemaphoreGive(uart_tx_mutex);
        }
    }
}

static void UART_SendBuffer(const char *buffer, int length)
{
    if ((buffer == NULL) || (length <= 0))
    {
        return;
    }

    if (uart_tx_mutex != NULL)
    {
        if (xSemaphoreTake(uart_tx_mutex, pdMS_TO_TICKS(100U)) == pdPASS)
        {
            HAL_UART_Transmit(&huart3,
                              (uint8_t *)buffer,
                              (uint16_t)length,
                              100U);

            xSemaphoreGive(uart_tx_mutex);
        }
    }
}

/* UART Task 생성 */
void UARTTask_Init(void)
{
    uart_rx_queue = xQueueCreate(UART_RX_QUEUE_LENGTH, sizeof(uint8_t));
    if (uart_rx_queue == NULL)
    {
        Error_Handler();
    }

    uart_tx_mutex = xSemaphoreCreateMutex();
    if (uart_tx_mutex == NULL)
    {
        Error_Handler();
    }

#if 1
    /* Base -> Pi 상태 전송 */
    if (xTaskCreate(UARTTelemetryTask,
                    "UART_TX",
                    384,
                    NULL,
                    1,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
#endif
    /* TeraTerm / Pi -> Base 명령 수신 */
    if (xTaskCreate(UARTCommandTask,
                    "UART_RX",
                    384,
                    NULL,
                    2,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);
}

/*
 * 100ms마다 Raspberry Pi에 상태 전송
 *
 * Format:
 * TEL,module_type,auth,pressure,temp_x10,target_temp_x10,peltier_duty
 */
static void UARTTelemetryTask(void *argument)
{
    char uart_buffer[256];
    int length;

    // 모듈 센서 값(무게, 온도)
    int32_t pressure_value;
    int32_t current_temp_x10;
    int32_t target_temp_x10;

    // 부하 모터
    int32_t target_rpm_x10;
    int32_t current_rpm_x10;
    int32_t motor_duty_x10;

    // 모듈 유형, 인증 결과
    ModuleType_t module_type;
    uint8_t auth_result;


    // 소비 전력 데이터
    int32_t base_power_mw;
    int32_t module_b_power_mw;
    int32_t cooling_power_mw;
    int32_t motor_power_mw;
    int32_t total_power_mw;

    // 전력 경고
    int32_t power_limit_mw;
    uint8_t power_warning_count;
    uint8_t power_fault;
    (void)argument;

    for (;;)
    {
        pressure_value = (int32_t)g_system_data.pressure_value;

        current_temp_x10 =
            (int32_t)(g_system_data.current_temp_c * 10.0f);

        target_temp_x10 =
            (int32_t)(g_system_data.target_temp_c * 10.0f);

        target_rpm_x10 =
            (int32_t)(g_system_data.target_speed_rpm * 10.0f);

        current_rpm_x10 =
            (int32_t)(g_system_data.current_speed_rpm * 10.0f);

        motor_duty_x10 =
            (int32_t)(g_system_data.motor_pwm_duty * 10.0f);

        base_power_mw    =
        		(int32_t)(g_system_data.base_power_w * 1000.0f);
        module_b_power_mw =
        		(int32_t)(g_system_data.module_b_power_w * 1000.0f);
        cooling_power_mw =
        		(int32_t)(g_system_data.cooling_power_w * 1000.0f);
        motor_power_mw   =
        		(int32_t)(g_system_data.motor_power_w * 1000.0f);
        total_power_mw   =
        		(int32_t)(g_system_data.total_power_w * 1000.0f);

        power_warning_count = g_system_data.power_warning_count;
        power_fault = g_system_data.power_fault;
        power_limit_mw = (int32_t)(g_system_data.power_limit_w * 1000.0f);

        module_type = ModuleManager_GetModuleType();
        auth_result = ModuleManager_IsAccepted();

/*
        if (auth_result == 1U)
        {
            g_system_data.relay_state = 1;
            g_system_data.fsm_state = FSM_ACTIVE;
        }
*/

        length = snprintf(uart_buffer,
                          sizeof(uart_buffer),
                          "RTOS_Data,%u,%u,%ld,%ld,%ld,%u,%u,%ld,%ld,%ld,%u,%u,%u,%ld,%ld,%ld,%ld,%ld,%u,%u,%ld\r\n",
                          (unsigned int)module_type,
                          (unsigned int)auth_result,
                          (long)pressure_value,
                          (long)current_temp_x10,
                          (long)target_temp_x10,
                          (unsigned int)g_system_data.motor_running,
                          (unsigned int)g_system_data.motor_speed_level,
                          (long)target_rpm_x10,
                          (long)current_rpm_x10,
                          (long)motor_duty_x10,
                          (unsigned int)g_system_data.detect_state,
                          (unsigned int)g_system_data.relay_state,
                          (unsigned int)g_system_data.fsm_state,
                          (long)base_power_mw,
                          (long)module_b_power_mw,
                          (long)cooling_power_mw,
                          (long)motor_power_mw,
                          (long)total_power_mw,
                          (unsigned int)power_warning_count,
                          (unsigned int)power_fault,
                          (long)power_limit_mw);

        if ((length > 0) &&
            (length < (int)sizeof(uart_buffer)))
        {
        	UART_SendBuffer(uart_buffer, length);
        }

        vTaskDelay(pdMS_TO_TICKS(UART_TELEMETRY_PERIOD_MS));
    }
}

/*
 * 문자 1개 명령 수신 Task
 *
 * 1 : START
 * 2 : SLOW
 * 3 : MID
 * 4 : FAST
 * 5 : STOP
 *
 * c : CONNECT_MODULE
 * d : DISCONNECT_MODULE
 * r : RETRY_AUTH
 */
static void UARTCommandTask(void *argument)
{
    uint8_t rx_char;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(uart_rx_queue,
                          &rx_char,
                          portMAX_DELAY) == pdPASS)
        {
            if ((rx_char == '\r') ||
                (rx_char == '\n') ||
                (rx_char == ' ')  ||
                (rx_char == '\t'))
            {
                continue;
            }

            UART_ProcessCommandChar(rx_char);
        }
    }
}

/*
 * 문자 1개 명령 처리
 */
static void UART_ProcessCommandChar(uint8_t command)
{
    uint32_t i;

    command = UART_NormalizeCommand(command);

    for (i = 0U; i < UART_COMMAND_TABLE_COUNT; i++)
    {
        if (uart_command_table[i].command == command)
        {
            uart_command_table[i].handler(command);
            return;
        }
    }

    UART_SendString("ERR,CMD\r\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART3)
    {
        if (uart_rx_queue != NULL)
        {
            xQueueSendFromISR(uart_rx_queue,
                              &uart_rx_byte,
                              &xHigherPriorityTaskWoken);
        }

        HAL_UART_Receive_IT(&huart3, &uart_rx_byte, 1U);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* Motor 제어 */
static void UART_HandleStart(uint8_t command)
{
    (void)command;

    Motor_RequestStart();
    UART_SendString("OK,START\r\n");
}

static void UART_HandleSlow(uint8_t command)
{
    (void)command;

    Motor_SetSpeedLevel(eMOTOR_SLOW);
    UART_SendString("OK,SLOW\r\n");
}

static void UART_HandleMid(uint8_t command)
{
    (void)command;

    Motor_SetSpeedLevel(eMOTOR_MID);
    UART_SendString("OK,MID\r\n");
}

static void UART_HandleFast(uint8_t command)
{
    (void)command;

    Motor_SetSpeedLevel(eMOTOR_FAST);
    UART_SendString("OK,HIGH\r\n");
}

static void UART_HandleStop(uint8_t command)
{
    (void)command;

    Motor_RequestStop();
    UART_SendString("OK,STOP\r\n");
}

/* 릴레이 제어 */
static void UART_HandleConnect(uint8_t command)
{
    (void)command;

    if (g_system_data.detect_state == 1U)
    {
        if (Relay_RequestConnect() != 0U)
        {
            UART_SendString("OK,CONNECT_MODULE\r\n");
        }
        else
        {
            UART_SendString("ERR,RELAY_BUSY\r\n");
        }
    }
    else
    {
        UART_SendString("ERR,NO_MODULE_DETECTED\r\n");
    }
}

static void UART_HandleDisconnect(uint8_t command)
{
    (void)command;

    Relay_RequestDisconnect();
    UART_SendString("OK,DISCONNECT_MODULE\r\n");
}

static void UART_HandleRetryAuth(uint8_t command)
{
    (void)command;

    if (g_system_data.detect_state == 1U)
    {
        if (Relay_RequestConnect() != 0U)
        {
            UART_SendString("OK,RETRY_AUTH\r\n");
        }
        else
        {
            UART_SendString("ERR,RELAY_BUSY\r\n");
        }
    }
    else
    {
        UART_SendString("ERR,NO_MODULE_DETECTED\r\n");
    }
}

/* 온도 제어 */
static void UART_HandleTargetTemp(uint8_t command)
{
    uint8_t target_temp_c;
    int16_t target_temp_x10;

    (void)command;

    if (xQueueReceive(uart_rx_queue,
                      &target_temp_c,
                      pdMS_TO_TICKS(100U)) == pdPASS)
    {
        if (target_temp_c > 30U)
        {
            target_temp_c = 30U;
        }

        target_temp_x10 = (int16_t)target_temp_c * 10;

        ModuleB_SendTargetTemp(target_temp_x10);

        UART_SendString("OK,TARGET_TEMP\r\n");
    }
    else
    {
        UART_SendString("ERR,TARGET_TEMP\r\n");
    }
}

/* 경고 명령 */
static void UART_HandleWarningL1(uint8_t command)
{
    (void)command;

    Warning_OnAiLevel(1U);
    UART_SendString("OK,WARNING_L1\r\n");
}

static void UART_HandleWarningL2(uint8_t command)
{
    (void)command;

    Warning_OnAiLevel(2U);
    UART_SendString("OK,WARNING_L2\r\n");
}

static void UART_HandleAiNormal(uint8_t command)
{
    (void)command;

    Warning_OnAiLevel(0U);
    Warning_OnGuiReset();

    UART_SendString("OK,AI_NORMAL\r\n");
}

static void UART_HandleWarningReset(uint8_t command)
{
    (void)command;

    Warning_OnGuiReset();
    UART_SendString("OK,WARNING_RESET\r\n");
}
