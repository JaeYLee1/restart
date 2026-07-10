/*
 * uart_task.c
 *
 * UART RX Polling -> Motor Command Task
 */
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

#define UART_TELEMETRY_PERIOD_MS     100U

/* main.c에 생성된 USART3 핸들 */
extern UART_HandleTypeDef huart3;

/* 내부 함수 선언 */
static void UARTTelemetryTask(void *argument);
static void UARTCommandTask(void *argument);
static void UART_ProcessCommandChar(uint8_t command);

/* UART Task 생성 */
void UARTTask_Init(void)
{
#if 0
    /* Base -> Pi 상태 전송 */
    if (xTaskCreate(UARTTelemetryTask,
                    "UART_TX",
                    256,
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
                    256,
                    NULL,
                    2,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}


/*
 * 100ms마다 Raspberry Pi에 상태 전송
 *
 * Format:
 * TEL,module_type,auth,pressure,temp_x10,target_temp_x10,peltier_duty
 */
static void UARTTelemetryTask(void *argument)
{
    char uart_buffer[128];
    int length;

    int32_t pressure_value;
    int32_t current_temp_x10;
    int32_t target_temp_x10;

    int32_t target_rpm_x10;
    int32_t current_rpm_x10;
    int32_t motor_duty_x10;

    ModuleType_t module_type;
    uint8_t auth_result;

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

        module_type = ModuleManager_GetModuleType();
        auth_result = ModuleManager_IsAccepted();

        if (auth_result == 1U)
        {
            g_system_data.relay_state = 1;
            g_system_data.fsm_state = FSM_ACTIVE;
        }

        length = snprintf(uart_buffer,
                          sizeof(uart_buffer),
                          "TEL,%u,%u,%ld,%ld,%ld,%u,%u,%u,%ld,%ld,%ld,%u,%u,%u\r\n",
                          (unsigned int)module_type,
                          (unsigned int)auth_result,
                          (long)pressure_value,
                          (long)current_temp_x10,
                          (long)target_temp_x10,
                          (unsigned int)g_system_data.peltier_pwm,
                          (unsigned int)g_system_data.motor_running,
                          (unsigned int)g_system_data.motor_speed_level,
                          (long)target_rpm_x10,
                          (long)current_rpm_x10,
                          (long)motor_duty_x10,
                          (unsigned int)g_system_data.detect_state,
                          (unsigned int)g_system_data.relay_state,
                          (unsigned int)g_system_data.fsm_state);

        if ((length > 0) &&
            (length < (int)sizeof(uart_buffer)))
        {
            HAL_UART_Transmit(&huart3,
                              (uint8_t *)uart_buffer,
                              (uint16_t)length,
                              100U);
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
        if (HAL_UART_Receive(&huart3,
                             &rx_char,
                             1U,
                             10U) == HAL_OK)
        {
            /*
             * 개행, 공백은 무시
             * GUI/service가 혹시 \n을 같이 보내도 문제 없게 처리
             */
            if ((rx_char == '\r') ||
                (rx_char == '\n') ||
                (rx_char == ' ')  ||
                (rx_char == '\t'))
            {
                /* ignore */
            }
            else
            {
                UART_ProcessCommandChar(rx_char);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
 * 문자 1개 명령 처리
 */
static void UART_ProcessCommandChar(uint8_t command)
{
    switch (command)
    {
        case '1':
            Motor_RequestStart();
            HAL_UART_Transmit(&huart3, (uint8_t *)"OK,START\r\n", 10U, 100U);
            break;

        case '2':
            Motor_SetSpeedLevel(eMOTOR_SLOW);
            HAL_UART_Transmit(&huart3, (uint8_t *)"OK,SLOW\r\n", 9U, 100U);
            break;

        case '3':
            Motor_SetSpeedLevel(eMOTOR_MID);
            HAL_UART_Transmit(&huart3, (uint8_t *)"OK,MID\r\n", 8U, 100U);
            break;

        case '4':
            Motor_SetSpeedLevel(eMOTOR_FAST);
            HAL_UART_Transmit(&huart3, (uint8_t *)"OK,HIGH\r\n", 9U, 100U);
            break;

        case '5':
            Motor_RequestStop();
            HAL_UART_Transmit(&huart3, (uint8_t *)"OK,STOP\r\n", 9U, 100U);
            break;

        case 'c':
        case 'C':
            if (g_system_data.detect_state == 1U)
            {
                g_system_data.relay_state = 1U;
                g_system_data.fsm_state = FSM_AUTHENTICATING;

                ModuleManager_OnAttached();

                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"OK,CONNECT_MODULE\r\n",
                                  19U,
                                  100U);
            }
            else
            {
                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"ERR,NO_MODULE_DETECTED\r\n",
                                  24U,
                                  100U);
            }
            break;

        case 'd':
        case 'D':
            g_system_data.relay_state = 0U;
            g_system_data.fsm_state = FSM_IDLE;

            //ModuleB_ResetTargetTemp();

            ModuleManager_OnDetached();

            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"OK,DISCONNECT_MODULE\r\n",
                              22U,
                              100U);
            break;

        case 'r':
        case 'R':
            if (g_system_data.detect_state == 1U)
            {
                g_system_data.relay_state = 1U;
                g_system_data.fsm_state = FSM_AUTHENTICATING;

                ModuleManager_OnAttached();

                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"OK,RETRY_AUTH\r\n",
                                  15U,
                                  100U);
            }
            else
            {
                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"ERR,NO_MODULE_DETECTED\r\n",
                                  24U,
                                  100U);
            }
            break;

        case 'T':
        case 't':
        {
            uint8_t target_temp_c;
            int16_t target_temp_x10;

            if (HAL_UART_Receive(&huart3,
                                 &target_temp_c,
                                 1U,
                                 100U) == HAL_OK)
            {
                /*
                 * GUI가 보낸 값:
                 * 25 -> 25°C
                 * 26 -> 26°C
                 */
                if (target_temp_c > 30)
                {
                    target_temp_c = 30;
                }

                target_temp_x10 = (int16_t)target_temp_c * 10;

                ModuleB_SendTargetTemp(target_temp_x10);

                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"OK,TARGET_TEMP\r\n",
                                  16U,
                                  100U);
            }
            else
            {
                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"ERR,TARGET_TEMP\r\n",
                                  17U,
                                  100U);
            }
            break;
        }

        case 'A':
        case 'a':
        {
            Warning_OnAiLevel(1U);

            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"OK,WARNING_L1\r\n",
                              15U,
                              100U);
            break;
        }

        case 'B':
        case 'b':
        {
            Warning_OnAiLevel(2U);

            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"OK,WARNING_L2\r\n",
                              15U,
                              100U);
            break;
        }

        case 'N':
        case 'n':
        {
            Warning_OnAiLevel(0U);

            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"OK,AI_NORMAL\r\n",
                              14U,
                              100U);
            break;
        }

        case 'K':
        case 'k':
        {
            Warning_OnGuiReset();

            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"OK,WARNING_RESET\r\n",
                              18U,
                              100U);
            break;
        }

        default:
            HAL_UART_Transmit(&huart3,
                              (uint8_t *)"ERR,CMD\r\n",
                              9U,
                              100U);
            break;
    }
}
