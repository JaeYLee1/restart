/*
 * module_b.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

/*
 * module_b.c
 *
 * Base STM32 RTOS
 * Module B 냉각 제어 명령 및 온도 데이터 처리
 */

#include "module_b.h"

#include "can_protocol.h"
#include "module_manager.h"
#include "system_data.h"

#define MODULE_B_TARGET_MIN_C     0.0f
#define MODULE_B_TARGET_MAX_C     30.0f

/*
 * CAN ID : 0x173
 *
 * Module B -> Base
 *
 * data[0] : current_temp_x10 LSB
 * data[1] : current_temp_x10 MSB
 */
void ModuleB_HandleTemperature(const CAN_Frame_t *frame)
{
    int16_t current_temp_x10;

    if (frame == NULL)
    {
        return;
    }

    if (frame->dlc < 2U)
    {
        return;
    }

    /* 승인된 Module B가 연결된 경우만 반영 */
    if ((ModuleManager_IsAccepted() == 0U) ||
        (ModuleManager_GetModuleType() != eMODULE_COLD_CHAIN))
    {
        return;
    }

    current_temp_x10 =
        (int16_t)(
            ((uint16_t)frame->data[0] << 0) |
            ((uint16_t)frame->data[1] << 8)
        );

    /* 예: 253 -> 25.3°C */
    g_system_data.current_temp_c =
        (float)current_temp_x10 / 10.0f;
}


/*
 * CAN ID : 0x172
 *
 * Base -> Module B
 *
 * data[0] : Peltier Duty (0 ~ 100)
 */
void ModuleB_SendDuty(uint8_t peltier_duty)
{
    CAN_Frame_t frame = {0};

    if ((ModuleManager_IsAccepted() == 0U) ||
        (ModuleManager_GetModuleType() != eMODULE_COLD_CHAIN))
    {
        return;
    }

    if (peltier_duty > 100U)
    {
        peltier_duty = 100U;
    }

    frame.std_id = CAN_ID_COLD_DUTY_COMMAND;
    frame.dlc = 1U;

    frame.data[0] = peltier_duty;

    /* Base 마지막 명령값 저장 */
    g_system_data.peltier_pwm = peltier_duty;

    (void)CAN_Send(&frame, pdMS_TO_TICKS(10));
}


/*
 * CAN ID : 0x174
 *
 * Base -> Module B
 *
 * data[0] : target_temp_x10 LSB
 * data[1] : target_temp_x10 MSB
 *
 * 예)
 * 5.0°C -> 50
 * -2.5°C -> -25
 */
void ModuleB_SendTargetTemp(int16_t target_temp_x10)
{
    CAN_Frame_t frame = {0};
    uint16_t raw_target_temp;

    if ((ModuleManager_IsAccepted() == 0U) ||
        (ModuleManager_GetModuleType() != eMODULE_COLD_CHAIN))
    {
        return;
    }

    raw_target_temp = (uint16_t)target_temp_x10;

    frame.std_id = CAN_ID_COLD_TARGET_TEMP;
    frame.dlc = 2U;

    frame.data[0] = (uint8_t)(raw_target_temp & 0xFFU);
    frame.data[1] = (uint8_t)((raw_target_temp >> 8) & 0xFFU);

    /* Base 마지막 목표 온도 저장 */
    g_system_data.target_temp_c =
        (float)target_temp_x10 / 10.0f;

    (void)CAN_Send(&frame, pdMS_TO_TICKS(10));
}


/* Module B CAN Callback 등록 */
void ModuleB_Init(void)
{
    CAN_RegisterRxCallback(CAN_ID_COLD_TEMPERATURE,
                           ModuleB_HandleTemperature);
}
