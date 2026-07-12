/*
 * module_a.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

/*
 * module_a.c
 *
 * Module A 압력 데이터 처리
 */

#include "module_a.h"

#include "can_protocol.h"
#include "module_manager.h"
#include "system_data.h"

#if 1
// test 용
#include <stdio.h>
extern UART_HandleTypeDef huart3;
#endif
/*
 * CAN ID : 0x171
 *
 * Module A -> Base
 *
 * data[0] : pressure_raw LSB
 * data[1] : pressure_raw MSB
 */

void ModuleA_HandlePressure(const CAN_Frame_t *frame)
{
    uint16_t pressure_raw;

    if (frame == NULL)
    {
        return;
    }

    /* 압력 데이터 4Byte 필요 */
    if (frame->dlc < 2U)
    {
        return;
    }

#if 0
    char debug_buffer[80];
    int debug_length;

    debug_length = snprintf(debug_buffer,
                            sizeof(debug_buffer),
                            "RX id=0x%03X dlc=%u data=%02X %02X auth=%u type=%u\r\n",
                            frame->std_id,
                            frame->dlc,
                            frame->data[0],
                            frame->data[1],
                            ModuleManager_IsAccepted(),
                            ModuleManager_GetModuleType());

    if ((debug_length > 0) &&
        (debug_length < (int)sizeof(debug_buffer)))
    {
        HAL_UART_Transmit(&huart3,
                          (uint8_t *)debug_buffer,
                          (uint16_t)debug_length,
                          100U);
    }
#endif

    /*
     * 현재 승인된 모듈이 Module A일 때만 수신값 반영
     *
     * Module B 연결 중이거나,
     * 인증 전 / 인증 실패 상태면 무시
     */
    if ((ModuleManager_IsAccepted() == 0U) ||
        (ModuleManager_GetModuleType() != eMODULE_GENERAL))
    {
        return;
    }

    /* Little Endian: data[0] = LSB, data[1] = MSB */
    pressure_raw =
        ((uint16_t)frame->data[0] << 0) |
        ((uint16_t)frame->data[1] << 8);

    /*
     * 현재는 ADC/압력 raw 값을 그대로 저장.
     * 센서 기준이 확정되면 여기서 N, kg, kgf 등으로 변환.
     */
    g_system_data.pressure_value = (float)pressure_raw;

    /* UART 출력 추가 */
#if 0
    {
        char uart_buffer[64];
        int length;

        length = snprintf(uart_buffer,
                          sizeof(uart_buffer),
                          "Pressure RX: %u\r\n",
                          pressure_raw);

        if (length > 0)
        {
            HAL_UART_Transmit(&huart3,
                              (uint8_t *)uart_buffer,
                              (uint16_t)length,
                              100U);
        }
    }
#endif
}

/* Module A 초기화 */
void ModuleA_Init(void)
{
    /* CAN 0x171 수신 시 ModuleA_HandlePressure() 실행 */
    CAN_RegisterRxCallback(CAN_ID_GENERAL_PRESSURE,
                           ModuleA_HandlePressure);
}

/*
Module A 압력센서
	→ CAN 0x171 전송
	→ can.c CanRxTask
	→ ModuleA_HandlePressure()
	→ g_system_data.pressure_value 저장
	→ 나중에 Raspberry Pi / GUI에서 읽기
*/
