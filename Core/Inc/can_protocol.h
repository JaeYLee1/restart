/*
 * can_protocol.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_CAN_PROTOCOL_H_
#define INC_CAN_PROTOCOL_H_

#include <stdint.h>

/* 모듈 종류 */
typedef enum
{
    eMODULE_NONE       = 0,
    eMODULE_GENERAL    = 1,
    eMODULE_COLD_CHAIN = 2,
    eMODULE_UNKNOWN    = 0xFF
} ModuleType_t;

/* CAN ID */
#define CAN_ID_MODULE_ANNOUNCE       0x110U  // 모듈 -> Base (ID로 연결 요청)
#define CAN_ID_LINK_RESULT           0x140U  // CAN 연결 결과를 알림

/* Module A */
#define CAN_ID_GENERAL_PRESSURE      0x171U  // Module A -> Base : 압력값

/* Module B */
#define CAN_ID_COLD_DUTY_COMMAND     0x172U  // Base -> Module B : 냉각 duty 명령
#define CAN_ID_COLD_TEMPERATURE      0x173U  // Module B -> Base : 현재 온도값
#define CAN_ID_COLD_TARGET_TEMP      0x174U  // Base -> Module B : 목표 온도

/* 등록된 모듈 ID */
#define MODULE_A_ID                  0x0000A001U
#define MODULE_B_ID                  0x0000B001U

/* 연결 결과 */
#define CAN_LINK_REASON_OK            0U
#define CAN_LINK_REASON_UNKNOWN_TYPE  1U
#define CAN_LINK_REASON_ID_MISMATCH   2U
#define CAN_LINK_REASON_TIMEOUT       3U

#endif /* INC_CAN_PROTOCOL_H_ */
