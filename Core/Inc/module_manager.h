/*
 * module_manager.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_MODULE_MANAGER_H_
#define INC_MODULE_MANAGER_H_

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "can.h"
#include "can_protocol.h"

/* 인증 상태 */
typedef enum
{
    eAUTH_IDLE = 0,
    eAUTH_WAIT_MODULE_ID,
    eAUTH_ACCEPTED,
    eAUTH_REJECTED
} AuthState_t;

/* 초기화 */
void ModuleManager_Init(void);

/* DetectTask에서 호출 */
void ModuleManager_OnAttached(void);
void ModuleManager_OnDetached(void);

/* CAN 0x110 Callback */
void ModuleManager_HandleAnnounce(const CAN_Frame_t *frame);

/* 상태 확인 */
AuthState_t ModuleManager_GetAuthState(void);
ModuleType_t ModuleManager_GetModuleType(void);
uint32_t ModuleManager_GetModuleId(void);
uint8_t ModuleManager_IsAccepted(void);

#endif /* INC_MODULE_MANAGER_H_ */
