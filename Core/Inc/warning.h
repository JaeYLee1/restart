/*
 * warning.h
 *
 *  Created on: 2026. 7. 9.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_WARNING_H_
#define INC_WARNING_H_

#include <stdint.h>

typedef enum
{
    WARNING_STATE_NORMAL = 0,
    WARNING_STATE_LEVEL1,
    WARNING_STATE_LEVEL2,
    WARNING_STATE_RECOVERY
} WarningState_t;

void Warning_Init(void);
void Warning_RtosInit(void);

/* AI UART event */
void Warning_OnAiLevel(uint8_t level);

/* GUI reset command */
void Warning_OnGuiReset(void);

uint8_t Warning_GetLevel(void);
WarningState_t Warning_GetState(void);

/*
 * motor.c/h 연결 전 임시 Hook
 * 나중에 motor 함수명 확정되면 warning.c 안에서 직접 연결하면 됨.
 */
void Warning_Motor_OnLevel2(void);
void Warning_Motor_OnRecovery(void);

#endif /* INC_WARNING_H_ */
