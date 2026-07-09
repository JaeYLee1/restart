/*
 * motor.h
 *
 *  Created on: 2026. 7. 6.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>

/* 속도 단계 */
typedef enum
{
    eMOTOR_STOP = 0,
    eMOTOR_SLOW,
    eMOTOR_MID,
    eMOTOR_FAST
} MotorSpeedLevel_t;


/* GPIO / PWM / Encoder 초기화 */
void Motor_Init(void);

/* MotorControlTask 생성 */
void Motor_RtosInit(void);


/* 외부 제어 API */
void Motor_RequestStart(void);
void Motor_RequestStop(void);

void Motor_SetSpeedLevel(MotorSpeedLevel_t level);


/* 상태 조회 */
uint8_t Motor_IsRunning(void);

float Motor_GetCurrentRpm(void);
float Motor_GetTargetRpm(void);
float Motor_GetDuty(void);

#endif /* INC_MOTOR_H_ */
