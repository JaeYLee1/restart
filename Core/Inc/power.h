/*
 * power.h
 *
 *  Created on: 2026. 7. 11.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_POWER_H_
#define INC_POWER_H_

#include "main.h"
#include <stdint.h>

typedef struct
{
    int32_t bus_mv;
    int32_t shunt_uv;
    int32_t current_ma;
    int32_t power_mw;
} Power_INA226Value_t;

/* I2C low-level */
void Power_I2C1_Init(void);

uint8_t Power_WriteReg16(int sensor, uint8_t reg, uint16_t data);
uint8_t Power_ReadReg16(int sensor, uint8_t reg, int16_t *out_data);

/* Sensor init */
void Power_INA3221_Init(void);

void Power_INA226_Init(void);          /* 기존 호환용: Cooling 0x40 */
void Power_INA226_Cooling_Init(void);  /* INA226 #1: 0x40 */
void Power_INA226_Motor_Init(void);    /* INA226 #2: 0x44 */

/* Sensor read */
uint8_t Power_INA226_Read(Power_INA226Value_t *out);          /* 기존 호환용: Cooling 0x40 */
uint8_t Power_INA226_Cooling_Read(Power_INA226Value_t *out);  /* INA226 #1 */
uint8_t Power_INA226_Motor_Read(Power_INA226Value_t *out);    /* INA226 #2 */

/* RTOS */
void PowerMonitor_Task(void *argument);
void PowerMonitor_RtosInit(void);

#endif /* INC_POWER_H_ */
