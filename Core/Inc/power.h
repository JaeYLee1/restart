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

void Power_I2C1_Init(void);

uint8_t Power_WriteReg16(int sensor, uint8_t reg, uint16_t data);
uint8_t Power_ReadReg16(int sensor, uint8_t reg, int16_t *out_data);

void Power_INA3221_Init(void);
void Power_INA226_Init(void);

uint8_t Power_INA226_Read(Power_INA226Value_t *out);

void PowerMonitor_Task(void *argument);
void PowerMonitor_RtosInit(void);

#endif /* INC_POWER_H_ */
