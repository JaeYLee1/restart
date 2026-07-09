/*
 * module_b.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_MODULE_B_H_
#define INC_MODULE_B_H_

#include <stdint.h>
#include "can_protocol.h"
#include "can.h"

/* Module B 초기화 및 온도 수신 Callback 등록 */
void ModuleB_Init(void);

/* Module B -> Base, CAN 0x173 */
void ModuleB_HandleTemperature(const CAN_Frame_t *frame);

/* Base -> Module B, CAN 0x172 */
void ModuleB_SendDuty(uint8_t peltier_duty);

/* Base -> Module B, CAN 0x174 */
void ModuleB_SendTargetTemp(int16_t target_temp_x10);
#endif /* INC_MODULE_B_H_ */
