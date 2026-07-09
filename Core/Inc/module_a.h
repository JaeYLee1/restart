/*
 * module_a.h
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_MODULE_A_H_
#define INC_MODULE_A_H_

#include "can.h"

/* Module A 초기화 및 CAN Callback 등록 */
void ModuleA_Init(void);

/* CAN 0x171 수신 Callback */
void ModuleA_HandlePressure(const CAN_Frame_t *frame);
#endif /* INC_MODULE_A_H_ */
