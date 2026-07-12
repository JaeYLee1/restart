/*
 * relay.h
 *
 *  Created on: 2026. 7. 10.
 *      Author: 한국전파진흥협회
 */

#ifndef INC_RELAY_H_
#define INC_RELAY_H_

#include <stdint.h>
#include "stm32f4xx.h"

typedef enum
{
    RELAY_STATE_OFF = 0,
    RELAY_STATE_PRECHARGE,
    RELAY_STATE_BYPASS,
    RELAY_STATE_CONNECTED,
    RELAY_STATE_DISCONNECTING
} RelayState_t;

void Relay_Init(void);
void Relay_RtosInit(void);

uint8_t Relay_RequestConnect(void);
uint8_t Relay_RequestDisconnect(void);

void Relay_ForceOff(void);

RelayState_t Relay_GetState(void);
uint8_t Relay_IsConnected(void);

/* Low-level control */
void Precharge_On(void);
void Precharge_Off(void);

void Bypass_On(void);
void Bypass_Off(void);

uint8_t Precharge_GetPinState(void);
uint8_t Bypass_GetPinState(void);

#endif /* INC_RELAY_H_ */
