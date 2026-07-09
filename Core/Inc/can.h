#ifndef INC_CAN_H_
#define INC_CAN_H_

#include <stdint.h>

#include "main.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* CAN Frame */
typedef struct
{
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

/* 수신 Callback 형식 (함수 포인터) */
typedef void (*CAN_RxCallback_t)(const CAN_Frame_t *frame);

/* 초기화 */
void CAN_Init(void);
void CAN_RtosInit(void);

/* 송신 */
BaseType_t CAN_Send(const CAN_Frame_t *frame,
                    TickType_t wait_time);

/* CAN RX IRQ에서 호출 */
void CAN_RxIrqHandler(void);

/* CAN ID별 수신 함수 등록 */
void CAN_RegisterRxCallback(uint16_t std_id,
                            CAN_RxCallback_t callback);

#endif /* INC_CAN_H_ */
