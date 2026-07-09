/*
 * can.c
 *
 * Bare-metal CAN Driver + FreeRTOS
 */

#include "can.h"

#include <stdio.h>
#include "main.h"

extern UART_HandleTypeDef huart3;

/* Callback 최대 개수 */
#define CAN_CALLBACK_MAX_COUNT    8U

/* Queue 길이 */
#define CAN_RX_QUEUE_LENGTH       10U
#define CAN_TX_QUEUE_LENGTH       10U


/* CAN ID별 Callback */
typedef struct
{
    uint16_t std_id;
    CAN_RxCallback_t callback;
} CAN_CallbackEntry_t;

/* Queue */
static QueueHandle_t can_rx_queue;
static QueueHandle_t can_tx_queue;

/* Callback Table */
static CAN_CallbackEntry_t callback_table[CAN_CALLBACK_MAX_COUNT];

/* 수신 Queue 유실 횟수 */
static volatile uint32_t rx_drop_count = 0U;


/* 내부 함수 선언 */
static void CAN1_GPIO_Init(void);
static void CAN1_Peripheral_Init(void);
static void CAN1_Filter_Init(void);
static void CAN1_RX_Interrupt_Init(void);

static BaseType_t CAN1_TrySend(const CAN_Frame_t *frame);

static void CanRxTask(void *argument);
static void CanTxTask(void *argument);

/* 데이터 ID 별 함수 처리 */
static void CAN_DispatchRxFrame(const CAN_Frame_t *frame);

/* ================= CAN 초기화 ================= */

void CAN_Init(void)
{
    CAN1_GPIO_Init();
    CAN1_Peripheral_Init();
    CAN1_Filter_Init();
}


void CAN_RtosInit(void)
{
    /* Rx / Tx Queue 생성 */
    can_rx_queue = xQueueCreate(CAN_RX_QUEUE_LENGTH,
                                sizeof(CAN_Frame_t));

    can_tx_queue = xQueueCreate(CAN_TX_QUEUE_LENGTH,
                                sizeof(CAN_Frame_t));

    if ((can_rx_queue == NULL) ||
        (can_tx_queue == NULL))
    {
        Error_Handler();
    }

    /* CAN Rx Task 생성 */
    if (xTaskCreate(CanRxTask,
                    "CAN_RX",
                    256,
                    NULL,
                    3,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    /* CAN Tx Task 생성 */
    if (xTaskCreate(CanTxTask,
                    "CAN_TX",
                    256,
                    NULL,
                    2,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }

    /* Queue, Task 생성 후 CAN Rx IRQ 활성화 */
    CAN1_RX_Interrupt_Init();
}


/* ================= GPIO / CAN Register ================= */

static void CAN1_GPIO_Init(void)
{
    /* GPIOD Clock Enable */
    RCC->AHB1ENR |= (1U << 3);

    /* PD0, PD1 = Alternate Function */
    GPIOD->MODER &= ~((3U << 0) | (3U << 2));
    GPIOD->MODER |=  ((2U << 0) | (2U << 2));

    /* No Pull */
    GPIOD->PUPDR &= ~((3U << 0) | (3U << 2));

    /* High Speed */
    GPIOD->OSPEEDR |= ((3U << 0) | (3U << 2));

    /* PD0, PD1 = AF9 CAN1 */
    GPIOD->AFR[0] &= ~((0xFU << 0) | (0xFU << 4));
    GPIOD->AFR[0] |=  ((9U << 0) | (9U << 4));
}


static void CAN1_Peripheral_Init(void)
{
    /* CAN1 Clock Enable */
    RCC->APB1ENR |= (1U << 25);

    /* Sleep mode 해제 */
    CAN1->MCR &= ~(1U << 1);

    while (CAN1->MSR & (1U << 1))
    {
    }

    /* Initialization mode 진입 */
    CAN1->MCR |= (1U << 0);

    while ((CAN1->MSR & (1U << 0)) == 0U)
    {
    }

    /*
     * PCLK1 = 42 MHz
     * Prescaler = 6
     * BS1 = 11 tq
     * BS2 = 2 tq
     * Bitrate = 500 kbps
     */
#if 1
    CAN1->BTR =
        (5U  << 0)  |   /* BRP */
        (10U << 16) |   /* TS1 */
        (1U  << 20) |   /* TS2 */
        (0U  << 24);    /* SJW */
#else
    CAN1->BTR =
        (11U << 0)  |
        (10U << 16) |
        (1U  << 20) |
        (0U  << 24);
#endif
    /*
     * TTCM = 0
     * ABOM = 0
     * AWUM = 0
     * NART = 0 : 자동 재전송
     * RFLM = 0
     * TXFP = 0 : ID 우선순위
     */
    CAN1->MCR &= ~((1U << 7) |
                   (1U << 6) |
                   (1U << 5) |
                   (1U << 4) |
                   (1U << 3) |
                   (1U << 2));

    /* Normal mode 진입 */
    CAN1->MCR &= ~(1U << 0);

    while (CAN1->MSR & (1U << 0))
    {
    }
}


static void CAN1_Filter_Init(void)
{
    /* Filter Init Mode */
    CAN1->FMR |= (1U << 0);

    /* Filter 0 Disable */
    CAN1->FA1R &= ~(1U << 0);

    /* Mask Mode */
    CAN1->FM1R &= ~(1U << 0);

    /* 32-bit Filter */
    CAN1->FS1R |= (1U << 0);

    /* FIFO0 사용 */
    CAN1->FFA1R &= ~(1U << 0);

    /* 모든 Standard ID 수신 허용 */
    CAN1->sFilterRegister[0].FR1 = 0x00000000U;
    CAN1->sFilterRegister[0].FR2 = 0x00000000U;

    /* Filter 0 Enable */
    CAN1->FA1R |= (1U << 0);

    /* Filter 적용 */
    CAN1->FMR &= ~(1U << 0);
}


static void CAN1_RX_Interrupt_Init(void)
{
    /* FIFO0 Message Pending Interrupt */
    CAN1->IER |= (1U << 1);

    /*
     * xQueueSendFromISR() 사용 가능 Priority
     * FreeRTOSConfig.h 값과 맞춰야 함
     */
    NVIC_SetPriority(CAN1_RX0_IRQn, 5);
    NVIC_EnableIRQ(CAN1_RX0_IRQn);
}


/* ================= CAN 송신 ================= */

BaseType_t CAN_Send(const CAN_Frame_t *frame,
                    TickType_t wait_time)
{
    if ((frame == NULL) ||
        (frame->dlc > 8U) ||
        (can_tx_queue == NULL))
    {
        return pdFAIL;
    }

    return xQueueSend(can_tx_queue,
                      frame,
                      wait_time);
}


static BaseType_t CAN1_TrySend(const CAN_Frame_t *frame)
{
    uint32_t mailbox;
    uint32_t tdlr;
    uint32_t tdhr;

    /* 빈 Tx Mailbox 탐색 */
    if (CAN1->TSR & (1U << 26))
    {
        mailbox = 0U;
    }
    else if (CAN1->TSR & (1U << 27))
    {
        mailbox = 1U;
    }
    else if (CAN1->TSR & (1U << 28))
    {
        mailbox = 2U;
    }
    else
    {
        return pdFAIL;
    }

    /* Data[0] ~ Data[3] */
    tdlr =
        ((uint32_t)frame->data[0] << 0)  |
        ((uint32_t)frame->data[1] << 8)  |
        ((uint32_t)frame->data[2] << 16) |
        ((uint32_t)frame->data[3] << 24);

    /* Data[4] ~ Data[7] */
    tdhr =
        ((uint32_t)frame->data[4] << 0)  |
        ((uint32_t)frame->data[5] << 8)  |
        ((uint32_t)frame->data[6] << 16) |
        ((uint32_t)frame->data[7] << 24);

    /* DLC */
    CAN1->sTxMailBox[mailbox].TDTR = frame->dlc;

    /* Data */
    CAN1->sTxMailBox[mailbox].TDLR = tdlr;
    CAN1->sTxMailBox[mailbox].TDHR = tdhr;

    /* Standard ID + TXRQ */
    CAN1->sTxMailBox[mailbox].TIR =
        ((uint32_t)(frame->std_id & 0x7FFU) << 21) |
        (1U << 0);

    return pdPASS;
}


static void CanTxTask(void *argument)
{
    CAN_Frame_t tx_frame;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(can_tx_queue,
                          &tx_frame,
                          portMAX_DELAY) == pdPASS)
        {
#if 0

            	if (tx_frame.std_id == 0x140U)
            	{
            	    char dbg[96];
            	    int len;

            	    len = snprintf(dbg,
            	                   sizeof(dbg),
            	                   "DBG,CAN_TX,id=0x140,dlc=%u,data=%02X %02X %02X\r\n",
            	                   (unsigned int)tx_frame.dlc,
            	                   tx_frame.data[0],
            	                   tx_frame.data[1],
            	                   tx_frame.data[2]);

            	    if ((len > 0) && (len < (int)sizeof(dbg)))
            	    {
            	        HAL_UART_Transmit(&huart3,
            	                          (uint8_t *)dbg,
            	                          (uint16_t)len,
            	                          100U);
            	    }
            	}
#endif
            /* Tx Mailbox 대기 */
            while (CAN1_TrySend(&tx_frame) != pdPASS)
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
    }
}


/* ================= CAN 수신 ================= */

/*
 * stm32f4xx_it.c의
 * CAN1_RX0_IRQHandler()에서 호출
 */
void CAN_RxIrqHandler(void)
{
    CAN_Frame_t rx_frame;
    BaseType_t higher_priority_task_woken = pdFALSE;

    /* FIFO0에 쌓인 메시지 처리 */
    while ((CAN1->RF0R & 0x3U) != 0U)
    {
        uint32_t rir;
        uint32_t rdtr;
        uint32_t rdlr;
        uint32_t rdhr;

        rir  = CAN1->sFIFOMailBox[0].RIR;
        rdtr = CAN1->sFIFOMailBox[0].RDTR;
        rdlr = CAN1->sFIFOMailBox[0].RDLR;
        rdhr = CAN1->sFIFOMailBox[0].RDHR;

        /* FIFO0 해제 */
        CAN1->RF0R |= (1U << 5);

        /* Extended ID / Remote Frame 무시 */
        if ((rir & (1U << 2)) ||
            (rir & (1U << 1)))
        {
            continue;
        }

        /* Standard ID */
        rx_frame.std_id =
            (uint16_t)((rir >> 21) & 0x7FFU);

        /* DLC */
        rx_frame.dlc = (uint8_t)(rdtr & 0x0FU);

        if (rx_frame.dlc > 8U)
        {
            rx_frame.dlc = 8U;
        }

        /* Data[0] ~ Data[3] */
        rx_frame.data[0] = (uint8_t)(rdlr & 0xFFU);
        rx_frame.data[1] = (uint8_t)((rdlr >> 8) & 0xFFU);
        rx_frame.data[2] = (uint8_t)((rdlr >> 16) & 0xFFU);
        rx_frame.data[3] = (uint8_t)((rdlr >> 24) & 0xFFU);

        /* Data[4] ~ Data[7] */
        rx_frame.data[4] = (uint8_t)(rdhr & 0xFFU);
        rx_frame.data[5] = (uint8_t)((rdhr >> 8) & 0xFFU);
        rx_frame.data[6] = (uint8_t)((rdhr >> 16) & 0xFFU);
        rx_frame.data[7] = (uint8_t)((rdhr >> 24) & 0xFFU);

        /* ISR → Rx Queue */
        if (xQueueSendFromISR(can_rx_queue,
                              &rx_frame,
                              &higher_priority_task_woken) != pdPASS)
        {
            rx_drop_count++;
        }

    }

    portYIELD_FROM_ISR(higher_priority_task_woken);
}


static void CanRxTask(void *argument)
{
    CAN_Frame_t rx_frame;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(can_rx_queue,
                          &rx_frame,
                          portMAX_DELAY) == pdPASS)
        {
#if 0
            /*
             * CAN 수신 디버그
             * Task Context라서 HAL_UART_Transmit 사용 가능
             */
        	if ((rx_frame.std_id == 0x110U) ||
        	    (rx_frame.std_id == 0x171U) ||
        	    (rx_frame.std_id == 0x140U))
        	{
        	    char dbg[128];
        	    int len;

        	    len = snprintf(dbg,
        	                   sizeof(dbg),
        	                   "DBG,CAN_RX,id=0x%03X,dlc=%u,data=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        	                   (unsigned int)rx_frame.std_id,
        	                   (unsigned int)rx_frame.dlc,
        	                   rx_frame.data[0],
        	                   rx_frame.data[1],
        	                   rx_frame.data[2],
        	                   rx_frame.data[3],
        	                   rx_frame.data[4],
        	                   rx_frame.data[5],
        	                   rx_frame.data[6],
        	                   rx_frame.data[7]);

        	    if ((len > 0) && (len < (int)sizeof(dbg)))
        	    {
        	        HAL_UART_Transmit(&huart3,
        	                          (uint8_t *)dbg,
        	                          (uint16_t)len,
        	                          100U);
        	    }
        	}
#endif
            CAN_DispatchRxFrame(&rx_frame);
        }
    }
}

/* ================= Callback 처리 ================= */

void CAN_RegisterRxCallback(uint16_t std_id,
                            CAN_RxCallback_t callback)
{
    uint32_t i;

    if (callback == NULL)
    {
        return;
    }

    /* 이미 등록된 ID면 Callback 교체 */
    for (i = 0U; i < CAN_CALLBACK_MAX_COUNT; i++)
    {
        if (callback_table[i].callback != NULL)
        {
            if (callback_table[i].std_id == std_id)
            {
                callback_table[i].callback = callback;
                return;
            }
        }
    }

    /* 빈 자리 등록 */
    for (i = 0U; i < CAN_CALLBACK_MAX_COUNT; i++)
    {
        if (callback_table[i].callback == NULL)
        {
            callback_table[i].std_id = std_id;
            callback_table[i].callback = callback;
            return;
        }
    }
}


static void CAN_DispatchRxFrame(const CAN_Frame_t *frame)
{
    uint32_t i;

    for (i = 0U; i < CAN_CALLBACK_MAX_COUNT; i++)
    {
        if ((callback_table[i].callback != NULL) &&
            (callback_table[i].std_id == frame->std_id))
        {
#if 1
        	if (frame->std_id == 0x110U)
            {
                HAL_UART_Transmit(&huart3,
                                  (uint8_t *)"DBG,DISPATCH_110\r\n",
                                  18U,
                                  100U);
            }
#endif
            /* Task Context에서 실행 (각 task에 따른 함수 처리) */
            callback_table[i].callback(frame);
            return;
        }
    }

#if 1
    if (frame->std_id == 0x110U)
    {
        HAL_UART_Transmit(&huart3,
                          (uint8_t *)"DBG,NO_CALLBACK_110\r\n",
                          21U,
                          100U);
    }
#endif
}

/*
 CAN_RegisterRxCallback(CAN_ID_MODULE_ANNOUNCE,
                       ModuleManager_HandleAnnounce);

CAN_RegisterRxCallback(CAN_ID_GENERAL_PRESSURE,
                       ModuleA_HandlePressure);

CAN_RegisterRxCallback(CAN_ID_COLD_STATUS,
                       ModuleB_HandleStatus);
*/
