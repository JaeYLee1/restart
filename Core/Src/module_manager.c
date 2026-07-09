/*
 * module_manager.c
 *
 *  Created on: 2026. 7. 3.
 *      Author: 한국전파진흥협회
 */


/*
 * module_manager.c
 *
 * Module 인증 및 연결 상태 관리
 */

#include "module_manager.h"

/* 인증 대기 시간 */
#define AUTH_TIMEOUT_MS      1000U

/* Task 주기 */
#define MODULE_MANAGER_MS    20U


/* 인증 상태 */
static volatile AuthState_t auth_state = eAUTH_IDLE;

/* 승인된 모듈 정보 */
static volatile ModuleType_t module_type = eMODULE_NONE;
static volatile uint32_t module_id = 0U;
static volatile uint8_t link_accepted = 0U;

/* 인증 시작 시간 */
static TickType_t auth_start_tick;

/* 내부 함수 선언 */
static void ModuleManagerTask(void *argument);

static uint8_t ModuleManager_CheckId(ModuleType_t received_type,
                                     uint32_t received_id,
                                     uint8_t *reason);

static void ModuleManager_SendLinkResult(ModuleType_t result_module_type,
                                         uint8_t accepted,
                                         uint8_t reason);

/* ================= 초기화 ================= */

void ModuleManager_Init(void)
{
    /* CAN 0x110 수신 Callback 등록 */
    CAN_RegisterRxCallback(CAN_ID_MODULE_ANNOUNCE,
                           ModuleManager_HandleAnnounce);

    /* 인증 Timeout 확인 Task */
    if (xTaskCreate(ModuleManagerTask,
                    "MODULE_MGR",
                    256,
                    NULL,
                    3,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}


/* ================= Detect 연동 ================= */

/* 모듈 결합 감지 */
void ModuleManager_OnAttached(void)
{
    taskENTER_CRITICAL();

    module_type = eMODULE_NONE;
    module_id = 0U;
    link_accepted = 0U;

    auth_state = eAUTH_WAIT_MODULE_ID;
    auth_start_tick = xTaskGetTickCount();

    taskEXIT_CRITICAL();
}


/* 모듈 분리 감지 */
void ModuleManager_OnDetached(void)
{
    taskENTER_CRITICAL();

    module_type = eMODULE_NONE;
    module_id = 0U;
    link_accepted = 0U;

    auth_state = eAUTH_IDLE;

    taskEXIT_CRITICAL();
}


/* ================= CAN 인증 처리 ================= */

/*
 * CAN ID : 0x110
 *
 * data[0]    : module_type
 * data[1~4]  : module_id
 */
void ModuleManager_HandleAnnounce(const CAN_Frame_t *frame)
{
    ModuleType_t received_type;
    uint32_t received_id;
    uint8_t accepted;
    uint8_t reason;

    if (frame == NULL)
    {
        return;
    }

    /* Type 1Byte + ID 4Byte */
    if (frame->dlc < 5U)
    {
        return;
    }

    /* 인증 대기 상태가 아니면 무시 */
    if (auth_state != eAUTH_WAIT_MODULE_ID) // 인증 대기 상태가 되야 함
    {
        return;
    }

    received_type = (ModuleType_t)frame->data[0];

    received_id =
        ((uint32_t)frame->data[1] << 0)  |
        ((uint32_t)frame->data[2] << 8)  |
        ((uint32_t)frame->data[3] << 16) |
        ((uint32_t)frame->data[4] << 24);

    // ID가 맞는 지 확인
    accepted = ModuleManager_CheckId(received_type,
                                     received_id,
                                     &reason);

    taskENTER_CRITICAL();

    // 같은 ID면
    if (accepted == 1)
    {
    	// 모듈 type, ID, 연결 여부, 인증 상태 등 연결 상태로 초기화
        module_type = received_type;
        module_id = received_id;
        link_accepted = 1U;

        auth_state = eAUTH_ACCEPTED;
        GPIOB->BSRR |= (1<<0);
    }
    else
    {
        module_type = eMODULE_NONE;
        module_id = 0U;
        link_accepted = 0U;

        auth_state = eAUTH_REJECTED;
        GPIOB->BSRR |= (1<<16);
    }

    taskEXIT_CRITICAL();

    /* 모듈에 승인 / 거절 결과 전송 */
    ModuleManager_SendLinkResult(received_type,
                                 accepted,
                                 reason);
}


/* 등록된 Module ID인지 확인 */
static uint8_t ModuleManager_CheckId(ModuleType_t received_type,
                                     uint32_t received_id,
                                     uint8_t *reason)
{
    if (reason == NULL)
    {
        return 0U;
    }

    /* Module A */
    if (received_type == eMODULE_GENERAL)
    {
        if (received_id == MODULE_A_ID)
        {
            *reason = CAN_LINK_REASON_OK;
            return 1;
        }

        *reason = CAN_LINK_REASON_ID_MISMATCH;
        return 0;
    }

    /* Module B */
    if (received_type == eMODULE_COLD_CHAIN)
    {
        if (received_id == MODULE_B_ID)
        {
            *reason = CAN_LINK_REASON_OK;
            return 1;
        }

        *reason = CAN_LINK_REASON_ID_MISMATCH;
        return 0;
    }

    /* 알 수 없는 모듈 종류 */
    *reason = CAN_LINK_REASON_UNKNOWN_TYPE;
    return 0;
}


/* ================= Timeout 확인 ================= */
/* 인증 대기 상태가 남아 있으면 안 되므로 20ms마다 시간 초과 여부를 검사 */
static void ModuleManagerTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (auth_state == eAUTH_WAIT_MODULE_ID)
        {
            if ((xTaskGetTickCount() - auth_start_tick) >=
                pdMS_TO_TICKS(AUTH_TIMEOUT_MS))
            {
                taskENTER_CRITICAL();

                module_type = eMODULE_NONE;
                module_id = 0U;
                link_accepted = 0U;

                auth_state = eAUTH_REJECTED;

                taskEXIT_CRITICAL();

                /*
                 * 여기서 나중에 RelayTask에
                 * "전원 차단 요청" 전달 가능
                 */
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MODULE_MANAGER_MS));
    }
}

static void ModuleManager_SendLinkResult(ModuleType_t result_module_type,
                                         uint8_t accepted,
                                         uint8_t reason)
{
    CAN_Frame_t frame = {0};

    frame.std_id = CAN_ID_LINK_RESULT;
    frame.dlc = 3U;

    frame.data[0] = (uint8_t)result_module_type;
    frame.data[1] = accepted;
    frame.data[2] = reason;

    (void)CAN_Send(&frame, pdMS_TO_TICKS(10));
}

/* ================= 상태 조회 ================= */

AuthState_t ModuleManager_GetAuthState(void)
{
    return auth_state;
}


ModuleType_t ModuleManager_GetModuleType(void)
{
    return module_type;
}


uint32_t ModuleManager_GetModuleId(void)
{
    return module_id;
}


uint8_t ModuleManager_IsAccepted(void)
{
    return link_accepted;
}
