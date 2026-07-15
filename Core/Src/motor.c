/*
 * motor.c
 *
 * Base STM32 Motor Control
 *
 * PA4  : L298N IN1
 * PB4  : L298N IN2
 * PC6  : TIM3_CH1 PWM
 * PD12 : TIM4_CH1 Encoder A
 * PD13 : TIM4_CH2 Encoder B
 */

#include "motor.h"

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#include "system_data.h"


/* ================= 제어 설정값 ================= */

/* MotorControlTask 주기 */
#define MOTOR_CONTROL_PERIOD_MS    100U

/*
 * 감속기 출력축 1회전당 엔코더 카운트 수
 * 실제 바퀴 1회전 기준으로 측정 후 수정
 */
#define COUNTS_PER_REV             1320.0f

/*
 * TIM3 Clock = 84MHz
 * PSC = 83 → 1MHz
 * ARR = 49 → 20kHz
 */
#define PWM_ARR                    (50 - 1)


/* 속도 단계별 목표 RPM */
#define MOTOR_RPM_SLOW             80.0f
#define MOTOR_RPM_MID              120.0f
#define MOTOR_RPM_FAST             150.0f


/* PI Gain */
#define MOTOR_KP                   0.35f
#define MOTOR_KI                   0.40f

/* 졸음 감지 */
#define MOTOR_WARNING_LIMIT_RPM       30.0f
#define MOTOR_RAMP_STEP_RPM           2.0f

/* ================= 내부 상태 ================= */

static volatile uint8_t motor_running = 0U;

static volatile MotorSpeedLevel_t motor_speed_level =
    eMOTOR_STOP;

static volatile float target_rpm = 0.0f;
static volatile float current_rpm = 0.0f;
static volatile float motor_duty = 0.0f;

static uint16_t prev_encoder_cnt = 0U;

static float pi_prev_error = 0.0f;
static float pi_prev_duty = 0.0f;

static volatile float user_target_rpm = 0.0f;
static volatile uint8_t warning_slowdown_active = 0U;
static volatile uint8_t warning_recovery_active = 0U;

/* ================= 내부 함수 선언 ================= */

static void Motor_GPIO_Init(void);

static void TIM3_PWM_Init(void);
static void TIM4_Encoder_Init(void);

static void Motor_StartHardware(void);
static void Motor_StopHardware(void);

static void Motor_SetDuty(float duty);

static void Encoder_UpdateRPM(void);
static void Motor_PI_Control(void);

static void MotorControlTask(void *argument);

static void Motor_UpdateRampTarget(void);

/* ================= 초기화 ================= */

void Motor_Init(void)
{
    Motor_GPIO_Init();

    TIM3_PWM_Init();
    TIM4_Encoder_Init();

    motor_running = 0U;
    motor_speed_level = eMOTOR_STOP;

    target_rpm = 0.0f;
    current_rpm = 0.0f;
    motor_duty = 0.0f;

    pi_prev_error = 0.0f;
    pi_prev_duty = 0.0f;

    user_target_rpm = 0.0f;
    warning_slowdown_active = 0U;
    warning_recovery_active = 0U;

    Motor_StopHardware();
}


void Motor_RtosInit(void)
{
    if (xTaskCreate(MotorControlTask,
                    "MOTOR",
                    256,
                    NULL,
                    3,
                    NULL) != pdPASS)
    {
        Error_Handler();
    }
}


/* ================= GPIO 초기화 ================= */

static void Motor_GPIO_Init(void)
{
    /* GPIOA, GPIOB, GPIOC, GPIOD Clock */
    RCC->AHB1ENR |= (1U << 0);
    RCC->AHB1ENR |= (1U << 1);
    RCC->AHB1ENR |= (1U << 2);
    RCC->AHB1ENR |= (1U << 3);

    /* PA4 = L298N IN1 Output */
    GPIOA->MODER &= ~(3U << (4U * 2U));
    GPIOA->MODER |=  (1U << (4U * 2U));

    GPIOA->PUPDR &= ~(3U << (4U * 2U));

    /* PB4 = L298N IN2 Output */
    GPIOB->MODER &= ~(3U << (4U * 2U));
    GPIOB->MODER |=  (1U << (4U * 2U));

    GPIOB->PUPDR &= ~(3U << (4U * 2U));

    /* PC6 = TIM3_CH1 PWM, AF2 */
    GPIOC->MODER &= ~(3U << (6U * 2U));
    GPIOC->MODER |=  (2U << (6U * 2U));

    GPIOC->PUPDR &= ~(3U << (6U * 2U));

    GPIOC->AFR[0] &= ~(0xFU << (6U * 4U));
    GPIOC->AFR[0] |=  (2U << (6U * 4U));

    /* PD12 = TIM4_CH1, PD13 = TIM4_CH2 */
    GPIOD->MODER &= ~((3U << (12U * 2U)) |
                      (3U << (13U * 2U)));

    GPIOD->MODER |=  ((2U << (12U * 2U)) |
                      (2U << (13U * 2U)));

    /* PD12 / PD13 = AF2 TIM4 */
    GPIOD->AFR[1] &= ~((0xFU << ((12U - 8U) * 4U)) |
                       (0xFU << ((13U - 8U) * 4U)));

    GPIOD->AFR[1] |=  ((2U << ((12U - 8U) * 4U)) |
                       (2U << ((13U - 8U) * 4U)));

    /* Encoder 입력 Pull-up */
    GPIOD->PUPDR &= ~((3U << (12U * 2U)) |
                      (3U << (13U * 2U)));

    GPIOD->PUPDR |=  ((1U << (12U * 2U)) |
                      (1U << (13U * 2U)));

    /* 초기 모터 정지 */
    GPIOA->BSRR = (1 << (4 + 16));
    GPIOB->BSRR = (1 << (4 + 16));
}


/* ================= PWM Timer ================= */

static void TIM3_PWM_Init(void)
{
    /* TIM3 Clock Enable */
    RCC->APB1ENR |= (1U << 1);

    /*
     * TIM3 Clock = 84MHz
     * PSC = 83 → 1MHz
     * ARR = 49 → PWM 20kHz
     */
    TIM3->PSC = 84 - 1;
    TIM3->ARR = PWM_ARR;
    TIM3->CCR1 = 0U;

    /* CH1 PWM Mode 1 */
    TIM3->CCMR1 &= ~(7U << 4);
    TIM3->CCMR1 |=  (6U << 4);

    /* CCR1 Preload */
    TIM3->CCMR1 |= (1U << 3);

    /* ARR Preload */
    TIM3->CR1 |= (1U << 7);

    /* CH1 Output Enable */
    TIM3->CCER |= (1U << 0);

    /* Register 적용 */
    TIM3->EGR |= (1U << 0);

    /* Timer Start */
    TIM3->CR1 |= (1U << 0);
}


/* ================= Encoder Timer ================= */

static void TIM4_Encoder_Init(void)
{
    /* TIM4 Clock Enable */
    RCC->APB1ENR |= (1U << 2);

    /* 최대 16bit Counter 사용 */
    TIM4->PSC = 0U;
    TIM4->ARR = 0xFFFFU;
    TIM4->CNT = 0U;

    /* CH1 = TI1, CH2 = TI2 */
    TIM4->CCMR1 &= ~((3U << 0) |
                     (3U << 8));

    TIM4->CCMR1 |=  ((1U << 0) |
                     (1U << 8));

    /* Encoder 입력 Filter */
    TIM4->CCMR1 |= ((0xFU << 4) |
                    (0xFU << 12));

    /* Input Capture Enable */
    TIM4->CCER |= ((1U << 0) |
                   (1U << 4));

    /*
     * Encoder Mode 3
     * TI1, TI2 두 위상 사용
     */
    TIM4->SMCR &= ~(7U << 0);
    TIM4->SMCR |=  (3U << 0);

    /* Register 적용 */
    TIM4->EGR |= (1U << 0);

    /* Timer Start */
    TIM4->CR1 |= (1U << 0);

    prev_encoder_cnt = (uint16_t)TIM4->CNT;
}


/* ================= 모터 HW 제어 ================= */

static void Motor_StartHardware(void)
{
    /* 정회전: IN1 = High, IN2 = Low */
    GPIOA->BSRR = (1 << 4);
    GPIOB->BSRR = (1 << (4 + 16));
}


static void Motor_StopHardware(void)
{
    /* PWM 출력 0 */
    TIM3->CCR1 = 0U;

    /* IN1 = Low, IN2 = Low */
    GPIOA->BSRR = (1U << (4 + 16));
    GPIOB->BSRR = (1U << (4 + 16));
}


static void Motor_SetDuty(float duty)
{
    uint32_t ccr_value;

    if (duty > 100.0f)
    {
        duty = 100.0f;
    }

    if (duty < 0.0f)
    {
        duty = 0.0f;
    }

    motor_duty = duty;

    ccr_value =
        (uint32_t)(((float)(PWM_ARR + 1U) * duty) / 100.0f);

    TIM3->CCR1 = ccr_value;
}


/* ================= RPM / PI 제어 ================= */

static void Encoder_UpdateRPM(void)
{
    uint16_t now_count;
    int16_t delta_count;

    now_count = (uint16_t)TIM4->CNT;

    /*
     * int16_t 변환으로
     * 정/역방향 및 Counter Overflow 대응
     */
    delta_count =
        (int16_t)(now_count - prev_encoder_cnt);

    prev_encoder_cnt = now_count;

    /*
     * 제어 주기 100ms 기준:
     * RPM = (count / counts_per_rev) * 600
     */
    current_rpm =
        ((float)delta_count / COUNTS_PER_REV) * 600.0f;
}


static void Motor_PI_Control(void)
{
    float error;
    float next_duty;
    float dt;

    dt = (float)MOTOR_CONTROL_PERIOD_MS / 1000.0f;

    error = target_rpm - current_rpm;

    /*
     * 속도형 PI 제어기
     *
     * u[k] = u[k-1]
     *      + Kp * (e[k] - e[k-1])
     *      + Ki * dt * e[k]
     */
    next_duty =
        pi_prev_duty
        + MOTOR_KP * (error - pi_prev_error)
        + MOTOR_KI * dt * error;

    Motor_SetDuty(next_duty);

    pi_prev_duty = motor_duty;
    pi_prev_error = error;
}

static void Motor_UpdateRampTarget(void)
{
    float desired_rpm;

    if (motor_running == 0U)
    {
        target_rpm = 0.0f;
        return;
    }

    if (warning_slowdown_active != 0U)
    {
        desired_rpm = MOTOR_WARNING_LIMIT_RPM;
    }
    else
    {
        desired_rpm = user_target_rpm;
    }

    if (target_rpm < desired_rpm)
    {
        target_rpm += MOTOR_RAMP_STEP_RPM;

        if (target_rpm > desired_rpm)
        {
            target_rpm = desired_rpm;
        }
    }
    else if (target_rpm > desired_rpm)
    {
        target_rpm -= MOTOR_RAMP_STEP_RPM;

        if (target_rpm < desired_rpm)
        {
            target_rpm = desired_rpm;
        }
    }

    if (warning_recovery_active != 0U)
    {
        if ((target_rpm >= user_target_rpm - 0.1f) &&
            (target_rpm <= user_target_rpm + 0.1f))
        {
            warning_recovery_active = 0U;
        }
    }
}

/* ================= 외부 제어 API ================= */

void Motor_RequestStart(void)
{
    motor_running = 1;

    /* 시작 기본 속도: 중간 */
    motor_speed_level = eMOTOR_MID;

    user_target_rpm = MOTOR_RPM_MID;
    target_rpm = user_target_rpm;

    warning_slowdown_active = 0U;
    warning_recovery_active = 0U;

    /* PI 초기화 */
    pi_prev_error = 0.0f;
    pi_prev_duty = 0.0f;

    Motor_StartHardware();
    Motor_SetDuty(0.0f);
}


void Motor_RequestStop(void)
{
    motor_running = 0;

    motor_speed_level = eMOTOR_STOP;

    user_target_rpm = 0.0f;
    target_rpm = 0.0f;

    warning_slowdown_active = 0U;
    warning_recovery_active = 0U;

    pi_prev_error = 0.0f;
    pi_prev_duty = 0.0f;

    Motor_StopHardware();
}

void Motor_SetSpeedLevel(MotorSpeedLevel_t level)
{
    switch (level)
    {
        case eMOTOR_SLOW:
            motor_speed_level = eMOTOR_SLOW;
            user_target_rpm = MOTOR_RPM_SLOW;
            break;

        case eMOTOR_MID:
            motor_speed_level = eMOTOR_MID;
            user_target_rpm = MOTOR_RPM_MID;
            break;

        case eMOTOR_FAST:
            motor_speed_level = eMOTOR_FAST;
            user_target_rpm = MOTOR_RPM_FAST;
            break;

        case eMOTOR_STOP:
        default:
            Motor_RequestStop();
            return;
    }

    if (warning_slowdown_active == 0U)
    {
        target_rpm = user_target_rpm;
    }
}

void Motor_WarningSlowdown(void)
{
    if (motor_running == 0U)
    {
        return;
    }

    warning_slowdown_active = 1U;
    warning_recovery_active = 0U;
}


void Motor_WarningRecover(void)
{
    if (motor_running == 0U)
    {
        warning_slowdown_active = 0U;
        warning_recovery_active = 0U;
        return;
    }

    warning_slowdown_active = 0U;
    warning_recovery_active = 1U;
}

/* ================= Motor Control Task ================= */

static void MotorControlTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* RPM은 정지 상태에서도 갱신 */
        Encoder_UpdateRPM();

        if (motor_running == 1U)
        {
        	Motor_UpdateRampTarget();
            Motor_PI_Control();
        }
        else
        {
            Motor_SetDuty(0.0f);
        }

        /* UART / GUI / Raspberry Pi 전달용 전역 데이터 */
        g_system_data.motor_running = motor_running;

        g_system_data.motor_speed_level =
            (uint8_t)motor_speed_level;

        g_system_data.target_speed_rpm = target_rpm;
        g_system_data.current_speed_rpm = current_rpm;
        g_system_data.motor_pwm_duty = motor_duty;

        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS));
    }
}


/* ================= 상태 조회 ================= */

uint8_t Motor_IsRunning(void)
{
    return motor_running;
}


float Motor_GetCurrentRpm(void)
{
    return current_rpm;
}


float Motor_GetTargetRpm(void)
{
    return target_rpm;
}


float Motor_GetDuty(void)
{
    return motor_duty;
}

