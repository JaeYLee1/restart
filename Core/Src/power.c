/*
 * power.c
 *
 *  Created on: 2026. 7. 11.
 *      Author: 한국전파진흥협회
 */

#include "main.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "power.h"
#include "system_data.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "module_manager.h"
#include "relay.h"
#include "warning.h"

/* =========================================================
 * External Handle
 * ========================================================= */
extern UART_HandleTypeDef huart3;

/* =========================================================
 * Sensor Use Select
 * ========================================================= */
#define POWER_USE_INA3221         (1)
#define POWER_USE_INA226          (1)

/* =========================================================
 * Sensor Address
 * =========================================================
 *
 * INA3221:
 * A0 = GND -> 0x40
 * A0 = VS  -> 0x41
 * A0 = SDA -> 0x42
 * A0 = SCL -> 0x43
 *
 */
#define INA3221_ADDR              (0x41)

/* INA226 #1: Fan + Peltier */
#define INA226_COOLING_ADDR       (0x40)

/* INA226 #2: Motor */
#define INA226_MOTOR_ADDR         (0x45)

/* Sensor ID */
#define POWER_SENSOR_INA3221          (3221)
#define POWER_SENSOR_INA226_COOLING   (2260)
#define POWER_SENSOR_INA226_MOTOR     (2261)
/* =========================================================
 * INA3221 Register
 * ========================================================= */
#define INA3221_CONFIG            (0x00)
#define INA3221_SHUNT1            (0x01)
#define INA3221_BUS1              (0x02)
#define INA3221_SHUNT2            (0x03)
#define INA3221_BUS2              (0x04)
#define INA3221_SHUNT3            (0x05)
#define INA3221_BUS3              (0x06)

/* =========================================================
 * INA226 Register
 * ========================================================= */
#define INA226_CONFIG             (0x00)
#define INA226_SHUNT              (0x01)
#define INA226_BUS                (0x02)
#define INA226_POWER              (0x03)
#define INA226_CURRENT            (0x04)
#define INA226_CALI               (0x05)
#define INA226_MANUF_ID           (0xFE)
#define INA226_DIE_ID             (0xFF)

/* =========================================================
 * User Setting
 * ========================================================= */
#define POWER_I2C_TIMEOUT_MS      (100U)
#define POWER_MONITOR_PERIOD_MS   (500U)

/*
 * INA226 단독 HAL 테스트에서 정상 확인한 설정값.
 * AVG 1회, VBUS/VSHUNT conversion 1.1ms, Shunt+Bus Continuous.
 */
#define INA226_CONFIG_VALUE       (0x4127U)

/*
 * 보드 션트저항 기준.
 * R100 = 0.1Ω = 100mΩ
 * R010 = 0.01Ω = 10mΩ
 */
#define INA3221_SHUNT_MOHM          (100U)  /* INA3221 보드가 R100이면 100mΩ */

#define INA226_COOLING_SHUNT_MOHM   (100U)  // 팬+펠티어 센서가 R100이면
#define INA226_MOTOR_SHUNT_MOHM     (10U)   // 모터는 R010

// 한계치 이상 전력 사용 시 경고 (과전력)
#define MODULE_B_POWER_LIMIT_W          (10.0f)  /* 임시값, 나중에 너희가 정하면 됨 */
#define POWER_OVER_WARNING_COUNT        (3U)
#define POWER_OVER_CUTOFF_COUNT         (4U)

// 고장 모듈 감지 (배선 끊어짐)
#define PELTIER_LOW_CURRENT_MA          (100L)
#define PELTIER_STABLE_TIME_MS          (500U)
#define PELTIER_LOW_CURRENT_TIME_MS     (2000U)
#define POWER_TASK_PERIOD_MS            (POWER_MONITOR_PERIOD_MS)

#define PELTIER_STABLE_COUNT            (PELTIER_STABLE_TIME_MS / POWER_TASK_PERIOD_MS)
#define PELTIER_LOW_CURRENT_COUNT       (PELTIER_LOW_CURRENT_TIME_MS / POWER_TASK_PERIOD_MS)
/* =========================================================
 * Internal Type
 * ========================================================= */
typedef struct
{
    int32_t bus_mv;
    int32_t shunt_uv;
    int32_t current_ma;
    int32_t power_mw;
} PowerTestValue_t;

/* =========================================================
 * I2C State Machine
 * ========================================================= */
typedef enum
{
    POWER_I2C_IDLE = 0,

    POWER_I2C_SEND_REG8_1,
    POWER_I2C_SEND_REG8_HIGH,
    POWER_I2C_SEND_REG8_LOW,
    POWER_I2C_SEND_REG8_2,

    POWER_I2C_SET_REG,
    POWER_I2C_READ_DATA,
    POWER_I2C_READ_DATA_H,
    POWER_I2C_READ_DATA_L

} PowerI2C_State_t;

typedef struct
{
    uint8_t slave_addr;
    uint8_t reg_addr;
    uint8_t data_h;
    uint8_t data_l;
    volatile uint8_t set_regs;
    volatile PowerI2C_State_t state;

} PowerI2C_Handle_t;

static volatile PowerI2C_Handle_t h_power_i2c;

static volatile uint8_t data_H;
static volatile uint8_t data_L;

static volatile uint8_t g_i2c_bus_error_flag = 0U;
static volatile uint32_t g_i2c_last_error_sr1 = 0U;
static uint8_t power_over_count = 0U;
static uint8_t power_cutoff_latched = 0U;

static uint8_t peltier_stable_count = 0U;
static uint8_t peltier_low_current_count = 0U;
static uint8_t peltier_fault_latched = 0U;
/* =========================================================
 * Internal Function
 * ========================================================= */

static void Power_TaskDelay1ms(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void Power_Print(const char *msg)
{
    HAL_UART_Transmit(&huart3,
                      (uint8_t *)msg,
                      strlen(msg),
                      100);
}

static void Power_I2C_Abort(void)
{
    I2C1->CR1 |= I2C_CR1_STOP;

    h_power_i2c.state = POWER_I2C_IDLE;
    h_power_i2c.set_regs = 0U;

    g_i2c_bus_error_flag = 1U;
}

static uint8_t Power_WaitIdle(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (h_power_i2c.state != POWER_I2C_IDLE)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            Power_I2C_Abort();
            return 0U;
        }

        Power_TaskDelay1ms();
    }

    return 1U;
}

static uint8_t Power_WaitSetRegDone(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (h_power_i2c.set_regs != 0U)
    {
        if ((HAL_GetTick() - start_tick) > timeout_ms)
        {
            Power_I2C_Abort();
            return 0U;
        }

        Power_TaskDelay1ms();
    }

    return 1U;
}

static uint8_t Power_GetWriteAddress(int sensor, uint8_t *addr)
{
    if (addr == NULL)
    {
        return 0U;
    }

    if (sensor == POWER_SENSOR_INA3221)
    {
        *addr = (uint8_t)((INA3221_ADDR << 1) | 0U);
        return 1U;
    }
    else if (sensor == POWER_SENSOR_INA226_COOLING)
    {
        *addr = (uint8_t)((INA226_COOLING_ADDR << 1) | 0U);
        return 1U;
    }
    else if (sensor == POWER_SENSOR_INA226_MOTOR)
    {
        *addr = (uint8_t)((INA226_MOTOR_ADDR << 1) | 0U);
        return 1U;
    }

    return 0U;
}

static uint8_t Power_GetReadAddress(int sensor, uint8_t *addr)
{
    if (addr == NULL)
    {
        return 0U;
    }

    if (sensor == POWER_SENSOR_INA3221)
    {
        *addr = (uint8_t)((INA3221_ADDR << 1) | 1U);
        return 1U;
    }
    else if (sensor == POWER_SENSOR_INA226_COOLING)
    {
        *addr = (uint8_t)((INA226_COOLING_ADDR << 1) | 1U);
        return 1U;
    }
    else if (sensor == POWER_SENSOR_INA226_MOTOR)
    {
        *addr = (uint8_t)((INA226_MOTOR_ADDR << 1) | 1U);
        return 1U;
    }

    return 0U;
}

static uint8_t Power_ReadStart(int sensor)
{
    if (Power_WaitSetRegDone(POWER_I2C_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (Power_GetReadAddress(sensor, (uint8_t *)&h_power_i2c.slave_addr) == 0U)
    {
        return 0U;
    }

    h_power_i2c.data_h = 0U;
    h_power_i2c.data_l = 0U;
    h_power_i2c.state = POWER_I2C_READ_DATA;

    I2C1->CR1 |= I2C_CR1_START;

    return 1U;
}

static int32_t Power_Abs32(int32_t value)
{
    if (value < 0)
    {
        return -value;
    }

    return value;
}

static void Power_UpdateModuleBTotalAndProtection(void)
{
	g_system_data.power_limit_w = MODULE_B_POWER_LIMIT_W;

    float module_b_total_w;

    module_b_total_w =
        g_system_data.module_b_power_w +
        g_system_data.cooling_power_w;

    /*
     * 변수명은 total_power_w지만,
     * 현재 프로젝트에서는 Module B 전체 전력으로 사용.
     *
     * total_power_w = Module B STM32 + Peltier + Fan
     */
    g_system_data.total_power_w = module_b_total_w;

    /*
     * Module B가 아니거나 인증 전이면 보호 로직 동작 안 함.
     */
    if ((ModuleManager_GetModuleType() != 2U) ||
        (ModuleManager_IsAccepted() == 0U) ||
        (Relay_IsConnected() == 0U))
    {
        power_over_count = 0U;

        /*
         * 차단 latch는 모듈 분리 또는 명시적인 reset 때만 해제
         */
        g_system_data.power_warning_count = 0U;

        return;
    }

    /*
     * 이미 차단된 상태면 반복 차단 방지.
     */
    if (power_cutoff_latched != 0U)
    {
        return;
    }

    if (module_b_total_w > MODULE_B_POWER_LIMIT_W)
    {
        if (power_over_count < POWER_OVER_CUTOFF_COUNT)
        {
            power_over_count++;
        }

        g_system_data.power_warning_count = power_over_count;

        if (power_over_count >= POWER_OVER_CUTOFF_COUNT)
        {
            power_cutoff_latched = 1U;

            ModuleB_Peltier_Off();
            Relay_ForceOff();

            g_system_data.power_fault = 1U;
            g_system_data.fsm_state = FSM_FAULT;
        }
    }
    else
    {
        power_over_count = 0U;
        g_system_data.power_warning_count = 0U;

        /*
         * 차단 전 정상 복귀 시에만 fault 해제.
         * 이미 차단된 fault는 유지.
         */
        if (power_cutoff_latched == 0U)
        {
            g_system_data.power_fault = 0U;
        }
    }

}

// 고장모듈 감지
static void Power_UpdatePeltierOpenFault(uint8_t ok_cooling,
                                         int32_t cooling_current_ma)
{
    uint8_t peltier_should_be_on;

    /*
     * Module B가 정상 인증되어 ACTIVE 상태이고,
     * 메인 릴레이가 연결된 상태면 펠티어 전류 감시 대상.
     *
     * 현재 구조상 이 조건이면 RelayTask에서 ModuleB_Peltier_On()이 유지됨.
     */
    peltier_should_be_on =
        (ModuleManager_GetModuleType() == 2U) &&
        (ModuleManager_IsAccepted() == 1U) &&
        (Relay_IsConnected() == 1U) &&
        (g_system_data.fsm_state == FSM_ACTIVE);

    if (peltier_should_be_on == 0U)
    {
        peltier_stable_count = 0U;
        peltier_low_current_count = 0U;

        if (peltier_fault_latched == 0U)
        {
            /* 정상 분리/대기 상태에서는 fault 해제 */
        }

        return;
    }

    if (peltier_fault_latched != 0U)
    {
        return;
    }

    /*
     * 릴레이 ON 직후 센서값 안정화 시간.
     * POWER_MONITOR_PERIOD_MS = 500ms면 1회 skip.
     */
    if (peltier_stable_count < PELTIER_STABLE_COUNT)
    {
        peltier_stable_count++;
        peltier_low_current_count = 0U;
        return;
    }

    /*
     * 센서 자체가 안 읽히면 단선 판정하지 않음.
     * 이건 I2C 센서 오류로 따로 봐야 함.
     */
    if (ok_cooling == 0U)
    {
        peltier_low_current_count = 0U;
        return;
    }

    /*
     * 릴레이 ON인데 전류가 기준 이하인 상태가 2초 지속되면 fault.
     */
    if (cooling_current_ma < PELTIER_LOW_CURRENT_MA)
    {
        if (peltier_low_current_count < PELTIER_LOW_CURRENT_COUNT)
        {
            peltier_low_current_count++;
        }

        if (peltier_low_current_count >= PELTIER_LOW_CURRENT_COUNT)
        {
            peltier_fault_latched = 1U;

            ModuleB_Peltier_Off();
            Relay_ForceOff();

            g_system_data.power_fault = 1U;
            g_system_data.fsm_state = FSM_FAULT;

            /*
             * 부저 + LED 매트릭스 경고.
             * warning.c에 있는 함수명에 맞춰 사용.
             * 예: Warning_OnAiLevel(2U) 또는 Warning_OnGuiReset 반대 함수.
             */
            Warning_OnAiLevel(2U);
        }
    }
    else
    {
        peltier_low_current_count = 0U;
    }
}

/* =========================================================
 * Public I2C Function
 * ========================================================= */

void Power_I2C1_Init(void)
{
    /*
     * I2C1 Pin:
     * PB8 = I2C1_SCL
     * PB9 = I2C1_SDA
     *
     * APB1 = 42MHz 기준
     * Standard Mode 100kHz
     */

    memset((void *)&h_power_i2c, 0, sizeof(h_power_i2c));
    h_power_i2c.state = POWER_I2C_IDLE;
    h_power_i2c.set_regs = 0U;

    data_H = 0U;
    data_L = 0U;
    g_i2c_bus_error_flag = 0U;
    g_i2c_last_error_sr1 = 0U;

    /* GPIOB Clock Enable */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* PB8, PB9 Alternate Function Mode */
    GPIOB->MODER &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->MODER |=  ((2UL << (8U * 2U)) | (2UL << (9U * 2U)));

    /* Open Drain */
    GPIOB->OTYPER |= (1UL << 8U) | (1UL << 9U);

    /* Pull-up */
    GPIOB->PUPDR &= ~((3UL << (8U * 2U)) | (3UL << (9U * 2U)));
    GPIOB->PUPDR |=  ((1UL << (8U * 2U)) | (1UL << (9U * 2U)));

    /* High Speed */
    GPIOB->OSPEEDR |= ((3UL << (8U * 2U)) | (3UL << (9U * 2U)));

    /* AF4 for I2C1 */
    GPIOB->AFR[1] &= ~((0xFUL << ((8U - 8U) * 4U)) |
                       (0xFUL << ((9U - 8U) * 4U)));
    GPIOB->AFR[1] |=  ((4UL << ((8U - 8U) * 4U)) |
                       (4UL << ((9U - 8U) * 4U)));

    /* I2C1 Clock Enable */
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /* I2C Software Reset */
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR1 = 0U;

    /*
     * CR2 FREQ = 42MHz
     * Interrupt enable
     */
    I2C1->CR2 = 42U | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;

    /*
     * Standard mode:
     * CCR = APB1 / (2 * I2C_SPEED)
     *     = 42MHz / 200kHz
     *     = 210
     */
    I2C1->CCR = 210U;
    I2C1->TRISE = 43U;

    /* I2C Enable */
    I2C1->CR1 |= I2C_CR1_PE;
}

uint8_t Power_WriteReg16(int sensor, uint8_t reg, uint16_t data)
{
    uint8_t addr;

    if (Power_WaitIdle(POWER_I2C_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (Power_GetWriteAddress(sensor, &addr) == 0U)
    {
        return 0U;
    }

    g_i2c_bus_error_flag = 0U;

    h_power_i2c.slave_addr = addr;
    h_power_i2c.reg_addr = reg;
    h_power_i2c.data_h = (uint8_t)(data >> 8);
    h_power_i2c.data_l = (uint8_t)(data & 0xFFU);
    h_power_i2c.state = POWER_I2C_SEND_REG8_1;

    I2C1->CR1 |= I2C_CR1_START;

    if (Power_WaitIdle(POWER_I2C_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (g_i2c_bus_error_flag != 0U)
    {
        return 0U;
    }

    return 1U;
}

uint8_t Power_ReadReg16(int sensor, uint8_t reg, int16_t *out_data)
{
    uint8_t addr;

    if (out_data == NULL)
    {
        return 0U;
    }

    if (Power_WaitIdle(POWER_I2C_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (Power_GetWriteAddress(sensor, &addr) == 0U)
    {
        return 0U;
    }

    g_i2c_bus_error_flag = 0U;

    h_power_i2c.slave_addr = addr;
    h_power_i2c.reg_addr = reg;
    h_power_i2c.data_h = 0U;
    h_power_i2c.data_l = 0U;
    h_power_i2c.set_regs = 1U;
    h_power_i2c.state = POWER_I2C_SET_REG;

    /*
     * Register Pointer Set:
     * START + SlaveAddr(W) + RegisterAddr
     */
    I2C1->CR1 |= I2C_CR1_START;

    /*
     * Repeated START + SlaveAddr(R)
     */
    if (Power_ReadStart(sensor) == 0U)
    {
        return 0U;
    }

    if (Power_WaitIdle(POWER_I2C_TIMEOUT_MS) == 0U)
    {
        return 0U;
    }

    if (g_i2c_bus_error_flag != 0U)
    {
        return 0U;
    }

    *out_data = (int16_t)(((uint16_t)data_H << 8) | (uint16_t)data_L);

    return 1U;
}

/* =========================================================
 * Sensor Init
 * ========================================================= */

void Power_INA3221_Init(void)
{
#if POWER_USE_INA3221
    /*
     * INA3221 전체 채널 활성화.
     * CH1, CH3 사용 예정.
     */
    (void)Power_WriteReg16(POWER_SENSOR_INA3221, INA3221_CONFIG, 0x7327U);
#else
    return;
#endif
}

static void Power_INA226_InitBySensor(int sensor)
{
#if POWER_USE_INA226
    /*
     * INA226 설정값:
     * AVG 1회, VBUS/VSHUNT conversion 1.1ms,
     * Shunt + Bus Continuous mode.
     */
    (void)Power_WriteReg16(sensor, INA226_CONFIG, INA226_CONFIG_VALUE);
#else
    (void)sensor;
    return;
#endif
}

void Power_INA226_Cooling_Init(void)
{
    Power_INA226_InitBySensor(POWER_SENSOR_INA226_COOLING);
}

void Power_INA226_Motor_Init(void)
{
    Power_INA226_InitBySensor(POWER_SENSOR_INA226_MOTOR);
}

/*
 * 기존 코드 호환용.
 * 기존 Power_INA226_Init()은 Cooling INA226 0x40 초기화로 유지.
 */
void Power_INA226_Init(void)
{
    Power_INA226_Cooling_Init();
}

/* =========================================================
 * INA3221 Read Function - Disabled
 * ========================================================= */

#if POWER_USE_INA3221
static uint8_t Power_Read_INA3221_Channel(uint8_t bus_reg,
                                          uint8_t shunt_reg,
                                          PowerTestValue_t *out)
{
    int16_t bus_raw;
    int16_t shunt_raw;

    int16_t bus_valid;
    int16_t shunt_valid;

    if (out == NULL)
    {
        return 0U;
    }

    if (Power_ReadReg16(POWER_SENSOR_INA3221, bus_reg, &bus_raw) == 0U)
    {
        return 0U;
    }

    if (Power_ReadReg16(POWER_SENSOR_INA3221, shunt_reg, &shunt_raw) == 0U)
    {
        return 0U;
    }

    /*
     * INA3221:
     * Bus register   : 상위 13bit 유효, LSB = 8mV
     * Shunt register : 상위 13bit 유효, LSB = 40uV
     */
    bus_valid = (int16_t)(bus_raw >> 3);
    shunt_valid = (int16_t)(shunt_raw >> 3);

    out->bus_mv = (int32_t)bus_valid * 8L;
    out->shunt_uv = (int32_t)shunt_valid * 40L;

    /*
     * current[mA] = shunt[uV] / shunt[mΩ]
     */
    out->current_ma = out->shunt_uv / (int32_t)INA3221_SHUNT_MOHM;

    if (out->current_ma < 0)
    {
        out->current_ma = -out->current_ma;
    }

    out->power_mw = (out->bus_mv * out->current_ma) / 1000L;

    if (out->power_mw < 0)
    {
        out->power_mw = -out->power_mw;
    }

    return 1U;
}
#endif

/* =========================================================
 * INA226 Read Function
 * ========================================================= */

static uint8_t Power_INA226_ReadBySensor(int sensor,
                                         uint32_t shunt_mohm,
                                         Power_INA226Value_t *out)
{
    int16_t bus_raw;
    int16_t shunt_raw;

    int32_t bus_mv;
    int32_t shunt_uv;
    int32_t current_ma;
    int32_t power_mw;

    if (out == NULL)
    {
        return 0U;
    }

    if (Power_ReadReg16(sensor, INA226_BUS, &bus_raw) == 0U)
    {
        return 0U;
    }

    if (Power_ReadReg16(sensor, INA226_SHUNT, &shunt_raw) == 0U)
    {
        return 0U;
    }

    bus_mv = ((int32_t)bus_raw * 125L) / 100L;
    shunt_uv = ((int32_t)shunt_raw * 25L) / 10L;

    current_ma = shunt_uv / (int32_t)shunt_mohm;

    if (current_ma < 0)
    {
        current_ma = -current_ma;
    }

    power_mw = (bus_mv * current_ma) / 1000L;
    power_mw = Power_Abs32(power_mw);

    out->bus_mv = bus_mv;
    out->shunt_uv = shunt_uv;
    out->current_ma = current_ma;
    out->power_mw = power_mw;

    return 1U;
}

uint8_t Power_INA226_Cooling_Read(Power_INA226Value_t *out)
{
    return Power_INA226_ReadBySensor(POWER_SENSOR_INA226_COOLING,
                                     INA226_COOLING_SHUNT_MOHM,
                                     out);
}

uint8_t Power_INA226_Motor_Read(Power_INA226Value_t *out)
{
    return Power_INA226_ReadBySensor(POWER_SENSOR_INA226_MOTOR,
                                     INA226_MOTOR_SHUNT_MOHM,
                                     out);
}

/*
 * 기존 코드 호환용.
 * 기존 Power_INA226_Read()는 Cooling INA226 0x40 읽기로 유지.
 */
uint8_t Power_INA226_Read(Power_INA226Value_t *out)
{
    return Power_INA226_Cooling_Read(out);
}

/* =========================================================
 * RTOS Task
 * ========================================================= */

void PowerMonitor_Task(void *argument)
{
    PowerTestValue_t ina3221_base;
    PowerTestValue_t ina3221_module_b;
    Power_INA226Value_t cooling_ina226;
    Power_INA226Value_t motor_ina226;

    char msg[256];

    uint8_t ok_base = 0U;
    uint8_t ok_module_b = 0U;
    uint8_t ok_cooling = 0U;
    uint8_t ok_motor = 0U;

    int32_t base_power_mw = 0;
    int32_t module_b_power_mw = 0;
    int32_t cooling_power_mw = 0;
    int32_t motor_power_mw = 0;

#if 0 // 테스트용
    int32_t total_power_mw = 0;
#endif

    (void)argument;

    Power_I2C1_Init();

    /*
     * 전체 센서 초기화
     */
    Power_INA3221_Init();          /* INA3221 0x41 */
    Power_INA226_Cooling_Init();   /* INA226 0x40 */
    Power_INA226_Motor_Init();     /* INA226 0x45 */

    for (;;)
    {
        if (g_i2c_bus_error_flag != 0U)
        {
            snprintf(msg,
                     sizeof(msg),
                     "[I2C ERROR] SR1=0x%04lX, Re-init\r\n",
                     g_i2c_last_error_sr1);

            Power_Print(msg);

            Power_I2C1_Init();

            Power_INA3221_Init();
            Power_INA226_Cooling_Init();
            Power_INA226_Motor_Init();

            g_i2c_bus_error_flag = 0U;

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        memset(&ina3221_base, 0, sizeof(ina3221_base));
        memset(&ina3221_module_b, 0, sizeof(ina3221_module_b));
        memset(&cooling_ina226, 0, sizeof(cooling_ina226));
        memset(&motor_ina226, 0, sizeof(motor_ina226));

        /*
         * INA3221
         * CH1: Base STM32 보드
         * CH3: Module B STM32 보드
         */

        ok_base = Power_Read_INA3221_Channel(INA3221_BUS1,
                                             INA3221_SHUNT1,
                                             &ina3221_base);

        ok_module_b = Power_Read_INA3221_Channel(INA3221_BUS3,
                                                 INA3221_SHUNT3,
                                                 &ina3221_module_b);

        /*
         * INA226
         * 0x40: Fan + Peltier
         * 0x45: Motor
         */
        ok_cooling = Power_INA226_Cooling_Read(&cooling_ina226);
        ok_motor = Power_INA226_Motor_Read(&motor_ina226);

        /*
         * 각 전력 mW 정리
         */
        base_power_mw = ok_base ? ina3221_base.power_mw : 0;
        module_b_power_mw = ok_module_b ? ina3221_module_b.power_mw : 0;
        cooling_power_mw = ok_cooling ? cooling_ina226.power_mw : 0;
        motor_power_mw = ok_motor ? motor_ina226.power_mw : 0;

#if 0 // 테스트용
        total_power_mw = module_b_power_mw + cooling_power_mw;
#endif
        /*
         * system_data는 기존 이름이 *_power_w 이므로 W 단위로 저장.
         * UART Telemetry에서 mW로 보낼 때는 *1000 해서 보내면 됨.
         */
        g_system_data.base_power_w = (float)base_power_mw / 1000.0f;
        g_system_data.module_b_power_w = (float)module_b_power_mw / 1000.0f;
        g_system_data.cooling_power_w = (float)cooling_power_mw / 1000.0f;
        g_system_data.motor_power_w = (float)motor_power_mw / 1000.0f;

        /*
         * total_power_w = Module B STM32 + Peltier + Fan
         * 과전력 경고/차단 판단까지 여기서 처리
         */

        Power_UpdateModuleBTotalAndProtection();

        Power_UpdatePeltierOpenFault(ok_cooling,
                                     cooling_ina226.current_ma);
#if 0 // 테스트용
        /*
         * 통합 테스트용 UART 출력.
         * GUI Telemetry와 충돌하면 나중에 #if 0 처리.
         */
        snprintf(msg,
                 sizeof(msg),
                 "PWR_FULL,"
                 "BASE:%ldmV,%ldmA,%ldmW,%s,"
                 "MODB:%ldmV,%ldmA,%ldmW,%s,"
                 "COOL:%ldmV,%ldmA,%ldmW,%s,"
                 "MOTOR:%ldmV,%ldmA,%ldmW,%s,"
                 "TOTAL:%ldmW\r\n",

                 ina3221_base.bus_mv,
                 ina3221_base.current_ma,
                 base_power_mw,
                 ok_base ? "OK" : "ERR",

                 ina3221_module_b.bus_mv,
                 ina3221_module_b.current_ma,
                 module_b_power_mw,
                 ok_module_b ? "OK" : "ERR",

                 cooling_ina226.bus_mv,
                 cooling_ina226.current_ma,
                 cooling_power_mw,
                 ok_cooling ? "OK" : "ERR",

                 motor_ina226.bus_mv,
                 motor_ina226.current_ma,
                 motor_power_mw,
                 ok_motor ? "OK" : "ERR",

                 total_power_mw);

        HAL_UART_Transmit(&huart3,
                          (uint8_t *)msg,
                          strlen(msg),
                          100);
#endif
#if 0	// 테스트용 - COOLING INA226만 출력
        snprintf(msg,
                 sizeof(msg),
                 "COOL:"
                 "%ldmV,%ldmA,%ldmW,%s\r\n",

                 cooling_ina226.bus_mv,
                 cooling_ina226.current_ma,
                 cooling_power_mw,
                 ok_cooling ? "OK" : "ERR");

        HAL_UART_Transmit(&huart3,
                          (uint8_t *)msg,
                          strlen(msg),
                          100);
#endif

        vTaskDelay(pdMS_TO_TICKS(POWER_MONITOR_PERIOD_MS));
    }
}

void PowerMonitor_RtosInit(void)
{
    BaseType_t ret;

    ret = xTaskCreate(PowerMonitor_Task,
                      "powerMonitor",
                      768,
                      NULL,
                      tskIDLE_PRIORITY + 1,
                      NULL);

    if (ret != pdPASS)
    {
        return;
    }
}

/* =========================================================
 * I2C1 Event IRQ Handler
 * 이름 변경 금지.
 * STM32 벡터 테이블에서 이 이름으로 호출됨.
 * ========================================================= */

void I2C1_EV_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;
    volatile uint32_t dummy;

    /*
     * EV5: START condition generated
     */
    if ((sr1 & I2C_SR1_SB) != 0U)
    {
        I2C1->DR = h_power_i2c.slave_addr;
        return;
    }

    /*
     * EV6: Address sent/matched
     */
    if ((sr1 & I2C_SR1_ADDR) != 0U)
    {
        if (h_power_i2c.state == POWER_I2C_READ_DATA)
        {
            h_power_i2c.state = POWER_I2C_READ_DATA_H;

            /*
             * 2-byte read를 위한 ACK enable.
             * 기존 팀원 코드 구조 유지.
             */
            I2C1->CR1 |= I2C_CR1_ACK;
        }

        dummy = I2C1->SR2;
        (void)dummy;

        return;
    }

    /*
     * EV8: TXE
     */
    if ((sr1 & I2C_SR1_TXE) != 0U)
    {
        switch (h_power_i2c.state)
        {
            case POWER_I2C_SEND_REG8_1:
                I2C1->DR = h_power_i2c.reg_addr;
                h_power_i2c.state = POWER_I2C_SEND_REG8_HIGH;
                break;

            case POWER_I2C_SEND_REG8_HIGH:
                I2C1->DR = h_power_i2c.data_h;
                h_power_i2c.state = POWER_I2C_SEND_REG8_LOW;
                break;

            case POWER_I2C_SEND_REG8_LOW:
                I2C1->DR = h_power_i2c.data_l;
                h_power_i2c.state = POWER_I2C_SEND_REG8_2;
                break;

            case POWER_I2C_SEND_REG8_2:
                I2C1->CR1 |= I2C_CR1_STOP;
                h_power_i2c.set_regs = 0U;
                h_power_i2c.state = POWER_I2C_IDLE;
                break;

            case POWER_I2C_SET_REG:
                I2C1->DR = h_power_i2c.reg_addr;
                h_power_i2c.state = POWER_I2C_SEND_REG8_2;
                break;

            default:
                break;
        }

        return;
    }

    /*
     * EV7: RXNE
     */
    if ((sr1 & I2C_SR1_RXNE) != 0U)
    {
        switch (h_power_i2c.state)
        {
            case POWER_I2C_READ_DATA_H:
                data_H = (uint8_t)I2C1->DR;

                /*
                 * NACK + STOP
                 */
                I2C1->CR1 &= ~I2C_CR1_ACK;
                I2C1->CR1 |= I2C_CR1_STOP;

                h_power_i2c.state = POWER_I2C_READ_DATA_L;
                break;

            case POWER_I2C_READ_DATA_L:
                data_L = (uint8_t)I2C1->DR;
                h_power_i2c.state = POWER_I2C_IDLE;
                break;

            default:
                break;
        }

        return;
    }
}

/* =========================================================
 * I2C1 Error IRQ Handler
 * 이름 변경 금지.
 * ========================================================= */

void I2C1_ER_IRQHandler(void)
{
    uint32_t sr1 = I2C1->SR1;

    g_i2c_last_error_sr1 = sr1;

    if ((sr1 & I2C_SR1_AF) != 0U)
    {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
    }

    if ((sr1 & I2C_SR1_BERR) != 0U)
    {
        I2C1->SR1 &= ~I2C_SR1_BERR;
    }

    if ((sr1 & I2C_SR1_ARLO) != 0U)
    {
        I2C1->SR1 &= ~I2C_SR1_ARLO;
    }

    if ((sr1 & I2C_SR1_OVR) != 0U)
    {
        I2C1->SR1 &= ~I2C_SR1_OVR;
    }

    h_power_i2c.state = POWER_I2C_IDLE;
    h_power_i2c.set_regs = 0U;

    g_i2c_bus_error_flag = 1U;
}
