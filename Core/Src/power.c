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

/* =========================================================
 * External Handle
 * ========================================================= */
extern UART_HandleTypeDef huart3;

/* =========================================================
 * Sensor Use Select
 * ========================================================= */
#define POWER_USE_INA3221         (0)
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
 * 현재 INA3221 보드는 SDA-GND 쇼트 의심으로 비활성화.
 */
#define INA3221_ADDR              (0x41)
#define INA226_ADDR               (0x40)

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
#define POWER_SHUNT_RESISTOR_MOHM (100U)

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

    if (sensor == 3221)
    {
        *addr = (uint8_t)((INA3221_ADDR << 1) | 0U);
        return 1U;
    }
    else if (sensor == 226)
    {
        *addr = (uint8_t)((INA226_ADDR << 1) | 0U);
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

    if (sensor == 3221)
    {
        *addr = (uint8_t)((INA3221_ADDR << 1) | 1U);
        return 1U;
    }
    else if (sensor == 226)
    {
        *addr = (uint8_t)((INA226_ADDR << 1) | 1U);
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
     * 현재 INA3221은 보드 불량 의심으로 비활성화.
     */
    (void)Power_WriteReg16(3221, INA3221_CONFIG, 0x7327U);
#else
    return;
#endif
}

void Power_INA226_Init(void)
{
#if POWER_USE_INA226
    /*
     * INA226 단독 HAL 테스트에서 확인한 설정값 사용.
     * Current/Power register는 Calibration 기반이라 사용하지 않고,
     * BUS/SHUNT raw를 직접 읽어서 계산함.
     */
    (void)Power_WriteReg16(226, INA226_CONFIG, INA226_CONFIG_VALUE);
#else
    return;
#endif
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

    if (Power_ReadReg16(3221, bus_reg, &bus_raw) == 0U)
    {
        return 0U;
    }

    if (Power_ReadReg16(3221, shunt_reg, &shunt_raw) == 0U)
    {
        return 0U;
    }

    /*
     * INA3221:
     * Bus LSB   = 8mV
     * Shunt LSB = 40uV
     */
    bus_valid = (int16_t)(bus_raw >> 3);
    shunt_valid = (int16_t)(shunt_raw >> 3);

    out->bus_mv = (int32_t)bus_valid * 8L;
    out->shunt_uv = (int32_t)shunt_valid * 40L;
    out->current_ma = out->shunt_uv / (int32_t)POWER_SHUNT_RESISTOR_MOHM;

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

uint8_t Power_INA226_Read(Power_INA226Value_t *out)
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

    if (Power_ReadReg16(226, INA226_BUS, &bus_raw) == 0U)
    {
        return 0U;
    }

    if (Power_ReadReg16(226, INA226_SHUNT, &shunt_raw) == 0U)
    {
        return 0U;
    }

    /*
     * INA226:
     * Bus Voltage LSB   = 1.25mV
     * Shunt Voltage LSB = 2.5uV
     *
     * bus_mv   = raw * 1.25mV
     * shunt_uv = raw * 2.5uV
     */
    bus_mv = ((int32_t)bus_raw * 125L) / 100L;
    shunt_uv = ((int32_t)shunt_raw * 25L) / 10L;

    /*
     * current[mA] = shunt[uV] / Rshunt[mΩ]
     *
     * R100 기준:
     * 700uV / 100mΩ = 7mA
     */
    current_ma = shunt_uv / (int32_t)POWER_SHUNT_RESISTOR_MOHM;

    if (current_ma < 0)
    {
        current_ma = -current_ma;
    }

    /*
     * power[mW] = bus[mV] * current[mA] / 1000
     */
    power_mw = (bus_mv * current_ma) / 1000L;

    power_mw = Power_Abs32(power_mw);

    out->bus_mv = bus_mv;
    out->shunt_uv = shunt_uv;
    out->current_ma = current_ma;
    out->power_mw = power_mw;

    return 1U;
}

/* =========================================================
 * RTOS Task
 * ========================================================= */

void PowerMonitor_Task(void *argument)
{
    Power_INA226Value_t ina226;
    char msg[192];

    (void)argument;

    Power_I2C1_Init();
    Power_INA226_Init();

    Power_Print("\r\n[INA226 RTOS TEST START]\r\n");

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
            Power_INA226_Init();

            g_i2c_bus_error_flag = 0U;

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        memset(&ina226, 0, sizeof(ina226));

        uint8_t ok_226 = Power_INA226_Read(&ina226);

        if (ok_226 != 0U)
        {
            /*
             * 프로젝트 내부 저장용.
             * 현재 system_data에 cooling_power_w는 있다고 보고 갱신.
             */

        	/* 실제 저장 단위는 mW. 변수명은 기존 system_data 구조 유지 때문에 cooling_power_w 사용 */

        	g_system_data.cooling_power_w = (float)ina226.power_mw / 1000.0f;
#if 1
        	// ina3321
        	g_system_data.base_power_w = 0.0f;
        	g_system_data.module_b_power_w = 0.0f;
        	g_system_data.motor_power_w = 0.0f;
#endif

            /*
             * system_data에 아래 필드가 있으면 나중에 활성화.
             *
             * g_system_data.cooling_voltage_v = (float)ina226.bus_mv / 1000.0f;
             * g_system_data.cooling_current_a = (float)ina226.current_ma / 1000.0f;
             */
        }

#if 0
        snprintf(msg,
                 sizeof(msg),
                 "INA226,BUS:%ldmV,SHUNT:%lduV,CURRENT:%ldmA,POWER:%ldmW,%s\r\n",
                 ina226.bus_mv,
                 ina226.shunt_uv,
                 ina226.current_ma,
                 ina226.power_mw,
                 ok_226 ? "OK" : "ERR");

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
