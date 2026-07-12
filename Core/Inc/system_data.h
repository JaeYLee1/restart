#ifndef INC_SYSTEM_DATA_H_
#define INC_SYSTEM_DATA_H_

#include <stdint.h>

/* GUI FSM 상태 */
#define FSM_IDLE            0U
#define FSM_DETECTED        1U
#define FSM_AUTHENTICATING  2U
#define FSM_ACTIVE          3U
#define FSM_FAULT           4U

/* Module A / B 공용 상태 */
typedef struct
{
    /* Module A */
    float pressure_value;

    /* Module B */
    float target_temp_c;
    float current_temp_c;
    uint8_t peltier_pwm;
    uint8_t fan_pwm;

    /* Base Motor */
    uint8_t motor_running;
    uint8_t motor_speed_level;   /* 0: STOP, 1: SLOW, 2: MID, 3: FAST */

    float target_speed_rpm;
    float current_speed_rpm;
    float motor_pwm_duty;

    /* GUI / Dock 상태 */
    uint8_t detect_state;        /* 0: NOT_DETECTED, 1: DETECTED */
    uint8_t relay_state;         /* 0: OFF, 1: ON, 현재는 가상 릴레이 */
    uint8_t fsm_state;           /* FSM_IDLE ~ FSM_FAULT */

    /* Power Monitor */
    float base_power_w;
    float module_b_power_w;
    float cooling_power_w;
    float motor_power_w;
    float total_power_w;

} SystemSharedData_t;

extern SystemSharedData_t g_system_data;

#endif /* INC_SYSTEM_DATA_H_ */
