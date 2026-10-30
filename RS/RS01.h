#ifndef RS01_H
#define RS01_H
#include "Robstride.h"
#include "fdcan.h"
#ifdef __cplusplus
extern "C" {
#endif
#define RS01_FEEDBACK_TIMEOUT_MS 100U
#define RS01_CONTROL_PERIOD_MS 10U
#define RS01_INIT_READY 9U
typedef enum {
    RS01_IDLE, RS01_MOVING, RS01_STOPPING, RS01_DONE, RS01_FAULT
} RS01_State;
typedef struct {
    uint8_t motor_id;
    int8_t direction;
    volatile float angle, speed, torque, temp, last_angle, total_angle, turns;
    volatile uint8_t angle_ready, feedback_valid, fault, pattern;
    volatile uint32_t feedback_tick, feedback_count;
    volatile int run_mode;
    uint8_t init_step, init_started, moving;
    uint32_t init_tick, init_feedback_count, control_tick, move_tick;
    uint32_t settle_tick, stop_tick, stop_cmd_tick;
    uint8_t settling, zero_sent;
    float limit_cur, acceleration;
    float target_angle, max_speed, position_kp, position_ki, position_kd;
    float integral, integral_limit, position_tolerance, speed_tolerance;
    uint32_t move_timeout_ms;
    RS01_State state;
    HAL_StatusTypeDef result;
} Motor_t;
extern Motor_t Motor_left, Motor_right;
/* Call after CAN start; sets configuration only, never starts a move. */
void RS01_Init(void);
HAL_StatusTypeDef RS01_WriteFloat(uint8_t id, uint16_t index, float value);
HAL_StatusTypeDef RS01_WriteU8(uint8_t id, uint16_t index, uint8_t value);
HAL_StatusTypeDef RS01_EnableMotor(uint8_t id);
HAL_StatusTypeDef RS01_SetSpeedTarget(uint8_t id, float speed);
HAL_StatusTypeDef RS01_SpeedInitTask(Motor_t *motor);
HAL_StatusTypeDef RS01_SpeedInit(Motor_t *motor, uint32_t timeout_ms);
HAL_StatusTypeDef RS01_StartMove(Motor_t *motor, float turns, float max_speed);
HAL_StatusTypeDef RS01_PositionTask(Motor_t *motor);
void RS01_Task(void);
void update_motorstate(Motor_t *motor, const uint8_t *data);
#ifdef __cplusplus
}
#endif
#endif
