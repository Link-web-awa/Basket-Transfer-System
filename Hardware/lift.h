#ifndef LIFT_H
#define LIFT_H
#include "RS01.h"

/* Lift configuration: speed rad/s, acceleration rad/s^2, current A. */
#define LIFT_SPEED                  2.0f
#define LIFT_CURRENT                4.0f
#define LIFT_ACCELERATION           0.2f
#define LIFT_RISE_TURNS             5.0f
#define LIFT_JOG_TURNS              0.02f
#define LIFT_UP_DIRECTION           (-1)
#define LIFT_BOOT_TURNS             0.0f
#define LIFT_BOOT_SPEED             LIFT_SPEED
#define LIFT_BOOT_TIMEOUT_MS        LIFT_TIMEOUT_MS
#define LIFT_TIMEOUT_MS             240000U
#define LIFT_JOG_TIMEOUT_MS         10000U
#define LIFT_INIT_TIMEOUT_MS        2000U
#define LIFT_WRITE_TIMEOUT_MS       100U
#define LIFT_WRITE_RETRY_MS         1U
#define LIFT_INIT_COMMAND_DELAY_MS  10U
#define LIFT_INIT_QUERY_DELAY_MS    20U
#define LIFT_ENABLE_DELAY_MS        20U
#define LIFT_ENABLE_GRACE_MS        300U
#define LIFT_FEEDBACK_TIMEOUT_MS    300U
#define LIFT_CURRENT_TIMEOUT_MS     300U
#define LIFT_POSITION_QUERY_MS      50U
#define LIFT_CURRENT_QUERY_MS       100U
#define LIFT_CURRENT_WARN_RATIO     0.9f
#define LIFT_POSITION_TOLERANCE_RAD 0.01f
#define LIFT_SPEED_TOLERANCE_RAD_S   RS01_SPEED_TOLERANCE
#define LIFT_SETTLE_SAMPLES         10U
#define LIFT_RAD_PER_TURN           6.283185307f
typedef enum { LIFT_UNREADY, LIFT_READY, LIFT_RISING, LIFT_HOLDING,
               LIFT_DESCENDING, LIFT_HOME, LIFT_FAULT, LIFT_PREPOSITION } Lift_State;
typedef struct {
    Lift_State state;
    HAL_StatusTypeDef result;
    float home_rad, target_rad, position_rad;
    uint32_t motion_tick;
    uint8_t error; /* 1 CAN, 2 feedback, 3 motor, 4 timeout */
    float current_a;       /* iqf, signed amperes */
    float peak_current_a;  /* sampled absolute peak since latest move */
    uint8_t current_valid; /* recent response within 300 ms */
    uint8_t near_current_limit; /* abs(iqf) >= 90% of configured limit; diagnostic only */
} Lift_Info;
extern volatile Lift_Info lift;
/* 1 rise five turns; 2 return home; 3 rise 0.02-turn direction check;
 * 4 DISABLE (load may fall). Boot preposition runs automatically first. */
extern volatile uint8_t lift_request;
extern volatile int8_t lift_up_direction; /* +1/-1 after unloaded direction check */
HAL_StatusTypeDef Lift_Init(void);
void Lift_Task(void);
#endif
