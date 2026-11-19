#ifndef COMBINED_H
#define COMBINED_H
#include "basket.h"
#include "lift.h"

#define COMBINED_PUSH_TURNS       1.5f   // 原位推出圈数，左右统一正方向
#define COMBINED_PUSH_SPEED       1.0f   // 推出和返回速度上限，rad/s
#define COMBINED_PUSH_TIMEOUT_MS  30000U // 每段推出/返回超时，ms
#define COMBINED_RAD_PER_TURN     6.283185307f
#define COMBINED_LASER_WAIT_MS    10000U // 升降到位后最多等待有效测距的时间，ms；期间保持升降位置

typedef enum { COMBINED_WAIT_HOME, COMBINED_PUSHING, COMBINED_RETRACTING,
               COMBINED_LIFTING, COMBINED_GRABBING, COMBINED_DONE,
               COMBINED_FAULT, COMBINED_LOWERING } Combined_State;
typedef struct {
    Combined_State state;
    uint8_t error; /* 1 init, 2 motor, 3 lift, 4 basket, 5 timeout, 6 abort */
    uint32_t stage_tick;
    float left_home, right_home;
    HAL_StatusTypeDef init_status;
    Combined_State fault_stage;       // 首次联调故障发生的阶段
    Basket_Error fault_basket_error;  // 中止子任务前保存的原始抓篮错误
    uint32_t fault_distance_mm;
    uint8_t fault_confidence;
} Combined_Info;
extern volatile Combined_Info combined;
/* 1: run one sequence from READY/HOME, repeatable after COMBINED_DONE;
 * 2: abort, fault locked until restart. Running requests are ignored.
 * In this mode use combined_request only, not basket_request/lift_request. */
extern volatile uint8_t combined_request;
HAL_StatusTypeDef Combined_Init(void);
void Combined_Task(void);
void Combined_ControlTick(void);
#endif
