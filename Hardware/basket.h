#ifndef BASKET_H
#define BASKET_H

#include "RS01.h"

#define BASKET_SLOW_DISTANCE_MM   80U  // 提前低速接近区的起始距离，mm；按实测制动距离增大
#define BASKET_APPROACH_SPEED     0.15f // 低速接近区的最终速度上限，rad/s；优先于最低速度设置

/* 抓篮参数：距离为激光到篮子的读数；电机圈数不直接等于直线行程。
 * 速度单位均为电机轴 rad/s，不是圈/s；1圈 = 2π rad。
 */
#define BASKET_TARGET_MM          30U      // 抓取目标距离，mm；含激光与夹爪安装位置的偏移
#define BASKET_TOLERANCE_MM       3U       // 目标距离允许偏差±mm；当前有效抓取区间为27～33 mm
#define BASKET_MIN_CONFIDENCE     50U      // 激光最低有效置信度，0～100；小于此值按无效测距处理
#define BASKET_LASER_TIMEOUT_MS   300U     // 有效测距允许的最大间隔，ms；前进/停稳阶段超时则故障
#define BASKET_MAX_FORWARD_TURNS  2.0f     // 从本次出发位置算起，每侧电机最多前进圈数；未到篮子就耗尽行程则故障
#define BASKET_MAX_SPEED          3.0f     // 接近篮子时的电机速度上限，rad/s；距离较近时会按比例降速
#define BASKET_RETURN_SPEED       3.0f     // 返回本次出发位置的电机速度上限，rad/s；由位置环计算实际指令
#define BASKET_MIN_SPEED          0.5f    // 前进时速度上限的最低设定值，rad/s；不是保证实际速度不低于此值
#define BASKET_DISTANCE_GAIN      0.1f   // 距离误差转速度上限的比例，(rad/s)/mm；速度上限=(距离-目标)*此值，再限幅
#define BASKET_SERVO_WAIT_MS      700U     // 张开、抓紧、释放指令后的等待时间，ms；仅计时，不代表已检测到夹紧
#define BASKET_STAGE_TIMEOUT_MS   30000U   // 单阶段最长允许时间，ms；超时进入故障，不是整套流程的总时间

typedef enum {
    BASKET_IDLE, BASKET_OPENING, BASKET_APPROACH, BASKET_STOPPING,
    BASKET_GRIPPING, BASKET_RETURNING, BASKET_RELEASING,
    BASKET_DONE, BASKET_FAULT
} Basket_State;

typedef enum {
    BASKET_ERROR_NONE, BASKET_ERROR_INIT, BASKET_ERROR_MOTOR,
    BASKET_ERROR_LASER, BASKET_ERROR_TOO_CLOSE, BASKET_ERROR_TRAVEL,
    BASKET_ERROR_TIMEOUT, BASKET_ERROR_SERVO, BASKET_ERROR_ABORT
} Basket_Error;

typedef struct {
    Basket_State state;
    Basket_Error error;
    uint32_t distance_mm;
    uint32_t fault_distance_mm;
    uint8_t confidence;
    uint32_t laser_tick;
    uint32_t stage_tick;
    float home_left;
    float home_right;
    HAL_StatusTypeDef init_status;
} Basket_Info;

extern volatile Basket_Info basket;
/* Set request=1 once to start, request=2 to abort. Faults require restart. */
extern volatile uint8_t basket_request;
HAL_StatusTypeDef Basket_Init(void);
void Basket_Task(void);              /* Main loop, at least every few ms. */
void Basket_ControlTick(void);       /* TIM6, every 10 ms, no UART waits. */

#endif
