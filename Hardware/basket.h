#ifndef BASKET_H
#define BASKET_H

#include "RS01.h"

/* Initial commissioning limits, NOT a conversion between turns and mm. */
#define BASKET_TARGET_MM          30U
#define BASKET_TOLERANCE_MM       3U
#define BASKET_MIN_CONFIDENCE     50U
#define BASKET_LASER_TIMEOUT_MS   300U
#define BASKET_MAX_FORWARD_TURNS  2.0f
#define BASKET_MAX_SPEED          1.5f
#define BASKET_RETURN_SPEED       1.5f
#define BASKET_MIN_SPEED          0.05f
#define BASKET_DISTANCE_GAIN      0.005f
#define BASKET_SERVO_WAIT_MS      700U
#define BASKET_STAGE_TIMEOUT_MS   30000U

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
