#include "basket.h"
#include "MINIF.h"
#include "servo.h"
#include "tim.h"
#include <math.h>

#define TWO_PI 6.283185307f
volatile Basket_Info basket;
volatile uint8_t basket_request;
static volatile uint8_t laser_ready;
static uint8_t near_count;
static uint8_t far_count;
static uint8_t servo_rx[32];
static uint32_t object_tick;
static uint32_t object_stable_tick, object_min_mm, object_max_mm;

void Basket_ResetDetection(void)
{
    basket.object_samples=0U;
    basket.object_detected=0U;
    object_tick=HAL_GetTick();
    object_stable_tick=object_tick;
    object_min_mm=object_max_mm=0U;
}

static void Enter(Basket_State state)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (basket.state != BASKET_FAULT)
    {
        basket.stage_tick = HAL_GetTick();
        basket.state = state;
    }
    __set_PRIMASK(mask);
}

static void Fail(Basket_Error error)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (basket.state != BASKET_FAULT)
    {
        basket.error = error;
        basket.fault_distance_mm = basket.distance_mm;
    }
    basket.state = BASKET_FAULT;
    Motor_left.result = Motor_right.result = HAL_ERROR;
    Motor_left.state = Motor_right.state = RS01_FAULT;
    __set_PRIMASK(mask);
}

static uint8_t BothDone(void)
{
    return Motor_left.state == RS01_DONE && Motor_right.state == RS01_DONE;
}

/* Register both relative targets atomically, with a shared failure outcome. */
static HAL_StatusTypeDef MovePair(float left, float right, float speed)
{
    HAL_StatusTypeDef status;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (basket.state == BASKET_FAULT)
    { __set_PRIMASK(mask); return HAL_ERROR; }
    status = RS01_StartMove(&Motor_left, left, speed);
    if (status == HAL_OK) status = RS01_StartMove(&Motor_right, right, speed);
    if (status != HAL_OK) Fail(BASKET_ERROR_MOTOR);
    __set_PRIMASK(mask);
    return status;
}

static void StopPair(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (basket.state == BASKET_FAULT)
    { __set_PRIMASK(mask); return; }
    Motor_left.target_angle = Motor_left.total_angle;
    Motor_right.target_angle = Motor_right.total_angle;
    Motor_left.integral = Motor_right.integral = 0.0f;
    Motor_left.settle_count = Motor_right.settle_count = 0U;
    Motor_left.move_tick = Motor_right.move_tick = HAL_GetTick();
    Motor_left.state = Motor_right.state = RS01_STOPPING;
    Enter(BASKET_STOPPING);
    __set_PRIMASK(mask);
}

HAL_StatusTypeDef Basket_Init(void)
{
    HAL_StatusTypeDef status;
    basket.state = BASKET_IDLE;
    basket.error = BASKET_ERROR_NONE;
    basket.init_status = HAL_BUSY;
    basket_request = laser_ready = near_count = far_count = 0U;
    basket.fault_distance_mm = 0U;
    Basket_ResetDetection();
    RS01_Init();
    Motor_left.position_kp = Motor_right.position_kp = 1.5f;
    status = FDCAN2_ConfigureRxFilters();
    if (status == HAL_OK) status = HAL_FDCAN_Start(&hfdcan2);
    if (status == HAL_OK)
        status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
    if (status == HAL_OK) status = RS01_SpeedInit(&Motor_left, 2000U);
    if (status == HAL_OK) status = RS01_SpeedInit(&Motor_right, 2000U);
    if (status != HAL_OK) Fail(BASKET_ERROR_INIT);
    /* Even on partial init failure the timer retries disabling BOTH motors. */
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
    {
        Fail(BASKET_ERROR_INIT);
        (void)RS01_PositionTask(&Motor_left);
        (void)RS01_PositionTask(&Motor_right);
        status = HAL_ERROR;
    }
    basket.init_status = status;
    return status;
}

void Basket_ControlTick(void)
{
    if (Motor_left.state == RS01_FAULT || Motor_right.state == RS01_FAULT)
        Fail(BASKET_ERROR_MOTOR);
    if ((basket.state == BASKET_APPROACH || basket.state == BASKET_STOPPING) &&
        (!laser_ready || (uint32_t)(HAL_GetTick() - basket.laser_tick) >= BASKET_LASER_TIMEOUT_MS))
        Fail(BASKET_ERROR_LASER);
    (void)RS01_PositionTask(&Motor_left);
    if (Motor_left.state == RS01_FAULT) Fail(BASKET_ERROR_MOTOR);
    (void)RS01_PositionTask(&Motor_right);
    if (Motor_right.state == RS01_FAULT)
    {
        Fail(BASKET_ERROR_MOTOR);
        (void)RS01_PositionTask(&Motor_left);
    }
}

void Basket_Task(void)
{
    LRM_t sample;
    uint32_t sample_tick;
    uint32_t now = HAL_GetTick();
    uint8_t fresh = MINIF_TryGetData(&sample, &sample_tick);
    if (fresh)
    {
        if ((uint32_t)(sample_tick-object_tick)>=BASKET_LASER_TIMEOUT_MS)
            basket.object_samples=0U;
        object_tick=sample_tick;
        if (sample.distance>=BASKET_DETECT_MIN_MM && sample.distance<=BASKET_DETECT_MAX_MM &&
            sample.distance>=DISTANCE_MIN && sample.distance<=DISTANCE_MAX &&
            sample.confidence>=BASKET_MIN_CONFIDENCE &&
            (uint32_t)(now-sample_tick)<BASKET_LASER_TIMEOUT_MS)
        {
            if (basket.object_samples==0U)
            {
                object_stable_tick=sample_tick;
                object_min_mm=object_max_mm=sample.distance;
            }
            else
            {
                if (sample.distance<object_min_mm) object_min_mm=sample.distance;
                if (sample.distance>object_max_mm) object_max_mm=sample.distance;
                if (object_max_mm-object_min_mm>BASKET_DETECT_STABLE_MM)
                {
                    /* A moving target restarts the full settling interval. */
                    basket.object_samples=0U;
                    object_stable_tick=sample_tick;
                    object_min_mm=object_max_mm=sample.distance;
                }
            }
            if (basket.object_samples<BASKET_DETECT_SAMPLES) ++basket.object_samples;
        }
        else basket.object_samples=0U;
        basket.object_detected=basket.object_samples>=BASKET_DETECT_SAMPLES &&
            (uint32_t)(sample_tick-object_stable_tick)>=BASKET_DETECT_STABLE_MS;
        basket.distance_mm = sample.distance;
        basket.confidence = sample.confidence;
        if (sample.distance >= DISTANCE_MIN && sample.distance <= DISTANCE_MAX &&
            sample.confidence >= BASKET_MIN_CONFIDENCE)
        {
            basket.laser_tick = sample_tick;
            laser_ready = 1U;
        }
        else
        {
            laser_ready = 0U;
            near_count = 0U;
        }
    }
    if ((uint32_t)(now-object_tick)>=BASKET_LASER_TIMEOUT_MS)
    {
        basket.object_samples=0U;
        basket.object_detected=0U;
    }
    /* Drain UART4 replies so an old reply cannot leave reception stopped. */
    Servo_getData(servo_rx);

    if (basket_request == 2U)
    {
        basket_request = 0U;
        Fail(BASKET_ERROR_ABORT);
    }
    if (basket.state == BASKET_FAULT) return;

    if (basket_request == 1U)
    {
        /* Keep start pending without motion until an object is stably present. */
        if ((basket.state==BASKET_IDLE || basket.state==BASKET_DONE) && !basket.object_detected)
            return;
        basket_request = 0U;
        if ((basket.state == BASKET_IDLE || basket.state == BASKET_DONE) &&
            laser_ready && (uint32_t)(now - basket.laser_tick) < BASKET_LASER_TIMEOUT_MS)
        {
            basket.home_left = Motor_left.total_angle;
            basket.home_right = Motor_right.total_angle;
            near_count = 0U;
            if (basket.distance_mm < BASKET_TARGET_MM - BASKET_TOLERANCE_MM)
                Fail(BASKET_ERROR_TOO_CLOSE);
            else if (Servo_release() != HAL_OK) Fail(BASKET_ERROR_SERVO);
            else Enter(BASKET_OPENING);
        }
        else if (basket.state == BASKET_IDLE || basket.state == BASKET_DONE)
            basket.error = BASKET_ERROR_LASER; /* No movement; retry when ready. */
    }

    /* Servo transmission may advance the tick, and Enter() records that later
     * tick. Refresh now to avoid unsigned underflow against the new stage. */
    now = HAL_GetTick();
    if (basket.state >= BASKET_OPENING && basket.state <= BASKET_RELEASING &&
        (uint32_t)(now - basket.stage_tick) >= BASKET_STAGE_TIMEOUT_MS)
    {
        Fail(BASKET_ERROR_TIMEOUT);
        return;
    }
    switch (basket.state)
    {
    case BASKET_OPENING:
        if ((uint32_t)(now - basket.stage_tick) < BASKET_SERVO_WAIT_MS) break;
        if (!laser_ready || (uint32_t)(now - basket.laser_tick) >= BASKET_LASER_TIMEOUT_MS)
        { Fail(BASKET_ERROR_LASER); break; }
        basket.error = BASKET_ERROR_NONE;
        Enter(BASKET_APPROACH);
        if (MovePair(BASKET_MAX_FORWARD_TURNS, BASKET_MAX_FORWARD_TURNS,
                     BASKET_MIN_SPEED) != HAL_OK) break;
        /* Fall through to apply the latest distance before the next tick. */
    case BASKET_APPROACH:
        if (!laser_ready) { Fail(BASKET_ERROR_LASER); break; }
        if (basket.distance_mm < BASKET_TARGET_MM - BASKET_TOLERANCE_MM)
        { Fail(BASKET_ERROR_TOO_CLOSE); break; }
        if (basket.distance_mm <= BASKET_TARGET_MM + BASKET_TOLERANCE_MM)
        { near_count = far_count = 0U; StopPair(); break; }
        if (Motor_left.state != RS01_MOVING || Motor_right.state != RS01_MOVING ||
            Motor_left.total_angle - basket.home_left >= BASKET_MAX_FORWARD_TURNS * TWO_PI - RS01_POSITION_TOLERANCE ||
            Motor_right.total_angle - basket.home_right >= BASKET_MAX_FORWARD_TURNS * TWO_PI - RS01_POSITION_TOLERANCE)
        { Fail(BASKET_ERROR_TRAVEL); break; }
        {
            float speed = (basket.distance_mm - BASKET_TARGET_MM) * BASKET_DISTANCE_GAIN;
            if (speed > BASKET_MAX_SPEED) speed = BASKET_MAX_SPEED;
            if (speed < BASKET_MIN_SPEED) speed = BASKET_MIN_SPEED;
            /* Apply last: a high minimum must not override near-target braking. */
            if (basket.distance_mm <= BASKET_SLOW_DISTANCE_MM && speed > BASKET_APPROACH_SPEED)
                speed = BASKET_APPROACH_SPEED;
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            Motor_left.max_speed = Motor_right.max_speed = speed;
            __set_PRIMASK(mask);
        }
        break;
    case BASKET_STOPPING:
        if (!laser_ready) { Fail(BASKET_ERROR_LASER); break; }
        if (fresh)
        {
            if (basket.distance_mm < BASKET_TARGET_MM - BASKET_TOLERANCE_MM)
            { Fail(BASKET_ERROR_TOO_CLOSE); break; }
            if (basket.distance_mm <= BASKET_TARGET_MM + BASKET_TOLERANCE_MM)
            {
                far_count = 0U;
                if (near_count < 3U) ++near_count;
            }
            else
            {
                near_count = 0U;
                if (far_count < 3U) ++far_count;
            }
        }
        if (BothDone() && far_count >= 3U)
        {
            float left = BASKET_MAX_FORWARD_TURNS -
                         (Motor_left.total_angle - basket.home_left) / TWO_PI;
            float right = BASKET_MAX_FORWARD_TURNS -
                          (Motor_right.total_angle - basket.home_right) / TWO_PI;
            if (left * TWO_PI <= RS01_POSITION_TOLERANCE ||
                right * TWO_PI <= RS01_POSITION_TOLERANCE)
            { Fail(BASKET_ERROR_TRAVEL); break; }
            /* Retain the original stage deadline across distance corrections. */
            if (MovePair(left, right, BASKET_MIN_SPEED) == HAL_OK)
            {
                uint32_t mask = __get_PRIMASK();
                __disable_irq();
                if (basket.state != BASKET_FAULT) basket.state = BASKET_APPROACH;
                __set_PRIMASK(mask);
            }
            break;
        }
        if (BothDone() && near_count >= 3U)
        {
            if (Servo_catch() != HAL_OK) Fail(BASKET_ERROR_SERVO);
            else Enter(BASKET_GRIPPING);
        }
        break;
    case BASKET_GRIPPING:
        if ((uint32_t)(now - basket.stage_tick) >= BASKET_SERVO_WAIT_MS)
        {
            float left = (basket.home_left - Motor_left.total_angle) / TWO_PI;
            float right = (basket.home_right - Motor_right.total_angle) / TWO_PI;
            if (MovePair(left, right, BASKET_RETURN_SPEED) == HAL_OK)
                Enter(BASKET_RETURNING);
        }
        break;
    case BASKET_RETURNING:
        if (BothDone())
        {
            if (Servo_release() != HAL_OK) Fail(BASKET_ERROR_SERVO);
            else Enter(BASKET_RELEASING);
        }
        break;
    case BASKET_RELEASING:
        if ((uint32_t)(now - basket.stage_tick) >= BASKET_SERVO_WAIT_MS)
            Enter(BASKET_DONE);
        break;
    default: break;
    }
}
