#include "lift.h"
#include "lift_can.h"
#include <math.h>

volatile Lift_Info lift;
volatile uint8_t lift_request;
volatile int8_t lift_up_direction = LIFT_UP_DIRECTION;
static uint32_t query_tick, enable_tick;
static uint32_t iq_query_tick;
static uint8_t enabled, settled;
static uint8_t boot_pending;
static uint32_t motion_timeout = LIFT_TIMEOUT_MS;

static HAL_StatusTypeDef Write(uint16_t index, float value)
{
    HAL_StatusTypeDef status;
    uint32_t start = HAL_GetTick();
    lift.tx_command=index;
    do {
        status = RS01_WriteFloat(RS01_DOWN_MOTOR_ID, index, value);
        if (status != HAL_BUSY) { lift.tx_status=status; return status; }
        HAL_Delay(LIFT_WRITE_RETRY_MS);
    } while ((uint32_t)(HAL_GetTick()-start) < LIFT_WRITE_TIMEOUT_MS);
    lift.tx_status=HAL_TIMEOUT;
    return HAL_TIMEOUT;
}

static HAL_StatusTypeDef Enable(void)
{
    HAL_StatusTypeDef status;
    uint32_t start=HAL_GetTick();
    lift.tx_command=0x0003U;
    do {
        status=RS01_EnableMotor(RS01_DOWN_MOTOR_ID);
        if(status!=HAL_BUSY) { lift.tx_status=status; return status; }
        HAL_Delay(LIFT_WRITE_RETRY_MS);
    } while((uint32_t)(HAL_GetTick()-start)<LIFT_WRITE_TIMEOUT_MS);
    lift.tx_status=HAL_TIMEOUT;
    return HAL_TIMEOUT;
}

static void Fault(uint8_t error)
{
    if (lift.state != LIFT_FAULT) lift.error = error;
    lift.state = LIFT_FAULT;
    lift.result = HAL_ERROR;
    /* PP zero velocity stops the trajectory without explicitly disabling torque.
     * Best effort only: communication loss cannot guarantee load retention. */
    (void)RS01_WriteFloat(RS01_DOWN_MOTOR_ID, 0x7024U, 0.0f);
}

void Lift_StopHold(void)
{
    Fault(5U);
}

HAL_StatusTypeDef Lift_Init(void)
{
    uint32_t start;
    HAL_StatusTypeDef status;
    lift.state = LIFT_UNREADY;
    lift.result = HAL_BUSY;
    lift_request = enabled = settled = 0U;
    boot_pending = 0U;
    lift.current_valid = lift.near_current_limit = 0U;
    lift.peak_current_a = 0.0f;
    down_iq_valid = 0U;
    iq_query_tick = HAL_GetTick();
    Motor_Down.limit_cur = LIFT_CURRENT;
    lift.step=1U;
    status=LiftCan_Start();
    lift.tx_status=status;
    if(status!=HAL_OK) { Fault(1U); return status; }
    HAL_Delay(LIFT_CAN_POWER_WAIT_MS);
    lift.step=2U;
    status = RS01_DisableMotor(RS01_DOWN_MOTOR_ID);
    if (status == HAL_OK) { HAL_Delay(LIFT_INIT_COMMAND_DELAY_MS); status = RS01_WriteU8(RS01_DOWN_MOTOR_ID,0x7005U,1U); }
    if (status == HAL_OK) { HAL_Delay(LIFT_INIT_COMMAND_DELAY_MS); status = Write(0x7018U,LIFT_CURRENT); }
    if (status == HAL_OK) { HAL_Delay(LIFT_INIT_COMMAND_DELAY_MS); status = Write(0x7024U,LIFT_SPEED); }
    if (status == HAL_OK) { HAL_Delay(LIFT_INIT_COMMAND_DELAY_MS); status = Write(0x7025U,LIFT_ACCELERATION); }
    if (status == HAL_OK) { HAL_Delay(LIFT_INIT_COMMAND_DELAY_MS); status = RS01_SetReport(RS01_DOWN_MOTOR_ID); }
    if (status != HAL_OK) { lift.tx_status=status; Fault(1U); return status; }
    start = HAL_GetTick();
    lift.step=3U;
    down_run_mode = -1;
    do {
        (void)RS01_RequestParameter(RS01_DOWN_MOTOR_ID,0x7005U);
        HAL_Delay(LIFT_INIT_QUERY_DELAY_MS);
    } while (down_run_mode != 1 && (uint32_t)(HAL_GetTick()-start)<LIFT_INIT_TIMEOUT_MS);
    if (down_run_mode != 1) { lift.tx_status=HAL_TIMEOUT; Fault(1U); return HAL_TIMEOUT; }
    start = HAL_GetTick();
    lift.step=4U;
    down_position_valid = 0U;
    do {
        (void)RS01_RequestParameter(RS01_DOWN_MOTOR_ID,0x7019U);
        HAL_Delay(LIFT_INIT_QUERY_DELAY_MS);
    } while (!down_position_valid && (uint32_t)(HAL_GetTick()-start)<LIFT_INIT_TIMEOUT_MS);
    if (!down_position_valid) { Fault(2U); return HAL_TIMEOUT; }
    lift.home_rad = lift.target_rad = down_mech_position;
    lift.step=5U;
    status = Write(0x7016U,lift.home_rad);
    if (status != HAL_OK) { Fault(1U); return status; }
    /* Main loop starts preposition once; commands wait until it completes. */
    boot_pending = 1U;
    lift.state = LIFT_PREPOSITION;
    Motor_Down.state = RS01_IDLE;
    lift.result = HAL_BUSY;
    query_tick = HAL_GetTick();
    return HAL_OK;
}

void Lift_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t request = lift_request;
    uint8_t is_boot = 0U;
    lift_request = 0U;
    if ((uint32_t)(now-query_tick)>=LIFT_POSITION_QUERY_MS)
    {
        (void)RS01_RequestParameter(RS01_DOWN_MOTOR_ID,0x7019U);
        query_tick = now;
    }
    lift.position_rad = down_mech_position;
    /* Separate requests so parameter replies remain independently timestamped. */
    if ((uint32_t)(now-iq_query_tick)>=LIFT_CURRENT_QUERY_MS)
    {
        if (RS01_RequestParameter(RS01_DOWN_MOTOR_ID,0x701AU)==HAL_OK) iq_query_tick=now;
    }
    {
        uint32_t mask=__get_PRIMASK();
        float current;
        uint32_t stamp;
        uint8_t valid;
        __disable_irq();
        current=down_iqf; stamp=down_iq_tick; valid=down_iq_valid;
        __set_PRIMASK(mask);
        lift.current_a=current;
        lift.current_valid=valid && (uint32_t)(now-stamp)<LIFT_CURRENT_TIMEOUT_MS;
        lift.near_current_limit=lift.current_valid && fabsf(current)>=LIFT_CURRENT_WARN_RATIO*LIFT_CURRENT;
        if (enabled && lift.current_valid && fabsf(current)>lift.peak_current_a)
            lift.peak_current_a=fabsf(current);
    }
    if (request == 4U)
    {
        (void)RS01_DisableMotor(RS01_DOWN_MOTOR_ID);
        enabled = 0U;
        Fault(3U);
        return;
    }
    if (lift.state == LIFT_FAULT || lift.state == LIFT_UNREADY) return;
    if (boot_pending)
    {
        boot_pending = 0U;
        is_boot = 1U;
        request = 5U; /* Internal only, not a public command. */
    }
    if (enabled && (uint32_t)(now-enable_tick)>LIFT_ENABLE_GRACE_MS &&
        (!Motor_Down.feedback_valid || (uint32_t)(now-Motor_Down.feedback_tick)>LIFT_FEEDBACK_TIMEOUT_MS ||
         !down_position_valid || (uint32_t)(now-down_position_tick)>LIFT_FEEDBACK_TIMEOUT_MS))
    { Fault(2U); return; }
    if (enabled && (Motor_Down.fault || Motor_Down.pattern != 2U) &&
        (uint32_t)(now-enable_tick)>LIFT_ENABLE_GRACE_MS) { Fault(3U); return; }
    if (is_boot || (request && lift.state != LIFT_RISING &&
        lift.state != LIFT_DESCENDING && lift.state != LIFT_PREPOSITION))
    {
        float target;
        if (!down_position_valid || (uint32_t)(now-down_position_tick)>LIFT_FEEDBACK_TIMEOUT_MS ||
            Motor_Down.fault || (lift_up_direction!=1 && lift_up_direction!=-1))
        { Fault(2U); return; }
        if (is_boot)
            target=down_mech_position + lift_up_direction*LIFT_BOOT_TURNS*LIFT_RAD_PER_TURN;
        else if (request==1U && (lift.state==LIFT_READY || lift.state==LIFT_HOME))
        {
            lift.home_rad = down_mech_position;
            target = lift.home_rad + lift_up_direction * LIFT_RISE_TURNS * LIFT_RAD_PER_TURN;
        }
        else if (request==2U && lift.state==LIFT_HOLDING) target=lift.home_rad;
        else if (request==3U && lift.state==LIFT_READY)
            target=down_mech_position + lift_up_direction*LIFT_JOG_TURNS*LIFT_RAD_PER_TURN;
        else return;
        if (!enabled)
        {
            lift.step=5U;
            /* Preload the CURRENT position only. Enable may reset loc_ref. */
            if (Write(0x7016U,down_mech_position)!=HAL_OK) { Fault(1U); return; }
            uint32_t mask=__get_PRIMASK();
            __disable_irq();
            Motor_Down.angle_ready=0U;
            Motor_Down.feedback_valid=0U;
            Motor_Down.state=RS01_IDLE;
            __set_PRIMASK(mask);
            lift.step=6U;
            if (Enable()!=HAL_OK) { Fault(1U); return; }
            enabled=1U; enable_tick=HAL_GetTick();
            HAL_Delay(LIFT_ENABLE_DELAY_MS);
        }
        /* PP protocol: enable, set limits, then publish the motion target. */
        lift.step=7U;
        if (Write(0x7018U,LIFT_CURRENT)!=HAL_OK ||
            Write(0x7024U,is_boot ? LIFT_BOOT_SPEED : LIFT_SPEED)!=HAL_OK ||
            Write(0x7025U,LIFT_ACCELERATION)!=HAL_OK || Write(0x7016U,target)!=HAL_OK)
        { Fault(1U); return; }
        motion_timeout = is_boot ? LIFT_BOOT_TIMEOUT_MS :
                         (request==3U ? LIFT_JOG_TIMEOUT_MS : LIFT_TIMEOUT_MS);
        lift.target_rad=target;
        lift.peak_current_a=0.0f;
        down_iq_valid=0U;
        lift.motion_tick=HAL_GetTick();
        lift.state=is_boot ? LIFT_PREPOSITION :
                   (request==2U ? LIFT_DESCENDING : LIFT_RISING);
        lift.result=HAL_BUSY;
        settled=0U;
    }
    now=HAL_GetTick();
    if (lift.state==LIFT_RISING || lift.state==LIFT_DESCENDING || lift.state==LIFT_PREPOSITION)
    {
        if ((uint32_t)(now-lift.motion_tick)>motion_timeout) { Fault(4U); return; }
        /* Count only new position replies, not repeated main-loop samples. */
        static uint32_t checked_tick;
        if (down_position_tick!=checked_tick)
        {
            checked_tick=down_position_tick;
            /* Match the validated RS01 low-speed tolerance. Position remains
             * strict: 0.01 rad (~0.57 degrees), for ten fresh position replies. */
            if (fabsf(down_mech_position-lift.target_rad)<LIFT_POSITION_TOLERANCE_RAD &&
                fabsf(Motor_Down.speed)<LIFT_SPEED_TOLERANCE_RAD_S)
            { if (settled<LIFT_SETTLE_SAMPLES) ++settled; }
            else settled=0U;
            Motor_Down.settle_count=settled;
            if (settled>=LIFT_SETTLE_SAMPLES)
            {
                if (lift.state==LIFT_PREPOSITION)
                {
                    lift.home_rad=down_mech_position;
                    lift.state=LIFT_READY;
                }
                else lift.state=lift.state==LIFT_DESCENDING ? LIFT_HOME : LIFT_HOLDING;
                lift.result=HAL_OK;
            }
        }
    }
}
