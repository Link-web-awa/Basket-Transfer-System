#include "combined.h"
#include "tim.h"

volatile Combined_Info combined;
volatile uint8_t combined_request;

static void Enter(Combined_State state)
{
    combined.stage_tick=HAL_GetTick();
    combined.state=state;
}

static void Fail(uint8_t error)
{
    if (combined.state!=COMBINED_FAULT) combined.error=error;
    combined.state=COMBINED_FAULT;
    basket_request=2U;
    Basket_Task(); /* Latch paired upper-axis stop. */
    Lift_StopHold(); /* Best effort PP stop, not disable of loaded lift. */
}

static uint8_t Done(void)
{
    return Motor_left.state==RS01_DONE && Motor_right.state==RS01_DONE;
}

static uint8_t Pair(float left, float right)
{
    HAL_StatusTypeDef status;
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    status=RS01_StartMove(&Motor_left,left,COMBINED_PUSH_SPEED);
    if (status==HAL_OK) status=RS01_StartMove(&Motor_right,right,COMBINED_PUSH_SPEED);
    if (status!=HAL_OK) Motor_left.state=Motor_right.state=RS01_FAULT;
    __set_PRIMASK(mask);
    if (status!=HAL_OK) { Fail(2U); return 0U; }
    return 1U;
}

HAL_StatusTypeDef Combined_Init(void)
{
    HAL_StatusTypeDef status;
    combined_request=0U;
    combined.error=0U;
    Enter(COMBINED_WAIT_HOME);
    /* Basket_Init owns the only RS01_Init, CAN startup and TIM6 startup. */
    status=Basket_Init();
    if (status==HAL_OK) status=Lift_Init();
    combined.init_status=status;
    if (status!=HAL_OK) Fail(1U);
    return status;
}

void Combined_ControlTick(void)
{
    Basket_ControlTick(); /* Lift uses internal PP, never the speed-mode task. */
}

void Combined_Task(void)
{
    uint8_t request=combined_request;
    combined_request=0U;
    /* Own child command channels so stray debugger requests cannot start axes. */
    lift_request=0U;
    if (combined.state!=COMBINED_GRABBING) basket_request=0U;
    if (request==2U) { Fail(6U); return; }
    if (combined.state==COMBINED_FAULT) return;
    Lift_Task();
    if (lift.state==LIFT_FAULT) { Fail(3U); return; }
    if (basket.state==BASKET_FAULT) { Fail(4U); return; }
    Basket_Task(); /* Keep laser fresh and drain servo replies, but no auto action. */
    if (basket.state==BASKET_FAULT) { Fail(4U); return; }

    if (request==1U && combined.state==COMBINED_WAIT_HOME &&
        (lift.state==LIFT_READY || lift.state==LIFT_HOME))
    {
        combined.left_home=Motor_left.total_angle;
        combined.right_home=Motor_right.total_angle;
        if (Pair(COMBINED_PUSH_TURNS,COMBINED_PUSH_TURNS)) Enter(COMBINED_PUSHING);
    }
    if ((combined.state==COMBINED_PUSHING || combined.state==COMBINED_RETRACTING) &&
        (uint32_t)(HAL_GetTick()-combined.stage_tick)>=COMBINED_PUSH_TIMEOUT_MS)
    { Fail(5U); return; }
    switch (combined.state)
    {
    case COMBINED_PUSHING:
        if (Done())
        {
            float left=(combined.left_home-Motor_left.total_angle)/COMBINED_RAD_PER_TURN;
            float right=(combined.right_home-Motor_right.total_angle)/COMBINED_RAD_PER_TURN;
            if (Pair(left,right)) Enter(COMBINED_RETRACTING);
        }
        break;
    case COMBINED_RETRACTING:
        if (Done())
        {
            lift_request=1U;
            Lift_Task();
            if (lift.state==LIFT_RISING) Enter(COMBINED_LIFTING);
            else Fail(3U);
        }
        break;
    case COMBINED_LIFTING:
        if (lift.state==LIFT_HOLDING)
        {
            basket_request=1U;
            Enter(COMBINED_GRABBING);
        }
        break;
    case COMBINED_GRABBING:
        if (basket.state==BASKET_DONE) Enter(COMBINED_DONE);
        /* No valid laser at launch: Basket_Task rejects request without moving. */
        else if (basket.state==BASKET_IDLE && basket_request==0U && basket.error!=BASKET_ERROR_NONE)
            Fail(4U);
        break;
    default: break;
    }
}
