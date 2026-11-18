#include "RS01.h"
#include <math.h>
#include <string.h>

/* 调用分工：主程序阻塞初始化并登记目标；CAN中断更新反馈；TIM6每10ms执行位置环。
 * 位置环输出速度，电机内部执行速度闭环。中断中禁止调用阻塞初始化接口。
 */

Motor_t Motor_left, Motor_right;
Motor_t Motor_Down;
static HAL_StatusTypeDef RS01_SendFrame(uint8_t id, uint8_t type, const uint8_t data[8]);
volatile float down_mech_position;
volatile uint32_t down_position_tick;
volatile uint8_t down_position_valid;
volatile int down_run_mode = -1;
volatile float down_iqf;
volatile uint32_t down_iq_tick;
volatile uint8_t down_iq_valid;

HAL_StatusTypeDef RS01_RequestParameter(uint8_t id, uint16_t index)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)index; data[1] = (uint8_t)(index >> 8);
    return RS01_SendFrame(id, 0x11U, data);
}

HAL_StatusTypeDef RS01_SetReport(uint8_t id)
{
    const uint8_t data[8] = {1,2,3,4,5,6,1,0};
    return RS01_SendFrame(id, 0x18U, data);
}

HAL_StatusTypeDef RS01_DisableMotor(uint8_t id)
{
    const uint8_t data[8] = {0};
    return RS01_SendFrame(id, 0x04U, data);
}
volatile RS01_DebugInfo rs01_debug;

/* 仅保存第一次超时现场；必须在失能和覆盖反馈时间之前调用。 */
static void RS01_RecordTimeout(Motor_t *motor, uint32_t now, uint32_t feedback_tick)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!rs01_debug.captured)
    {
        rs01_debug.motor_id = motor->motor_id;
        rs01_debug.time_ms = now;
        rs01_debug.feedback_age = now - feedback_tick;
        rs01_debug.left_age = now - Motor_left.feedback_tick;
        rs01_debug.right_age = now - Motor_right.feedback_tick;
        rs01_debug.previous_state = motor->state;
        rs01_debug.can_psr = hfdcan2.Instance->PSR;
        rs01_debug.can_ecr = hfdcan2.Instance->ECR;
        rs01_debug.can_rxf0s = hfdcan2.Instance->RXF0S;
        rs01_debug.can_ir = hfdcan2.Instance->IR;
        rs01_debug.captured = 1U;
    }
    __set_PRIMASK(primask);
}

/* 初始化一次只处理一个电机；模式回复不再占用电机结构体。 */
static Motor_t *volatile init_motor;
static volatile int init_run_mode = -1;

static float RS01_LimitValue(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

/* 非阻塞发帧：短临界区保护 HAL 发送队列，队列满立即返回。 */
static HAL_StatusTypeDef RS01_SendFrame(uint8_t id, uint8_t type, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = ((uint32_t)type << 24) | (0xFDUL << 8) | id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    HAL_StatusTypeDef status = HAL_BUSY;
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) > 0U)
    {
        status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
    }

    __set_PRIMASK(primask);
    return status;
}

/* 类型18：参数地址低字节在前，浮点值放在Byte4~7；与类型2整数反馈的字节序不同。 */
HAL_StatusTypeDef RS01_WriteFloat(uint8_t id, uint16_t index, float value)
{
    if (!isfinite(value))
    {
        return HAL_ERROR;
    }

    uint8_t data[8] = {0};
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8);
    memcpy(&data[4], &value, sizeof(value));
    return RS01_SendFrame(id, 0x12U, data);
}

/* 类型18：写入单字节参数，例如0x7005运行模式。 */
HAL_StatusTypeDef RS01_WriteU8(uint8_t id, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8);
    data[4] = value;
    return RS01_SendFrame(id, 0x12U, data);
}

/* 类型3：使能电机；使能前由初始化过程设置限流、加速度和零速度。 */
HAL_StatusTypeDef RS01_EnableMotor(uint8_t id)
{
    const uint8_t data[8] = {0};
    return RS01_SendFrame(id, 0x03U, data);
}

/* 周期速度发布入口：仅写0x700A，不重复切模式或使能，不等待发送完成。 */
HAL_StatusTypeDef RS01_SetSpeedTarget(uint8_t id, float speed)
{
    if (!isfinite(speed) || fabsf(speed) > 44.0f)
    {
        return HAL_ERROR;
    }
    HAL_StatusTypeDef status = RS01_WriteFloat(id, 0x700A, speed);
    if (id == Motor_left.motor_id)
    {
        rs01_debug.left_speed_cmd = speed;
        rs01_debug.left_tx_status = status;
    }
    else if (id == Motor_right.motor_id)
    {
        rs01_debug.right_speed_cmd = speed;
        rs01_debug.right_tx_status = status;
    }
    return status;
}

/* 默认只启用P；主程序可在启动运动前覆盖参数，不在运动中直接改参数。 */
static void RS01_SetDefaultConfig(Motor_t *motor, uint8_t id, int8_t direction)
{
    memset(motor, 0, sizeof(*motor));
    motor->motor_id = id;
    motor->direction = direction;
    motor->limit_cur = 1.0f;
    motor->acceleration = 5.0f;
    motor->position_kp = 1.0f;
    motor->integral_limit = 1.0f;
    motor->state = RS01_UNINITIALIZED;
    motor->result = HAL_BUSY;
}

/* 上电调用一次，只设置参数，不发送运动命令。 */
void RS01_Init(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RS01_SetDefaultConfig(&Motor_left, RS01_MOTOR_1_ID, RS01_LEFT_DIRECTION);
    RS01_SetDefaultConfig(&Motor_right, RS01_MOTOR_2_ID, RS01_RIGHT_DIRECTION);
    RS01_SetDefaultConfig(&Motor_Down, RS01_DOWN_MOTOR_ID, 1);
    down_position_valid = 0U;
    down_iq_valid = 0U;
    memset((void *)&rs01_debug, 0, sizeof(rs01_debug));
    rs01_debug.left_start_status = HAL_BUSY;
    rs01_debug.right_start_status = HAL_BUSY;
    __set_PRIMASK(primask);
}

/* 仅初始化使用：允许等待，且使用整个初始化过程的统一超时时间。 */
static HAL_StatusTypeDef RS01_InitSend(Motor_t *motor, uint8_t type, const uint8_t data[8],
                                       uint32_t start, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;
    do
    {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        status = RS01_SendFrame(motor->motor_id, type, data);
        if (status == HAL_BUSY)
        {
            HAL_Delay(1U);
        }
    } while (status == HAL_BUSY);

    if (status != HAL_OK)
    {
        return status;
    }

    /* 入队后等待该帧真正发出，避免初始化提前结束。 */
    uint32_t buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2);
    while (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, buffer))
    {
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, buffer);
            return HAL_TIMEOUT;
        }
        HAL_Delay(1U);
    }
    return (hfdcan2.Instance->TXBTO & buffer) ? HAL_OK : HAL_ERROR;
}

/* 主程序调用一次：失能、配置、查询模式、使能，最后等待新的运行反馈。
 * timeout_ms覆盖整个过程，模式读取每100ms重试；仅在主程序且中断开启时调用。
 * 控制定时器必须尚未启动，避免与周期控制竞争初始化用的发送确认。
 */
HAL_StatusTypeDef RS01_SpeedInit(Motor_t *motor, uint32_t timeout_ms)
{
    if (motor == NULL || timeout_ms == 0U || __get_IPSR() != 0U || __get_PRIMASK() != 0U)
    {
        return HAL_ERROR;
    }

    if (!isfinite(motor->limit_cur) || motor->limit_cur <= 0.0f || motor->limit_cur > 23.0f ||
        !isfinite(motor->acceleration) || motor->acceleration <= 0.0f ||
        (motor->direction != 1 && motor->direction != -1))
    {
        return HAL_ERROR;
    }

    if (motor->state != RS01_UNINITIALIZED)
    {
        return HAL_BUSY;
    }

    uint32_t start = HAL_GetTick();
    uint32_t query_tick;
    uint32_t primask;
    const uint8_t enable_data[8] = {0};
    const uint8_t mode_request[8] = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
    HAL_StatusTypeDef status = HAL_OK;
    init_run_mode = -1;
    init_motor = motor;
    rs01_debug.init_motor_id = motor->motor_id;
    rs01_debug.init_run_mode = -1;
    rs01_debug.init_query_count = 0U;

    /* 失能 -> 速度模式 -> 限流 -> 加速度 -> 零速度 -> 主动上报 -> 查询模式。 */
    for (uint8_t step = 0U; step < 7U; ++step)
    {
        rs01_debug.init_step = step;
        uint8_t data[8] = {0};
        uint8_t type = 0x12U;
        float value;

        switch (step)
        {
        case 0:
            type = 0x04U;
            break;
        case 1:
            data[0] = 0x05;
            data[1] = 0x70;
            data[4] = 2;
            break;
        case 2:
            data[0] = 0x18;
            data[1] = 0x70;
            value = motor->limit_cur;
            memcpy(&data[4], &value, sizeof(value));
            break;
        case 3:
            data[0] = 0x22;
            data[1] = 0x70;
            value = motor->acceleration;
            memcpy(&data[4], &value, sizeof(value));
            break;
        case 4:
            data[0] = 0x0A;
            data[1] = 0x70;
            break;
        case 5:
            type = 0x18U;
            for (uint8_t i = 0; i < 6U; ++i)
            {
                data[i] = i + 1U;
            }
            data[6] = 1;
            break;
        case 6:
            type = 0x11U;
            data[0] = 0x05;
            data[1] = 0x70;
            break;
        }
        status = RS01_InitSend(motor, type, data, start, timeout_ms);
        if (status != HAL_OK)
        {
            goto init_failed;
        }
        if (step == 6U) ++rs01_debug.init_query_count;
    }

    /* 失能状态下可能没有类型2反馈，不能把反馈作为使能前提。 */
    rs01_debug.init_step = 7U;
    query_tick = HAL_GetTick();
    while (init_run_mode != 2)
    {
        if (motor->fault)
        {
            status = HAL_ERROR;
            goto init_failed;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            status = HAL_TIMEOUT;
            goto init_failed;
        }
        /* 模式读取可以重试，配置命令无需反复下发。 */
        if ((uint32_t)(HAL_GetTick() - query_tick) >= 100U)
        {
            status = RS01_InitSend(motor, 0x11U, mode_request, start, timeout_ms);
            if (status != HAL_OK) goto init_failed;
            ++rs01_debug.init_query_count;
            query_tick = HAL_GetTick();
        }
        HAL_Delay(1U);
    }

    /* 使能前清除反馈标志，必须等到使能后的新反馈。 */
    primask = __get_PRIMASK();
    __disable_irq();
    motor->feedback_valid = 0U;
    __set_PRIMASK(primask);

    rs01_debug.init_step = 8U;
    status = RS01_InitSend(motor, 0x03U, enable_data, start, timeout_ms);
    if (status != HAL_OK)
    {
        goto init_failed;
    }

    rs01_debug.init_step = 9U;
    while (!motor->feedback_valid || motor->pattern != 2U)
    {
        if (motor->fault)
        {
            status = HAL_ERROR;
            goto init_failed;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= timeout_ms)
        {
            status = HAL_TIMEOUT;
            goto init_failed;
        }
        HAL_Delay(1U);
    }

    if (motor->fault)
    {
        status = HAL_ERROR;
        goto init_failed;
    }
    init_motor = NULL;
    motor->result = HAL_OK;
    motor->state = RS01_IDLE;
    rs01_debug.init_step = 10U;
    return HAL_OK;

init_failed:
    init_motor = NULL;
    motor->result = status;
    motor->state = RS01_FAULT;
    const uint8_t stop_data[8] = {0};
    (void)RS01_SendFrame(motor->motor_id, 0x04U, stop_data);
    return status;
}

/* 主程序下发一次目标；临界区保证定时器不会读到更新一半的命令。 */
HAL_StatusTypeDef RS01_StartMove(Motor_t *motor, float turns, float max_speed)
{
    if (motor == NULL || !isfinite(turns) || !isfinite(max_speed) || max_speed <= 0.0f ||
        max_speed > 44.0f || !isfinite(motor->position_kp) || motor->position_kp <= 0.0f ||
        !isfinite(motor->position_ki) || motor->position_ki < 0.0f ||
        !isfinite(motor->position_kd) || motor->position_kd < 0.0f ||
        !isfinite(motor->integral_limit) || motor->integral_limit < 0.0f)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_OK;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (motor->state == RS01_FAULT)
    {
        status = motor->result;
    }
    else if (motor->state != RS01_IDLE && motor->state != RS01_DONE)
    {
        status = HAL_BUSY;
    }
    else if (motor->fault || motor->pattern != 2U)
    {
        status = HAL_ERROR;
    }
    else if (!motor->feedback_valid ||
             (uint32_t)(HAL_GetTick() - motor->feedback_tick) >= RS01_FEEDBACK_TIMEOUT_MS)
    {
        status = HAL_TIMEOUT;
    }
    else
    {
        float target = motor->total_angle + turns * 6.283185307f;
        if (!isfinite(target))
        {
            status = HAL_ERROR;
        }
        else
        {
            motor->target_angle = target;
            motor->max_speed = max_speed;
            motor->integral = 0.0f;
            motor->settle_count = 0U;
            motor->move_tick = HAL_GetTick();
            motor->result = HAL_BUSY;
            motor->state = RS01_MOVING;
        }
    }

    __set_PRIMASK(primask);
    return status;
}

/* 故障停车只尝试入队，下一个定时周期会再次调用。 */
static HAL_StatusTypeDef RS01_StopOnFault(Motor_t *motor)
{
    const uint8_t data[8] = {0};
    (void)RS01_SendFrame(motor->motor_id, 0x04U, data);
    return motor->result;
}

/* 独立定时器每10ms调用；所有发帧均为非阻塞，不调用HAL_Delay。
 * IDLE/DONE也检查反馈有效性；失联仍会故障停车，不能以“已到位”为由忽略失联。
 */
HAL_StatusTypeDef RS01_PositionTask(Motor_t *motor)
{
    if (motor == NULL)
    {
        return HAL_ERROR;
    }
    ++rs01_debug.control_count;
    if (motor->state == RS01_UNINITIALIZED)
    {
        return HAL_BUSY;
    }
    if (motor->state == RS01_FAULT)
    {
        return RS01_StopOnFault(motor);
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    float position = motor->total_angle;
    float speed = motor->speed;
    uint32_t feedback_tick = motor->feedback_tick;
    uint8_t valid = motor->feedback_valid;
    uint8_t fault = motor->fault;
    uint8_t pattern = motor->pattern;
    __set_PRIMASK(primask);

    uint32_t now = HAL_GetTick();
    if (!valid || (uint32_t)(now - feedback_tick) >= RS01_FEEDBACK_TIMEOUT_MS ||
        !isfinite(position) || !isfinite(speed))
    {
        RS01_RecordTimeout(motor, now, feedback_tick);
        motor->result = HAL_TIMEOUT;
        motor->state = RS01_FAULT;
    }
    if (fault || pattern != 2U)
    {
        motor->result = HAL_ERROR;
        motor->state = RS01_FAULT;
    }

    /* 故障状态保持锁存，每10ms重试失能，不能在中断中等待。 */
    if (motor->state == RS01_FAULT)
    {
        return RS01_StopOnFault(motor);
    }

    if (motor->state == RS01_IDLE || motor->state == RS01_DONE)
    {
        /* 到位后保持零速度目标并继续通信；这不是位置闭环保持。 */
        return RS01_SetSpeedTarget(motor->motor_id, 0.0f);
    }

    if ((uint32_t)(now - motor->move_tick) >= RS01_MOVE_TIMEOUT_MS)
    {
        /* 本版STOPPING也计入任务总时限，首次超时现场保留原状态。 */
        RS01_RecordTimeout(motor, now, feedback_tick);
        motor->result = HAL_TIMEOUT;
        motor->state = RS01_FAULT;
        return RS01_StopOnFault(motor);
    }

    /* 到位后持续发零速度，再等10个低速反馈周期确认停止。 */
    if (motor->state == RS01_STOPPING)
    {
        HAL_StatusTypeDef status = RS01_SetSpeedTarget(motor->motor_id, 0.0f);
        if (status != HAL_OK)
        {
            return status;
        }

        if (fabsf(speed) <= RS01_SPEED_TOLERANCE)
        {
            ++motor->settle_count;
        }
        else
        {
            motor->settle_count = 0U;
        }

        if (motor->settle_count >= 10U)
        {
            motor->result = HAL_OK;
            motor->state = RS01_DONE;
        }
        return HAL_OK;
    }

    float error = motor->target_angle - position;
    if (fabsf(error) <= RS01_POSITION_TOLERANCE && fabsf(speed) <= RS01_SPEED_TOLERANCE)
    {
        ++motor->settle_count;
    }
    else
    {
        motor->settle_count = 0U;
    }

    if (motor->settle_count >= 10U)
    {
        motor->settle_count = 0U;
        motor->integral = 0.0f;
        motor->state = RS01_STOPPING;
        return RS01_SetSpeedTarget(motor->motor_id, 0.0f);
    }

    /* 位置PID输出速度，D项直接使用速度反馈，避免位置差分噪声。 */
    const float dt = RS01_CONTROL_PERIOD_MS * 0.001f;
    float integral_next = RS01_LimitValue(motor->integral + motor->position_ki * error * dt,
                                          -motor->integral_limit, motor->integral_limit);
    float speed_cmd = motor->position_kp * error + integral_next - motor->position_kd * speed;

    if ((speed_cmd > motor->max_speed && error > 0.0f) ||
        (speed_cmd < -motor->max_speed && error < 0.0f))
    {
        integral_next = motor->integral;
        speed_cmd = motor->position_kp * error + integral_next - motor->position_kd * speed;
    }
    if (!isfinite(speed_cmd))
    {
        motor->result = HAL_ERROR;
        motor->state = RS01_FAULT;
        return RS01_StopOnFault(motor);
    }

    speed_cmd = RS01_LimitValue(speed_cmd, -motor->max_speed, motor->max_speed);
    primask = __get_PRIMASK();
    __disable_irq();
    HAL_StatusTypeDef status;
    if (motor->state == RS01_FAULT || motor->fault || !motor->feedback_valid)
    {
        motor->state = RS01_FAULT;
        if (motor->result == HAL_BUSY)
        {
            motor->result = HAL_ERROR;
        }
        status = RS01_StopOnFault(motor);
    }
    else
    {
        status = RS01_SetSpeedTarget(motor->motor_id, speed_cmd * motor->direction);
    }
    __set_PRIMASK(primask);
    if (status == HAL_OK)
    {
        motor->integral = integral_next;
    }
    else if (status != HAL_BUSY)
    {
        motor->result = status;
        motor->state = RS01_FAULT;
    }
    return status;
}

/* 类型2：保留原来的解析写法，先跨边界修正，再转换安装方向。 */
void update_motorstate(Motor_t *motor, const uint8_t *data)
{
    if (motor == NULL || data == NULL)
    {
        return;
    }

    uint16_t raw_angle = ((uint16_t)data[0] << 8) | data[1];
    uint16_t raw_speed = ((uint16_t)data[2] << 8) | data[3];
    uint16_t raw_torque = ((uint16_t)data[4] << 8) | data[5];
    uint16_t raw_temp = ((uint16_t)data[6] << 8) | data[7];

    float angle = (float)raw_angle * 25.14f / 65535.0f - 12.57f;
    uint32_t now = HAL_GetTick();

    if (motor->angle_ready && (uint32_t)(now - motor->feedback_tick) >= RS01_FEEDBACK_TIMEOUT_MS)
    {
        RS01_RecordTimeout(motor, now, motor->feedback_tick);
        motor->result = HAL_TIMEOUT;
        motor->state = RS01_FAULT; // 丢帧后不能自动认为累计位置仍然可靠
    }
    if (!motor->angle_ready)
    {
        motor->total_angle = 0.0f;
        motor->angle_ready = 1U;
    }
    else if (motor->state != RS01_FAULT)
    {
        float delta = angle - motor->last_angle;
        if (delta < -12.57f)
        {
            delta += 25.14f;
        }
        else if (delta > 12.57f)
        {
            delta -= 25.14f;
        }
        motor->total_angle += delta * motor->direction;
    }

    motor->angle = angle;
    motor->last_angle = angle;
    motor->turns = motor->total_angle / 6.283185307f;
    motor->speed = ((float)raw_speed * 88.0f / 65535.0f - 44.0f) * motor->direction;
    motor->torque = ((float)raw_torque * 34.0f / 65535.0f - 17.0f) * motor->direction;
    motor->temp = (float)raw_temp * 0.1f;
    motor->feedback_tick = now;
    motor->feedback_valid = motor->state != RS01_FAULT;
}

/* 唯一FIFO0接收入口：取完现有帧，按电机ID和通信类型分发。
 * 类型2更新反馈；类型17提供初始化模式确认；类型21锁存故障。
 * 此处不发送命令、不做PID，也不等待电机回复。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance != FDCAN2 || (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
        {
            ++rs01_debug.rx_errors;
            break;
        }
        ++rs01_debug.rx_total;
        if (header.IdType != FDCAN_EXTENDED_ID || header.RxFrameType != FDCAN_DATA_FRAME ||
            header.DataLength != FDCAN_DLC_BYTES_8)
        {
            continue;
        }

        uint8_t type = (header.Identifier >> 24) & 0x1FU;
        uint8_t motor_id = (header.Identifier >> 8) & 0xFFU;
        uint8_t host_id = header.Identifier & 0xFFU;
        /* 在主机地址过滤前记录右电机帧，避免把解析过滤误认为没收到。 */
        if (motor_id == Motor_right.motor_id)
        {
            ++rs01_debug.right_any_count;
            rs01_debug.last_right_id = header.Identifier;
            if (type == 0x11U)
            {
                rs01_debug.right_param_index = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
                rs01_debug.right_param_value = data[4];
            }
        }
        if (host_id != 0xFDU)
        {
            continue;
        }

        Motor_t *motor = NULL;
        if (motor_id == Motor_left.motor_id)
        {
            motor = &Motor_left;
        }
        else if (motor_id == Motor_right.motor_id)
        {
            motor = &Motor_right;
        }
        else if (motor_id == Motor_Down.motor_id)
        {
            motor = &Motor_Down;
        }
        if (motor == NULL)
        {
            continue;
        }

        if (type == 0x11U && motor == &Motor_Down && data[0] == 0x05U && data[1] == 0x70U)
        {
            down_run_mode = data[4];
        }
        else if (type == 0x11U && motor == &Motor_Down && data[0] == 0x19U && data[1] == 0x70U)
        {
            float value;
            memcpy(&value, &data[4], sizeof(value));
            if (isfinite(value))
            {
                down_mech_position = value;
                down_position_tick = HAL_GetTick();
                down_position_valid = 1U;
            }
        }
        else if (type == 0x11U && motor == &Motor_Down && data[0] == 0x1AU && data[1] == 0x70U)
        {
            float value;
            memcpy(&value, &data[4], sizeof(value));
            if (isfinite(value))
            {
                down_iqf = value;
                down_iq_tick = HAL_GetTick();
                down_iq_valid = 1U;
            }
        }
        else if (type == 2U || type == 0x18U)
        {
            if (motor == &Motor_left) ++rs01_debug.left_count;
            else ++rs01_debug.right_count;
            motor->fault |= (header.Identifier >> 16) & 0x3FU;
            motor->pattern = (header.Identifier >> 22) & 0x03U;
            update_motorstate(motor, data);
        }
        else if (type == 0x15U)
        {
            motor->fault = 0xFFU;
        }
        else if (type == 0x11U && motor == init_motor &&
                 ((header.Identifier >> 16) & 0xFFU) == 0U && data[0] == 0x05U && data[1] == 0x70U)
        {
            init_run_mode = data[4];
            rs01_debug.init_run_mode = data[4];
        }
    }
}
