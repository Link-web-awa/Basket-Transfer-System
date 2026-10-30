#include "RS01.h"
#include <math.h>
#include <string.h>

Motor_t Motor_left, Motor_right;
static float clamp(float x, float lo, float hi)
{ return x < lo ? lo : (x > hi ? hi : x); }
static HAL_StatusTypeDef RS01_SendFrame(uint8_t id, uint8_t type, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef h = {0};
    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U) return HAL_BUSY;
    h.Identifier = ((uint32_t)type << 24) | (0xFDUL << 8) | id;
    h.IdType = FDCAN_EXTENDED_ID;
    h.TxFrameType = FDCAN_DATA_FRAME;
    h.DataLength = FDCAN_DLC_BYTES_8;
    h.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    h.BitRateSwitch = FDCAN_BRS_OFF;
    h.FDFormat = FDCAN_CLASSIC_CAN;
    h.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &h, data);
}
HAL_StatusTypeDef RS01_WriteFloat(uint8_t id, uint16_t index, float value)
{
    uint8_t data[8] = {(uint8_t)index, (uint8_t)(index >> 8), 0,0,0,0,0,0};
    if (!isfinite(value)) return HAL_ERROR;
    memcpy(data + 4, &value, 4);
    return RS01_SendFrame(id, 0x12U, data);
}
HAL_StatusTypeDef RS01_WriteU8(uint8_t id, uint16_t index, uint8_t value)
{
    uint8_t data[8] = {(uint8_t)index, (uint8_t)(index >> 8), 0,0,value,0,0,0};
    return RS01_SendFrame(id, 0x12U, data);
}
HAL_StatusTypeDef RS01_EnableMotor(uint8_t id)
{ const uint8_t data[8] = {0}; return RS01_SendFrame(id, 3, data); }
HAL_StatusTypeDef RS01_SetSpeedTarget(uint8_t id, float speed)
{
    if (!isfinite(speed) || fabsf(speed) > 44.0f) return HAL_ERROR;
    return RS01_WriteFloat(id, 0x700A, speed);
}
static void defaults(Motor_t *m, uint8_t id, int8_t dir)
{
    memset(m, 0, sizeof(*m));
    m->motor_id = id; m->direction = dir; m->run_mode = -1;
    m->limit_cur = 1.0f; m->acceleration = 5.0f;
    m->position_kp = 1.0f; /* Initial P only; tune for the mechanism. */
    m->integral_limit = 1.0f;
    m->position_tolerance = 0.02f; m->speed_tolerance = 0.05f;
    m->move_timeout_ms = 30000U; m->result = HAL_BUSY;
}
void RS01_Init(void)
{
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    defaults(&Motor_left, RS01_MOTOR_1_ID, 1);
    defaults(&Motor_right, RS01_MOTOR_2_ID, -1);
    __set_PRIMASK(mask);
}
static void stop_request(Motor_t *m, HAL_StatusTypeDef result)
{
    m->state = RS01_STOPPING; m->result = result;
    m->zero_sent = 0; m->stop_tick = HAL_GetTick();
    m->stop_cmd_tick = m->stop_tick - RS01_CONTROL_PERIOD_MS; m->integral = 0;
}
HAL_StatusTypeDef RS01_SpeedInitTask(Motor_t *m)
{
    HAL_StatusTypeDef s;
    uint32_t now = HAL_GetTick();
    if (!m) return HAL_ERROR;
    if (m->state == RS01_STOPPING || m->state == RS01_FAULT) return HAL_ERROR;
    if (!isfinite(m->limit_cur) || m->limit_cur <= 0 || m->limit_cur > 23 ||
        !isfinite(m->acceleration) || m->acceleration <= 0) {
        m->state = RS01_FAULT; m->result = HAL_ERROR; return HAL_ERROR;
    }
    if (m->init_step == RS01_INIT_READY) return HAL_OK;
    if (!m->init_started) { m->init_started = 1; m->init_tick = now; }
    if ((uint32_t)(now - m->init_tick) >= 2000U || m->fault) {
        stop_request(m, m->fault ? HAL_ERROR : HAL_TIMEOUT); return m->result;
    }
    switch (m->init_step) {
    case 0: s = RS01_WriteU8(m->motor_id, 0x7005, 2); break;
    case 1: s = RS01_WriteFloat(m->motor_id, 0x7018, m->limit_cur); break;
    case 2: s = RS01_WriteFloat(m->motor_id, 0x7022, m->acceleration); break;
    case 3: s = RS01_SetSpeedTarget(m->motor_id, 0); break;
    case 4: {
        const uint8_t d[8] = {1,2,3,4,5,6,1,0};
        s = RS01_SendFrame(m->motor_id, 0x18, d); break;
    }
    case 5: {
        const uint8_t d[8] = {5,0x70,0,0,0,0,0,0};
        s = RS01_SendFrame(m->motor_id, 0x11, d); break;
    }
    case 6:
        if (m->run_mode != 2) return HAL_BUSY;
        ++m->init_step; return HAL_BUSY;
    case 7:
        m->init_feedback_count = m->feedback_count;
        s = RS01_EnableMotor(m->motor_id); break;
    case 8:
        if (m->feedback_count == m->init_feedback_count || m->pattern != 2 ||
            !m->feedback_valid || (uint32_t)(now-m->feedback_tick) >= RS01_FEEDBACK_TIMEOUT_MS)
            return HAL_BUSY;
        m->init_step = RS01_INIT_READY; m->result = HAL_OK; return HAL_OK;
    default: return HAL_ERROR;
    }
    if (s == HAL_OK) ++m->init_step;
    else if (s != HAL_BUSY) stop_request(m, s);
    return s == HAL_OK ? HAL_BUSY : s;
}
HAL_StatusTypeDef RS01_SpeedInit(Motor_t *m, uint32_t timeout_ms)
{
    if (!m || !timeout_ms) return HAL_ERROR;
    uint32_t start = HAL_GetTick();
    for (;;) {
        HAL_StatusTypeDef s = RS01_SpeedInitTask(m);
        if (s != HAL_BUSY) return s;
        if ((uint32_t)(HAL_GetTick()-start) >= timeout_ms) {
            stop_request(m, HAL_TIMEOUT); return HAL_TIMEOUT;
        }
        HAL_Delay(1);
    }
}
HAL_StatusTypeDef RS01_StartMove(Motor_t *m, float turns, float max_speed)
{
    if (!m || !isfinite(turns) || !isfinite(max_speed) || max_speed <= 0 || max_speed > 44)
        return HAL_ERROR;
    if (m->moving || m->state == RS01_STOPPING) return HAL_BUSY;
    if (m->state == RS01_FAULT) return m->result;
    if (m->init_step != RS01_INIT_READY) return HAL_BUSY;
    if ((m->direction != 1 && m->direction != -1) ||
        !isfinite(m->position_kp) || m->position_kp <= 0 ||
        !isfinite(m->position_ki) || m->position_ki < 0 ||
        !isfinite(m->position_kd) || m->position_kd < 0 ||
        !isfinite(m->integral_limit) || m->integral_limit < 0 ||
        !isfinite(m->position_tolerance) || m->position_tolerance <= 0 ||
        !isfinite(m->speed_tolerance) || m->speed_tolerance <= 0 || !m->move_timeout_ms)
        return HAL_ERROR;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    float pos = m->total_angle;
    uint32_t tick = m->feedback_tick;
    uint8_t valid = m->feedback_valid, fault = m->fault, pattern = m->pattern;
    __set_PRIMASK(mask);
    if (fault || pattern != 2) return HAL_ERROR;
    if (!valid || (uint32_t)(HAL_GetTick()-tick) >= RS01_FEEDBACK_TIMEOUT_MS) return HAL_TIMEOUT;
    float target = pos + turns * 6.283185307f;
    if (!isfinite(target)) return HAL_ERROR;
    m->target_angle = target; m->max_speed = max_speed; m->integral = 0;
    m->settling = 0; m->moving = 1; m->state = RS01_MOVING;
    m->move_tick = HAL_GetTick(); m->control_tick = m->move_tick;
    m->result = HAL_BUSY;
    return HAL_OK;
}
HAL_StatusTypeDef RS01_PositionTask(Motor_t *m)
{
    if (!m) return HAL_ERROR;
    uint32_t now = HAL_GetTick();
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    float pos = m->total_angle, speed = m->speed;
    uint32_t tick = m->feedback_tick;
    uint8_t valid = m->feedback_valid, fault = m->fault, pattern = m->pattern;
    __set_PRIMASK(mask);
    uint8_t fresh = valid && (uint32_t)(now-tick) < RS01_FEEDBACK_TIMEOUT_MS;
    if (m->state == RS01_FAULT) return m->result;
    if ((m->state != RS01_STOPPING || m->result == HAL_OK) &&
        m->init_step == RS01_INIT_READY &&
        (!fresh || fault || pattern != 2 || !isfinite(pos) || !isfinite(speed)))
        stop_request(m, fault || pattern != 2 ? HAL_ERROR : HAL_TIMEOUT);
    if (m->state == RS01_STOPPING) {
        if ((uint32_t)(now-m->stop_cmd_tick) < RS01_CONTROL_PERIOD_MS) return HAL_BUSY;
        m->stop_cmd_tick = now;
        if (m->result != HAL_OK) {
            const uint8_t d[8] = {0};
            HAL_StatusTypeDef s = RS01_SendFrame(m->motor_id, 4, d);
            /* Retry disable until new feedback confirms non-running. */
            if (m->angle_ready && (uint32_t)(now-tick) < RS01_FEEDBACK_TIMEOUT_MS &&
                (uint32_t)(tick-m->stop_tick) < 0x80000000UL && pattern != 2) {
                m->state = RS01_FAULT; m->moving = 0;
            }
            return s == HAL_OK ? m->result : s;
        }
        if ((uint32_t)(now-m->stop_tick) >= 2000U) {
            stop_request(m, HAL_TIMEOUT); return HAL_TIMEOUT;
        }
        HAL_StatusTypeDef s = RS01_SetSpeedTarget(m->motor_id, 0);
        if (s != HAL_OK) return s;
        if (!m->zero_sent) { m->zero_sent = 1; m->stop_tick = now; }
        if (fresh && fabsf(speed) <= m->speed_tolerance &&
            (uint32_t)(tick-m->stop_tick) < 0x80000000UL &&
            (uint32_t)(now-m->stop_tick) >= 100U) {
            m->state = RS01_DONE; m->moving = 0;
        }
        return HAL_OK;
    }
    if (!m->moving) return m->result;
    if ((uint32_t)(now-m->move_tick) >= m->move_timeout_ms) {
        stop_request(m, HAL_TIMEOUT); return HAL_TIMEOUT;
    }
    uint32_t elapsed = now-m->control_tick;
    if (elapsed < RS01_CONTROL_PERIOD_MS) return HAL_BUSY;
    m->control_tick = now;
    if (elapsed >= RS01_FEEDBACK_TIMEOUT_MS) {
        stop_request(m, HAL_TIMEOUT); return HAL_TIMEOUT;
    }
    float error = m->target_angle-pos;
    if (fabsf(error) <= m->position_tolerance && fabsf(speed) <= m->speed_tolerance) {
        if (!m->settling) { m->settling = 1; m->settle_tick = now; }
        if ((uint32_t)(now-m->settle_tick) >= 100U) {
            stop_request(m, HAL_OK); return HAL_BUSY;
        }
    } else m->settling = 0;
    float next = clamp(m->integral + m->position_ki*error*((float)elapsed*0.001f),
                       -m->integral_limit, m->integral_limit);
    float out = m->position_kp*error + next - m->position_kd*speed;
    if ((out > m->max_speed && error > 0) || (out < -m->max_speed && error < 0)) {
        next = m->integral;
        out = m->position_kp*error + next - m->position_kd*speed;
    }
    if (!isfinite(out)) { stop_request(m, HAL_ERROR); return HAL_ERROR; }
    HAL_StatusTypeDef s = RS01_SetSpeedTarget(m->motor_id,
        clamp(out, -m->max_speed, m->max_speed)*m->direction);
    if (s == HAL_OK) m->integral = next;
    else if (s != HAL_BUSY) stop_request(m, s);
    return s;
}
void RS01_Task(void)
{
    Motor_t *motors[2] = {&Motor_left, &Motor_right};
    for (unsigned i=0; i<2; ++i) {
        Motor_t *m = motors[i];
        if (m->init_step != RS01_INIT_READY && m->state != RS01_STOPPING && m->state != RS01_FAULT)
            (void)RS01_SpeedInitTask(m);
        (void)RS01_PositionTask(m);
    }
}
void update_motorstate(Motor_t *m, const uint8_t *d)
{
    if (!m || !d) return;
    uint32_t now = HAL_GetTick();
    float a = (float)(((uint16_t)d[0]<<8)|d[1])*25.14f/65535.0f-12.57f;
    if (m->angle_ready && (uint32_t)(now-m->feedback_tick) >= RS01_FEEDBACK_TIMEOUT_MS)
        m->feedback_valid = 0; /* Lost tracking must be explicitly reinitialized. */
    if (!m->angle_ready) {
        m->angle_ready = 1; m->feedback_valid = 1; m->total_angle = 0;
    } else if (m->feedback_valid) {
        float delta = a-m->last_angle;
        if (delta < -12.57f) delta += 25.14f;
        else if (delta > 12.57f) delta -= 25.14f;
        m->total_angle += delta*m->direction;
    }
    m->angle = a; m->last_angle = a;
    m->turns = m->total_angle/6.283185307f;
    m->speed = ((float)(((uint16_t)d[2]<<8)|d[3])*88.0f/65535.0f-44.0f)*m->direction;
    m->torque = ((float)(((uint16_t)d[4]<<8)|d[5])*34.0f/65535.0f-17.0f)*m->direction;
    m->temp = (float)(((uint16_t)d[6]<<8)|d[7])*0.1f;
    m->feedback_tick = now; ++m->feedback_count;
}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t events)
{
    if (h->Instance != FDCAN2 || !(events & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)) return;
    while (HAL_FDCAN_GetRxFifoFillLevel(h, FDCAN_RX_FIFO0)) {
        FDCAN_RxHeaderTypeDef head; uint8_t d[8];
        if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &head, d) != HAL_OK) break;
        if (head.IdType != FDCAN_EXTENDED_ID || head.RxFrameType != FDCAN_DATA_FRAME ||
            head.DataLength != FDCAN_DLC_BYTES_8 || (head.Identifier&255U) != 0xFDU) continue;
        uint8_t id = (head.Identifier>>8)&255U;
        Motor_t *m = id == Motor_left.motor_id ? &Motor_left :
                     id == Motor_right.motor_id ? &Motor_right : NULL;
        if (!m) continue;
        uint8_t type = (head.Identifier>>24)&31U;
        if (type == 2) {
            m->fault |= (head.Identifier>>16)&63U;
            m->pattern = (head.Identifier>>22)&3U;
            update_motorstate(m, d);
        } else if (type == 0x15) m->fault = 0xFFU;
        else if (type == 0x11 && ((head.Identifier>>16)&255U) == 0 && d[0] == 5 && d[1] == 0x70)
            m->run_mode = d[4];
    }
}
