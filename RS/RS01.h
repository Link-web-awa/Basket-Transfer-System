#ifndef RS01_H
#define RS01_H

#include "Robstride.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* 保留本次实测通过的参数；位置、速度的单位不要混用。 */
#define RS01_CONTROL_PERIOD_MS 10U       // TIM6控制周期，ms
#define RS01_FEEDBACK_TIMEOUT_MS 100U    // 类型2反馈超过此间隔即判为超时
#define RS01_MOVE_TIMEOUT_MS 30000U      // 从启动运动到停止确认的总时限，ms
#define RS01_POSITION_TOLERANCE 0.15f    // 到位位置误差，rad，约8.59°
#define RS01_SPEED_TOLERANCE 0.15f       // 到位/停止允许速度，rad/s

typedef enum
{
    RS01_UNINITIALIZED, // 软件参数已建立，尚未完成速度模式初始化
    RS01_IDLE,          // 初始化成功，等待第一条运动命令
    RS01_MOVING,        // 位置环计算速度，向目标累计角度运动
    RS01_STOPPING,      // 已满足到位条件，发布零速度并确认停止
    RS01_DONE,          // 本次运动完成，可直接接受下一条相对运动命令
    RS01_FAULT         // 故障锁存，定时中断持续尝试发送失能帧
} RS01_State;

typedef struct
{
    /* 电机配置：初始化前设置，运行时不要修改。 */
    uint8_t motor_id;
    int8_t direction;   // +1 正常安装，-1 镜像安装
    float limit_cur;    // 电流限制，A
    float acceleration; // 加速度，rad/s²

    /* CAN 接收中断更新的反馈。 */
    volatile float angle; // 原始循环角度，rad
    volatile float speed; // 统一机械方向，rad/s
    volatile float torque;     // 统一机械方向的力矩，N·m
    volatile float temp;       // 电机温度，℃
    volatile float last_angle; // 上一次原始反馈角度，用于判断跨边界
    volatile float total_angle; // 相对首帧的累计角度，rad
    volatile float turns;             // 净转动圈数，正转和反转相互抵消
    volatile uint32_t feedback_tick;   // 最近一次类型2反馈的HAL毫秒时间
    volatile uint8_t angle_ready;     // 已接收第一帧角度，建立相对零点
    volatile uint8_t feedback_valid;  // 当前累计位置是否仍可信
    volatile uint8_t fault;   // 故障锁存，重新初始化时清除
    volatile uint8_t pattern; // 电机上报的运行状态

    /* 主程序登记目标，10ms 定时中断执行 PID。 */
    float target_angle; // 与total_angle同坐标系的累计角度目标，rad
    float max_speed;    // 本次运动的速度幅值上限，rad/s
    float position_kp;  // 位置误差到速度的比例系数
    float position_ki;  // 位置误差积分系数，默认不启用
    float position_kd;  // 速度反馈阻尼系数，默认不启用
    float integral;     // 积分项当前的速度输出，rad/s
    float integral_limit; // 积分项速度输出限制，rad/s
    uint32_t move_tick; // 本次任务接受时间，STOPPING仍使用这个起点判断总超时
    uint8_t settle_count;      // 连续到位/停止的周期数
    volatile RS01_State state; // 用一个状态代替 moving、init_step 等标志
    volatile HAL_StatusTypeDef result; // BUSY表示任务未结束，OK表示初始化/运动成功
} Motor_t;

extern Motor_t Motor_left, Motor_right;

/* 独立诊断记录：不参与控制；计数持续更新，超时现场仅保存首次。
 * 复位或调用RS01_Init后清零。调试器可直接监视rs01_debug。
 */
typedef struct
{
    uint32_t rx_total;    // FIFO0成功取出的总帧数，包含之后被过滤的帧
    uint32_t left_count;  // 左电机通过检查的类型2反馈数
    uint32_t right_count; // 右电机通过检查的类型2反馈数，不代表所有回复数
    uint32_t rx_errors;   // HAL从FIFO0取帧失败的次数
    uint32_t control_count;     // 两个电机的位置任务调用总次数
    uint32_t init_motor_id;
    uint32_t init_step;         // 0~6发配置，7等模式，8使能，9等运行，10成功
    int init_run_mode;          // 模式读取回复，-1表示尚未收到
    uint32_t init_query_count;  // 模式读取发送成功的次数
    uint32_t right_any_count;   // ID7有效扩展帧总数，包含被主机地址检查过滤的帧
    uint32_t last_right_id;     // 右电机最近一帧的完整扩展ID
    uint32_t right_param_index; // 最近一条类型17回复的参数地址
    uint32_t right_param_value; // 该回复的Byte4，运行模式为uint8
    HAL_StatusTypeDef left_start_status;  // 主程序最近一次左电机启动返回值
    HAL_StatusTypeDef right_start_status; // 主程序最近一次右电机启动返回值
    float left_speed_cmd;       // 最近一次尝试发布的速度，电机自身方向
    float right_speed_cmd;
    HAL_StatusTypeDef left_tx_status;  // 最近一次速度发布结果，OK仅代表成功入队
    HAL_StatusTypeDef right_tx_status;
    uint32_t captured;          // 首次超时后置1，后续不覆盖
    uint32_t motor_id;     // 首次超时的电机ID
    uint32_t time_ms;      // 首次超时发生时的HAL毫秒时间
    uint32_t feedback_age;      // 实际触发判断使用的反馈间隔
    uint32_t left_age;     // 超时现场中左电机反馈距当前的时间，ms
    uint32_t right_age;    // 超时现场中右电机反馈距当前的时间，ms
    RS01_State previous_state; // 故障前状态，用于区分运动、停止确认及完成后超时
    uint32_t can_psr;      // 以下为首次超时现场的FDCAN原始寄存器值
    uint32_t can_ecr;
    uint32_t can_rxf0s;
    uint32_t can_ir;
} RS01_DebugInfo;

extern volatile RS01_DebugInfo rs01_debug;

/* 上电初始化软件参数和诊断记录；不发帧、不启动电机，重复调用会清累计位置。 */
void RS01_Init(void);
/* 仅主程序调用：阻塞完成配置、确认和使能；控制定时器须处于停止状态。 */
HAL_StatusTypeDef RS01_SpeedInit(Motor_t *motor, uint32_t timeout_ms);
/* 主程序收到新指令时调用一次。turns是相对圈数，正负表示统一机械方向。
 * max_speed是正的速度幅值上限。HAL_OK仅代表接受任务，完成看state==RS01_DONE。
 * 只允许从IDLE或DONE启动；更新目标及清积分在同一临界区完成。
 */
HAL_StatusTypeDef RS01_StartMove(Motor_t *motor, float turns, float max_speed);
/* 仅由独立的10ms定时器中断调用，无等待、无延时。 */
HAL_StatusTypeDef RS01_PositionTask(Motor_t *motor);

/* 以下发送接口均非阻塞：队列满返回HAL_BUSY，HAL_OK仅代表成功入队。 */
HAL_StatusTypeDef RS01_WriteFloat(uint8_t id, uint16_t index, float value);
HAL_StatusTypeDef RS01_WriteU8(uint8_t id, uint16_t index, uint8_t value);
HAL_StatusTypeDef RS01_EnableMotor(uint8_t id);
/* speed使用电机自身方向，单位rad/s；上层只需进行一次direction转换。 */
HAL_StatusTypeDef RS01_SetSpeedTarget(uint8_t id, float speed);
void update_motorstate(Motor_t *motor, const uint8_t *data);

#ifdef __cplusplus
}
#endif
#endif
