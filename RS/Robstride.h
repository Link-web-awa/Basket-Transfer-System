#ifndef __RobStride_H__
#define __RobStride_H__

#include "stm32h7xx_hal.h"

/* Default motor for legacy C APIs; per-motion APIs take an explicit ID. */
#define RS01_MOTOR_ID 0x05U
#define RS01_MOTOR_1_ID 0x05U
#define RS01_MOTOR_2_ID 0x07U

/* PP acceleration used by RS01_MoveTurns(), in rad/s^2. */
#define RS01_MOVE_ACCEL_RAD_S2 1.0f

/* Position-motion current limit in A; valid range: (0, 23]. */
#define RS01_MOVE_CURRENT_LIMIT_A 1.0f

#ifdef __cplusplus

 #define Set_mode 		 'j'
 #define Set_parameter 'p'

 #define move_control_mode  0
 #define Pos_control_mode   1
 #define Speed_control_mode 2
 #define Elect_control_mode 3
 #define Set_Zero_mode      4
 #define CSP_control_mode		5

#define Communication_Type_Get_ID 0x00
#define Communication_Type_MotionControl 0x01
#define Communication_Type_MotorRequest 0x02
#define Communication_Type_MotorEnable 0x03
#define Communication_Type_MotorStop 0x04
#define Communication_Type_SetPosZero 0x06
#define Communication_Type_Can_ID 0x07
#define Communication_Type_Control_Mode 0x12
#define Communication_Type_GetSingleParameter 0x11
#define Communication_Type_SetSingleParameter 0x12
#define Communication_Type_ErrorFeedback 0x15

#define Communication_Type_MotorDataSave 0x16
#define Communication_Type_BaudRateChange 0x17
#define Communication_Type_ProactiveEscalationSet 0x18
#define Communication_Type_MotorModeSet 0x19

class data_read_write_one
{
	public:
		data_read_write_one() : index(0), data(0.0f) {}
        uint16_t index;
		float data;
};
static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

class data_read_write
{

	public:
		data_read_write_one run_mode;
		data_read_write_one iq_ref;
		data_read_write_one spd_ref;
		data_read_write_one imit_torque;
		data_read_write_one cur_kp;
		data_read_write_one cur_ki;
		data_read_write_one cur_filt_gain;
		data_read_write_one loc_ref;
		data_read_write_one limit_spd;
		data_read_write_one limit_cur;

		data_read_write_one mechPos;
		data_read_write_one iqf;
		data_read_write_one	mechVel;
		data_read_write_one	VBUS;
		data_read_write_one	rotation;
		data_read_write(const uint16_t *index_list=Index_List);
};
typedef struct
{
	float Angle;
	float Speed;
	float Torque;
	float Temp;
	int pattern;
}Motor_Pos_RobStride_Info;
typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_acceleration;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_limit_speed;
	float set_Kp;
	float set_Ki;
	float set_Kd;
}Motor_Set;

enum MIT_TYPE
{
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
};

class RobStride_Motor
{
private:
	uint8_t CAN_ID;
	uint64_t Unique_ID;
	uint16_t Master_CAN_ID;
	float (*Motor_Offset_MotoFunc)(float Motor_Tar);

	Motor_Set Motor_Set_All;
	uint8_t error_code;

	bool MIT_Mode;
	MIT_TYPE MIT_Type;

	void Set_MIT_Mode(bool MIT_Mode);
	void Set_MIT_Type(MIT_TYPE MIT_Type);

public:
    float output;
    int Can_Motor;
    Motor_Pos_RobStride_Info Pos_Info;
    data_read_write drw;

    RobStride_Motor(uint8_t CAN_Id, bool MIT_mode);
    RobStride_Motor(float (*offset)(float), uint8_t CAN_Id, bool MIT_mode);
    void RobStride_Motor_Analysis(uint8_t *data, uint32_t identifier);
    void Enable_Motor();                 //按本地协议标志选择报文
    void Disenable_Motor(uint8_t clear_error);
    bool Get_MIT_Mode();
    MIT_TYPE get_MIT_Type();

    // ==================== 私有协议：扩展帧 ====================
    void RobStride_Get_CAN_ID();
    void Set_RobStride_Motor_parameter(uint16_t index, float value, char value_mode);
    void Get_RobStride_Motor_parameter(uint16_t index);
    void RobStride_Motor_move_control(float torque, float angle, float speed, float kp, float kd);
    void RobStride_Motor_Pos_control(float speed, float angle);
    void RobStride_Motor_CSP_control(float angle, float limit_spd);
    void RobStride_Motor_Speed_control(float speed, float limit_cur);
    void RobStride_Motor_current_control(float current);
    void RobStride_Motor_Set_Zero_control();
    void Set_CAN_ID(uint8_t id);
    void Set_ZeroPos();
    void RobStride_Motor_MotorDataSave();
    void RobStride_Motor_BaudRateChange(uint8_t command);
    void RobStride_Motor_ProactiveEscalationSet(uint8_t command);
    void RobStride_Motor_MotorModeSet(uint8_t protocol); // 0 私有、1 CANopen、2 MIT

    // ==================== MIT 协议：标准帧 ====================
    void RobStride_Motor_MIT_Enable();
    void RobStride_Motor_MIT_Disable();
    void RobStride_Motor_MIT_SetMotorType(uint8_t mode); // 0 运控、1 位置、2 速度
    void RobStride_Motor_MIT_Control(float angle, float speed, float kp, float kd, float torque);
    void RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s);
    void RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit);
    void RobStride_Motor_MIT_SetZeroPos();
    void RobStride_Motor_MIT_ClearOrCheckError(uint8_t command);
    void RobStride_Motor_MIT_SetMotorId(uint8_t id);
    void RobStride_Motor_MIT_MotorModeSet(uint8_t protocol); // 重新上电生效
};

#endif /* __cplusplus */

#ifdef __cplusplus
extern "C" {
#endif

/* main.c 使用的上电运动接口；单位为 rad、rad/s。 */
HAL_StatusTypeDef RS01_ReadPosition(float *position_rad);
/* 扫描私有协议设备，返回首个回复的 CAN ID；不修改电机配置。 */
/* Single connected motor: two matching UID replies plus a run-mode read reply.
   On failure, *motor_id remains 0xFF; no motion or configuration writes. */
HAL_StatusTypeDef RS01_FindMotorId(uint8_t *motor_id);
HAL_StatusTypeDef RS01_SetCspMode(void);
HAL_StatusTypeDef RS01_SetSpeedLimit(float speed_rad_s);
HAL_StatusTypeDef RS01_Enable(void);
HAL_StatusTypeDef RS01_SetPosition(float position_rad);
/* 私有协议速度控制：切换模式、限流、使能并持续转动；速度设 0 可停车。 */
HAL_StatusTypeDef RS01_SpeedControl(float speed_rad_s, float current_limit_a);
/* Relative output-shaft motion for motor_id: signed turns, positive speed in rad/s (0,44].
   Blocks until arrival/error, then attempts disable; current limit uses RS01_MOVE_CURRENT_LIMIT_A.
   Caller must enforce available travel. Blocking: call motors sequentially, not concurrently. */
HAL_StatusTypeDef RS01_MoveTurns(uint8_t motor_id, float turns, float speed_rad_s);
/* Configure/enable once, run relative segments while holding between them,
   then disable explicitly. MoveTurns blocks; this is not dual-axis synchronization.
   Speed is rad/s; current limit uses RS01_MOVE_CURRENT_LIMIT_A. A segment failure attempts disable. */
HAL_StatusTypeDef RS01_MotionBegin(uint8_t motor_id, float speed_rad_s);
HAL_StatusTypeDef RS01_MotionMoveTurns(uint8_t motor_id, float turns);
HAL_StatusTypeDef RS01_MotionEnd(uint8_t motor_id);
void RS01_PollFeedback(void);

#ifdef __cplusplus
}
#endif

#endif /* ROBSTRIDE_H */
