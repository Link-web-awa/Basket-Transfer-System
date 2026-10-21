#include "Robstride.h"
#include "string.h"

#include "fdcan.h"

static FDCAN_HandleTypeDef &hcan = hfdcan2;
volatile HAL_StatusTypeDef robstride_last_tx_status = HAL_OK;

/* 原 bxCAN 接口适配到 STM32H7 FDCAN；仅用于经典 CAN 的 8 字节帧。 */
struct CAN_TxHeaderTypeDef {
    uint32_t StdId, ExtId, IDE, RTR, DLC;
};
#define CAN_ID_STD 0U
#define CAN_ID_EXT 1U
#define CAN_RTR_DATA 0U
#define CAN_RTR_REMOTE 1U

static HAL_StatusTypeDef HAL_CAN_AddTxMessage(
    FDCAN_HandleTypeDef *can, const CAN_TxHeaderTypeDef *message,
    const uint8_t *data, uint32_t *mailbox)
{
    FDCAN_TxHeaderTypeDef header = {0};
    if (message->DLC != 8U || message->IDE > CAN_ID_EXT ||
        message->RTR > CAN_RTR_REMOTE ||
        (message->IDE == CAN_ID_STD && message->StdId > 0x7FFU) ||
        (message->IDE == CAN_ID_EXT && message->ExtId > 0x1FFFFFFFU))
        return robstride_last_tx_status = HAL_ERROR;
    header.Identifier = message->IDE ? message->ExtId : message->StdId;
    header.IdType = message->IDE ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType = message->RTR ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    const uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(can) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= 100U)
            return robstride_last_tx_status = HAL_TIMEOUT;
        HAL_Delay(1U);
    }
    robstride_last_tx_status = HAL_FDCAN_AddMessageToTxFifoQ(can, &header, data);
    if (robstride_last_tx_status == HAL_OK) *mailbox = 0U;
    return robstride_last_tx_status;
}

 #define P_MIN -12.5f
 #define P_MAX 12.5f
 #define V_MIN -44.0f 
 #define V_MAX 44.0f
 #define KP_MIN 0.0f
 #define KP_MAX 500.0f
 #define KD_MIN 0.0f
 #define KD_MAX 5.0f
 #define T_MIN -17.0f
 #define T_MAX 17.0f

uint32_t Mailbox; // 定义邮箱变量

/*******************************************************************************
* @功能     		: RobStride电机实例化的构造函数
* @参数         : CAN ID
* @返回值 			: void
* @概述  				: 初始化电机ID。
*******************************************************************************/
RobStride_Motor::RobStride_Motor(uint8_t CAN_Id, bool MIT_mode)
{
    memset(&Pos_Info, 0, sizeof(Pos_Info));
    memset(&Motor_Set_All, 0, sizeof(Motor_Set_All));
    Unique_ID = 0; error_code = 0; output = 0.0f; Can_Motor = 0; Motor_Offset_MotoFunc = 0;
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0xFD;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	MIT_Mode = MIT_mode;
	MIT_Type = operationControl;
}
RobStride_Motor::RobStride_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, bool MIT_mode)
{
    memset(&Pos_Info, 0, sizeof(Pos_Info));
    memset(&Motor_Set_All, 0, sizeof(Motor_Set_All));
    Unique_ID = 0; error_code = 0; output = 0.0f; Can_Motor = 0; Motor_Offset_MotoFunc = 0;
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0xFD;	
	Motor_Set_All.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;
	MIT_Mode = MIT_mode;
	MIT_Type = operationControl;
}
/*******************************************************************************
* @功能     		: uint16_t型转float型浮点数
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits){
    uint32_t span = (1 << bits) - 1;
		x &= span; 
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/*******************************************************************************
* @功能     		: float浮点数转int型
* @参数1        : 需要转换的值
* @参数2        : x的最小值
* @参数3        : x的最大值
* @参数4        : 需要转换的进制数
* @返回值 			: 十进制的int型整数
* @概述  				: None
*******************************************************************************/
int float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}
/*******************************************************************************
* @功能     		: uint8_t数组转float浮点数
* @参数        	: 需要转换的数组
* @返回值 			: 十进制的float型浮点数
* @概述  				: None
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float; memcpy(&data_float, &data, sizeof(data_float));
  return data_float;  
}  
/*******************************************************************************
* @功能     	: MIT错位码 转换至 私有模式错位码
* @参数        	: MIT错位码
* @返回值 		: uint8_t
* @概述  		: None
*******************************************************************************/
uint8_t mapFaults(uint16_t fault16) {
    uint8_t fault8 = 0;

    if (fault16 & (1 << 14)) fault8 |= (1 << 4); // 过载故障
    if (fault16 & (1 << 7))  fault8 |= (1 << 5); // 未标定
    if (fault16 & (1 << 3))  fault8 |= (1 << 3); // 磁编码故障
    if (fault16 & (1 << 2))  fault8 |= (1 << 0); // 欠压故障
    if (fault16 & (1 << 1))  fault8 |= (1 << 1); // 驱动故障
    if (fault16 & (1 << 0))  fault8 |= (1 << 2); // 过温

    return fault8;
}
//int count_num = 0 ;
/*******************************************************************************
* @功能     	: 接收处理函数		（通信类型2 17应答帧 0应答帧）
* @参数1        : 接收到的数据
* @参数2        : 接收到的CANID
* @返回值 		: None
* @概述  		: drw只有通过通信17发送以后才有值
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId)
{
	if(MIT_Mode)
	{
		if((ID_ExtId & 0xFF) == 0XFD)
		{
			if(DataFrame[3] == 0x00 && DataFrame[4] == 0x00 && DataFrame[5] == 0x00 && DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
			{
				uint16_t fault16 = 0;
				memcpy(&fault16, &DataFrame[1], 2);
				error_code = mapFaults(fault16);
			}
			else
			{
				Pos_Info.Angle =  uint16_to_float((DataFrame[1]<<8) | (DataFrame[2]),P_MIN,P_MAX,16);
				Pos_Info.Speed =  uint16_to_float((DataFrame[3]<<4) | (DataFrame[4]>>4),V_MIN,V_MAX,12);
				Pos_Info.Torque = uint16_to_float((DataFrame[4]<<8) | (DataFrame[5]),T_MIN,T_MAX,12);
				Pos_Info.Temp = ((DataFrame[6]<<8) | DataFrame[7])*0.1;
			}
		}
		else
		{
		    memcpy(&Unique_ID, DataFrame, 8);
		}
	}
	else 
	{
		if (uint8_t((ID_ExtId&0xFF00)>>8) == CAN_ID)
		{		
			if (int((ID_ExtId&0x3F000000)>>24) == 2)
			{
				Pos_Info.Angle =  uint16_to_float(DataFrame[0]<<8|DataFrame[1],P_MIN,P_MAX,16);
				Pos_Info.Speed =  uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
				Pos_Info.Torque = uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
				Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7])*0.1;
				error_code = uint8_t((ID_ExtId&0x3F0000)>>16);
				Pos_Info.pattern = uint8_t((ID_ExtId&0xC00000)>>22);
			}
			else if (int((ID_ExtId&0x3F000000)>>24) == 17)
			{
				for (int index_num = 0; index_num <= 13; index_num++)
				{
					if ((DataFrame[1]<<8|DataFrame[0]) == Index_List[index_num])
						switch(index_num)
						{
							case 0:
								drw.run_mode.data = uint8_t(DataFrame[4]);
								break;
							case 1:
								drw.iq_ref.data = Byte_to_float(DataFrame);
								break;
							case 2:
								drw.spd_ref.data = Byte_to_float(DataFrame);
								break;
							case 3:
								drw.imit_torque.data = Byte_to_float(DataFrame);
								break;
							case 4:
								drw.cur_kp.data = Byte_to_float(DataFrame);
								break;
							case 5:
								drw.cur_ki.data = Byte_to_float(DataFrame);
								break;
							case 6:
								drw.cur_filt_gain.data = Byte_to_float(DataFrame);
								break;
							case 7:
								drw.loc_ref.data = Byte_to_float(DataFrame);
								break;
							case 8:
								drw.limit_spd.data = Byte_to_float(DataFrame);
								break;
							case 9:
								drw.limit_cur.data = Byte_to_float(DataFrame);
								break;	
							case 10:
								drw.mechPos.data = Byte_to_float(DataFrame);
								break;	
							case 11:
								drw.iqf.data = Byte_to_float(DataFrame);
								break;	
							case 12:
								drw.mechVel.data =Byte_to_float(DataFrame);
								break;	
							case 13:
								drw.VBUS.data = Byte_to_float(DataFrame);
								break;	
						}
				}
			}
			else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
			{
				CAN_ID = uint8_t((ID_ExtId & 0xFF00)>>8);	
				memcpy(&Unique_ID, DataFrame, 8);
			}
		}
	}
}
/*******************************************************************************
* @功能     		: RobStride电机获取设备ID和MCU（通信类型0）
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Get_CAN_ID()
{
		uint8_t txdata[8] = {0};						   	//发送数据
		CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		TxMessage.ExtId = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStride电机运控模式  （通信类型1）
* @参数1        : 力矩（-4Nm~4Nm）
* @参数2        : 目标角度(-4π~4π)
* @参数3        : 目标角速度(-30rad/s~30rad/s)
* @参数4        : Kp(0.0~500.0)
* @参数5        : Kp(0.0~5.0)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 					//发送邮箱
	Motor_Set_All.set_Torque = Torque;
	Motor_Set_All.set_angle = Angle;	
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_Kp = Kp;
	Motor_Set_All.set_Kd = Kd;
	if (drw.run_mode.data != 0)
	{
		Set_RobStride_Motor_parameter(0X7005, move_control_mode, Set_mode);		//设置电机模式
		Get_RobStride_Motor_parameter(0x7005);
		Enable_Motor();
		Motor_Set_All.set_motor_mode = move_control_mode;
	}
	if(Pos_Info.pattern != 2)
	{
		Enable_Motor();
	}
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID;
	txdata[0] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16)>>8; 
	txdata[1] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16); 
	txdata[2] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16)>>8; 
	txdata[3] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16); 
	txdata[4] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16)>>8; 
	txdata[5] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16); 
	txdata[6] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16)>>8; 
	txdata[7] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16); 
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
//MIT模式使能
void RobStride_Motor::RobStride_Motor_MIT_Enable()
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID;
	txMsg.IDE = CAN_ID_STD;
	txMsg.DLC = 8;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFC;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

//MIT模式失能
void RobStride_Motor::RobStride_Motor_MIT_Disable()
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID;
	txMsg.IDE = CAN_ID_STD;
	txMsg.DLC = 8;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFD;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

//MIT模式清除或检查错误
void RobStride_Motor::RobStride_Motor_MIT_ClearOrCheckError(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFB;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}

//MIT设置电机运行模式
void RobStride_Motor::RobStride_Motor_MIT_SetMotorType(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFC;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}

//MIT设置电机ID
void RobStride_Motor::RobStride_Motor_MIT_SetMotorId(uint8_t F_CMD)
{
    uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0x01;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}



//MIT控制模式
void RobStride_Motor::RobStride_Motor_MIT_Control(float Angle, float Speed, float Kp, float Kd, float Torque)
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	txdata[0] = float_to_uint(Angle, P_MIN,P_MAX, 16)>>8;
	txdata[1] = float_to_uint(Angle, P_MIN,P_MAX, 16);
	txdata[2] = float_to_uint(Speed, V_MIN,V_MAX, 12)>>4;
	txdata[3] = float_to_uint(Speed, V_MIN,V_MAX, 12)<<4 | float_to_uint(Kp, KP_MIN, KP_MAX, 12)>>8;
	txdata[4] = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
	txdata[5] = float_to_uint(Kd, KD_MIN, KD_MAX, 12)>>4;
	txdata[6] = float_to_uint(Kd, KD_MIN, KD_MAX, 12)<<4 | float_to_uint(Torque, T_MIN, T_MAX, 12)>>8;
	txdata[7] = float_to_uint(Torque, T_MIN, T_MAX, 12);
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}

//MIT位置模式
void RobStride_Motor::RobStride_Motor_MIT_PositionControl(float position_rad, float speed_rad_per_s)
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = (1 << 8) | CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	memcpy(&txdata[0], &position_rad, 4); 	//将位置数据复制到发送数据数组中
	memcpy(&txdata[4], &speed_rad_per_s, 4); 	//将速度数据复制到发送数据数组中
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}
// MIT模式速度控制实现
void RobStride_Motor::RobStride_Motor_MIT_SpeedControl(float speed_rad_per_s, float current_limit)
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = (2 << 8) | CAN_ID;
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	memcpy(&txdata[0], &speed_rad_per_s, 4);
	memcpy(&txdata[4], &current_limit, 4);
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox);
}

//MIT零点设置模式
void RobStride_Motor::RobStride_Motor_MIT_SetZeroPos()
{
	uint8_t txdata[8] = {0}; 	//定义发送数据数组
	CAN_TxHeaderTypeDef txMsg = {0}; 	//发送邮箱
	txMsg.StdId = CAN_ID; 	//设置标准ID
	txMsg.IDE = CAN_ID_STD; 	//设置标识符类型
	txMsg.RTR = CAN_RTR_DATA; 	//设置远程传输请求
	txMsg.DLC = 8; 	//设置数据长度
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = 0xFF;
	txdata[7] = 0xFE;
	HAL_CAN_AddTxMessage(&hcan, &txMsg, txdata, &Mailbox); 	//发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机位置模式(PP插补位置模式控制)
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标角度(-4π~4π)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Pos_control(float Speed, float Angle)
{
		Motor_Set_All.set_speed = Speed;
		Motor_Set_All.set_angle = Angle;
//		Motor_Set_All.set_limit_speed = vel_max;
//		Motor_Set_All.set_acceleration = acc_set;
//		if (drw.run_mode.data != 1 && Pos_Info.pattern == 2)
		if (drw.run_mode.data != 1)
		{
			Set_RobStride_Motor_parameter(0X7005, Pos_control_mode, Set_mode);		//设置电机模式
			Get_RobStride_Motor_parameter(0x7005);
			Motor_Set_All.set_motor_mode = Pos_control_mode;
			Enable_Motor();
			Set_RobStride_Motor_parameter(0X7024, Motor_Set_All.set_limit_speed, Set_parameter);
			Set_RobStride_Motor_parameter(0X7025, Motor_Set_All.set_acceleration, Set_parameter);
		}	
		HAL_Delay(1);
		Set_RobStride_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
}
/*******************************************************************************
* @功能     		: RobStride电机位置模式(CSP位置模式控制)
* @参数1        : 目标角度(-4π~4π)
* @参数2        : 目标角速度(0rad/s~44rad/s)
* @返回值 				: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_CSP_control(float Angle, float limit_spd)
{
    if (MIT_Mode) {
        RobStride_Motor_MIT_PositionControl(Angle, limit_spd);
        return;
    }
    Motor_Set_All.set_angle = Angle;
    Motor_Set_All.set_limit_speed = limit_spd;
    const bool mode_changed = drw.run_mode.data != CSP_control_mode;
    if (mode_changed) {
        Set_RobStride_Motor_parameter(0x7005, CSP_control_mode, Set_mode);
        if (robstride_last_tx_status != HAL_OK) return;
        HAL_Delay(20U);
        Get_RobStride_Motor_parameter(0x7005);
        if (robstride_last_tx_status != HAL_OK) return;
        HAL_Delay(10U);
    }
    Set_RobStride_Motor_parameter(0x7017, limit_spd, Set_parameter);
    if (robstride_last_tx_status != HAL_OK) return;
    HAL_Delay(10U);
    if (mode_changed || Pos_Info.pattern != 2) {
        Enable_Motor();
        if (robstride_last_tx_status != HAL_OK) return;
        HAL_Delay(20U);
    }
    Set_RobStride_Motor_parameter(0x7016, Angle, Set_parameter);
}

/*******************************************************************************
* @功能     		: RobStride电机速度模式 
* @参数1        : 目标角速度(-30rad/s~30rad/s)
* @参数2        : 目标电流限制(0~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStride_Motor::RobStride_Motor_Speed_control(float Speed, float limit_cur)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_limit_cur = limit_cur;
	if (drw.run_mode.data != 2)
	{
		Set_RobStride_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//设置电机模式
		Get_RobStride_Motor_parameter(0x7005);
		Enable_Motor();
		Motor_Set_All.set_motor_mode = Speed_control_mode;
		Set_RobStride_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
		Set_RobStride_Motor_parameter(0X7022, 10, Set_parameter);	
//		Set_RobStride_Motor_parameter(0X7022, Motor_Set_All.set_acceleration, Set_parameter);	
	}
	Set_RobStride_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);
}
/*******************************************************************************
* @功能     		: RobStride电机电流模式
* @参数         : 目标电流(-23~23A)
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStride_Motor::RobStride_Motor_current_control(float current)
{
	Motor_Set_All.set_current = current;
	output = Motor_Set_All.set_current;
	if (Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStride_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//设置电机模式
		Get_RobStride_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = Elect_control_mode;
		Enable_Motor();
	}
	Set_RobStride_Motor_parameter(0X7006, Motor_Set_All.set_current, Set_parameter);
}
/*******************************************************************************
* @功能     	: RobStride电机零点模式(电机回到机械零点)
* @参数         : None
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_Set_Zero_control()
{
	Set_RobStride_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//设置电机模式
}
/*******************************************************************************
* @功能     		: RobStride电机使能 （通信类型3）
* @参数         : None
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::Enable_Motor()
{
	if (MIT_Mode)
	{
		RobStride_Motor_MIT_Enable();
	}
	else
	{
		uint8_t txdata[8] = {0};				//发送数据
		CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
		
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		TxMessage.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_ID;
		HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
	}
}
/*******************************************************************************
* @功能     		: RobStride电机失能 （通信类型4）
* @参数         : 是否清除错误位（0不清除 1清除）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::Disenable_Motor(uint8_t clear_error)
{
	if (MIT_Mode)
	{
		RobStride_Motor_MIT_Disable();
	}
	else
	{
		uint8_t txdata[8] = {0};					   	//发送数据
		CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱

		txdata[0] = clear_error;
		TxMessage.IDE = CAN_ID_EXT;
		TxMessage.RTR = CAN_RTR_DATA;
		TxMessage.DLC = 8;
		TxMessage.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|CAN_ID;
		HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
		Set_RobStride_Motor_parameter(0X7005, move_control_mode, Set_mode);
	}

}
/*******************************************************************************
* @功能     		: RobStride电机写入参数 （通信类型18）
* @参数1        : 参数地址
* @参数2        : 参数数值
* @参数3        : 选择是传入控制模式 还是其他参数 （Set_mode设置控制模式 Set_parameter设置参数）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::Set_RobStride_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = Index;
	txdata[1] = Index>>8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;	
	if (Value_mode == 'p')
	{
		memcpy(&txdata[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		Motor_Set_All.set_motor_mode = int(Value);
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;	
		txdata[6] = 0x00;	
		txdata[7] = 0x00;	
	}
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStride电机单个参数读取 （通信类型17）
* @参数         : 参数地址
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::Get_RobStride_Motor_parameter(uint16_t Index)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	txdata[0] = Index;
	txdata[1] = Index>>8;
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStride电机设置CAN_ID （通信类型7）
* @参数         : 修改后（预设）CANID
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}
/*******************************************************************************
* @功能     		: RobStride电机设置机械零点 （通信类型6）
* @参数         : None
* @返回值 			: void
* @概述  				: 会把当前电机位置设为机械零位， 会先失能电机, 再使能电机
*******************************************************************************/
void RobStride_Motor::Set_ZeroPos()
{
	Disenable_Motor(0);							//失能电机
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 1;
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
	Enable_Motor();
}

/*******************************************************************************
* @功能     		: RobStride电机数据保存 （通信类型22）
* @参数      		: None
* @返回值 				: void
* @概述  				: 会把当前单参数写入表中的数据写为默认值，重新上电后参数保持为此指令运行时的参数
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_MotorDataSave()
{
	uint8_t txdata[8] = {0};				//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorDataSave<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = 0x07;
	txdata[7] = 0x08;
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机波特率修改 （通信类型23）
* @参数      		: 波特率模式:	 01（1M）
									02（500K）
									03（250K）
									04（125K）
* @返回值 				: void
* @概述  				: 将电机波特率修改为对应的值，例如参数为01，波特率修改为1M
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_BaudRateChange(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_BaudRateChange<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;	//这一字节无所谓，填什么都可以，这里写的是0x08
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机主动上报设置 （通信类型24）
* @参数      		: 上报模式：	00（关闭）
														01（开启）
* @返回值 				: void
* @概述  				: 开启/关闭 电机主动上报，默认上报周期为10ms
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_ProactiveEscalationSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_ProactiveEscalationSet<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;	//这一字节无所谓，填什么都可以，这里写的是0x08
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

/*******************************************************************************
* @功能     		: RobStride电机协议修改 （通信类型25）
* @参数      		: 协议类型：		00（私有协议）
										01（Canopen）
										02（MIT协议）
* @返回值 				: void
* @概述  				: None
*******************************************************************************/
void RobStride_Motor::RobStride_Motor_MIT_MotorModeSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};				//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.StdId = CAN_ID;
	txdata[0] = 0xFF;
	txdata[1] = 0xFF;
	txdata[2] = 0xFF;
	txdata[3] = 0xFF;
	txdata[4] = 0xFF;
	txdata[5] = 0xFF;
	txdata[6] = F_CMD;
	txdata[7] = 0xFD;	//这一字节无所谓，填什么都可以，这里写的是0x08
  HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}


/*******************************************************************************
* @功能     		: RobStride电机数据的参数地址初始化
* @参数         : 数据的参数地址数组
* @返回值 			: void
* @概述  				: 会在创建电机类时自动调用
*******************************************************************************/
data_read_write::data_read_write(const uint16_t *index_list)
{
	run_mode.index = index_list[0];
	iq_ref.index = index_list[1];
	spd_ref.index = index_list[2];
	imit_torque.index = index_list[3];
	cur_kp.index = index_list[4];
	cur_ki.index = index_list[5];
	cur_filt_gain.index = index_list[6];
	loc_ref.index = index_list[7];
	limit_spd.index = index_list[8];
	limit_cur.index = index_list[9];
	mechPos.index = index_list[10];
	iqf.index = index_list[11];
	mechVel.index = index_list[12];
	VBUS.index = index_list[13];	
	rotation.index = index_list[14];
}

void RobStride_Motor::RobStride_Motor_MotorModeSet(uint8_t F_CMD)
{
	uint8_t txdata[8] = {0};						   	//发送数据
	CAN_TxHeaderTypeDef TxMessage = {0}; 	//发送邮箱
	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8;
	TxMessage.ExtId = Communication_Type_MotorModeSet<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 0x01;
	txdata[1] = 0x02;
	txdata[2] = 0x03;
	txdata[3] = 0x04;
	txdata[4] = 0x05;
	txdata[5] = 0x06;
	txdata[6] = F_CMD;
	txdata[7] = 0x08;	//这个一位随便填，此处填0x08
  	HAL_CAN_AddTxMessage(&hcan, &TxMessage, txdata, &Mailbox); // 发送CAN消息
}

bool RobStride_Motor::Get_MIT_Mode() { return MIT_Mode; }
MIT_TYPE RobStride_Motor::get_MIT_Type() { return MIT_Type; }
void RobStride_Motor::Set_MIT_Mode(bool mode) { MIT_Mode = mode; }
void RobStride_Motor::Set_MIT_Type(MIT_TYPE type) { MIT_Type = type; }

/* 保留 main.c 已使用的 5 个 C 接口，启动流程不变。 */
#include "fdcan.h"
#include <string.h>

static const uint8_t MOTOR_ID = 0x06U;
static const uint8_t MASTER_ID = 0xFDU;
static RobStride_Motor rs01_motor(MOTOR_ID, false);
volatile uint32_t rs01_status_rx_count;
volatile uint32_t rs01_status_rx_id;
volatile uint8_t rs01_status_rx_data[8];
/* 仅记录上电控制序列，周期查询不会覆盖它，便于核对实际发送字节。 */
struct RS01_StartupFrame {
    uint32_t id;
    uint8_t data[8];
};
volatile RS01_StartupFrame rs01_startup_tx[7];
volatile uint32_t rs01_startup_tx_count;
volatile uint32_t rs01_parameter_rx_error_count;
volatile uint32_t rs01_fault_rx_count;
volatile uint8_t rs01_fault_rx_data[8];
/* 1 失能，2 设置模式，3 确认模式，4 确认使能，5 写参数，6 已下发速度。 */
volatile uint8_t rs01_speed_stage;
volatile uint32_t rs01_rx_count;
volatile RS01_StartupFrame rs01_startup_rx[16];
volatile uint32_t rs01_last_rx_id;
volatile uint32_t rs01_mode_rx_count;
volatile uint32_t rs01_mode_rx_id;
volatile uint8_t rs01_mode_rx_data[8];

/* 解析反馈，调试器中可查看 rs01_motor 的模式、速度和故障。 */
void RS01_PollFeedback(void)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
    {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0,
                                  &header, data) != HAL_OK) return;
        if (rs01_rx_count < 16U)
        {
            volatile RS01_StartupFrame *frame = &rs01_startup_rx[rs01_rx_count];
            frame->id = header.Identifier;
            for (uint32_t i = 0; i < 8U; ++i) frame->data[i] = data[i];
        }
        ++rs01_rx_count;
        rs01_last_rx_id = header.Identifier;
        /* 在目标 ID 和读取状态过滤之前保存模式回包，定位解析条件。 */
        if (header.IdType == FDCAN_EXTENDED_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.DataLength == FDCAN_DLC_BYTES_8 &&
            ((header.Identifier >> 24) & 0x1FU) == 0x11U &&
            data[0] == 0x05U && data[1] == 0x70U)
        {
            ++rs01_mode_rx_count;
            rs01_mode_rx_id = header.Identifier;
            for (uint32_t i = 0; i < 8U; ++i) rs01_mode_rx_data[i] = data[i];
        }
        if (header.IdType == FDCAN_EXTENDED_ID &&
            header.RxFrameType == FDCAN_DATA_FRAME &&
            header.DataLength == FDCAN_DLC_BYTES_8 &&
            ((header.Identifier >> 8) & 0xFFU) == MOTOR_ID &&
            (header.Identifier & 0xFFU) == MASTER_ID)
        {
            const uint32_t type = (header.Identifier >> 24) & 0x1FU;
            if (type == 0x11U && ((header.Identifier >> 16) & 0xFFU) != 0U)
            {
                ++rs01_parameter_rx_error_count;
                continue;
            }
            if (type == 0x15U)
            {
                ++rs01_fault_rx_count;
                for (uint32_t i = 0; i < 8U; ++i) rs01_fault_rx_data[i] = data[i];
            }
            if (((header.Identifier >> 24) & 0x1FU) == 2U)
            {
                rs01_status_rx_id = header.Identifier;
                for (uint32_t i = 0; i < 8U; ++i) rs01_status_rx_data[i] = data[i];
                ++rs01_status_rx_count;
            }
            rs01_motor.RobStride_Motor_Analysis(data, header.Identifier);
        }
    }
    /* 轮流读回模式、目标速度、限流和实际速度，确认电机是否接受设置。 */
    static uint32_t query_tick;
    static uint8_t query_index;
    static const uint16_t parameters[] = {0x7005U, 0x700AU, 0x7018U, 0x701BU};
    if (rs01_speed_stage == 6U &&
        (uint32_t)(HAL_GetTick() - query_tick) >= 100U)
    {
        rs01_motor.Get_RobStride_Motor_parameter(parameters[query_index]);
        query_index = (uint8_t)((query_index + 1U) % 4U);
        query_tick = HAL_GetTick();
    }
}

/* 私有协议：扩展帧 ID = 通信类型 | 主机 ID | 电机 ID。 */
static HAL_StatusTypeDef SendFrameTo(uint8_t target_id, uint8_t type, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = ((uint32_t)type << 24)
                      | ((uint32_t)MASTER_ID << 8) | target_id;
    header.IdType = FDCAN_EXTENDED_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    if (type != 0x00U && type != 0x11U && rs01_startup_tx_count < 7U)
    {
        volatile RS01_StartupFrame *frame = &rs01_startup_tx[rs01_startup_tx_count];
        frame->id = header.Identifier;
        for (uint32_t i = 0; i < 8U; ++i) frame->data[i] = data[i];
        ++rs01_startup_tx_count;
    }

    const uint32_t start = HAL_GetTick();
    while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) >= 100U)
            return HAL_TIMEOUT;
        HAL_Delay(1U);
    }
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header, data);
    if (status != HAL_OK) return status;
    const uint32_t buffer = HAL_FDCAN_GetLatestTxFifoQRequestBuffer(&hfdcan2);
    const uint32_t tx_start = HAL_GetTick();
    while (HAL_FDCAN_IsTxBufferMessagePending(&hfdcan2, buffer) != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - tx_start) >= 100U)
        {
            (void)HAL_FDCAN_AbortTxRequest(&hfdcan2, buffer);
            return HAL_TIMEOUT;
        }
        HAL_Delay(1U);
    }
    /* 入队不等于成功发送：TXBTO 才表示控制器已成功发出这一帧。 */
    return (hfdcan2.Instance->TXBTO & buffer) != 0U ? HAL_OK : HAL_ERROR;
}

static HAL_StatusTypeDef SendFrame(uint8_t type, const uint8_t data[8])
{
    return SendFrameTo(MOTOR_ID, type, data);
}

/* 查询私有协议设备，不修改 ID，不发送使能或运动指令。
   遍历 0x00~0xFF，每个地址等待 25 ms，返回第一个有效回复的 ID。 */
HAL_StatusTypeDef RS01_FindMotorId(uint8_t *motor_id)
{
    if (motor_id == NULL) return HAL_ERROR;
    const uint8_t request[8] = {0};
    for (uint32_t candidate = 0; candidate <= 0xFFU; ++candidate)
    {
        HAL_StatusTypeDef status = SendFrameTo((uint8_t)candidate, 0x00U, request);
        if (status != HAL_OK) return status;
        const uint32_t start = HAL_GetTick();
        do
        {
            while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
            {
                FDCAN_RxHeaderTypeDef header;
                uint8_t data[8];
                status = HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &header, data);
                if (status != HAL_OK) return status;
                if (header.IdType == FDCAN_EXTENDED_ID &&
                    header.RxFrameType == FDCAN_DATA_FRAME &&
                    header.DataLength == FDCAN_DLC_BYTES_8 &&
                    ((header.Identifier >> 24) & 0x1FU) == 0U &&
                    (header.Identifier & 0xFFU) == 0xFEU)
                {
                    *motor_id = (uint8_t)((header.Identifier >> 8) & 0xFFU);
                    return HAL_OK;
                }
            }
            HAL_Delay(1U);
        } while ((uint32_t)(HAL_GetTick() - start) < 25U);
    }
    return HAL_TIMEOUT;
}

/* 参数浮点数按小端顺序写入数据区 Byte4~7。 */
static HAL_StatusTypeDef WriteFloat(uint16_t index, float value)
{
    uint8_t data[8] = {0};
    data[0] = (uint8_t)index;
    data[1] = (uint8_t)(index >> 8);
    memcpy(&data[4], &value, sizeof(value));
    return SendFrame(0x12U, data);
}

/* 每 100 ms 查询实际位置，最多等待 3 秒。 */
HAL_StatusTypeDef RS01_ReadPosition(float *position_rad)
{
    if (position_rad == NULL) return HAL_ERROR;
    const uint8_t request[8] = {0x19U, 0x70U, 0, 0, 0, 0, 0, 0};
    const uint32_t start = HAL_GetTick();
    uint32_t last_request = start - 100U;
    while ((uint32_t)(HAL_GetTick() - start) < 3000U)
    {
        if ((uint32_t)(HAL_GetTick() - last_request) >= 100U)
        {
            HAL_StatusTypeDef status = SendFrame(0x11U, request);
            if (status != HAL_OK) return status;
            last_request = HAL_GetTick();
        }
        while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0U)
        {
            FDCAN_RxHeaderTypeDef header;
            uint8_t data[8];
            HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(
                &hfdcan2, FDCAN_RX_FIFO0, &header, data);
            if (status != HAL_OK) return status;
            if (header.IdType == FDCAN_EXTENDED_ID &&
                header.RxFrameType == FDCAN_DATA_FRAME &&
                header.DataLength == FDCAN_DLC_BYTES_8 &&
                ((header.Identifier >> 24) & 0x1FU) == 0x11U &&
                ((header.Identifier >> 8) & 0xFFU) == MOTOR_ID &&
                (header.Identifier & 0xFFU) == MASTER_ID &&
                data[0] == 0x19U && data[1] == 0x70U)
            {
                memcpy(position_rad, &data[4], sizeof(*position_rad));
                return HAL_OK;
            }
        }
        HAL_Delay(1U);
    }
    return HAL_TIMEOUT;
}

HAL_StatusTypeDef RS01_SetCspMode(void)
{
    /* 0x7005 为 uint8 模式参数，5 表示 CSP。 */
    const uint8_t data[8] = {0x05U, 0x70U, 0, 0, 5U, 0, 0, 0};
    return SendFrame(0x12U, data);
}

HAL_StatusTypeDef RS01_SetSpeedLimit(float speed_rad_s)
{
    if (!(speed_rad_s > 0.0f && speed_rad_s <= 0.15f)) return HAL_ERROR;
    return WriteFloat(0x7017U, speed_rad_s);
}

HAL_StatusTypeDef RS01_Enable(void)
{
    const uint8_t data[8] = {0};
    return SendFrame(0x03U, data);
}

HAL_StatusTypeDef RS01_SetPosition(float position_rad)
{
    return WriteFloat(0x7016U, position_rad);
}

HAL_StatusTypeDef RS01_SpeedControl(float speed_rad_s, float current_limit_a)
{
    if (!(speed_rad_s >= -44.0f && speed_rad_s <= 44.0f) ||
        !(current_limit_a > 0.0f && current_limit_a <= 23.0f)) return HAL_ERROR;

    /* 恢复 0x06 电机已验证的启动流程：先查询位置，确认电机就绪。 */
    rs01_speed_stage = 0U;
    float initial_position;
    HAL_StatusTypeDef status = RS01_ReadPosition(&initial_position);
    if (status != HAL_OK) return status;

    /* 先失能，再配置速度模式。 */
    rs01_speed_stage = 1U;
    const uint8_t stop[8] = {0};
    status = SendFrame(0x04U, stop);
    if (status != HAL_OK) return status;
    HAL_Delay(20U);

    const uint8_t mode[8] = {0x05U, 0x70U, 0, 0, 2U, 0, 0, 0};
    rs01_speed_stage = 2U;
    status = SendFrame(0x12U, mode);
    if (status != HAL_OK) return status;
    HAL_Delay(20U);
    /* 等待真正的模式读回，不能仅靠固定延时。 */
    rs01_speed_stage = 3U;
    rs01_motor.drw.run_mode.data = -1.0f;
    const uint8_t mode_request[8] = {0x05U, 0x70U, 0, 0, 0, 0, 0, 0};
    uint32_t wait_start = HAL_GetTick();
    uint32_t request_tick = wait_start - 100U;
    while (rs01_motor.drw.run_mode.data != 2.0f)
    {
        if ((uint32_t)(HAL_GetTick() - request_tick) >= 100U)
        {
            status = SendFrame(0x11U, mode_request);
            if (status != HAL_OK) return status;
            request_tick = HAL_GetTick();
        }
        RS01_PollFeedback();
        if ((uint32_t)(HAL_GetTick() - wait_start) >= 2000U) return HAL_TIMEOUT;
        HAL_Delay(1U);
    }

    /* 排空旧状态后使能；只接受使能之后收到的新运行状态。 */
    rs01_speed_stage = 4U;
    bool running = false;
    for (uint32_t attempt = 0; attempt < 3U && !running; ++attempt)
    {
        RS01_PollFeedback();
        const uint32_t previous_count = rs01_status_rx_count;
        status = RS01_Enable();
        if (status != HAL_OK) return status;
        wait_start = HAL_GetTick();
        while ((uint32_t)(HAL_GetTick() - wait_start) < 300U)
        {
            RS01_PollFeedback();
            if (rs01_status_rx_count != previous_count &&
                rs01_motor.Pos_Info.pattern == 2)
            {
                running = true;
                break;
            }
            HAL_Delay(1U);
        }
    }
    if (!running) return HAL_TIMEOUT;

    rs01_speed_stage = 5U;
    status = WriteFloat(0x7018U, current_limit_a);
    if (status != HAL_OK) return status;
    HAL_Delay(10U);
    status = WriteFloat(0x7022U, 1.0f);
    if (status != HAL_OK) return status;
    HAL_Delay(10U);
    status = WriteFloat(0x700AU, 0.0f);
    if (status != HAL_OK) return status;
    HAL_Delay(10U);
    status = WriteFloat(0x700AU, speed_rad_s);
    if (status == HAL_OK) rs01_speed_stage = 6U;
    return status;
}
