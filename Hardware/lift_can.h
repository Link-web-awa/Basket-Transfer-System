#ifndef LIFT_CAN_H
#define LIFT_CAN_H
#include "fdcan.h"
#define LIFT_CAN_RAM_OFFSET 256U // 32位字偏移；与FDCAN2的RAM区域分开
#define LIFT_CAN_IRQ_PRIORITY 5U
#define LIFT_CAN_POWER_WAIT_MS 3000U // OUT1开启后等待电机启动
extern FDCAN_HandleTypeDef hfdcan1;
HAL_StatusTypeDef LiftCan_Start(void);
#endif
