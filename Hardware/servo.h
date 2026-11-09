#ifndef __SERVO_H
#define __SERVO_H

#include "usart.h"

#define SERVO_POSITION_RANGE_DEG 270

extern uint8_t rxByte;
extern volatile uint8_t frameReady;

enum servo_state
{
    servo_catch,
    servo_release
};

typedef struct
{
    uint8_t install_dir;
    float cur_angle;
    uint8_t servo_state;
}servo_t;



void Servo_Init(void);
HAL_StatusTypeDef Servo_position_move(uint16_t id, float position_deg, uint16_t time_ms);
HAL_StatusTypeDef Servo_multi_position_move(
    uint16_t id1, float position1_deg, uint16_t time1_ms,
    uint16_t id2, float position2_deg, uint16_t time2_ms);
HAL_StatusTypeDef Servo_set_id(uint16_t current_id, uint16_t new_id);
HAL_StatusTypeDef Servo_SendCommand(const char *cmd);
void Servo_getData(uint8_t* ReceivedataBuffer);
HAL_StatusTypeDef Servo_set_mod(uint16_t id, uint8_t mode);
HAL_StatusTypeDef Servo_set_zero(uint16_t id);
void Servo_catch(void);
void Servo_release(void);
void Servo_RxByteHandler(uint8_t data);
#endif



