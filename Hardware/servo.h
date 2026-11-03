#ifndef __SERVO_H
#define __SERVO_H

#include "usart.h"

#define  ChangeID_000to001 #000PID001!
#define  ChangeID_000to002 #000PID002!
#define  ChangeID_001to001 #001PID001!

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
HAL_StatusTypeDef Servo_SendCommand(const char *cmd);
void Servo_getData(uint8_t* ReceivedataBuffer);
void Servo_catch(uint8_t servo_t);
void Servo_RxByteHandler(uint8_t data);
#endif
