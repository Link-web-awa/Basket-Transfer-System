#include "servo.h"
#include "string.h"
#include <stdio.h>
uint8_t rxByte;                 // 单字节接收
uint8_t servo_rx_buf[32];       // 完整数据帧缓冲区
volatile uint8_t rxIndex = 0;   // 当前接收位置
volatile uint8_t frameReady = 0;// 完整帧标志
volatile uint8_t frameLen = 0;  // 完整帧长度

HAL_StatusTypeDef Servo_SendCommand(const char *cmd)
{
    size_t cmd_length;
    uint32_t timeout_ms;
    HAL_StatusTypeDef status;
    HAL_StatusTypeDef restore_status;

    if (cmd == NULL)
        return HAL_ERROR;

    cmd_length = strlen(cmd);
    if (cmd_length == 0U || cmd_length > 65535U || huart4.Init.BaudRate == 0U)
        return HAL_ERROR;

    /* UART4 uses 8N1: ten wire bits per byte, plus a 10 ms margin. */
    timeout_ms = (uint32_t)((cmd_length * 10000UL + huart4.Init.BaudRate - 1U)
                           / huart4.Init.BaudRate) + 10U;

    // 暂停当前接收中断
    status = HAL_UART_AbortReceive(&huart4);
    if (status != HAL_OK) return status;

    // 切换到发送模式
    status = HAL_HalfDuplex_EnableTransmitter(&huart4);

    // 发送 ASCII 指令
    if (status == HAL_OK) status = HAL_UART_Transmit(
        &huart4,
        (uint8_t *)cmd,
        (uint16_t)cmd_length,
        timeout_ms
    );

    // 切换回接收模式
    restore_status = HAL_HalfDuplex_EnableReceiver(&huart4);
    if (status == HAL_OK) status = restore_status;

    // 重新开启单字节中断接收
    if (restore_status == HAL_OK && !frameReady)
    {
        restore_status = HAL_UART_Receive_IT(&huart4, &rxByte, 1);
        if (status == HAL_OK) status = restore_status;
    }

    return status;
}

HAL_StatusTypeDef Servo_position_move(uint16_t id, float position_deg, uint16_t time_ms)
{
    char cmd[16];
    uint16_t pulse;
    int length;

    /* This comparison also rejects NaN and infinity. */
    if (id > 255U || time_ms > 9999U ||
        !(position_deg >= 0.0f && position_deg <= SERVO_POSITION_RANGE_DEG))
    {
        return HAL_ERROR;
    }

    pulse = (uint16_t)(500.0f + position_deg *
                      (2000.0f / SERVO_POSITION_RANGE_DEG) + 0.5f);
    length = snprintf(cmd, sizeof(cmd), "#%03uP%04uT%04u!",
                      (unsigned int)id, (unsigned int)pulse,
                      (unsigned int)time_ms);
    if (length < 0 || length >= (int)sizeof(cmd))
    {
        return HAL_ERROR;
    }

    return Servo_SendCommand(cmd);
}

HAL_StatusTypeDef Servo_multi_position_move(
    uint16_t id1, float position1_deg, uint16_t time1_ms,
    uint16_t id2, float position2_deg, uint16_t time2_ms)
{
    char cmd[38];
    uint16_t pulse1, pulse2;
    int length;

    if (id1 > 254U || id2 > 254U || time1_ms > 9999U || time2_ms > 9999U ||
        !(position1_deg >= 0.0f && position1_deg <= SERVO_POSITION_RANGE_DEG) ||
        !(position2_deg >= 0.0f && position2_deg <= SERVO_POSITION_RANGE_DEG))
        return HAL_ERROR;

    pulse1 = (uint16_t)(500.0f + position1_deg *
                       (2000.0f / SERVO_POSITION_RANGE_DEG) + 0.5f);
    pulse2 = (uint16_t)(500.0f + position2_deg *
                       (2000.0f / SERVO_POSITION_RANGE_DEG) + 0.5f);
    length = snprintf(cmd, sizeof(cmd), "{G0000#%03uP%04uT%04u!#%03uP%04uT%04u!}",
                      (unsigned int)id1, (unsigned int)pulse1, (unsigned int)time1_ms,
                      (unsigned int)id2, (unsigned int)pulse2, (unsigned int)time2_ms);
    if (length < 0 || length >= (int)sizeof(cmd))
        return HAL_ERROR;

    return Servo_SendCommand(cmd);
}

HAL_StatusTypeDef Servo_set_mod(uint16_t id, uint8_t mode)
{
    char cmd[11];
    int length;

    if (id > 255U || mode < 1U || mode > 8U)
    {
        return HAL_ERROR;
    }

    length = snprintf(cmd, sizeof(cmd), "#%03uPMOD%u!",
                      (unsigned int)id, (unsigned int)mode);
    if (length < 0 || length >= (int)sizeof(cmd))
    {
        return HAL_ERROR;
    }

    return Servo_SendCommand(cmd);
}

HAL_StatusTypeDef Servo_set_id(uint16_t current_id, uint16_t new_id)
{
    char cmd[12];
    int length;

    if (current_id > 255U || new_id > 254U)
    {
        return HAL_ERROR;
    }

    length = snprintf(cmd, sizeof(cmd), "#%03uPID%03u!",
                      (unsigned int)current_id, (unsigned int)new_id);
    if (length < 0 || length >= (int)sizeof(cmd))
    {
        return HAL_ERROR;
    }

    return Servo_SendCommand(cmd);
}

HAL_StatusTypeDef Servo_set_zero(uint16_t id)
{
    char cmd[10];
    int length;

    if (id > 255U)
        return HAL_ERROR;

    length = snprintf(cmd, sizeof(cmd), "#%03uPSCK!", (unsigned int)id);
    if (length < 0 || length >= (int)sizeof(cmd))
        return HAL_ERROR;

    return Servo_SendCommand(cmd);
}

void Servo_getData(uint8_t* ReceivedataBuffer)
{
    if(frameReady)
    {
        for(uint8_t i=0;i<=frameLen;i++)
        {
            ReceivedataBuffer[i]=servo_rx_buf[i];
        }
        frameReady=0;
        HAL_UART_Receive_IT(&huart4, &rxByte, 1);
    }
    
   
}

HAL_StatusTypeDef Servo_catch(void)
{
    return Servo_multi_position_move(1,90.0f,500,2,90.0f,500);
}

HAL_StatusTypeDef Servo_release(void)
{
    return Servo_multi_position_move(1,10.0f,500,2,10.0f,500);
}

void Servo_Init(void)
{
    HAL_HalfDuplex_EnableReceiver(&huart4);
    HAL_UART_Receive_IT(&huart4,&rxByte, 1);
}


void Servo_RxByteHandler(uint8_t data)
{
    // 已经接收到完整帧，暂时不处理新数据
    if (frameReady)
        return;

    // 检测帧头
    if (data == '#')
    {
        rxIndex = 0;
        servo_rx_buf[rxIndex++] = data;
        return;
    }

    // 尚未找到帧头，忽略数据
    if (rxIndex == 0)
        return;

    // 检查缓冲区是否溢出
    if (rxIndex >= sizeof(servo_rx_buf) - 1)
    {
        rxIndex = 0;
        return;
    }

    // 存储接收到的数据
    servo_rx_buf[rxIndex++] = data;

    // 检测帧尾
    if (data == '!')
    {
        servo_rx_buf[rxIndex] = '\0';

        frameLen = rxIndex;
        frameReady = 1;
    }
}
