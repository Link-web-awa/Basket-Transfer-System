#include "servo.h"
#include "string.h"
uint8_t rxByte;                 // 单字节接收
uint8_t servo_rx_buf[32];       // 完整数据帧缓冲区
volatile uint8_t rxIndex = 0;   // 当前接收位置
volatile uint8_t frameReady = 0;// 完整帧标志
volatile uint8_t frameLen = 0;  // 完整帧长度

HAL_StatusTypeDef Servo_SendCommand(const char *cmd)
{
    if (cmd == NULL)
        return HAL_ERROR;

    // 暂停当前接收中断
    HAL_UART_AbortReceive(&huart4);

    // 切换到发送模式
    HAL_HalfDuplex_EnableTransmitter(&huart4);

    // 发送 ASCII 指令
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        &huart4,
        (uint8_t *)cmd,
        strlen(cmd),
        10
    );

    // 切换回接收模式
    HAL_HalfDuplex_EnableReceiver(&huart4);

    // 重新开启单字节中断接收
    if (!frameReady)
    {
        HAL_UART_Receive_IT(&huart4, &rxByte, 1);
    }

    return status;
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
