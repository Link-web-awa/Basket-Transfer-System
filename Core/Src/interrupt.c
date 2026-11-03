#include "interrupt.h"
#include "MINIF.h"
#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        ++minif_debug.rx_count;
        Processing_Data(uart7_rx_data);
        minif_debug.rx_status = HAL_UART_Receive_IT(&huart7, &uart7_rx_data, 1);
    }
    if(huart->Instance ==UART4)
    {
        Servo_RxByteHandler(rxByte);
        if (!frameReady)
        {
            HAL_UART_Receive_IT(&huart4, &rxByte, 1);
        }
    }
}
