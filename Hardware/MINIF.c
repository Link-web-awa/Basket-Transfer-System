#include "MINIF.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

/* huart7使用CubeMX在usart.c中定义的句柄，不能重复定义。 */
uint8_t uart7_rx_data;
volatile uint16_t distance_value = 0;
volatile uint8_t confidence_value = 0;
volatile uint8_t data_ready = 0;
volatile MINIF_DebugInfo minif_debug;


void MINIF_Init(void)
{
    minif_debug.rx_status = HAL_UART_Receive_IT(&huart7,&uart7_rx_data,1);
}

void Processing_Data(uint8_t RXdata) {
static uint8_t recv_buf[16] = {0}; // 接收缓冲区（最大11字节数据） / Receive buffer (max 11 bytes data)

static uint8_t index = 0;           
// 当前缓冲区位置 / Current buffer position
static uint8_t parsing = 0;         // 解析状态 / Parsing state
static uint8_t comma_pos = 0;       // 逗号位置（用于截取距离数据） / Commaposition (for extracting distance data)

// 解析状态 / Parsing state
// 逗号位置（用于截取距离数据） / Comma 
// 防溢出：如果缓冲区满则重置状态机 / Overflow protection: reset state machine if buffer full
if (index >= sizeof(recv_buf)) {
index = 0;        
parsing = 0;
comma_pos = 0;  // 重置逗号位置 / Reset comma position
return;
}
// 存储接收到的字节 / Store received byte
recv_buf[index++] = RXdata;
  // 状态机解析 / State machine parsing
    switch (parsing) {
        case 0:  // 等待帧头0x20 (空格)  Wait for header 0x20 (space)
            if (RXdata == 0x20) {
                parsing = 1;  // 进入距离解析状态  Enter distance parsing state
                index = 1;     
            } else {
                index = 0;     // 非帧头字符，重置  Non-header character, reset
            }
            break;
        case 1:  // 解析距离值 / Parse distance value
            if (RXdata == 0x2C) {  // 遇到逗号  Encounter comma
                parsing = 2;       // 进入分隔符检查状态  Enter separator check state

                comma_pos = index - 1; // 记录逗号位置  Record comma position
            }
            break;
        case 2:  // 检查分隔符0x20 (空格)  Check separator 0x20 (space)
            if (RXdata == 0x20) {
                parsing = 3; // 进入置信度解析状态  Enter confidence parsing state
            } else {
                // 格式错误，重置状态机  Format error, reset state machine
                parsing = 0;
                index = 0;
                comma_pos = 0;  // 重置逗号位置 / Reset comma position
            }
            break;
        case 3:  // 解析置信度 / Parse confidence value
            if (RXdata == 0x0A) { 
            
                uint8_t dist_len = comma_pos - 1;  // 距离值长度  Distance value length

                if (dist_len > 5) dist_len = 5;   // 防止溢出  Prevent overflow
                char dist_str[6] = {0};            
                memcpy(dist_str, &recv_buf[1], dist_len); 
                dist_str[dist_len] = '\0';         // 添加结束符  Add null terminator

                // 提取置信度字符串 / Extract confidence string
                uint8_t conf_start = comma_pos + 2;  
                uint8_t conf_len = index - conf_start - 1;  
                if (conf_len > 3) conf_len = 3;      // 置信度0~100，最多3字符

                char conf_str[4] = {0};              // 3位数值及字符串结束符
                memcpy(conf_str, &recv_buf[conf_start], conf_len);
                conf_str[conf_len] = '\0';            // 同上
                // 转换为数值 / Convert to numerical values
                distance_value = atoi(dist_str);      // 距离值转换  Distance conversion

                confidence_value = atoi(conf_str);    // 置信度转换  Confidence conversion

                ++minif_debug.frame_count;
                minif_debug.raw_distance = distance_value;
                minif_debug.raw_confidence = confidence_value;

                // 检查数据有效性  Check data validity
                if ((distance_value < DISTANCE_MIN) || (distance_value > DISTANCE_MAX) || (confidence_value > CONFIDENCE_MAX)) 
                {
                    // 无效数据，清零  Invalid data, clear values
                    ++minif_debug.invalid_count;
                    distance_value = 0;
                    confidence_value = 0;
                }
                
                // 设置数据就绪标志  Set data ready flag
                data_ready = 1;
                // 重置接收状态机  Reset state machine
                index = 0;
                parsing = 0;
                comma_pos = 0;  
            }
            break;
        default:
            // 未知状态，重置状态机  Unknown state, reset state machine
            parsing = 0;
            index = 0;
            comma_pos = 0;
            break;
    }
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        ++minif_debug.rx_count;
        Processing_Data(uart7_rx_data);
        minif_debug.rx_status = HAL_UART_Receive_IT(&huart7, &uart7_rx_data, 1);
    }
}

void get_LRMdata(LRM_t* LRM)
{
    if (LRM == NULL) return;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if(data_ready==1)
    {
    LRM->distance=distance_value;
    LRM->confidence=confidence_value;
    data_ready=0;
    ++minif_debug.copied_count;
    }
    __set_PRIMASK(primask);
}
