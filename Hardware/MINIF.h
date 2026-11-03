#ifndef __MINIF_H
#define __MINIF_H
#include "main.h"

#define DISTANCE_MIN 20
#define DISTANCE_MAX 5000  // 通讯协议规定的最大有效距离，mm
#define CONFIDENCE_MAX 100 // 通讯协议规定的最大置信度


typedef struct
{
    uint32_t distance;
    uint8_t confidence;
}LRM_t;

/* 调试记录独立于测距结果，不改变原来的解析状态机。 */
typedef struct
{
    uint32_t rx_count;         // 接收字节数
    uint32_t frame_count;      // 完成解析的帧数
    uint32_t invalid_count;    // 被范围检查清零的帧数
    uint32_t copied_count;     // 主程序成功取走结果的次数
    uint32_t raw_distance;     // 最近一帧范围检查前的距离
    uint32_t raw_confidence;   // 最近一帧范围检查前的置信度
    HAL_StatusTypeDef rx_status;
} MINIF_DebugInfo;

extern volatile MINIF_DebugInfo minif_debug;
extern uint8_t uart7_rx_data;
void Processing_Data(uint8_t RXdata);




void MINIF_Init(void);
void get_LRMdata(LRM_t* LRM);


#endif

