#include "lift_can.h"
FDCAN_HandleTypeDef hfdcan1;
HAL_StatusTypeDef LiftCan_Start(void)
{
    GPIO_InitTypeDef gpio={0};
    FDCAN_FilterTypeDef filter={0};
    HAL_StatusTypeDef status;
    /* Main already configures the shared FDCAN kernel clock and CAN2 timing.
     * Do not change the kernel clock while CAN2 is running. */
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
    gpio.Pin=GPIO_PIN_14; gpio.Mode=GPIO_MODE_OUTPUT_PP;
    gpio.Pull=GPIO_NOPULL; gpio.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC,&gpio);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET); /* OUT1 power */
    gpio.Pin=GPIO_PIN_0|GPIO_PIN_1; gpio.Mode=GPIO_MODE_AF_PP;
    gpio.Alternate=GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD,&gpio); /* PD0 RX / PD1 TX */
    hfdcan1.Instance=FDCAN1;
    hfdcan1.Init=hfdcan2.Init;
    hfdcan1.Init.MessageRAMOffset=LIFT_CAN_RAM_OFFSET;
    status=HAL_FDCAN_Init(&hfdcan1);
    if(status!=HAL_OK) return status;
    filter.IdType=FDCAN_EXTENDED_ID; filter.FilterIndex=0U;
    filter.FilterType=FDCAN_FILTER_MASK; filter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;
    status=HAL_FDCAN_ConfigFilter(&hfdcan1,&filter);
    if(status==HAL_OK) status=HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
    if(status==HAL_OK) status=HAL_FDCAN_Start(&hfdcan1);
    if(status==HAL_OK) status=HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0U);
    if(status==HAL_OK)
    {
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn,LIFT_CAN_IRQ_PRIORITY,0U);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
    return status;
}
void FDCAN1_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan1); }
