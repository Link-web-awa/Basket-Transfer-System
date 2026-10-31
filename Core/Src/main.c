/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../RS/Robstride.h"
#include "RS01.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim6;
volatile uint8_t step = 0U; // 正反转测试阶段，放在全局便于调试器监视

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void MX_TIM6_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */

  if (FDCAN2_ConfigureRxFilters() != HAL_OK) Error_Handler();
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) Error_Handler();
  if (HAL_FDCAN_ActivateNotification(&hfdcan2,
      FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) Error_Handler();
  /* 等待电机上电，然后阻塞完成两个电机的速度模式初始化。 */
  HAL_Delay(3000U);
  RS01_Init();

  if (RS01_SpeedInit(&Motor_left, 2000U) != HAL_OK) Error_Handler();
  if (RS01_SpeedInit(&Motor_right, 2000U) != HAL_OK) Error_Handler();

  /* 初始化成功后才启动位置控制中断。 */
  MX_TIM6_Init();
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */

  /* 本次实测参数：位置环使用P控制，速度和电流环由电机内部执行。 */
  Motor_left.position_kp = 1.5f;
  Motor_right.position_kp = 1.5f;
  step = 0U;
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    /* 上电测试：正转2圈 -> 两边完成 -> 反转2圈 -> 两边完成，不自动重复。 */
    /* 任一电机故障只暂停后续命令；另一电机仍由定时器独立执行已有任务。 */
    if (Motor_left.state == RS01_FAULT ||
        Motor_right.state == RS01_FAULT)
    {
        HAL_Delay(1U);
        continue;
    }

    switch (step)
    {
    case 0:
        // 接受任务后才推进阶段；不在主循环中等待运动结束
        rs01_debug.left_start_status = RS01_StartMove(&Motor_left, 2.0f, 2.5f);
        if (rs01_debug.left_start_status == HAL_OK)
            step = 1U;
        break;

    case 1:
        // 左右电机独立启动，相同正圈数已在库内进行镜像方向转换
        rs01_debug.right_start_status = RS01_StartMove(&Motor_right, 2.0f, 2.5f);
        if (rs01_debug.right_start_status == HAL_OK)
            step = 2U;
        break;

    case 2:
        // 两边都完成正转后，才能开始反转
        if (Motor_left.state == RS01_DONE &&
            Motor_right.state == RS01_DONE)
            step = 3U;
        break;

    case 3:
        // 负圈数表示相对当前位置反转，不是指定绝对位置
        rs01_debug.left_start_status = RS01_StartMove(&Motor_left, -2.0f, 2.5f);
        if (rs01_debug.left_start_status == HAL_OK)
            step = 4U;
        break;

    case 4:
        // 右电机的反转命令也只登记一次
        rs01_debug.right_start_status = RS01_StartMove(&Motor_right, -2.0f, 2.5f);
        if (rs01_debug.right_start_status == HAL_OK)
            step = 5U;
        break;

    case 5:
        // 等待两边反转结束，期间位置环仍在TIM6中断执行
        if (Motor_left.state == RS01_DONE &&
            Motor_right.state == RS01_DONE)
            step = 6U;
        break;

    case 6:
        // 正反转测试完成，不再下发命令
        break;
    }

    HAL_Delay(1U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 32;
  RCC_OscInitStruct.PLL.PLLN = 129;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* TIM6：当前 APB1 不分频，定时器时钟等于 PCLK1。 */
static void MX_TIM6_Init(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = timer_clock / 10000U - 1U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 100U - 1U;  // 10kHz / 100 = 100Hz，即10ms
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);

    /* CAN接收优先级为5；控制中断为6，接收可以抢占控制。 */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* 这里只运行非阻塞位置控制，禁止延时、等待回复或重新初始化电机。 */
        RS01_PositionTask(&Motor_left);
        RS01_PositionTask(&Motor_right);
    }
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
