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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
/* 测试用：记录 IO 翻转次数，方便调试时观察 */
volatile uint32_t debug_1_toggle_cnt = 0;

/* USART1 测试相关变量 */
static uint8_t usart1_rx_byte = 0;           /* 中断接收用的单字节缓冲 */
volatile uint32_t usart1_rx_cnt = 0;         /* 累计接收字节数 */
volatile uint32_t usart1_last_byte = 0;      /* 最近一次收到的字节 */
volatile uint32_t usart1_err_cnt = 0;        /* 串口错误计数 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * printf 重定向：printf() -> USART1
 * 说明：syscalls.c 中的 _write() 会调用 __io_putchar()，这里实现它即可。
 *       使用阻塞发送，简单可靠，适合调试输出。
 */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/* 启动 USART1 单字节中断接收（用于回显测试） */
static void USART1_StartRxIT(void)
{
  HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
}

/* USART1 接收完成回调：回显收到的字节，并重新开启下一次接收 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    usart1_rx_cnt++;
    usart1_last_byte = usart1_rx_byte;

    /* 回显：原样发回 PC */
    HAL_UART_Transmit(&huart1, &usart1_rx_byte, 1, 10);

    /* 重新开启下一字节接收 */
    HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
  }
}

/* USART1 错误回调：清除错误标志并重新开启接收，避免接收停摆 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    usart1_err_cnt++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
  }
}

/*
 * USART1 测试主函数（在 while(1) 中周期调用）
 * 测试内容：
 *   1. 启动时用 printf 打印横幅和串口配置
 *   2. 每秒 printf 一次运行信息（验证 TX 阻塞发送）
 *   3. 每 3 秒用 DMA 发送一段字符串（验证 DMA TX）
 *   4. 收到任意字符原样回显（验证 RX 中断接收）
 */
static void USART1_Test(void)
{
  static uint32_t last_tick = 0;
  static uint32_t cnt = 0;
  static const uint8_t dma_msg[] = "USART1 DMA TX test OK\r\n";
  uint32_t now = HAL_GetTick();

  /* 1) 上电初始化一次 */
  if (cnt == 0U)
  {
    printf("\r\n===== USART1 Test Start =====\r\n");
    printf("USART1: PB6(TX) / PB7(RX), 115200 8N1, remap ON\r\n");
    printf("Send any character, it will be echoed back.\r\n");
    printf("==============================\r\n");
    USART1_StartRxIT();
    last_tick = now;
  }

  /* 2) 每 1s 用 printf 打印一次状态 */
  if ((now - last_tick) >= 1000U)
  {
    last_tick = now;
    cnt++;

    printf("[%lu] printf OK, tick=%lu, rx_cnt=%lu, last=0x%02lX, err=%lu\r\n",
           (unsigned long)cnt,
           (unsigned long)now,
           (unsigned long)usart1_rx_cnt,
           (unsigned long)usart1_last_byte,
           (unsigned long)usart1_err_cnt);

    /* 3) 每 3s 用 DMA 方式发送一段字符串 */
    if ((cnt % 3U) == 0U)
    {
      /* 等待上一次发送完成，避免与 printf 抢占 TX */
      if (huart1.gState == HAL_UART_STATE_READY)
      {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dma_msg, sizeof(dma_msg) - 1U);
      }
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 测试开始前先把 PA6 置为已知状态（低电平） */
  HAL_GPIO_WritePin(debug_1_GPIO_Port, debug_1_Pin, GPIO_PIN_RESET);

  /* 启动 USART1 测试（打印横幅 + 开启中断接收） */
  USART1_Test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* USART1 测试：每秒 printf 一次状态，每 3 秒 DMA 发送一次 */
    USART1_Test();

    /* PA6 每 500ms 翻转一次 -> 周期 1s，即 1Hz 方波
       - 用万用表/示波器测 PA6，应能看到 1Hz 方波
       - 或接 LED+限流电阻（PA6 -> 1k -> LED -> GND），LED 1秒闪烁一次 */
    HAL_GPIO_TogglePin(debug_1_GPIO_Port, debug_1_Pin);
    debug_1_toggle_cnt++;

    /* 每个周期读取一次输出电平，验证读写一致（仅调试用，可在观察窗口查看） */
    if (HAL_GPIO_ReadPin(debug_1_GPIO_Port, debug_1_Pin) == GPIO_PIN_SET)
    {
      /* PA6 当前为高电平：可在此加断点或置标志位 */
      __NOP();
    }
    else
    {
      /* PA6 当前为低电平 */
      __NOP();
    }

    HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(debug_1_GPIO_Port, debug_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : debug_1_Pin */
  GPIO_InitStruct.Pin = debug_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(debug_1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
