/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "memory.h"
#include "stdio.h"
#include "math.h"

#include "stm32f4xx_hal_u8g2.h"
#include "mpu6050.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  Success, Failure
} UARTControlResult;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU6050 0
#define GY955 1

#define SAMPLE_PROVIDER GY955 //角速度提供者

#define SEND_INFO 0 //是否向上位机发送状态信息

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for logicTask */
osThreadId_t logicTaskHandle;
const osThreadAttr_t logicTask_attributes = {
  .name = "logicTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayTask */
osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "displayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sampleTask */
osThreadId_t sampleTaskHandle;
const osThreadAttr_t sampleTask_attributes = {
  .name = "sampleTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for uartHandlerTask */
osThreadId_t uartHandlerTaskHandle;
const osThreadAttr_t uartHandlerTask_attributes = {
  .name = "uartHandlerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for periodTask */
osThreadId_t periodTaskHandle;
const osThreadAttr_t periodTask_attributes = {
  .name = "periodTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for btUartRxQueue */
osMessageQueueId_t btUartRxQueueHandle;
const osMessageQueueAttr_t btUartRxQueue_attributes = {
  .name = "btUartRxQueue"
};
/* Definitions for sensorUartRxQueue */
osMessageQueueId_t sensorUartRxQueueHandle;
const osMessageQueueAttr_t sensorUartRxQueue_attributes = {
  .name = "sensorUartRxQueue"
};
/* Definitions for pidStatusMutex */
osMutexId_t pidStatusMutexHandle;
const osMutexAttr_t pidStatusMutex_attributes = {
  .name = "pidStatusMutex"
};
/* USER CODE BEGIN PV */
char sprintfBuffer[64];
char uart1RxBuffer[UART1_RX_BUFFER_SIZE];
char uart6RxBuffer[UART6_RX_BUFFER_SIZE];

u8g2_t u8g2;
static MPU6050_t mpu6050;
PIDController pidX = {
    .T = 0.1f,
    .Kp = 0.025f,
    .Ti = 0.8f,
    .Td = 0.075f,
    .limMax = 1.0f,
    .limMin = -1.0f,
    .limMaxInt = 0.5f,
    .limMinInt = -0.5f,
}, pidY;

float rotX, rotY;
float thetaX, thetaY;
float targetThetaX = .0f, targetThetaY = .0f;
float vPeekX = .0f, vPeekY = .0f;
float phiX = .0f, phiY = .0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
void StartLogicTask(void *argument);
void StartPIDTask(void *argument);
void StartDisplayTask(void *argument);
void StartSampleTask(void *argument);
void StartUartHandlerTask(void *argument);
void StartPeriodControlTask(void *argument);

/* USER CODE BEGIN PFP */

void SetPIDParameters(float t, float kp, float ki, float kd) {
  pidX.T = pidY.T = t;
  pidX.Kp = pidY.Kp = kp;
  pidX.Ti = pidY.Ti = ki;
  pidX.Td = pidY.Td = kd;
}



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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of pidStatusMutex */
  pidStatusMutexHandle = osMutexNew(&pidStatusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of btUartRxQueue */
  btUartRxQueueHandle = osMessageQueueNew (1, 64, &btUartRxQueue_attributes);

  /* creation of sensorUartRxQueue */
  sensorUartRxQueueHandle = osMessageQueueNew (1, 64, &sensorUartRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of logicTask */
  logicTaskHandle = osThreadNew(StartLogicTask, NULL, &logicTask_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(StartPIDTask, NULL, &pidTask_attributes);

  /* creation of displayTask */
  displayTaskHandle = osThreadNew(StartDisplayTask, NULL, &displayTask_attributes);

  /* creation of sampleTask */
  sampleTaskHandle = osThreadNew(StartSampleTask, NULL, &sampleTask_attributes);

  /* creation of uartHandlerTask */
  uartHandlerTaskHandle = osThreadNew(StartUartHandlerTask, NULL, &uartHandlerTask_attributes);

  /* creation of periodTask */
  periodTaskHandle = osThreadNew(StartPeriodControlTask, NULL, &periodTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, uart1RxBuffer, UART1_RX_BUFFER_SIZE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

float degToRad(float degree) {
  return degree / 57.295779513f;
}

float radToDeg(float rad) {
  return rad * 57.295779513f;
}

float getPeekV(float d) {
  const float L = 0.54f, H = 0.79f, g = 9.79f;
  return radToDeg(sqrtf(2.0f) * sqrtf(g / L - sqrtf(g * g * L * L * (H * H - d * d)) / (H * L * L)));
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLogicTask */
/**
  * @brief  Function implementing the logicTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLogicTask */
void StartLogicTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for( ;; ) {
    for(float d = 0.05f; d <= 0.60f; d += 0.05f) {
      vPeekX = getPeekV(d);
      vPeekY = .0f;
      osDelayUntil(osKernelGetTickCount() + pdMS_TO_TICKS(5000));
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void *argument)
{
  /* USER CODE BEGIN StartPIDTask */
  PIDController_Init(&pidX);
  PIDController_Init(&pidY);
  pidY = pidX;

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);

  /* Infinite loop */
  for (;;) {
    osDelayUntil(osKernelGetTickCount() + pdMS_TO_TICKS((int) (pidX.T * 1000.0f)));
    if (osMutexAcquire(pidStatusMutexHandle, 100) == osOK) {
      PIDController_Update(&pidX, targetThetaX, thetaX);
      PIDController_Update(&pidY, targetThetaY, thetaY);
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t) ((float) UINT16_MAX * fmaxf(pidX.out, .0f)));
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t) ((float) UINT16_MAX * fmaxf(-pidX.out, .0f)));
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, (uint16_t) ((float) UINT16_MAX * fmaxf(-pidY.out, .0f)));
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (uint16_t) ((float) UINT16_MAX * fmaxf(pidY.out, .0f)));
      osMutexRelease(pidStatusMutexHandle);
    }
  }
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_stm32_hw_i2c, u8x8_stm32_gpio_and_delay);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetFont(&u8g2, u8g2_font_courB10_tr);
  u8g2_SetFontRefHeightExtendedText(&u8g2);
  u8g2_SetDrawColor(&u8g2, 1);
  u8g2_SetFontPosTop(&u8g2);
  u8g2_SetFontDirection(&u8g2, 0);
  /* Infinite loop */
  for (;;) {
    u8g2_FirstPage(&u8g2);
    do {
      sprintf(sprintfBuffer, "THX: %7.02f", thetaX);
      u8g2_DrawStr(&u8g2, 0, 0, sprintfBuffer);
      sprintf(sprintfBuffer, "THY: %7.02f", thetaY);
      u8g2_DrawStr(&u8g2, 0, 11, sprintfBuffer);
      sprintf(sprintfBuffer, "PIDX: %7.02f%%", pidX.out * 100.0f);
      u8g2_DrawStr(&u8g2, 0, 22, sprintfBuffer);
      sprintf(sprintfBuffer, "PIDY: %7.02f%%", pidY.out * 100.0f);
      u8g2_DrawStr(&u8g2, 0, 33, sprintfBuffer);
    } while (u8g2_NextPage(&u8g2));
    osDelay(pdMS_TO_TICKS(10));
  }

  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartSampleTask */
/**
* @brief Function implementing the sampleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSampleTask */
void StartSampleTask(void *argument)
{
  /* USER CODE BEGIN StartSampleTask */
#if SAMPLE_PROVIDER == GY955
  uint8_t txBuffer[3] = {0xAA, 0b10101100,};
  txBuffer[2] = txBuffer[0] + txBuffer[1];
  HAL_UART_Transmit(&huart6, txBuffer, sizeof(txBuffer), 100); //初始化为仅欧拉角的连续测量
  uint8_t rxBuffer[UART6_RX_BUFFER_SIZE];
  uint8_t *rxPtrEnd = rxBuffer + UART6_RX_BUFFER_SIZE;
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart6, uart6RxBuffer, UART6_RX_BUFFER_SIZE);
  for (;;) {
//    触发测量
//    txBuffer[0] = 0xA5;
//    txBuffer[1] = 0x45;
//    txBuffer[2] = txBuffer[0] + txBuffer[1];
//    HAL_UART_Transmit(&huart6, txBuffer, sizeof(txBuffer), 100);

    if (osMessageQueueGet(sensorUartRxQueueHandle, rxBuffer, NULL, 100) == osOK) { //等待串口收到数据
      uint8_t *rxPtr = rxBuffer;
      for (;;) {
        while (rxPtr < rxPtrEnd && *rxPtr++ != 0x5A); //Byte 0：0x5A 由于未知原因，在连续发送模式下接收到的第一位不一定是0x5A，需要忽略之前的所有数据
        if (*rxPtr++ != 0x5A || rxPtr + 2 >= rxPtrEnd) break; //Byte 1：0x5A
        uint8_t type = *rxPtr++; //Byte 2：类型
        size_t remainder = *rxPtr++; //Byte 3：除去校验位剩下的位数
        if (rxPtr + remainder >= rxPtrEnd) break;
        uint8_t check = 0x5A * 2 + type + remainder; // 校验位
        for (uint8_t *checkPtr = rxPtr; checkPtr < rxPtr + remainder; checkPtr++) check += *checkPtr;
        if (check != rxPtr[remainder]) continue;
        if (type & 0x01) {
          float accX = (float) (int16_t) ((rxPtr[0] << 8) | rxPtr[1]);
          float accY = (float) (int16_t) ((rxPtr[2] << 8) | rxPtr[3]);
          float accZ = (float) (int16_t) ((rxPtr[4] << 8) | rxPtr[5]);
          rxPtr += 6;
        }
        if (type & 0x02) {
          float magX = (float) (int16_t) ((rxPtr[0] << 8) | rxPtr[1]) / 16.0f;
          float magY = (float) (int16_t) ((rxPtr[2] << 8) | rxPtr[3]) / 16.0f;
          float magZ = (float) (int16_t) ((rxPtr[4] << 8) | rxPtr[5]) / 16.0f;
          rxPtr += 6;
        }
        if (type & 0x04) {
          float gyrX = (float) (int16_t) ((rxPtr[0] << 8) | rxPtr[1]) / 16.0f;
          float gyrY = (float) (int16_t) ((rxPtr[2] << 8) | rxPtr[3]) / 16.0f;
          float gyrZ = (float) (int16_t) ((rxPtr[4] << 8) | rxPtr[5]) / 16.0f;
          thetaX = gyrX;
          thetaY = gyrY;
          rxPtr += 6;
        }
        if (type & 0x08) { // 欧拉角
          float yaw = (float) (uint16_t) ((rxPtr[0] << 8) | rxPtr[1]) / 100.0f;
          float roll = (float) (int16_t) ((rxPtr[2] << 8) | rxPtr[3]) / 100.0f;
          float pitch = (float) (int16_t) ((rxPtr[4] << 8) | rxPtr[5]) / 100.0f;
          rotX = roll;
          rotY = pitch;
          rxPtr += 6;
        }
        if (type & 0x10) {
          float q1 = (float) (int16_t) ((rxPtr[0] << 8) | rxPtr[1]) / 10000.0f;
          float q2 = (float) (int16_t) ((rxPtr[2] << 8) | rxPtr[3]) / 10000.0f;
          float q3 = (float) (int16_t) ((rxPtr[4] << 8) | rxPtr[5]) / 10000.0f;
          float q4 = (float) (int16_t) ((rxPtr[6] << 8) | rxPtr[7]) / 10000.0f;
          rxPtr += 8;
        }
        rxPtr++; //跳过校验位
      }
      memset(rxBuffer, 0x00, sizeof(rxBuffer));
    }
  }
#endif
#if SAMPLE_PROVIDER == MPU6050
  while(MPU6050_Init(&hi2c1));
  for( ;; ) {
    MPU6050_Read_All(&hi2c1, &mpu6050);
    thetaX = mpu6050.Gx;
    thetaY = mpu6050.Gy;
    rotX = mpu6050.KalmanAngleX;
    rotY = mpu6050.KalmanAngleY;
    osDelayUntil(osKernelGetTickCount() + pdMS_TO_TICKS(10));
  }
#endif
  /* USER CODE END StartSampleTask */
}

/* USER CODE BEGIN Header_StartUartHandlerTask */
/**
* @brief Function implementing the uartHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartHandlerTask */
void StartUartHandlerTask(void *argument)
{
  /* USER CODE BEGIN StartUartHandlerTask */
  char rxBuffer[UART1_RX_BUFFER_SIZE];
  float T, Kp, Ki, Kd, vx, vy;
#if SEND_INFO
  uint8_t txBuffer[64];
  /* Infinite loop */
  for(;;)
  {
    uint8_t *txPtr = txBuffer;
    *txPtr++ = 0x03;
    *txPtr++ = 0xFC;
    *txPtr++ = (uint8_t)(thetaX / 180.0f * UINT8_MAX);
    *txPtr++ = (uint8_t)(thetaY / 180.0f * UINT8_MAX);
    *txPtr++ = (uint8_t)(targetThetaX / 180.0f * UINT8_MAX);
    *txPtr++ = (uint8_t)(targetThetaY / 180.0f * UINT8_MAX);
    *txPtr++ = 0xFC;
    *txPtr++ = 0x03;

    HAL_UART_Transmit(&huart1, txBuffer, txPtr - txBuffer, 10);
#else
  for (;;) {
#endif
    if (osMessageQueueGet(btUartRxQueueHandle, rxBuffer, NULL, 100) == osOK &&
        osMutexAcquire(pidStatusMutexHandle, 100) == osOK) {
      if (sscanf(rxBuffer, "%f,%f:%f,%f,%f,%f", &vx, &vy, &Kp, &Ki, &Kd, &T) == 6) {
        SetPIDParameters(T, Kp, Ki, Kd);
        vPeekX = vx;
        vPeekY = vy;
        PIDController_Init(&pidX);
        PIDController_Init(&pidY);
      }
      memset(rxBuffer, 0x00, sizeof(rxBuffer));
      osMutexRelease(pidStatusMutexHandle);
    }
  }
  /* USER CODE END StartUartHandlerTask */
}

/* USER CODE BEGIN Header_StartPeriodControlTask */
/**
* @brief Function implementing the periodTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPeriodControlTask */
void StartPeriodControlTask(void *argument)
{
  /* USER CODE BEGIN StartPeriodControlTask */
  /* Infinite loop */
  const float T = 1.333333333f, deltaT = 0.01f;
  float tx, ty;
  sin(M_PI * 2);
  for (;;) {
    osDelayUntil(osKernelGetTickCount() + pdMS_TO_TICKS((uint32_t) (deltaT * 1000)));
    tx += deltaT;
    ty += deltaT;
    if (tx > T) tx -= T;
    if (ty > T) ty -= T;
    targetThetaX = sinf(M_PI * 2.0f * tx / T + phiX) * vPeekX;
    targetThetaY = sinf(M_PI * 2.0f * ty / T + phiY) * vPeekY;
  }
  /* USER CODE END StartPeriodControlTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
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
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
