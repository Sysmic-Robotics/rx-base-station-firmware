/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "nrf24.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum {
  SEND_FLAG_RESET = 0,
  SEND_FLAG_SET
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_GPIO_PORT           GPIOC
#define LED_GPIO_PIN            GPIO_PIN_13

#define nRF24L01_SYSMIC_CHANNEL 0x6B
#define TX_ONLY         0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
uint8_t serial_data[32]="Rx Basestation ON              \n";
volatile uint32_t serial_len;
volatile uint8_t send_flag = 0;

uint8_t status;

nRF24_Handler_t rx_device;
uint8_t rx_len = 32;

uint8_t tx_node_addr[5] = {'s', 'y', 's', 't', 'x'}; //ref in ssl robot
uint8_t rx_node_addr[5] = {'s', 'y', 's', 'r', 'x'};

uint8_t debug_reg[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */



  nRF24_HW_Init(&rx_device, &hspi1, GPIOB, GPIO_PIN_0, GPIOA, GPIO_PIN_4);
  nRF24_Init(&rx_device);
  nRF24_SetRFChannel(&rx_device, nRF24L01_SYSMIC_CHANNEL);
  nRF24_ReadMBReg(&rx_device, nRF24_REG_RX_ADDR_P0, debug_reg, 5);


  tx_node_addr[4]=0;
  nRF24_SetAddr(&rx_device, nRF24_PIPE0, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE0, nRF24_AA_OFF, 32);

  tx_node_addr[4]=1;
  nRF24_SetAddr(&rx_device, nRF24_PIPE1, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE1, nRF24_AA_OFF, 32);

  tx_node_addr[4]=2;
  nRF24_SetAddr(&rx_device, nRF24_PIPE2, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE2, nRF24_AA_OFF, 32);

  tx_node_addr[4]=3;
  nRF24_SetAddr(&rx_device, nRF24_PIPE3, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE3, nRF24_AA_OFF, 32);

  tx_node_addr[4]=4;
  nRF24_SetAddr(&rx_device, nRF24_PIPE4, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE4, nRF24_AA_OFF, 32);

  tx_node_addr[4]=5;
  nRF24_SetAddr(&rx_device, nRF24_PIPE5, tx_node_addr);
  nRF24_SetRXPipe(&rx_device, nRF24_PIPE5, nRF24_AA_OFF, 32);



  nRF24_DisableAA(&rx_device, nRF24_PIPETX);
  nRF24_SetPowerMode(&rx_device, nRF24_PWR_UP);
  nRF24_SetOperationalMode(&rx_device, nRF24_MODE_RX);
  nRF24_RX_ON(&rx_device);


  // Link rx_buffer address to UART-DMARX
	HAL_UART_Receive_DMA(&huart1, serial_data, 32);
	__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT | DMA_IT_TC);
	__HAL_DMA_DISABLE(huart1.hdmarx);

	// LISR must be cleared before re-enabling DMA stream
	__HAL_DMA_CLEAR_FLAG(huart1.hdmarx, DMA_FLAG_GL3);
	huart1.hdmarx->Instance->CNDTR = 32;

	// Re-enable UART to listen Nextion again
	__HAL_DMA_ENABLE(huart1.hdmarx);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);


  nRF24_ReadMBReg(&rx_device, nRF24_REG_TX_ADDR, debug_reg, 5);
 // nRF24_ReadMBReg(&tx_device, nRF24_REG_TX_ADDR, debug_reg, 5);



  send_flag = SEND_FLAG_RESET;

  //HAL_UART_Transmit(&huart1, rx_device.rx_data, rx_len, HAL_MAX_DELAY);

  nRF24_FlushRX(&rx_device);
	nRF24_ClearIRQFlags(&rx_device);

	HAL_UART_Transmit(&huart1, serial_data, rx_len, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	status = nRF24_GetStatus(&rx_device);
	
	if (status & nRF24_FLAG_RX_DR) {
    // Leer todos los paquetes disponibles
    while (nRF24_ReadPayload(&rx_device, rx_device.rx_data, &rx_len) != nRF24_RX_EMPTY) {
        // Procesar el contenido de rx_device.rx_data
        HAL_UART_Transmit(&huart1, rx_device.rx_data, rx_len, HAL_MAX_DELAY);
    }

    // Al terminar de leer todos los paquetes, limpiar las banderas e, incluso, hacer flush si lo deseas
    nRF24_FlushRX(&rx_device);
    nRF24_ClearIRQFlags(&rx_device);
  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  huart1.Init.BaudRate = 460800;//115200;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	__HAL_DMA_DISABLE(huart->hdmarx);

	// Store length of message before resetting NDTR
	serial_len = 30 - huart->hdmarx->Instance->CNDTR;
	
	// LISR must be cleared before re-enabling DMA stream
  __HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAG_GL3);
	huart->hdmarx->Instance->CNDTR = 30;

	// Re-enable UART
	__HAL_DMA_ENABLE(huart->hdmarx);
	send_flag = SEND_FLAG_SET;
}
/* USER CODE END 4 */

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
    NVIC_SystemReset(); // Reset the system to recover from error
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
