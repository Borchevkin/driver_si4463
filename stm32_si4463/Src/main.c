/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "si4463.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
si4463_t si4463;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
bool SI4463_IsCTS(void);
void SI4463_WriteRead(uint8_t * pTxData, uint8_t * pRxData, uint16_t sizeTxData);
void SI4463_SetShutdown(void);
void SI4463_ClearShutdown(void);
void SI4463_Select(void);
void SI4463_Deselect(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */

  /* Assign functions */
  si4463.IsCTS = SI4463_IsCTS;
  si4463.WriteRead = SI4463_WriteRead;
  si4463.Select = SI4463_Select;
  si4463.Deselect = SI4463_Deselect;
  si4463.SetShutdown = SI4463_SetShutdown;
  si4463.ClearShurdown = SI4463_ClearShutdown;
  si4463.DelayMs = HAL_Delay;
  /* Init Si4463 with structure */
  SI4463_Init(&si4463);
  SI4463_StartRx(&si4463);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //uint8_t testMessage[7] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7};
	  //SI4463_WriteTxFifo(&si4463, testMessage, 7);
	  //SI4463_StartTx(&si4463);

	  HAL_Delay(100);

	  uint8_t incoming[64] = {0};
	  memset(incoming, 0x00, 64);
	  SI4463_ReadRxFifo(&si4463, incoming, 64);

	  uint8_t currentState = 0;
	  SI4463_GetCurrentState(&si4463, &currentState);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(nSEL_GPIO_Port, nSEL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_ONBOARD_Pin */
  GPIO_InitStruct.Pin = LED_ONBOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_ONBOARD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CTS_Pin */
  GPIO_InitStruct.Pin = CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CTS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHUTDOWN_Pin */
  GPIO_InitStruct.Pin = SHUTDOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHUTDOWN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nIRQ_Pin */
  GPIO_InitStruct.Pin = nIRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(nIRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : nSEL_Pin */
  GPIO_InitStruct.Pin = nSEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(nSEL_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint8_t buffer[SI4463_CMD_BUF_LEN];
  memset(buffer, 0x00, SI4463_CMD_BUF_LEN);
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* Get interrupts and work with it */
  SI4463_GetInterrupts(&si4463);

  /* Handling PH interrupts */
  if (si4463.interrupts.filterMatch)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.filterMatch = false;
  }
  if (si4463.interrupts.filterMiss)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.filterMiss = false;
  }
  if (si4463.interrupts.packetSent)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.packetSent = false;
	  /*Toggle led for indication*/
	  HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
  }
  if (si4463.interrupts.packetRx)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.packetRx = false;
  }
  if (si4463.interrupts.crcError)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.crcError = false;
  }
  if (si4463.interrupts.txFifoAlmostEmpty)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.txFifoAlmostEmpty = false;
  }
  if (si4463.interrupts.rxFifoAlmostFull)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rxFifoAlmostFull = false;
	  /*Toggle led for indication*/
	  HAL_GPIO_TogglePin(LED_ONBOARD_GPIO_Port, LED_ONBOARD_Pin);
  }

  /* Handling Modem interrupts */
  if (si4463.interrupts.postambleDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.postambleDetect = false;
  }
  if (si4463.interrupts.invalidSync)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.invalidSync = false;
  }
  if (si4463.interrupts.rssiJump)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rssiJump = false;
  }
  if (si4463.interrupts.rssi)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.rssi = false;
  }
  if (si4463.interrupts.invalidPreamble)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.invalidPreamble = false;
  }
  if (si4463.interrupts.preambleDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.preambleDetect = false;
  }
  if (si4463.interrupts.syncDetect)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.syncDetect = false;
  }

  /* Handling Chip interrupts */
  if (si4463.interrupts.cal)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.cal = false;
  }
  if (si4463.interrupts.fifoUnderflowOverflowError)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.fifoUnderflowOverflowError = false;
  }
  if (si4463.interrupts.stateChange)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.stateChange = false;
  }
  if (si4463.interrupts.cmdError)
  {
	  /* Handling this interrupt here */
	  SI4463_GetChipStatus(&si4463);
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.stateChange = false;
  }
  if (si4463.interrupts.chipReady)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.chipReady = false;
  }
  if (si4463.interrupts.lowBatt)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.lowBatt = false;
  }
  if (si4463.interrupts.wut)
  {
	  /* Handling this interrupt here */
	  /* Following instruction only for add breakpoints. May be deleted */
	  si4463.interrupts.wut = false;
  }

  /* GetChipStatus used for clearing Chip interrupts such CMD_ERROR
   * which cannot clear by SI4463_ClearAllInterrupts */
  SI4463_GetChipStatus(&si4463);

  /* Clear All interrupts before exit */
  SI4463_ClearAllInterrupts(&si4463);
}

bool SI4463_IsCTS(void)
{
	if(HAL_GPIO_ReadPin(CTS_GPIO_Port, CTS_Pin) == GPIO_PIN_SET)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void SI4463_WriteRead(uint8_t * pTxData, uint8_t * pRxData, uint16_t sizeTxData)
{
	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, sizeTxData, 100);
}

void SI4463_SetShutdown(void)
{
	HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_SET);
}

void SI4463_ClearShutdown(void)
{
	HAL_GPIO_WritePin(SHUTDOWN_GPIO_Port, SHUTDOWN_Pin, GPIO_PIN_RESET);
}

void SI4463_Select(void)
{
	HAL_GPIO_WritePin(nSEL_GPIO_Port, nSEL_Pin, GPIO_PIN_RESET);
}

void SI4463_Deselect(void)
{
	HAL_GPIO_WritePin(nSEL_GPIO_Port, nSEL_Pin, GPIO_PIN_SET);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
