/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "input_signal.h"
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
FMAC_HandleTypeDef hfmac;
DMA_HandleTypeDef hdma_fmac_write;
DMA_HandleTypeDef hdma_fmac_read;
DMA_HandleTypeDef hdma_fmac_preload;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* declare a filter configuration structure */

FMAC_FilterConfigTypeDef sFmacConfig;


static int16_t aInputValues_q15[BLOCK_SIZE];
const uint16_t CurrentInputArraySize = BLOCK_SIZE;


static int16_t aCalculatedFilteredData_q15[BLOCK_SIZE];

#ifdef DATA_VALIDATION
	static int16_t aCalculatedFilteredData_Validation_q15[INPUT_SIGNAL_SIZE];
	volatile int16_t output;
#endif



const uint16_t ExpectedCalculatedFilteredDataSize = BLOCK_SIZE;

static int16_t aFilterPreloadValues_q15[PRELOAD_SIZE] = {0};

/* Status of the calculation */
__IO uint32_t FilterConfigCallbackCount    = 0;
__IO uint32_t FilterPreloadCallbackCount   = 0;
__IO uint32_t HalfGetDataCallbackCount     = 0;
__IO uint32_t GetDataCallbackCount         = 0;
__IO uint32_t OutputDataReadyCallbackCount = 0;
__IO uint32_t ErrorCount                   = 0;

uint32_t OldValue;
uint16_t currentBlock = 0;
uint16_t i;

uint32_t frameCout;

#ifdef PERFORMANCE_MEASUREMENT
volatile uint32_t dwtCycleCountStart;
volatile uint32_t dwtCycleCountStop;
volatile uint32_t dwtCycleCount;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FMAC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
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

#ifdef PERFORMANCE_MEASUREMENT
    // Enable the DWT unit
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // Enable the cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // Reset the cycle counter
    DWT->CYCCNT = 0;

#endif
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
  MX_FMAC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* Fill first frame with input signal and zero the other frame */
  /* Normally samples would come from ADC or other source, in this example the */
  /* same data is reloaded from Flash */

  for (i = 0; i<BLOCK_SIZE; i++)
  {
	  aInputValues_q15[i] = aInputSignal_q15[i];
  }

  /* Configure the FMAC */
  /* Set the coefficient buffer base address */
  sFmacConfig.CoeffBaseAddress = 0;
  /* Set the coefficient buffer size to the number of coeffs */
  sFmacConfig.CoeffBufferSize = NO_OF_TAPS;
  /* Set the Input buffer base address to the next free address */
  sFmacConfig.InputBaseAddress = NO_OF_TAPS;
  /* Set the input buffer size greater than the number of coeffs */
  sFmacConfig.InputBufferSize = X1_BUFF_SIZE;
  /* Set the input watermark to zero since we are using DMA */
  sFmacConfig.InputThreshold = 0;
  /* Set the Output buffer base address to the next free address */
  sFmacConfig.OutputBaseAddress = NO_OF_TAPS + X1_BUFF_SIZE;
  /* Set the output buffer size */
  sFmacConfig.OutputBufferSize = Y_BUFF_SIZE;
  /* Set the output watermark to zero since we are using DMA */
  sFmacConfig.OutputThreshold = 0;
  /* No A coefficients since FIR */
  sFmacConfig.pCoeffA = NULL;
  sFmacConfig.CoeffASize = 0;
  /* Pointer to the coefficients in memory */
  sFmacConfig.pCoeffB = aFirCoeff_q15;
  /* Number of coefficients */
  sFmacConfig.CoeffBSize = NO_OF_TAPS;
  /* Select FIR filter function */
  sFmacConfig.Filter = FMAC_FUNC_CONVO_FIR;
  /* Enable DMA input transfer */
  sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_DMA;
  /* Enable DMA output transfer */
  sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_DMA;
  /* Enable clipping of the output at 0x7FFF and 0x8000 */
  sFmacConfig.Clip = FMAC_CLIP_ENABLED;
  /* P parameter contains number of coefficients */
  sFmacConfig.P = NO_OF_TAPS;
  /* Q parameter is not used */
  sFmacConfig.Q = 0;
  /* R parameter contains the post-shift value (none) */
  sFmacConfig.R = 0;

  __HAL_RCC_FMAC_CLK_ENABLE();

  /*### Main loop repeats each frame ########################################*/
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /* Configure the FMAC */
  if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
  /* Configuration Error */
  Error_Handler();



  if (HAL_FMAC_FilterPreload_DMA(&hfmac,
  aFilterPreloadValues_q15, (PRELOAD_SIZE), NULL, 0) != HAL_OK)
  Error_Handler();


  while(FilterPreloadCallbackCount == 0);

  /*  Before starting a new process, you need to check the current state of the peripheral;
	  if it's busy you need to wait for the end of current transfer before starting the calculation.
	  For simplicity reasons, this example is just waiting till the end of the
	  process, but the application may perform other tasks while the transfer is ongoing. */
  while (HAL_FMAC_GetState(&hfmac) != HAL_FMAC_STATE_READY)
  {
  }

  /* Start calculation of FIR filter in DMA mode */
  if (HAL_FMAC_FilterStart(&hfmac, aCalculatedFilteredData_q15, &ExpectedCalculatedFilteredDataSize) != HAL_OK)
  {
	/* Processing Error */
	Error_Handler();
		  }

  /*## Append data to start the DMA process after the preloaded data handling ##*/
   if (HAL_FMAC_AppendFilterData(&hfmac,
							   aInputValues_q15,
							   &CurrentInputArraySize) != HAL_OK)
   {
	 ErrorCount++;
   }

   currentBlock++;


	  /* Repeat until required number of frames have been processed */
	  do {

		    /* While processing is going on, fill next frame with input signal from Flash */
		   while(HalfGetDataCallbackCount == OldValue);
		   while(GetDataCallbackCount == OldValue){
			  for (i = 0; i < HALF_BLOCK_SIZE; i++)
			  {
				  /*write new data */
				  aInputValues_q15[i] = aInputSignal_q15[i+BLOCK_SIZE*currentBlock];
				#ifdef DATA_VALIDATION
				  /*store data for validation*/
				  aCalculatedFilteredData_Validation_q15[i+BLOCK_SIZE*(currentBlock-1)] = aCalculatedFilteredData_q15[i];
				#endif
			  }

		   }

			   /* Wait for FMAC to finish processing frame */
			   while (OutputDataReadyCallbackCount == OldValue);
				  for (i = HALF_BLOCK_SIZE; i<BLOCK_SIZE; i++)
				  {
					  /*write new data */
					  aInputValues_q15[i] = aInputSignal_q15[i+BLOCK_SIZE*currentBlock];

					 #ifdef DATA_VALIDATION
					  /*store data for validation*/
					 aCalculatedFilteredData_Validation_q15[i+BLOCK_SIZE*(currentBlock-1)] = aCalculatedFilteredData_q15[i];
					#endif
				  }

				 OldValue = OutputDataReadyCallbackCount;
				 currentBlock++;

				  /* end of loop */
		  } while(OutputDataReadyCallbackCount < NO_OF_BLOCKS);
		  {
		    if(ErrorCount != 0)
		    {
		      /* Processing Error */
		      Error_Handler();
		    }
		  }


		   /* Stop the calculation of FIR filter in polling/DMA mode */
			if (HAL_FMAC_FilterStop(&hfmac) != HAL_OK)
			{
			  /* Processing Error */
			  Error_Handler();
			}

		  /*## Check the final error status ############################################*/
		  if(ErrorCount != 0)
		  {
		    /* Processing Error */
		    Error_Handler();
		  }

		  /* Reached the end of processing : Turn LED2 on */
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

#ifdef DATA_VALIDATION
/*print filter output */
	for(i = 0; i < INPUT_SIGNAL_SIZE; i++)
	{
		// assign to global var to print in STM32CubeMonitor
	  output = aCalculatedFilteredData_Validation_q15[i];
	  // print via UART
	  printf("%d\r\n", output);
	  HAL_Delay(1);
	}
#endif

#ifdef PERFORMANCE_MEASUREMENT

		  for(i = 0; i < NO_OF_BLOCKS; i++)
		  {
			 dwtCycleCount = aDWTCycleCount[i];
			 printf("%lu\r\n", dwtCycleCount);
			 HAL_Delay(1);

		  }

#endif







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 28;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_FMAC_FilterPreloadCallback(FMAC_HandleTypeDef *hfmac)
{
  FilterPreloadCallbackCount++;;
}


void HAL_FMAC_HalfGetDataCallback(FMAC_HandleTypeDef *hfmac)
{
  HalfGetDataCallbackCount++;
}


void HAL_FMAC_GetDataCallback(FMAC_HandleTypeDef *hfmac)
{
#ifdef DATA_VALIDATION
	if(GetDataCallbackCount==NO_OF_BLOCKS-1){
		DMA1_Channel2->CCR &= ~DMA_CCR_EN; // Disable the DMA channel
	}
#endif

  GetDataCallbackCount++;
}


void HAL_FMAC_OutputDataReadyCallback(FMAC_HandleTypeDef *hfmac)
{
#ifdef DATA_VALIDATION
	if(OutputDataReadyCallbackCount==NO_OF_BLOCKS){
		DMA1_Channel3->CCR &= ~DMA_CCR_EN; // Disable the DMA channel
	}
#endif

#ifdef PERFORMANCE_MEASUREMENT

	dwtCycleCountStop = DWT->CYCCNT;

	  aDWTCycleCount[currentBlock-1]  = dwtCycleCountStop - dwtCycleCountStart;
	  dwtCycleCountStart = dwtCycleCountStop;
#endif

	OutputDataReadyCallbackCount++;

}

/**
  * @brief FMAC error callback
  * @par hfmac: FMAC HAL handle
  * @retval None
  */
void HAL_FMAC_ErrorCallback(FMAC_HandleTypeDef *hfmac)
{
  ErrorCount++;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  *   None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
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
