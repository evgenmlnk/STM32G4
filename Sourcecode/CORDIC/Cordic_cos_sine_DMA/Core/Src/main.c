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
#include "cordic.h"
#include "dma.h"
#include "hrtim.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi  	3.1415926535897932384626433


#define TIMER_PERIOD	65503

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/**
 * @name q31_to_f32
 * @brief convert a q1.31 notation integer into a 32 bits float
 * @param x a q1.31 notation integer
 * @return floating point value to convert
 */

#define q31_to_f32(x)	ldexp((int32_t) x, -31)

/**
 * @name f32_to_q31
 * @brief convert a f32 bits float  notation  into a q1.31 notation integer
 * @param x a q1.31 notation integer
 * @return floating point value to convert
 */

static inline int32_t f32_to_q31(double input)
{
	const float q31_max_F= 0x0.FFFFFFp0f; // represent 0.9999847412109375
	const float q31_min_F= -1.0f;
	return (int32_t)roundf(scalbnf(fmaxf(fminf(input, q31_max_F),q31_min_F), 31)); /*!fminf give the min value between input and q31_max
																				fmaxf gives the max betwen the result for fminf and q31_min_F
																				scalbnf multiply the result of fmaxf() with 2^31. This is used to convert a float value
																				in a specified range into an integer representation with a certain scale.
																				roundf() This function rounds the previous result to the nearest integer.
																				This converts the scaled float value to a whole number, taking rounding into account.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int32_t arr_of_angles[361];
static int32_t alpha_arr[2*361];
float cordic_cos_dma_result=0.0f, cordic_sine_dma_result=0.0f;
volatile uint16_t  start_time, overflow_count, start_overflow_count, end_time1;
uint32_t execution_time;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */


HAL_StatusTypeDef cordic_cos_sin_array_dma(int32_t *angles, int32_t* results, size_t size)
{
	CORDIC_ConfigTypeDef sConfig;
	HAL_StatusTypeDef status =HAL_OK;
	uint16_t end_time=0, end_overflow_count=0, total_overflow;

	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_2;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	if((status = HAL_CORDIC_Configure(&hcordic, &sConfig)) != HAL_OK)
	{
		return status;
	}


	if((status = HAL_CORDIC_Calculate_DMA(&hcordic, angles, results,
			size, CORDIC_DMA_DIR_IN_OUT)) != HAL_OK)
	{
		return status;
	}

	//check the current state of the the peripheral
	while(HAL_CORDIC_GetState(&hcordic) != HAL_CORDIC_STATE_READY)
	{

	}
	end_time= (uint16_t) hhrtim1.Instance->sTimerxRegs->CNTxR;
	end_overflow_count =overflow_count;

	total_overflow = end_overflow_count - start_overflow_count;
	execution_time =total_overflow*(TIMER_PERIOD +1) + end_time - start_time;

	/*if(end < start_time)
	{
		cos_sine_dma_exe_time= TIMER_PERIOD -start_time +end +1;
	}
	else
	{
		cos_sine_dma_exe_time = end -start_time;
	}*/

	return status;
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
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_HRTIM1_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start(&htim2);

  HAL_HRTIM_SimpleBaseStart_IT(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);

  for(int i=0; i <361; i++)
  {
	  arr_of_angles[i]= f32_to_q31((double) (i)/360.0f) <<1;
  }

  if(cordic_cos_sin_array_dma(arr_of_angles, alpha_arr, 180) !=HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for(int i=0; i < 2*361; i +=2)
	  {
		  cordic_cos_dma_result= q31_to_f32(alpha_arr[i])*5;
		  cordic_sine_dma_result = q31_to_f32(alpha_arr[i+1])*5;
		  HAL_Delay(5);
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

/* USER CODE BEGIN 4 */
/*
 *  @brief  Callback function invoked when the timer counter matches the value
  *         programmed in the compare 1 register
  * @param  hhrtim pointer to HAL HRTIM handle
  * @param  TimerIdx Timer index
 * */

void HAL_HRTIM_Compare1EventCallback(HRTIM_HandleTypeDef * hhrtim,
                                             uint32_t TimerIdx)
{
	overflow_count++;
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
