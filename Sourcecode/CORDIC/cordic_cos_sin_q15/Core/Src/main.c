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
#include "hrtim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi  	3.1415926535897932384626433

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**
 * @name q15_to_f16
 * @brief convert a q1.15 notation integer into a 16 bits float
 * @param x a q1.15 notation integer
 * @return floating point value to convert
 */
#define q15_to_f16(x)	ldexp((int16_t) x, -15)

/**
 * @name q31_to_f32
 * @brief convert a q1.31 notation integer into a 32 bits float
 * @param x a q1.31 notation integer
 * @return the converted floating point value
 */

#define q31_to_f32(x)	ldexp((int32_t) x, -31)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int16_t extime;
float cos_res_q15, sin_res_q15;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
 * @name f16_to_q15
 * @brief Convert a 16 Bits float to a q1.15 notation interger
 * @param input floating point value to convert
 * @return a q1.15 notation integer
 */
static inline int16_t f16_to_q15(double input)
{

	const float q15_max_F= 0x0.FFFp0f; // represent 0.9999847412109375
	const float q15_min_F= -1.0f;
	return  (int16_t)(roundf(scalbnf(fmaxf(fminf(input, q15_max_F), q15_min_F), 15))); /*!fminf give the min value between input and q15_max
																				fmaxf gives the max betwen the result for fminf and q15_min_F
																				scalbnf multiply the result of fmaxf() with 2^15. This is used to convert a float value
																				in a specified range into an integer representation with a certain scale.
																				roundf() This function rounds the previous result to the nearest integer.
																				This converts the scaled float value to a whole number, taking rounding into account.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */

}



static inline  HAL_StatusTypeDef config_cos_sinf_q15()
{
  HAL_StatusTypeDef status;
  CORDIC_ConfigTypeDef  sConfig;
  sConfig.Function = CORDIC_FUNCTION_COSINE; //Function Cosine
  sConfig.InSize = CORDIC_INSIZE_16BITS;      /* !< 16 bits input data size (Q1.15 format)*/
  sConfig.OutSize = CORDIC_OUTSIZE_16BITS; /*  !< 16 bits output data size (Q1.15 format)*/
  sConfig.NbWrite = CORDIC_NBWRITE_1; 		/*!< One 32-bits write containing either only one 3 2-bit data input (Q1.31 format), or two 16-bit data input (Q1.15 format) packed in one 32 bits Data*/
  sConfig.NbRead = CORDIC_NBREAD_1; 	/*!< One 32-bits read containing either only one  32-bit data output (Q1.31 format), or two 16-bit data output (Q1.15 format) packed in one 32 bits  Data*/
  sConfig.Scale= CORDIC_SCALE_0; 	/*! No Scaling*/
  sConfig.Precision = CORDIC_PRECISION_6CYCLES ; /*! Precision 6 cycles: 24 iterations*/
  status =HAL_CORDIC_Configure(&hcordic, &sConfig);
 return status;
}

/**
 * @name cordic_cos_sinf_q15
 * @brief calculate the cosine and sine of angle  a q1.31 notation integer into a 32 bits float
 * @param angle and module will convert to a q1.15 notation integer.
 *  the two 16-bit data input (Q1.15 format) packed in one 32bits data input for the processing
 * @return a int32_t value. it content the cosine and sine Result.
 * LSB = cosine value and MSB = sine value of the calculate angle
 */
static inline void cos_sinf_q15(float angle, float* pcos, float* psin)
{

	int16_t angle_in = f16_to_q15(fmod(angle, 2.0f*pi)/ (2.0*pi))<<1;
	int16_t mod = f16_to_q15(0.9999);
	int32_t	InData= mod<<16;
		InData	|= angle_in;
	int16_t cos_res, sin_res;
	int32_t OutData;

	if(HAL_CORDIC_CalculateZO(&hcordic, &InData, &OutData, 1, 0) != HAL_OK)
		Error_Handler();

	cos_res=(int16_t) OutData;
	sin_res= (int16_t)(OutData>>16 & 0x0000FFFF);

	*pcos=q15_to_f16(cos_res);
	*psin=q15_to_f16(sin_res);

return;
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
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
  if(config_cos_sinf_q15()!=HAL_OK)
	  Error_Handler();

  float x=0;

  int16_t k=0;
  uint16_t pTime[361]={0};
  uint8_t to_send[5];
  uint8_t* c="\r\n";

  for(int i=0; i<361; i++)
  {
	  pTime[i]=0;
  }

  for(int i=0; i<361; i++)
  {
	  x = (float)((i)*pi/180.0f);	// Convert degrees to radiant
	  cos_sinf_q15(x, &cos_res_q15, &sin_res_q15);
	  pTime[k]=extime;
	  k++;
	  HAL_Delay(5);
  }


  for(int j=0; j<361; j++)
   {
 	  sprintf(to_send,"%d", pTime[j]);
 	  HAL_UART_Transmit(&huart2, to_send, 5, 5);
 	  HAL_Delay(3);
 	  HAL_UART_Transmit(&huart2, c, 2, 2);
 	  HAL_Delay(3);
   }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	 /* for(int i=0; i<181; i++)
	  {
		  x = (float)((i)*pi/180.0f);	// Convert degrees to radiant
		  cos_sinf_q15(x, &cos_res_q15, &sin_res_q15);
		  HAL_Delay(5);
	  }*/
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
