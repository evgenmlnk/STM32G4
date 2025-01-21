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
#include "tim.h"
#include "usart.h"
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float cordic_phase=0.0f, cordic_modulus=0.0f;
uint32_t phasef_exTime, mod_exTime;
volatile  uint16_t Extime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/**
 * @name f32_to_q31
 * @brief Convert a 32 Bits float to a q1.31 notation interger
 * @param input floating point value to convert
 * @return a q1.15 notation integer
 */

static inline int f32_to_q31(double input)
{
	const float q31_max_F= 0x0.FFFFFFp0f; // represent 0.9999847412109375
	const float q31_min_F= -1.0f;
	return (int)roundf(scalbnf(fmaxf(fminf(input, q31_max_F),q31_min_F), 31)); /*!fminf give the min value between input and q31_max
																				fmaxf gives the max betwen the result for fminf and q31_min_F
																				scalbnf multiply the result of fmaxf() with 2^31. This is used to convert a float value
																				in a specified range into an integer representation with a certain scale.
																				roundf() This function rounds the previous result to the nearest integer.
																				This converts the scaled float value to a whole number, taking rounding into account.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */
}

/**
 * @name q31_to_f32
 * @brief convert a q1.31 notation integer into a 32 bits float
 * @param x a q1.31 notation integer
 * @return floating point value to convert
 */
#define q31_to_f32(x)	ldexp((int32_t) x, -31)

static inline HAL_StatusTypeDef  config_phasef()
{

	CORDIC_ConfigTypeDef  sConfig;
	HAL_StatusTypeDef status;

	sConfig.Function = CORDIC_FUNCTION_PHASE; //Function Phase

	sConfig.InSize = CORDIC_INSIZE_32BITS;       /*!< 32 bits input data size (Q1.31 format) */

	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  /*!< 32 bits output data size (Q1.31 format) */

	sConfig.NbWrite = CORDIC_NBWRITE_2; 		/*!< Two 32-bits write containing either only one
													 32-bit data input (Q1.31 format), or two 16-bit
													  data input (Q1.15 format) packed in one 32 bits Data */

	sConfig.NbRead = CORDIC_NBREAD_1; 	/*!< One 32-bits read containing either only one
												  32-bit data output (Q1.31 format), or two 16-bit
												  data output (Q1.15 format) packed in one 32 bits
												  Data */
	sConfig.Scale= CORDIC_SCALE_0; 	/*! No Scaling */
	sConfig.Precision = CORDIC_PRECISION_6CYCLES ; /*! Precision 6 cycles: 24 iterations */

	status =HAL_CORDIC_Configure(&hcordic, &sConfig);

	return status;

}

static inline float phasef(float x, float y)
{
	int32_t inBuffer_q31[2], outBuffer_q31;

	inBuffer_q31[0] = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1;
	inBuffer_q31[1] = f32_to_q31(fmod(y, 2.0f*pi) / (2.0f*pi)) <<1;

	if(HAL_CORDIC_CalculateZO(&hcordic, inBuffer_q31, &outBuffer_q31, 1, 0)!=HAL_OK)
		Error_Handler();

	return (q31_to_f32(outBuffer_q31)*pi);
}

/*static inline float cordic_modulusf(float x_par, float y_par)
{
	int32_t inBuffer_q31[2], outBuffer_q31;

   inBuffer_q31[0] = f32_to_q31(fmod(x_par, 2.0f*pi) / (2.0f*pi)) <<1;
   inBuffer_q31[1] = f32_to_q31(fmod(y_par, 2.0f*pi) / (2.0f*pi)) <<1;


	CORDIC_ConfigTypeDef  sConfig;

	sConfig.Function = CORDIC_FUNCTION_MODULUS; //Function Modulus

	sConfig.InSize = CORDIC_INSIZE_32BITS;       !< 32 bits input data size (Q1.15 format)

	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  !< 32 bits output data size (Q1.15 format)

	sConfig.NbWrite = CORDIC_NBWRITE_2; 		!< One 32-bits write containing either only one
													 32-bit data input (Q1.31 format), or two 16-bit
													  data input (Q1.15 format) packed in one 32 bits Data

	sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one
												  32-bit data output (Q1.31 format), or two 16-bit
												  data output (Q1.15 format) packed in one 32 bits
												  Data
	sConfig.Scale= CORDIC_SCALE_0; 	! No Scaling
	sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations

	HAL_CORDIC_Configure(&hcordic, &sConfig);

	if(HAL_CORDIC_CalculateZO(&hcordic, inBuffer_q31, &outBuffer_q31, 1, 0)==HAL_OK)
		mod_exTime = Extime;
	else
		Error_Handler();

	return q31_to_f32(outBuffer_q31);

}*/
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
  /*float x, y;*/


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
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
 // HAL_TIM_Base_Start(&htim2);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
  config_phasef();

  /*uint16_t pExeTime[100];
  uint16_t Sqrt_Time[6];*/
  float x,y;
 //int k=0;

    uint8_t to_send[6];
    uint8_t* c="\r\n";

	/*for(int i=0; i <28; i+=3)
	  {
		  x = ((float)(i)*2.0f*pi)/360.0f;
		  for(int j =0; j<28; j+=3)
		  {
			  y = ((float)(j)*2.0f*pi)/360.0f;

			  cordic_phase = phasef(x,y);
			  pExeTime[k]=Extime;
			  k++;
			  //cordic_modulus= cordic_modulusf(x, y);
			  HAL_Delay(3);
		  }
	  }*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  for(int i=0; i <27; i+=3)
		  {
			  x = ((float)(i)*2.0f*pi)/360.0f;
			  for(int j =0; j<27; j+=3)
			  {
				  y = ((float)(j)*2.0f*pi)/360.0f;

				  cordic_phase = phasef(x,y);
				  //cordic_modulus= cordic_modulusf(x, y);
				  HAL_Delay(10);
			  }
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
