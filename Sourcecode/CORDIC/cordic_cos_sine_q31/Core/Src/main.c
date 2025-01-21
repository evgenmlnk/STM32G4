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
 * @name q31_to_f32
 * @brief convert a q1.31 notation integer into a 32 bits float
 * @param x a q1.31 notation integer
 * @return the converted floating point value
 */

#define q31_to_f32(x)	ldexp((int32_t) x, -31)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float cordic_cos_result=0.0f, cordic_sine_result=0.0f, sine_result=0.0, cos_result=0.0;
volatile uint32_t exe_time_cos,exe_time_sin, execution_time, exe_time_cos_sin;
uint32_t hrtim_cos_exe_time, hrtim_sine_exe_time;
volatile uint16_t hrtim_exe_time=0;
uint32_t exe_time_cos_IT, exe_time_sine_IT, exe_time_ISR, cos_time, sine_time;
volatile uint32_t Systick_counter=0, ex_Time_sin_Syst,  ex_Time_cos_Syst, start_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/*
 * Get the SysTick Counter value
 */
static inline uint32_t Get_SysTick_Counter()
{
	return Systick_counter;
}

/**
 * @name f32_to_q31
 * @brief Convert a 32 Bits float to a q1.31 notation integer
 * @param input floating point value to convert
 * @return a q1.31 notation integer
 */
static inline int32_t f32_to_q31(float input)
{
	const float q31_max_F= 0x0.FFFFFFp0f; /* represent 0.9999847412109375*/
	const float q31_min_F= -1.0f;
	return (int32_t)roundf(scalbnf(fmaxf(fminf(input, q31_max_F), q31_min_F), 31)); /*!fminf give the min value between input and q31_max
																				fmaxf gives the max between the result for fminf and q31_min_F
																				scalbnf multiply the result of fmaxf() with 2^31. This is used to convert a float value
																				in a specified range into an integer representation with a certain scale.
																				roundf() This function rounds the previous result to the nearest integer.
																				This converts the scaled float value to a whole number, taking rounding into account.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */
}

static inline HAL_StatusTypeDef config_cos_and_sin(void)
{
	CORDIC_ConfigTypeDef sConfig;

	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_2;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	if((HAL_CORDIC_Configure(&hcordic, &sConfig))!= HAL_OK)
		return HAL_ERROR;
	else
		return HAL_OK;


}

static inline int32_t* cordic_cos_sinf(float x)
{

	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1; //input
	static int32_t output_q31[2];

	if( HAL_CORDIC_CalculateZO(&hcordic, &input_q31, output_q31, 1, 0) !=HAL_OK)
		Error_Handler();

	return output_q31;
}


/**
 * @name f32_to_q31
 * @brief Convert a 32 Bits float to a q1.31 notation integer
 * @param input floating point value to convert
 * @return a q1.31 notation integer
 */
static inline HAL_StatusTypeDef config_cos_q31(void)
{
	CORDIC_ConfigTypeDef sConfig;
	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	if((HAL_CORDIC_Configure(&hcordic, &sConfig))!= HAL_OK)
		return HAL_ERROR;
	else
		return HAL_OK;
}


/**
 * @name cordic_cosinef
 * @brief Calculate the cosine of an angle in zero overhead mode in q1.31 notation integer
 * @param x is an angle in Degree
 * @return the calculate value in float
 */
static inline float cordic_cosinef(float x)
{

	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1; //input
	int32_t output_q31;

	if(HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0)==HAL_OK)
		Error_Handler();

	return q31_to_f32(output_q31);
}

/**
 * @name cordic_cosineIT
 * @brief Convert a 32 Bits float to a q1.31 notation interger
 * @param input floating point value to convert
 * @return a q1.32 notation integer
 */
static inline void cordic_cosineIT(float x)
{
	CORDIC_ConfigTypeDef sConfig;

	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1; //input
	int32_t output_q31;
	//uint32_t start=0;

	sConfig.Function = CORDIC_FUNCTION_COSINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	HAL_CORDIC_Configure(&hcordic, &sConfig);

	if(HAL_CORDIC_Calculate_IT(&hcordic, &input_q31, &output_q31, 1) == HAL_OK)
		exe_time_cos_IT =execution_time /10;



	return;

}

static inline HAL_StatusTypeDef config_sin_q31(void)
{
	CORDIC_ConfigTypeDef sConfig;
	sConfig.Function = CORDIC_FUNCTION_SINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	if((HAL_CORDIC_Configure(&hcordic, &sConfig))!= HAL_OK)
		return HAL_ERROR;
	else
		return HAL_OK;
}

static inline float cordic_sinef(float x)
{


	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1; //input
	int32_t output_q31;

	if(HAL_CORDIC_CalculateZO(&hcordic, &input_q31, &output_q31, 1, 0)!=HAL_OK)
		Error_Handler();

	return q31_to_f32(output_q31);

}


static inline void cordic_sineIT(float x)
{
	CORDIC_ConfigTypeDef sConfig;

	int32_t input_q31 = f32_to_q31(fmod(x, 2.0f*pi) / (2.0f*pi)) <<1; //input
	int32_t output_q31;
	//uint32_t start_time=0;

	sConfig.Function = CORDIC_FUNCTION_SINE;
	sConfig.InSize = CORDIC_INSIZE_32BITS;
	sConfig.OutSize= CORDIC_OUTSIZE_32BITS;
	sConfig.Precision= CORDIC_PRECISION_6CYCLES;
	sConfig.Scale= CORDIC_SCALE_0;
	sConfig.NbRead= CORDIC_NBREAD_1;
	sConfig.NbWrite = CORDIC_NBWRITE_1;

	HAL_CORDIC_Configure(&hcordic, &sConfig);

	//start_time = htim2.Instance->CNT;

	if(HAL_CORDIC_Calculate_IT(&hcordic, &input_q31, &output_q31, 1) == HAL_OK)
		exe_time_sine_IT = execution_time/10;
	else
		Error_Handler();


	//exe_time_sine_IT = (htim2.Instance->CNT - start_time)/100;

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
	double x_in_rad;
	int32_t* result;
	//uint32_t start_time;
	//uint32_t start_time;
	int j=0;
	int16_t buffer_exe_hrtime[361]={0};
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
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim2);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);

  if(config_cos_q31()!= HAL_OK)
  {
	  Error_Handler();
  }

  if(config_sin_q31()!= HAL_OK)
  {
	  Error_Handler();
  }
/*
  if(config_cos_and_sin()!=HAL_OK)
  {
	  Error_Handler();
  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
    /* USER CODE END WHILE */


    /* USER CODE BEGIN 3 */
  while (1)
  {
    for(int i=0; i< 361; i++)
    {
    	x_in_rad = (float)(i)*pi/180.0;	/* Convert degrees to radiant*/

    	/*CORDIC in Zero mode*/
    	cordic_cos_result = cordic_cosinef(x_in_rad);
    	HAL_Delay(3);

    	cordic_sine_result = cordic_sinef(x_in_rad)*10;
	    HAL_Delay(3);


		/* result = cordic_cos_sinf(x_in_rad);
		 cordic_cos_result = q31_to_f32(result[0]) *10;
		 cordic_sine_result = q31_to_f32(result[1]) *10;
		 HAL_Delay(5);*/

    }


  /* USER CODE END 3 */
}

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
