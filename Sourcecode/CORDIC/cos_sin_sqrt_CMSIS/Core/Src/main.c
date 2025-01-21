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
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include "arm_const_structs.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define pi  	3.1415926535897932384626433
#define TIMER_PERIOD 65503
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float32_t cos_result, sin_result;
q31_t q31_cos_result, q31_sin_result, q31_sqrt_result;
q15_t q15_cos_result, q15_sin_result, q15_sqrt_result;
uint32_t cos_sin_exe_time, cos_exe_time, sin_exe_time, ex_time_sqrt_x, exe_Systick_time;
float32_t sqrt_x;
uint16_t sin_ex_time_hrtm, cos_ex_time_hrtm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
	//uint32_t start=0;
	uint16_t start_time_hrtm;
	float32_t x=0.0;
	uint16_t buffer_of_exe_time[361]={0};
	uint16_t sqrt_buffer[6]={0};


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
  MX_TIM2_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_TIM_Base_Start(&htim2);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);
  int i =0;
  q31_t x_q31;
  q15_t x_q15;
  uint16_t end_time;
  //uint32_t end_time=0;
/*
 for(float theta=0; theta <361; theta++)
  {
	  start_time_hrtm =(int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
	  arm_sin_cos_f32(theta, &sin_result, &cos_result);  //Theta in Degrees
	  buffer_of_exe_time[i]=  (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
	  i++;
	  HAL_Delay(3);
  }*/

/* arm_sin_cos_q31*/
 /* for(float theta=0; theta <361; theta++)
  {
    x=  (theta*pi)/180.0f;
    arm_float_to_q31(&x, &x_q31,1);
    start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
    arm_sin_cos_q31(theta, &q31_sin_result, &q31_cos_result);
    buffer_of_exe_time[i] = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
    i++;
    HAL_Delay(3);
  }*/

 /*  arm_cos_f32
 for(float theta=0; theta <361; theta++)
 {
	  x = (theta*pi)/180.0f;
	  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
	  cos_result= arm_cos_f32(x);
	  buffer_of_exe_time[i] = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
	  i++;
	  HAL_Delay(3);
 }*/

 /* arm_sin_f32*/
 /*for(float theta=0; theta <361; theta++)
  {
 	  x = (theta*pi)/180.0f;
 	  start_time_hrtm = (uint16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
 	  sin_result= arm_sin_f32(x);
 	  buffer_of_exe_time[i] = (uint16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
 	  i++;
 	  HAL_Delay(3);
  }*/

  /* calculate sine in q31 Format*/
for(float theta=0; theta <361; theta++)
{
  x=  (theta*pi)/180.0f;
  arm_float_to_q31(&x, &x_q31,1);
  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
  q31_sin_result= arm_sin_q31(x_q31);
  end_time=(int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR);

  if(start_time_hrtm > end_time)
  {
	  buffer_of_exe_time[i]=  TIMER_PERIOD -start_time_hrtm +end_time+1;
  }
  else
  {
	  buffer_of_exe_time[i] =end_time-start_time_hrtm;
  }
  i++;
  HAL_Delay(3);
}

/* calculate cosine in q31 Format*/
/*for(float theta=0; theta <361; theta++)
{
  x=  (theta*pi)/180.0f;
  arm_float_to_q31(&x, &x_q31,1);
  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
  q31_cos_result= arm_cos_q31(x_q31);
  buffer_of_exe_time[i] = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
  i++;
  HAL_Delay(3);
}*/

/* calculate sine in q15 Format*/
/*for(float theta=0; theta <361; theta++)
 {
	 x=  (theta*pi)/180.0f;
      arm_float_to_q15(&x, &x_q15,1);
	  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
	  q15_sin_result= arm_sin_q15(x_q15);
	  buffer_of_exe_time[i] = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
	  i++;
	  HAL_Delay(3);
 }*/

/* calculate cos in q15 Format*/
/*for(float theta=0; theta <361; theta++)
 {
	 x=  (theta*pi)/180.0f;
	  arm_float_to_q15(&x, &x_q15,1);
	  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
	  q15_cos_result= arm_cos_q15(x_q15);
	  buffer_of_exe_time[i] = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
	  i++;
	  HAL_Delay(3);
 }*/


/* f32 square root of x*/
/*for(float x1=1.75; x1<= 2.341; x1 +=0.1)
{
  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
  arm_sqrt_f32(x1, &sqrt_x);
  sqrt_buffer[i] = (uint16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
  i++;
  HAL_Delay(3);

}*/


/* q31 square root of x*/
/*for(float x1=1.75; x1<= 2.341; x1 +=0.1)
{
  arm_float_to_q31(&x1, &x_q31,1);
  start_time_hrtm = (uint16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
  arm_sqrt_q31(x_q31, &q31_sqrt_result);
  sqrt_buffer[i] = (uint16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
  i++;
  HAL_Delay(3);
}*/

/* q15 square root of x*/
  for(float x1=1.75; x1<= 2.341; x1 +=0.1)
  {
  	  arm_float_to_q15(&x, &x_q15,1);
	  start_time_hrtm = (uint16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
	  arm_sqrt_q15(x, &q15_sqrt_result);
	  sqrt_buffer[i] = (uint16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
	  i++;
	  HAL_Delay(3);

  }

uint8_t to_send[5]; /* for execution time of  sqrt*/
//uint8_t to_send[5]; /* for execution time of  cosine and sine*/
uint8_t* c="\r\n";
 for(int i=0;i<361; i++)
 {

	sprintf(to_send,"%d", buffer_of_exe_time[i]);
	HAL_UART_Transmit(&huart2, to_send, 5, 5);
	HAL_Delay(3);
	HAL_UART_Transmit(&huart2, c, 2, 5);

	HAL_Delay(3);
 }

 /*for(int i=0;i<6; i++)
  {

 	sprintf(to_send,"%d", sqrt_buffer[i]);
 	HAL_UART_Transmit(&huart2, to_send, 5, 5);
 	HAL_Delay(3);
 	HAL_UART_Transmit(&huart2, c, 2, 5);

 	HAL_Delay(3);
  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*cosine and sine processing*/
	 /* for(int theta=0; theta <361; theta++)
	  {
		  start=htim2.Instance->CNT;
		  arm_sin_cos_f32(theta, &sin_result, &cos_result);  Theta in Degrees
		  cos_sin_exe_time= htim2.Instance->CNT -start;
		  HAL_Delay(3);*/

		/*  x = (theta*pi)/180.0f;
		  //start=htim2.Instance->CNT;
		  //cos_exe_time = htim2.Instance->CNT- start;
		  start_time_hrtm = (int16_t)hhrtim1.Instance->sTimerxRegs->CNTxR;
		  cos_result= arm_cos_f32(x);
		  cos_ex_time_hrtm = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
		  HAL_Delay(3);

		  //start=htim2.Instance->CNT;
		  start_time_hrtm = (int16_t) hhrtim1.Instance->sTimerxRegs->CNTxR;
		  sin_result= arm_sin_f32(x);
		  sin_ex_time_hrtm = (int16_t)(hhrtim1.Instance->sTimerxRegs->CNTxR - start_time_hrtm);
		  //sin_exe_time = htim2.Instance->CNT - start;
		  HAL_Delay(3);*/
	 // }
	  /* square root of x*/
	 /* for(x=1.75; x<= 2.341; x +=0.4)
	  {
		  start=htim2.Instance->CNT;
		  arm_sqrt_f32(x, &sqrt_x);
		  ex_time_sqrt_x = htim2.Instance->CNT- start;
		  HAL_Delay(5);
	  } */

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
