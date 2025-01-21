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
#define RES1	0
#define RES2	1
#define RNUMBER	2
#define x_min	-1.118
#define x_max	1.118
#define TIMER_PERIOD	65503
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float cosh_res, sinh_res, Expo_x, Expo_minusX, sqrtX_res_q31, sqrtX_res_q15;
float cosh_q15_res,sinh_q15_res;
volatile uint16_t execution_time;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static inline int16_t f16_to_q15(double input)
{
	const float q15_max_F= 0x0.FFFp0f; // represent 0.9999847412109375
	const float q15_min_F= -1.0f;
	return (int16_t)roundf(scalbnf(fmaxf(fminf(input,q15_max_F),q15_min_F), 15)); /*!fminf give the min value between input and q15_max
																				fmaxf gives the max betwen the result for fminf and q15_min_F
																				scalbnf multiply the result of fmaxf() with 2^15. This is used to convert a float value
																				in a specified range into an integer representation with a certain scale.
																				roundf() This function rounds the previous result to the nearest integer.
																				This converts the scaled float value to a whole number, taking rounding into account.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */
}

/**
 * @name f32_to_q31
 * @brief Convert a 32 Bits float to a q1.31 notation interger
 * @param input floating point value to convert
 * @return a q1.31 notation integer
 */
static inline int f32_to_q31(double input)
{
	const float q31_max_F= 0x0.FFFFFFp0f; // represent 0.9999847412109375
	const float q31_min_F= -1.0f;
	return (int)roundf(scalbnf(fmaxf(fminf(input, q31_max_F),q31_min_F), 31)); /*!fminf give the min value between input and q31_max
																				fmaxf gives the max between the result for fminf and q31_min_F
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

/*

typedef enum
{
	COSH = 1,
	SINH=2,
	BOTH =3

}FunctionTypeDef;
static inline HAL_StatusTypeDef Config_hyperboFunction_q31(FunctionTypeDef func)
{
	HAL_StatusTypeDef status;
	CORDIC_ConfigTypeDef  sConfig;
	switch (func)
	{
		case COSH:
			sConfig.Function = CORDIC_FUNCTION_HCOSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_32BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one  32-bit data input (Q1.31 format), or two 16-bit data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one  32-bit data output (Q1.31 format), Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
		case SINH:
			sConfig.Function = CORDIC_FUNCTION_HSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_32BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one 32-bit data input (Q1.31 format), or two 16-bit  data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one 32-bit data output (Q1.31 format),   Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
		case BOTH:
			sConfig.Function = CORDIC_FUNCTION_HSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_32BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one  32-bit data input (Q1.31 format), or two 16-bit  data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_2; 	!< One 32-bits read containing either only one   32-bit data output (Q1.31 format),  Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
	  }
	return status;
}


static inline HAL_StatusTypeDef  Config_hyperboFunction_q15(FunctionTypeDef func)
{
	HAL_StatusTypeDef status;
	CORDIC_ConfigTypeDef  sConfig;
	switch (func)
	{
		case COSH:
			sConfig.Function = CORDIC_FUNCTION_HCOSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_16BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_16BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one 32-bit data input (Q1.31 format), or two 16-bit  data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one   32-bit data output (Q1.31 format),  Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
		case SINH:
			sConfig.Function = CORDIC_FUNCTION_HSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_16BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_16BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one 32-bit data input (Q1.31 format), or two 16-bit  data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one  32-bit data output (Q1.31 format),   Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
		case BOTH:
			sConfig.Function = CORDIC_FUNCTION_HCOSINE; //Function Cosine
			sConfig.InSize = CORDIC_INSIZE_16BITS;       !< 32 bits input data size (Q1.31 format)
			sConfig.OutSize = CORDIC_OUTSIZE_16BITS;  !< 32 bits output data size (Q1.31 format)
			sConfig.NbWrite = CORDIC_NBWRITE_1; 		!< One 32-bits write containing either only one 32-bit data input (Q1.31 format), or two 16-bit data input (Q1.15 format) packed in one 32 bits Data
			sConfig.NbRead = CORDIC_NBREAD_1; 	!< One 32-bits read containing either only one 32-bit data output (Q1.31 format),   Data
			sConfig.Scale= CORDIC_SCALE_1; 	! Scaling Factor 1
			sConfig.Precision = CORDIC_PRECISION_6CYCLES ; ! Precision 6 cycles: 24 iterations
			status = HAL_CORDIC_Configure(&hcordic, &sConfig);
			break;
	  }
	return status;
}

static inline void cosh_sinhf_q31(float x, float *pcosh_res, float* psinh_res)
{
	int32_t RES[2], q31_angleX;
	q31_angleX= f32_to_q31(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q31_angleX, RES, 1, 0) != HAL_OK)
		Error_Handler();

	*pcosh_res = (q31_to_f32(RES[0]) * 2);
	*psinh_res= (q31_to_f32(RES[1]) * 2);

	return;
}
static inline void cosh_sinhf_q15(float x, float *pcosh_res, float* psinh_res)
{
	int32_t RES[1], q31_angleX;
	int16_t cosh_low;
	q31_angleX= f32_to_q31(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q31_angleX, RES, 1, 0) != HAL_OK)
		Error_Handler();

	 cosh_low = (int16_t)(RES[0] & 0x0000FFFF);
	*pcosh_res = (q15_to_f16(cosh_low) * 2);
	*psinh_res= (q15_to_f16((int16_t)(RES[0]& 0xFFFF0000)) * 2);

	return;
}

static inline void coshf_q31(float x, float *p_cosh_res)
{
	int32_t RES[1], q31_angleX;
	q31_angleX= f32_to_q31(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q31_angleX, RES, 1, 0) != HAL_OK)
		 Error_Handler();


	*p_cosh_res = q31_to_f32(RES[0])*2;
	return;
}



static inline void coshf_q15(float x, float *p_cosh_res)
{
	int32_t RES[1], q15_angleX;
	int16_t res;
	q15_angleX= f16_to_q15(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q15_angleX, RES, 1, 0) != HAL_OK)
		Error_Handler();

	 res = (int16_t)RES[0];
	*p_cosh_res = (q15_to_f16(res)*2);
	return;
}

static inline void sinhf_q15(float x, float *p_sinh_res)
{
	int32_t RES[1], q15_angleX;
	int16_t res;
	q15_angleX= f16_to_q15(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q15_angleX, RES, 1, 0) != HAL_OK)
		Error_Handler();

	 res= (int16_t) RES[0];
	*p_sinh_res = (q15_to_f16(res)*2);
	return;
}

static inline void sinhf_q31(float x, float *sinh_res)
{
	int32_t RES[1], q31_angleX;
	q31_angleX= f32_to_q31(x*0.5f);

	 if(HAL_CORDIC_CalculateZO(&hcordic, &q31_angleX, RES, 1, 0) != HAL_OK)
		Error_Handler();

	*sinh_res = (q31_to_f32(RES[0])*2);
	return;
}
*/


static inline HAL_StatusTypeDef  config_Sqrt_q31(void)
{
	CORDIC_ConfigTypeDef  sConfig;
	 HAL_StatusTypeDef status;

	sConfig.Function = CORDIC_FUNCTION_SQUAREROOT; //Function Square root

	sConfig.InSize = CORDIC_INSIZE_32BITS;       /*!< 32 bits input data size (Q1.15 format) */

	sConfig.OutSize = CORDIC_OUTSIZE_32BITS;  /*!< 32 bits output data size (Q1.15 format) */

	sConfig.NbWrite = CORDIC_NBWRITE_1; 		/*!< One 32-bits write containing either only one
													 32-bit data input (Q1.31 format), or two 16-bit
													  data input (Q1.15 format) packed in one 32 bits Data */

	sConfig.NbRead = CORDIC_NBREAD_1; 	/*!< One 32-bits read containing either only one
												  32-bit data output (Q1.31 format), or two 16-bit
												  data output (Q1.15 format) packed in one 32 bits
												  Data */
	sConfig.Scale= CORDIC_SCALE_2; 	/*! 2 Scaling */
	sConfig.Precision = CORDIC_PRECISION_3CYCLES ; /*! Precision 6 cycles: 24 iterations */

	status= HAL_CORDIC_Configure(&hcordic, &sConfig);

	return status;

}

static inline HAL_StatusTypeDef  config_Sqrt_q15(void)
{
	CORDIC_ConfigTypeDef  sConfig;
	 HAL_StatusTypeDef status;

	sConfig.Function = CORDIC_FUNCTION_SQUAREROOT; //Function Square root

	sConfig.InSize = CORDIC_INSIZE_16BITS;       /*!< 32 bits input data size (Q1.15 format) */

	sConfig.OutSize = CORDIC_OUTSIZE_16BITS;  /*!< 32 bits output data size (Q1.15 format) */

	sConfig.NbWrite = CORDIC_NBWRITE_1; 		/*!< One 32-bits write containing either only one
													 32-bit data input (Q1.31 format), or two 16-bit
													  data input (Q1.15 format) packed in one 32 bits Data */

	sConfig.NbRead = CORDIC_NBREAD_1; 	/*!< One 32-bits read containing either only one
												  32-bit data output (Q1.31 format), or two 16-bit
												  data output (Q1.15 format) packed in one 32 bits
												  Data */
	sConfig.Scale= CORDIC_SCALE_2; 	/*! 2 Scaling */
	sConfig.Precision = CORDIC_PRECISION_3CYCLES ; /*! Precision 6 cycles: 24 iterations */

	status= HAL_CORDIC_Configure(&hcordic, &sConfig);

	return status;

}

static inline float sqrtX_q31(float x)
{

	int32_t RES[1];
	x = x*0.25;
	int32_t X_q31= f32_to_q31(x);

	HAL_CORDIC_CalculateZO(&hcordic, &X_q31, RES, 1, 0);

	return (q31_to_f32(RES[0])*4);
}

static inline float sqrtX_q15(float x)
{

	int32_t RES[1];
	x = x*0.25;
	int32_t X_q15= f32_to_q31(x);

	HAL_CORDIC_CalculateZO(&hcordic, &X_q15, RES, 1, 0);

	return (q15_to_f16(RES[0]&0x0000FFFF)*4);
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
	//float x=0;
	//float cosh_x, sinh_x;

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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_CORDIC_Init();
  MX_TIM2_Init();
  MX_HRTIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  int i =0;
  int16_t pExeTime[224], sqrtExTime[12];
  uint8_t to_send[5];
  uint8_t* c="\r\n";
  //HAL_TIM_Base_Start(&htim2);
  HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A);

  /*if(Config_hyperboFunction_q31(COSH)!=HAL_OK)
	  Error_Handler();*/

/*  if(Config_hyperboFunction_q31(SINH)!=HAL_OK)
 	  Error_Handler();*/

 /* if(Config_hyperboFunction_q15(SINH)!=HAL_OK)
  	  Error_Handler();*/
/*
  if(Config_hyperboFunction_q15(COSH)!=HAL_OK)
  	  Error_Handler();*/

 /* if(Config_hyperboFunction_q31(BOTH)!=HAL_OK)
  	  Error_Handler();*/

  /*if(Config_hyperboFunction_q15(BOTH)!=HAL_OK)
  	  Error_Handler();*/

  /*if(config_Sqrt_q31() !=HAL_OK)
	  Error_Handler();
*/
  if(config_Sqrt_q15()!=HAL_OK)
	  Error_Handler();
  /*Calculate the cosineh in range -1.118 to 1.118 in q31 Format */
  /*	 for (float x = x_min;  x <= x_max; x +=0.01)
  	 {
  		coshf_q31(x,  &cosh_res);
  		pExeTime[i]=execution_time;
  		i++;
  		HAL_Delay(5);
  	 }*/

	/*Calculate the cosineh in range -1.118 to 1.118 in q15 Format */
	 /*for (float x = x_min;  x <= x_max; x +=0.01)
	 {
		coshf_q15(x,  &cosh_q15_res);
		pExeTime[i]=execution_time;
		i++;
		HAL_Delay(5);
	 }
*/
 /*Calculate the sineh in range -1.118 to 1.118 in q31 Format */
	/* for (float x = x_min;  x <= x_max; x +=0.01)
	 {
		sinhf_q31(x,  &sinh_res);
		pExeTime[i]=execution_time;
		i++;
		HAL_Delay(5);
	 }*/
 /*Calculate the sineh in range -1.118 to 1.118 in q15 Format */
	/* for (float x = x_min;  x <= x_max; x +=0.01)
	 {
		sinehf_q15(x,  &sineh_q15_res);
		pExeTime[i]=execution_time;
		i++;
		HAL_Delay(5);
	 }*/
 /*Calculate the sineh and cosineh in range -1.118 to 1.118 in q31 Format */
/*	 for (float x = x_min;  x <= x_max; x +=0.01)
	 {
		cosh_sinhf_q31(x,  &cosh_res, &sinh_res);
		pExeTime[i]=execution_time;
		i++;
		HAL_Delay(5);
	 }*/

 /*Calculate the sineh and cosineh in range -1.118 to 1.118 in q15 Format*/
	/* for (float x = x_min;  x <= x_max; x +=0.01)
	 {
		cosh_sinhf_q15(x,  &cosh_q15_res, &sinh_q15_res);
		pExeTime[i]=execution_time;
		i++;
		HAL_Delay(5);
	 }*/

	/*calculate the square function for 1.175<= x1 <= 2.341 in q31 Format*/
	 /* for (float x1 = 1.75; x1 <= 2.341; x1 +=0.05)
		{
			sqrtX_res_q31 = sqrtX_q31(x1);
			sqrtExTime[i]=execution_time;
			i++;
			HAL_Delay(1);
		}*/

	/*calculate the square function for 1.175<= x1 <= 2.341 in q15 Format*/
	  for (float x1 = 1.75; x1 <= 2.341; x1 +=0.05)
		{
			sqrtX_res_q15 = sqrtX_q15(x1);
			sqrtExTime[i]=execution_time;
			i++;
			HAL_Delay(1);
		}

 /*send execution time for cosh and sineh */
  for(int j=0; j<224; j++)
  {
	  sprintf(to_send,"%d", pExeTime[j]);
	  HAL_UART_Transmit(&huart2, to_send, 5, 5);
	  HAL_Delay(3);
	  HAL_UART_Transmit(&huart2, c, 2, 2);
	  HAL_Delay(3);
  }

  /*send execution time for sqrt */
 /*  for(int j=0; j<12; j++)
   {
	  sprintf(to_send,"%d", sqrtExTime[j]);
	  HAL_UART_Transmit(&huart2, to_send, 5, 5);
	  HAL_Delay(3);
	  HAL_UART_Transmit(&huart2, c, 2, 2);
	  HAL_Delay(3);
   }
*/



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE BEGIN 3 */
	  /*Calculate the cosineh in range -1.118 to 1.118 in q31 Format */
	  for (float x = x_min;  x <= x_max; x +=0.01)
	 {

		coshf_q31(x,  &cosh_res);
		HAL_Delay(5);
	 }
	  /*Calculate the sineh in range -1.118 to 1.118 in q31 Format*/
	  for(float x = x_min; x<= x_max; x +=0.01)
	  {
		  sinhf_q31(x, &sinh_res);
		  HAL_Delay(5);
	  }
  /*Calculate the cosineh in range -1.118 to 1.118 in q15 Format */
	 /* for (float x = x_min;  x <= x_max; x +=0.01)
	 {

		coshf_q15(x,  &cosh_q15_res);
		Expo_x =  cosh_res +  sinh_res;
		Expo_minusX = cosh_res - sinh_res;
		HAL_Delay(5);
	 }*/

  /*Calculate the sineh in range -1.118 to 1.118 in q15 Format*/
	  /*for(float x = x_min; x<= x_max; x +=0.01)
	  {
		  sinhf_q15(x, &sinh_q15_res);
		  HAL_Delay(5);
	  }*/



	  /*calculate the square function for 1.175<= x1 <= 2.341 */
	  /*for (float x1 = 1.75; x1 <= 2.341; x1 +=0.05)
		{
			sqrtX_res_q31 = sqrtX_q31(x1);
			HAL_Delay(1);
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
