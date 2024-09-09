/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

//#define DATA_VALIDATION
#define PERFORMANCE_MEASUREMENT

#define BLOCK_SIZE 256

/* FMAC Parameters */
#define NO_OF_TAPS 32

#if NO_OF_TAPS == 32
    #define X1_BUFF_SIZE  112
    #define Y_BUFF_SIZE   112
#elif NO_OF_TAPS == 64
    #define X1_BUFF_SIZE  96
    #define Y_BUFF_SIZE   96
#elif NO_OF_TAPS == 127
    #define X1_BUFF_SIZE  128
    #define Y_BUFF_SIZE   1
#endif

// Define the preload size
#define PRELOAD_SIZE (NO_OF_TAPS - 1)


// Conditional block size and number of blocks
#ifdef DATA_VALIDATION
    // Override settings for data validation
    #undef BLOCK_SIZE
    #define BLOCK_SIZE 256
	#define HALF_BLOCK_SIZE        (BLOCK_SIZE / 2)
    #define NO_OF_BLOCKS 4
#else
    // Use block size settings based on BLOCK_SIZE value
    #if BLOCK_SIZE == 256
        #define HALF_BLOCK_SIZE        (BLOCK_SIZE / 2)
        #define NO_OF_BLOCKS           800
    #elif BLOCK_SIZE == 2048
        #define HALF_BLOCK_SIZE        (BLOCK_SIZE / 2)
        #define NO_OF_BLOCKS           100
    #elif BLOCK_SIZE == 8192
        #define HALF_BLOCK_SIZE        (BLOCK_SIZE / 2)
        #define NO_OF_BLOCKS           25
    #else
        #error "Unsupported BLOCK_SIZE"
    #endif
#endif

// Define the input signal size
#define INPUT_SIGNAL_SIZE        (BLOCK_SIZE * NO_OF_BLOCKS)



/* Polling timeout */
#define POLLING_TIMEOUT         1000





/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
