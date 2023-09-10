/* USER CODE BEGIN Header */
/**
 *
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  *
  * Copyright (c) 2023 BYU-Idaho   ECEN Dept
  * Author Lynn Watson
  *
  * Note that most of this comes from the CubeMX make.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define       LAB_04

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

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
#define LM35_IN_Pin GPIO_PIN_1
#define LM35_IN_GPIO_Port GPIOC
#define Potentiometer_Pin GPIO_PIN_0
#define Potentiometer_GPIO_Port GPIOA
#define Button_1_Pin GPIO_PIN_1
#define Button_1_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Button_2_Pin GPIO_PIN_4
#define Button_2_GPIO_Port GPIOA
#define LED_D2_Pin GPIO_PIN_6
#define LED_D2_GPIO_Port GPIOA
#define Button_3_Pin GPIO_PIN_0
#define Button_3_GPIO_Port GPIOB
#define SevenSeg_CLK_Pin GPIO_PIN_8
#define SevenSeg_CLK_GPIO_Port GPIOA
#define SevenSeg_DATA_Pin GPIO_PIN_9
#define SevenSeg_DATA_GPIO_Port GPIOA
#define SPI1_NSS_Pin GPIO_PIN_11
#define SPI1_NSS_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SevenSeg_LATCH_Pin GPIO_PIN_5
#define SevenSeg_LATCH_GPIO_Port GPIOB
#define LED_D4_Pin GPIO_PIN_6
#define LED_D4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE 50

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
