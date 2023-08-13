/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define SW01_Pin GPIO_PIN_6
#define SW01_GPIO_Port GPIOA
#define STOP_BTN_Pin GPIO_PIN_7
#define STOP_BTN_GPIO_Port GPIOA
#define SW02_Pin GPIO_PIN_0
#define SW02_GPIO_Port GPIOB
#define RELAY2_Pin GPIO_PIN_3
#define RELAY2_GPIO_Port GPIOB
#define RELAY1_Pin GPIO_PIN_4
#define RELAY1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */




#define relay1_on HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin,GPIO_PIN_SET)
#define relay1_off HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin,GPIO_PIN_RESET)


#define relay2_on  HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin,GPIO_PIN_SET)
#define relay2_off HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin,GPIO_PIN_RESET)

#define led_toggle  HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin)
#define led_ON      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET)
#define led_OFF      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET)



#define right_btn  HAL_GPIO_ReadPin(SW02_GPIO_Port,SW02_Pin)
#define left_btn   HAL_GPIO_ReadPin(SW01_GPIO_Port,SW01_Pin)
#define stop_btn   HAL_GPIO_ReadPin(STOP_BTN_GPIO_Port,STOP_BTN_Pin)


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
