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

// Define this to 1 to enable PID tuning and test mode via Bluetooth commands.
// Define to 0 or comment out to enable the standard maze-solving mode.
#define DEBUG_TEST 1

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
#define TIM_ANGLE htim3
#define PWM_Timer_L htim2
#define UART_BT huart2
#define PWM_Timer_R htim1
#define PWM_TIM_ChR TIM_CHANNEL_2
#define I2C_GYRO hi2c2
#define PID_Timer htim4
#define PWM_TIM_ChL TIM_CHANNEL_2
#define I2C_SENSORS hi2c1
#define TOF_L_XSHUT_Pin GPIO_PIN_13
#define TOF_L_XSHUT_GPIO_Port GPIOC
#define TOF_FL_XSHUT_Pin GPIO_PIN_14
#define TOF_FL_XSHUT_GPIO_Port GPIOC
#define ENC_L_B_Pin GPIO_PIN_15
#define ENC_L_B_GPIO_Port GPIOC
#define ENC_L_B_EXTI_IRQn EXTI15_10_IRQn
#define ENC_L_A_Pin GPIO_PIN_0
#define ENC_L_A_GPIO_Port GPIOA
#define ENC_L_A_EXTI_IRQn EXTI0_IRQn
#define LED_USER_Pin GPIO_PIN_1
#define LED_USER_GPIO_Port GPIOA
#define USER_BTN_Pin GPIO_PIN_4
#define USER_BTN_GPIO_Port GPIOA
#define ENC_R_A_Pin GPIO_PIN_7
#define ENC_R_A_GPIO_Port GPIOA
#define ENC_R_A_EXTI_IRQn EXTI9_5_IRQn
#define ENC_R_B_Pin GPIO_PIN_1
#define ENC_R_B_GPIO_Port GPIOB
#define ENC_R_B_EXTI_IRQn EXTI1_IRQn
#define IN1_R_Pin GPIO_PIN_15
#define IN1_R_GPIO_Port GPIOB
#define IN2_R_Pin GPIO_PIN_8
#define IN2_R_GPIO_Port GPIOA
#define IN1_L_Pin GPIO_PIN_10
#define IN1_L_GPIO_Port GPIOA
#define IN2_L_Pin GPIO_PIN_12
#define IN2_L_GPIO_Port GPIOA
#define FLOOR_SENS_Pin GPIO_PIN_15
#define FLOOR_SENS_GPIO_Port GPIOA
#define TOF_R_XSHUT_Pin GPIO_PIN_4
#define TOF_R_XSHUT_GPIO_Port GPIOB
#define LED_BATT_LOW_Pin GPIO_PIN_5
#define LED_BATT_LOW_GPIO_Port GPIOB
#define TOF_FR_XSHUT_Pin GPIO_PIN_8
#define TOF_FR_XSHUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
