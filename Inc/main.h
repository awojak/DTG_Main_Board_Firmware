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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"
#include "stdbool.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_DB6_Pin GPIO_PIN_2
#define LCD_DB6_GPIO_Port GPIOE
#define LCD_DB7_Pin GPIO_PIN_3
#define LCD_DB7_GPIO_Port GPIOE
#define LCD_E_Pin GPIO_PIN_4
#define LCD_E_GPIO_Port GPIOE
#define LED1_CONTROL_Pin GPIO_PIN_5
#define LED1_CONTROL_GPIO_Port GPIOE
#define LED2_CONTROL_Pin GPIO_PIN_6
#define LED2_CONTROL_GPIO_Port GPIOE
#define LCD_RS_Pin GPIO_PIN_13
#define LCD_RS_GPIO_Port GPIOC
#define INPUT2_Pin GPIO_PIN_1
#define INPUT2_GPIO_Port GPIOC
#define INPUT1_Pin GPIO_PIN_2
#define INPUT1_GPIO_Port GPIOC
#define LED_STATUS_Pin GPIO_PIN_3
#define LED_STATUS_GPIO_Port GPIOC
#define ENC1_A_Pin GPIO_PIN_0
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_1
#define ENC1_B_GPIO_Port GPIOA
#define DETECT_5V_Pin GPIO_PIN_2
#define DETECT_5V_GPIO_Port GPIOA
#define DETECT_12V_Pin GPIO_PIN_3
#define DETECT_12V_GPIO_Port GPIOA
#define EMERGENCY_Pin GPIO_PIN_4
#define EMERGENCY_GPIO_Port GPIOA
#define PRINTER_POWER_DETECT_Pin GPIO_PIN_4
#define PRINTER_POWER_DETECT_GPIO_Port GPIOC
#define OPC1_SIGNAL_Pin GPIO_PIN_5
#define OPC1_SIGNAL_GPIO_Port GPIOC
#define OPE_SIGNAL_Pin GPIO_PIN_2
#define OPE_SIGNAL_GPIO_Port GPIOB
#define PHOTO_SENSOR_Pin GPIO_PIN_7
#define PHOTO_SENSOR_GPIO_Port GPIOE
#define TABLE_SENSOR_Pin GPIO_PIN_8
#define TABLE_SENSOR_GPIO_Port GPIOE
#define INPUT3_Pin GPIO_PIN_9
#define INPUT3_GPIO_Port GPIOE
#define TABLE_COIL_Pin GPIO_PIN_10
#define TABLE_COIL_GPIO_Port GPIOE
#define OUTPUT1_Pin GPIO_PIN_13
#define OUTPUT1_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_8
#define KEY_1_GPIO_Port GPIOD
#define KEY_2_Pin GPIO_PIN_9
#define KEY_2_GPIO_Port GPIOD
#define KEY_3_Pin GPIO_PIN_10
#define KEY_3_GPIO_Port GPIOD
#define KEY_4_Pin GPIO_PIN_11
#define KEY_4_GPIO_Port GPIOD
#define OUTPUT2_Pin GPIO_PIN_13
#define OUTPUT2_GPIO_Port GPIOD
#define EEPROM_P_Pin GPIO_PIN_15
#define EEPROM_P_GPIO_Port GPIOD
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOC
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOC
#define LIMIT_Z_UP_Pin GPIO_PIN_10
#define LIMIT_Z_UP_GPIO_Port GPIOC
#define Z_INDEX_Pin GPIO_PIN_11
#define Z_INDEX_GPIO_Port GPIOC
#define Z_DIAG_Pin GPIO_PIN_12
#define Z_DIAG_GPIO_Port GPIOC
#define Z_ENABLE_Pin GPIO_PIN_0
#define Z_ENABLE_GPIO_Port GPIOD
#define LIMIT_Z_DOWN_Pin GPIO_PIN_1
#define LIMIT_Z_DOWN_GPIO_Port GPIOD
#define LIMIT_Y_FRONT_Pin GPIO_PIN_2
#define LIMIT_Y_FRONT_GPIO_Port GPIOD
#define Y_INDEX_Pin GPIO_PIN_3
#define Y_INDEX_GPIO_Port GPIOD
#define Y_DIAG_Pin GPIO_PIN_4
#define Y_DIAG_GPIO_Port GPIOD
#define TMC_TX_Pin GPIO_PIN_5
#define TMC_TX_GPIO_Port GPIOD
#define TMC_RX_Pin GPIO_PIN_6
#define TMC_RX_GPIO_Port GPIOD
#define Y_ENABLE_Pin GPIO_PIN_7
#define Y_ENABLE_GPIO_Port GPIOD
#define Z_STEP_Pin GPIO_PIN_4
#define Z_STEP_GPIO_Port GPIOB
#define Z_DIR_Pin GPIO_PIN_5
#define Z_DIR_GPIO_Port GPIOB
#define Y_STEP_Pin GPIO_PIN_6
#define Y_STEP_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_7
#define Y_DIR_GPIO_Port GPIOB
#define LIMIT_Y_BACK_Pin GPIO_PIN_8
#define LIMIT_Y_BACK_GPIO_Port GPIOB
#define LCD_PWM_Pin GPIO_PIN_9
#define LCD_PWM_GPIO_Port GPIOB
#define LCD_DB4_Pin GPIO_PIN_0
#define LCD_DB4_GPIO_Port GPIOE
#define LCD_DB5_Pin GPIO_PIN_1
#define LCD_DB5_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
