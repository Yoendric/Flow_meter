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
#include "stm32l0xx_hal.h"

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
void CASES_CHOICE(char* buff);
void COUNTER_CONSUMER(void);
void TRANSMIT_WSSFM10R2AT(char*,int);
uint32_t CONFIG_CHANNEL_ADC(uint32_t);
uint32_t GET_MEAS_BAT(void);
uint32_t GET_MEAS_HALL(void);
void WAKE_WSSFM10R2AT(void);
void RESET_WSSFM10R2AT(void);
void DEEP_SLEEP_WSSFM10R2AT(void);
void DATA_ASSIGMENT(char*, uint16_t, uint8_t, uint8_t);
void BUILD_DATA_TO_SEND(char*);
void stm32l_lowPowerSetup(void);
void stm32l_lowPowerResume(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HALL_SENS_IRQ_Pin GPIO_PIN_0
#define HALL_SENS_IRQ_GPIO_Port GPIOA
#define HALL_SENS_IRQ_EXTI_IRQn EXTI0_1_IRQn
#define BAT_MEAS_AN_Pin GPIO_PIN_1
#define BAT_MEAS_AN_GPIO_Port GPIOA
#define HALL_SENS_AN_Pin GPIO_PIN_3
#define HALL_SENS_AN_GPIO_Port GPIOA
#define EN_BAT_MEAS_Pin GPIO_PIN_4
#define EN_BAT_MEAS_GPIO_Port GPIOA
#define WAKE_DS_WISOL_Pin GPIO_PIN_5
#define WAKE_DS_WISOL_GPIO_Port GPIOA
#define RST_WISOL_Pin GPIO_PIN_6
#define RST_WISOL_GPIO_Port GPIOA
#define EN_VCC3V3s_Pin GPIO_PIN_7
#define EN_VCC3V3s_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
