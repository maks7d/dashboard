/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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

/* USER CODE BEGIN Private defines */
#define POWER_HOLD_PIN   GPIO_PIN_3
#define POWER_HOLD_PORT  GPIOE
#define POWER_BTN_PIN    GPIO_PIN_7
#define POWER_BTN_PORT   GPIOG

#define DISP_PIN         GPIO_PIN_7   /* PH7 — display standby (active high) */
#define DISP_PORT        GPIOH
#define BL_PWM_PIN       GPIO_PIN_11  /* PB11 — DIO5661 EN (active high)     */
#define BL_PWM_PORT      GPIOB

#define CAN_STB_PIN      GPIO_PIN_15  /* PF15 — TJA1044 STB (bas = mode normal) */
#define CAN_STB_PORT     GPIOF

#define EN_ESP32_PIN     GPIO_PIN_5   /* PH5 — enable ESP32 (actif haut) */
#define EN_ESP32_PORT    GPIOH

#define EN_IP23_PIN      GPIO_PIN_8   /* PI8 — enable chargeur IP2312 (U10) */
#define EN_IP23_PORT     GPIOI

#define BAT_STAT_PIN     GPIO_PIN_13  /* PC13 — statut charge IP2312 (sortie open-drain) */
#define BAT_STAT_PORT    GPIOC

#define LAP_DET_PIN      GPIO_PIN_0   /* PA0 — capteur détection tour */
#define LAP_DET_PORT     GPIOA

#define UWB_RST_PIN      GPIO_PIN_2   /* PG2 — reset module UWB (DW3000) */
#define UWB_RST_PORT     GPIOG

#define RPM_PIN          GPIO_PIN_15  /* PH15 — entrée capteur RPM (EXTI) */
#define RPM_PORT         GPIOH

#define CS_FLASH_GPIO_PIN  GPIO_PIN_0   /* PI0 — SPI2_CS, flash externe IC1 */
#define CS_FLASH_GPIO_PORT GPIOI
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
