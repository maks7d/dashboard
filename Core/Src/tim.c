/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */
DMA_HandleTypeDef hdma_tim16_ch1;
TIM_HandleTypeDef htim16;
/* USER CODE END 0 */

TIM_HandleTypeDef htim2;

/* TIM2 : uniquement la base de temps du PWM de rétroéclairage (CH4 -> PB11, voir display.c).
   Prescaler 0 : tick = 200 MHz (2 x PCLK1), période 20000 ticks = 10 kHz avec 20000 niveaux de
   luminosité. display.c recalcule ARR à partir de ces valeurs. Pas d'interruption nécessaire. */
void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 1 */
void MX_TIM16_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim16.Instance               = TIM16;
  htim16.Init.Prescaler         = 0;
  htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim16.Init.Period            = 249;   /* 200MHz / 250 = 800kHz */
  htim16.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
    Error_Handler();

  sConfigOC.OCMode       = TIM_OCMODE_PWM1;
  sConfigOC.Pulse        = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM2)
  {
    __HAL_RCC_TIM2_CLK_ENABLE();   /* la broche PB11 est configurée dans display.c */
  }
  else if (htim->Instance == TIM16)
  {
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PF6 → TIM16_CH1 */
    GPIO_InitStruct.Pin       = GPIO_PIN_6;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* DMA1 Stream0 — requête TIM16_CH1 */
    hdma_tim16_ch1.Instance                 = DMA1_Stream0;
    hdma_tim16_ch1.Init.Request             = DMA_REQUEST_TIM16_CH1;
    hdma_tim16_ch1.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_tim16_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim16_ch1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim16_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tim16_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_tim16_ch1.Init.Mode                = DMA_NORMAL;
    hdma_tim16_ch1.Init.Priority            = DMA_PRIORITY_HIGH;
    hdma_tim16_ch1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim16_ch1) != HAL_OK)
      Error_Handler();

    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim16_ch1);

    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  }
}
/* ===== TIM5 : chronométrage (LAP_DET sur PA0 = CH1, RPM sur PH11 = CH2) =====
 * Compteur 32 bits libre à 200 MHz (1 tick = 5 ns). À chaque front, le MATÉRIEL copie le
 * compteur dans CCRx puis un DMA copie CCRx dans un tampon circulaire (app_timing.c) :
 * aucune interruption, aucun cycle CPU par impulsion, et la précision ne dépend ni de la
 * latence d'interruption ni de FreeRTOS. Les NVIC des streams DMA ne sont volontairement PAS
 * activés. DMA1 Stream1 = CH1, Stream2 = CH2 (Stream0 est pris par TIM16/WS2812). */
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;

void MX_TIM5_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
    Error_Handler();

  sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter    = 0x0F;   /* rejette les parasites < ~1,3 us (8 échantillons à fDTS/32) */
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
    Error_Handler();
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM5)
  {
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* PA0 -> TIM5_CH1 (LAP_DET), PH11 -> TIM5_CH2 (RPM) */
    GPIO_InitStruct.Pin       = GPIO_PIN_0;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* DMA circulaire périphérique -> mémoire, mots de 32 bits (CCRx de TIM5 = 32 bits) */
    hdma_tim5_ch1.Instance                 = DMA1_Stream1;
    hdma_tim5_ch1.Init.Request             = DMA_REQUEST_TIM5_CH1;
    hdma_tim5_ch1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_tim5_ch1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim5_ch1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim5_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch1.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch1.Init.Mode                = DMA_CIRCULAR;
    hdma_tim5_ch1.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_tim5_ch1.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch1) != HAL_OK)
      Error_Handler();
    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC1], hdma_tim5_ch1);

    hdma_tim5_ch2.Instance                 = DMA1_Stream2;
    hdma_tim5_ch2.Init.Request             = DMA_REQUEST_TIM5_CH2;
    hdma_tim5_ch2.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_tim5_ch2.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_tim5_ch2.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_tim5_ch2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim5_ch2.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    hdma_tim5_ch2.Init.Mode                = DMA_CIRCULAR;
    hdma_tim5_ch2.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
    hdma_tim5_ch2.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_tim5_ch2) != HAL_OK)
      Error_Handler();
    __HAL_LINKDMA(htim, hdma[TIM_DMA_ID_CC2], hdma_tim5_ch2);
  }
}
/* USER CODE END 1 */

