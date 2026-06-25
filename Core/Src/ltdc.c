/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ltdc.c
  * @brief   This file provides code for the configuration
  *          of the LTDC instances.
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
#include "ltdc.h"

/* USER CODE BEGIN 0 */
#include "display.h"
/* USER CODE END 0 */

LTDC_HandleTypeDef hltdc;

/* LTDC init function */
void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg  = {0};
  LTDC_LayerCfgTypeDef pLayerCfg1 = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */

  /* Riverdi RVT43HLTFWN00 — 480x272, DCLK 9 MHz
   *
   * Horizontal (DCLK units):
   *   Thw=4 (sync), Thbp=43 (incl. sync), Active=480, Thfp=8
   *   Total = 43 + 480 + 8 = 531 DCLK  ✓ (datasheet typ)
   *
   * Vertical (HSYNC lines):
   *   Tvw=4 (sync), Tvbp=12 (incl. sync), Active=272, Tvfp=8
   *   Total = 12 + 272 + 8 = 292 HSYNC  ✓ (datasheet typ)
   *
   * Frame rate = 9 000 000 / (531 × 292) ≈ 58 fps
   */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;   /* HSYNC active low  */
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;   /* VSYNC active low  */
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AH;   /* DE active high (data valid) */
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;  /* LCD samples on rising DCLK edge */

  hltdc.Init.HorizontalSync     = 3;    /* Thw  - 1 = 3              */
  hltdc.Init.AccumulatedHBP     = 42;   /* Thbp - 1 = 42             */
  hltdc.Init.AccumulatedActiveW = 522;  /* Thbp + W - 1 = 42 + 480  */
  hltdc.Init.TotalWidth         = 530;  /* Thbp + W + Thfp - 1      */

  hltdc.Init.VerticalSync       = 3;    /* Tvw  - 1 = 3              */
  hltdc.Init.AccumulatedVBP     = 11;   /* Tvbp - 1 = 11             */
  hltdc.Init.AccumulatedActiveH = 283;  /* Tvbp + H - 1 = 11 + 272  */
  hltdc.Init.TotalHeigh         = 291;  /* Tvbp + H + Tvfp - 1      */

  hltdc.Init.Backcolor.Blue  = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red   = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  /* Layer 0 — full-screen RGB888 framebuffer (480 x 272 x 3 = 383 KB in AXI SRAM) */
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = DISPLAY_WIDTH;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = DISPLAY_HEIGHT;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg.Alpha  = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t)display_fb;
  pLayerCfg.ImageWidth  = DISPLAY_WIDTH;
  pLayerCfg.ImageHeight = DISPLAY_HEIGHT;
  pLayerCfg.Backcolor.Blue  = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red   = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Layer 1 — disabled (Alpha=0, zero-size window) */
  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 0;
  pLayerCfg1.WindowY0 = 0;
  pLayerCfg1.WindowY1 = 0;
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB888;
  pLayerCfg1.Alpha  = 0;
  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg1.FBStartAdress = 0;
  pLayerCfg1.ImageWidth  = 0;
  pLayerCfg1.ImageHeight = 0;
  pLayerCfg1.Backcolor.Blue  = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red   = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

void HAL_LTDC_MspInit(LTDC_HandleTypeDef* ltdcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(ltdcHandle->Instance==LTDC)
  {
  /* USER CODE BEGIN LTDC_MspInit 0 */

  /* USER CODE END LTDC_MspInit 0 */

  /** Initializes the peripherals clock
   *  PLL3 for 9 MHz LTDC pixel clock:
   *    HSE=25 MHz / M=5 → VCI=5 MHz (range 3: 4-8 MHz)
   *    × N=36 → VCO=180 MHz (medium: 150-420 MHz)
   *    / R=20 → 9 MHz  ✓
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    PeriphClkInitStruct.PLL3.PLL3M = 5;
    PeriphClkInitStruct.PLL3.PLL3N = 36;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 20;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* LTDC clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();

    /**LTDC GPIO Configuration (netlist v45)
    PE4      ------> LTDC_B0
    PE5      ------> LTDC_G0
    PE6      ------> LTDC_G1
    PE11     ------> LTDC_G3
    PE12     ------> LTDC_B4
    PI9      ------> LTDC_VSYNC
    PI10     ------> LTDC_HSYNC
    PI14     ------> LTDC_CLK
    PI11     ------> LTDC_G6   (AF9)
    PF10     ------> LTDC_DE
    PC0      ------> LTDC_R5
    PA1      ------> LTDC_R2
    PA5      ------> LTDC_R4   (was PJ3 in old CubeMX output — netlist fix)
    PA6      ------> LTDC_G2   (was PI15 in old CubeMX output — netlist fix)
    PA10     ------> LTDC_B1
    PH2      ------> LTDC_R0
    PH3      ------> LTDC_R1
    PH4      ------> LTDC_G5   (AF9)
    PB0      ------> LTDC_R3   (AF9)
    PB1      ------> LTDC_R6   (AF9)
    PB10     ------> LTDC_G4
    PJ0      ------> LTDC_R7   (AF9)
    PD6      ------> LTDC_B2
    PD10     ------> LTDC_B3
    PK2      ------> LTDC_G7
    PK4      ------> LTDC_B5
    PK5      ------> LTDC_B6
    PK6      ------> LTDC_B7
    */

    /* GPIOE: B0, G0, G1, G3, B4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* GPIOI: VSYNC, HSYNC, CLK */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* GPIOI: G6 (AF9) */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /* GPIOF: DE */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* GPIOC: R5 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* GPIOA: R2, R4 (PA5), G2 (PA6), B1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* GPIOH: R0, R1 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* GPIOH: G5 (AF9) */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /* GPIOB: R3 (PB0), R6 (PB1) — AF9 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIOB: G4 (PB10) */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* GPIOJ: R7 (PJ0) — AF9 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
    HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

    /* GPIOD: B2, B3 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* GPIOK: G7, B5, B6, B7 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
    HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /* USER CODE BEGIN LTDC_MspInit 1 */
  /* PH7 = DISP — display standby control (active high = normal operation)
   * Held low; Display_Init() raises it after 10ms.
   */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_7, GPIO_PIN_RESET);

  /* PB11 = BL_PWM — DIO5661 EN (active high = backlight on, full brightness)
   * Held low; Display_Init() raises it after 250ms.
   */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
  /* USER CODE END LTDC_MspInit 1 */
  }
}

void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* ltdcHandle)
{

  if(ltdcHandle->Instance==LTDC)
  {
  /* USER CODE BEGIN LTDC_MspDeInit 0 */

  /* USER CODE END LTDC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LTDC_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_11|GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOI, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_14);
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11);
    HAL_GPIO_DeInit(GPIOJ, GPIO_PIN_0);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_6|GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOK, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

  /* USER CODE BEGIN LTDC_MspDeInit 1 */

  /* USER CODE END LTDC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
