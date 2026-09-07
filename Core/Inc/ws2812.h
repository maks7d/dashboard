#pragma once
#include "stm32h7xx_hal.h"

#define WS2812B_NUM_LEDS    10
#define WS2812B_T0H         80    /* ~0.4µs @ 200MHz (TIM16 clock = APB2x2 = 200MHz)(voir tim.c) */
#define WS2812B_T1H         160   /* ~0.8µs @ 200MHz */
#define WS2812B_RESET_BITS  50    // >50µs reset

#define WS2812B_BUF_SIZE    (WS2812B_NUM_LEDS * 24 + WS2812B_RESET_BITS)

void WS2812B_Init(void);
void WS2812B_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void WS2812B_SetAll(uint8_t r, uint8_t g, uint8_t b);
void WS2812B_Clear(void);
void WS2812B_Send(void);
uint8_t WS2812B_IsReady(void);
