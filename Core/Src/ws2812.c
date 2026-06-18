#include "ws2812b.h"
#include "tim.h"   // htim16 généré par CubeMX
#include <string.h>

extern TIM_HandleTypeDef htim16;

static uint16_t  dma_buf[WS2812B_BUF_SIZE];
static uint8_t   colors[WS2812B_NUM_LEDS][3]; // [i] = {R, G, B}
static volatile uint8_t busy = 0;

void WS2812B_Init(void) {
    memset(dma_buf, 0, sizeof(dma_buf));
    memset(colors, 0, sizeof(colors));
}

uint8_t WS2812B_IsReady(void) { return !busy; }

void WS2812B_SetColor(uint8_t i, uint8_t r, uint8_t g, uint8_t b) {
    if (i >= WS2812B_NUM_LEDS) return;
    colors[i][0] = r;
    colors[i][1] = g;
    colors[i][2] = b;
}

void WS2812B_SetAll(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < WS2812B_NUM_LEDS; i++)
        WS2812B_SetColor(i, r, g, b);
}

void WS2812B_Clear(void) { WS2812B_SetAll(0, 0, 0); }

static void encode_byte(uint8_t byte, uint16_t *buf) {
    for (int b = 7; b >= 0; b--)
        *buf++ = (byte & (1 << b)) ? WS2812B_T1H : WS2812B_T0H;
}

void WS2812B_Send(void) {
    if (busy) return;
    busy = 1;

    // Encoder GRB (ordre WS2812B)
    for (int i = 0; i < WS2812B_NUM_LEDS; i++) {
        encode_byte(colors[i][1], &dma_buf[i * 24]);      // G
        encode_byte(colors[i][0], &dma_buf[i * 24 + 8]);  // R
        encode_byte(colors[i][2], &dma_buf[i * 24 + 16]); // B
    }
    // Les derniers RESET_BITS sont déjà à 0 (memset initial)

    HAL_TIM_PWM_Start_DMA(&htim16, TIM_CHANNEL_1,
                          (uint32_t *)dma_buf, WS2812B_BUF_SIZE);
}

// Appelé automatiquement par HAL quand la DMA a fini
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM16) {
        HAL_TIM_PWM_Stop_DMA(&htim16, TIM_CHANNEL_1);
        busy = 0;
    }
}
