#include "display.h"
#include "ltdc.h"
#include "tim.h"
#include <string.h>

/*
 * Framebuffer in AXI SRAM (.dma_buffers, 0x24000000).
 * Aligned to 32 bytes for D-cache line management (STM32H7 cache line = 32B).
 * Size: 480 * 272 * 3 = 391 680 bytes (~383 KB).
 */
__attribute__((section(".dma_buffers"), aligned(32)))
uint8_t display_fb[DISPLAY_WIDTH * DISPLAY_HEIGHT * 3];

/* DISP = PH7 (display enable, active high) */
#define DISP_PORT   GPIOH
#define DISP_PIN    GPIO_PIN_7

/* BL_PWM = PB11 = TIM2_CH4 (AF1), DIO5661 EN pin: PWM on EN dims the backlight.
 * TIM2 is shared with the LAP_DET input capture (CH1, PA0); that capture only
 * uses the interrupt, not the counter value, so TIM2's period can be set for PWM.
 * Check the DIO5661 datasheet for the allowed EN PWM frequency range. */
#define BL_PORT         GPIOB
#define BL_PIN          GPIO_PIN_11
#define BL_PWM_CHANNEL  TIM_CHANNEL_4
#define BL_PWM_FREQ_HZ  10000U

/* Timer ticks per PWM period (= ARR + 1), computed in Display_BacklightPwmInit() */
static uint32_t bl_pwm_period;

static void Display_BacklightPwmInit(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef oc = {0};

    /* APB1 prescaler != 1 -> TIM2 kernel clock = 2 x PCLK1 (200 MHz here) */
    uint32_t tick_hz = (HAL_RCC_GetPCLK1Freq() * 2U) / (htim2.Init.Prescaler + 1U);
    bl_pwm_period = tick_hz / BL_PWM_FREQ_HZ;

    /* PB11: GPIO output -> TIM2_CH4 */
    gpio.Pin       = BL_PIN;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(BL_PORT, &gpio);

    /* TIM2 was initialised by CubeMX as free-running 32-bit input capture:
     * give it a real period, then force an update so the counter restarts
     * from 0 (otherwise it would first have to count up to 2^32). */
    __HAL_TIM_SET_AUTORELOAD(&htim2, bl_pwm_period - 1U);
    HAL_TIM_GenerateEvent(&htim2, TIM_EVENTSOURCE_UPDATE);

    oc.OCMode     = TIM_OCMODE_PWM1;        /* high while CNT < CCR */
    oc.Pulse      = 0;                      /* start at 0% */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;    /* EN active high */
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc, BL_PWM_CHANNEL) != HAL_OK)
        Error_Handler();
    if (HAL_TIM_PWM_Start(&htim2, BL_PWM_CHANNEL) != HAL_OK)
        Error_Handler();
}

void Display_Init(void)
{
    /* Clear framebuffer to black before enabling display */
    memset(display_fb, 0, sizeof(display_fb));
    Display_Flush();

    /* Set LTDC layer 0 framebuffer address now that display_fb is ready */
    HAL_LTDC_SetAddress(&hltdc, (uint32_t)display_fb, LTDC_LAYER_1);

    /* Riverdi datasheet power-on sequence:
     *  t0: VDD stable + reset high (done at startup)
     *  t1 (min 10ms): DISP high
     *  t2 (min 250ms after display signals start): backlight on
     */
    HAL_Delay(10);
    HAL_GPIO_WritePin(DISP_PORT, DISP_PIN, GPIO_PIN_SET);

    HAL_Delay(250);
    Display_BacklightPwmInit();
    Display_SetBrightness(100);
}

void Display_PowerOff(void)
{
    /* Riverdi datasheet power-off sequence:
     *   backlight off -> 5ms -> DISP low -> 80ms (internal voltage discharge)
     */
    Display_SetBrightness(0);
    HAL_Delay(5);
    HAL_GPIO_WritePin(DISP_PORT, DISP_PIN, GPIO_PIN_RESET);
    HAL_Delay(80);
}

void Display_SetBrightness(uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    /* duty = percent %: CCR = 0 keeps EN low (DIO5661 shutdown),
     * CCR = period (> ARR) keeps EN high (100 %). */
    uint32_t pulse = (bl_pwm_period * percent) / 100U;
    __HAL_TIM_SET_COMPARE(&htim2, BL_PWM_CHANNEL, pulse);
}

void Display_Clear(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t *p = display_fb;
    uint32_t n = DISPLAY_WIDTH * DISPLAY_HEIGHT;

    if (r == g && g == b) {
        memset(display_fb, r, sizeof(display_fb));
    } else {
        for (uint32_t i = 0; i < n; i++) {
            *p++ = b;
            *p++ = g;
            *p++ = r;
        }
    }
    Display_Flush();
}

void Display_DrawPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b)
{
    if (x >= DISPLAY_WIDTH || y >= DISPLAY_HEIGHT)
        return;

    /* RGB888, BGR byte order: [B][G][R] at increasing addresses */
    uint32_t offset = ((uint32_t)y * DISPLAY_WIDTH + x) * 3;
    display_fb[offset + 0] = b;
    display_fb[offset + 1] = g;
    display_fb[offset + 2] = r;
}

void Display_Flush(void)
{
    /* Clean D-cache for the entire framebuffer so LTDC sees CPU writes */
    SCB_CleanDCache_by_Addr((uint32_t *)display_fb, sizeof(display_fb));
}

void Display_ShowText(uint16_t x, uint16_t y, char *text, uint16_t color, uint16_t bgcolor)
{
    /* Simple text rendering: draw each character as a 5x7 pixel font.
     * For simplicity, this implementation is omitted. In practice, you would
     * use a font library or implement a basic bitmap font renderer here. */
    (void)x;
    (void)y;
    (void)text;
    (void)color;
    (void)bgcolor;
}