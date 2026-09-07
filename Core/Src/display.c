#include "display.h"
#include "ltdc.h"
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

/* BL_PWM = PB11 (DIO5661 EN pin, active high for full brightness) */
#define BL_PORT     GPIOB
#define BL_PIN      GPIO_PIN_11

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
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_SET);
}

void Display_PowerOff(void)
{
    /* Riverdi datasheet power-off sequence:
     *   backlight off -> 5ms -> DISP low -> 80ms (internal voltage discharge)
     */
    HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(DISP_PORT, DISP_PIN, GPIO_PIN_RESET);
    HAL_Delay(80);
}

void Display_SetBrightness(uint8_t percent)
{
    /* GPIO-only implementation: 0=DIO5661 shutdown, >0=full brightness.
     * For smooth dimming, configure LPTIM2 on PB11 (AF3=LPTIM2_OUT) and
     * replace this function with a PWM duty-cycle update. */
    if (percent == 0)
        HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(BL_PORT, BL_PIN, GPIO_PIN_SET);
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