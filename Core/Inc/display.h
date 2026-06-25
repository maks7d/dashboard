#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define DISPLAY_WIDTH   480
#define DISPLAY_HEIGHT  272

/*
 * Framebuffer layout: RGB888, BGR byte order in memory (LTDC convention)
 *   offset+0 = Blue, offset+1 = Green, offset+2 = Red
 * Size: 480 * 272 * 3 = 391 680 bytes, placed in AXI SRAM (.dma_buffers section)
 */
extern uint8_t display_fb[DISPLAY_WIDTH * DISPLAY_HEIGHT * 3];

/* Power sequence (follows Riverdi RVT43HLTFWN00 datasheet):
 *   Init:  DISP high after 10ms, backlight on after 250ms
 *   Off:   backlight off, 5ms, DISP low, 80ms
 */
void Display_Init(void);
void Display_PowerOff(void);

/*
 * Brightness: 0=off (DIO5661 shutdown), 1-100=full on (GPIO only, no PWM dimming yet)
 * PWM dimming via LPTIM2 on PB11 can be added later for smooth control.
 */
void Display_SetBrightness(uint8_t percent);

void Display_Clear(uint8_t r, uint8_t g, uint8_t b);
void Display_DrawPixel(uint16_t x, uint16_t y, uint8_t r, uint8_t g, uint8_t b);

/* Flush D-cache to make CPU writes visible to LTDC over AXI bus */
void Display_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __DISPLAY_H__ */
