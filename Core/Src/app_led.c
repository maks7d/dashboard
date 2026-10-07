/**
  ******************************************************************************
  * @file    app_led.c
  * @brief   Bandeau de 10 LEDs WS2812B (TIM16 CH1 + DMA1 Stream0).
  *
  * WS2812B_SetAll()/SetColor() ne font que modifier un tableau en mémoire ;
  * il faut appeler WS2812B_Send() pour que le DMA envoie réellement les couleurs aux LEDs.
  ******************************************************************************
  */
#include "app.h"
#include "ws2812.h"

void Led_HwInit(void)
{
  WS2812B_Init();
}

/* Tâche de démonstration : cycle rouge / vert / bleu (à remplacer par les vraies
   indications : shift-light RPM, alertes température, etc.) */
void vTaskLED(void *pvParameters)
{
  (void)pvParameters;
  uint8_t color = 0;

  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;) {
    switch (color % 3) {
      case 0: WS2812B_SetAll(255, 0, 0); break; /* Rouge */
      case 1: WS2812B_SetAll(0, 255, 0); break; /* Vert  */
      case 2: WS2812B_SetAll(0, 0, 255); break; /* Bleu  */
    }
    WS2812B_Send();
    color++;
    vTaskDelay(pdMS_TO_TICKS(1000));

    WS2812B_Clear();
    WS2812B_Send();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
