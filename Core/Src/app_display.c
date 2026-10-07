/**
  ******************************************************************************
  * @file    app_display.c
  * @brief   Tâche d'affichage : lux, dernier temps au tour, dernier bouton pressé.
  *
  * La tâche est la SEULE à écrire dans le framebuffer : en donnant la propriété d'une
  * ressource à une seule tâche, on évite d'avoir besoin de mutex. Les autres modules lui
  * envoient leurs données par queue.
  ******************************************************************************
  */
#include "app.h"
#include "display.h"
#include <stdio.h>

void Display_HwInit(void)
{
  Display_Init();  /* ~350 ms d'attentes imposées par la datasheet de l'écran */
}

void vTaskDisplay(void *pvParameters)
{
  (void)pvParameters;
  uint16_t lux = 0;
  uint32_t lapTimeUs = 0;
  uint16_t pin = 0;
  char buffer[48];

  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;) {
    /* xQueuePeek : lit SANS retirer la valeur (la dernière valeur de lux reste dispo) */
    if (xQueuePeek(xLightSensorQueue, &lux, 0) == pdPASS) {
      /* snprintf (et non sprintf) : ne dépasse jamais la taille du buffer.
         Le "%-10u" complète avec des espaces pour effacer les anciens chiffres. */
      snprintf(buffer, sizeof(buffer), "Lux: %-10u", lux);
      Display_ShowText(10, 10, buffer, WHITE, BLACK);
    }

    /* xQueueReceive avec timeout 0 : ne bloque pas, prend un temps de tour s'il y en a un */
    if (xQueueReceive(xLapTimeQueue, &lapTimeUs, 0) == pdPASS) {
      /* la queue contient des microsecondes : affichage "secondes.millisecondes" */
      snprintf(buffer, sizeof(buffer), "Lap: %lu.%03lu s    ",
               (unsigned long)(lapTimeUs / 1000000u), (unsigned long)((lapTimeUs / 1000u) % 1000u));
      Display_ShowText(10, 30, buffer, WHITE, BLACK);
    }

    if (xQueueReceive(xButtonQueue, &pin, 0) == pdPASS) {
      snprintf(buffer, sizeof(buffer), "Btn: %-4s", Buttons_Name(pin));
      Display_ShowText(10, 50, buffer, WHITE, BLACK);
    }

    vTaskDelay(pdMS_TO_TICKS(100));  /* rafraîchissement à 10 Hz ; la tâche dort le reste du temps */
  }
}
