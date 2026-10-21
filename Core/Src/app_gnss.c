/**
  ******************************************************************************
  * @file    app_gnss.c
  * @brief   GNSS u-blox NEO-M9N (USART1) : configuration au démarrage puis tâche de réception.
  *
  * La tâche dort jusqu'à l'interruption IDLE de l'UART (fin d'un NAV-PVT, 25 fois par
  * seconde), décode la trame et publie la dernière solution dans xGnssFixQueue.
  * Les autres modules lisent avec Gnss_GetFix() : jamais bloquant.
  *
  * Pilote (UBX, DMA, parseur) : gnss.c.
  ******************************************************************************
  */
#include "app.h"

#define GNSS_RETRY_MS       2000    /* module absent ou refus de configuration : nouvel essai */
#define GNSS_LOST_MS        1000    /* sans NAV-PVT depuis 1 s => on publie "plus de fix"      */
#define GNSS_RECONFIG_MS    5000    /* sans NAV-PVT depuis 5 s => on reconfigure le module     */

/* Dernière solution. Queue de taille 1 + xQueueOverwrite = "boîte aux lettres" (comme la lumière) */
QueueHandle_t xGnssFixQueue = NULL;

void Gnss_CreateObjects(void)
{
  xGnssFixQueue = xQueueCreate(1, sizeof(Gnss_Fix_t));
  if (xGnssFixQueue == NULL) { Error_Handler(); }
}

void Gnss_HwInit(void)
{
  GNSS_Init();   /* DMA + IDLE sur USART1 ; le module est configuré par la tâche (peut durer ~1 s) */
}

bool Gnss_GetFix(Gnss_Fix_t *fix)
{
  return xQueuePeek(xGnssFixQueue, fix, 0) == pdTRUE;
}

void vTaskGNSS(void *pvParameters)
{
  (void)pvParameters;

  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;) {
    /* Configuration faite ici et non dans Gnss_HwInit : si le module est absent ou lent à
       démarrer, ça ne retarde pas l'affichage. On réessaie jusqu'à ce que ça réponde. */
    if (!GNSS_Configure()) {
      vTaskDelay(pdMS_TO_TICKS(GNSS_RETRY_MS));
      continue;
    }

    TickType_t lastPvt = xTaskGetTickCount();
    bool       lost    = false;

    for (;;) {
      GNSS_Wait(100);   /* réveillée à chaque fin de rafale UART ; 100 ms = garde-fou */

      Gnss_Fix_t fix;
      if (GNSS_Poll(&fix) > 0) {   /* plusieurs PVT d'un coup : seul le plus récent est publié */
        xQueueOverwrite(xGnssFixQueue, &fix);
        lastPvt = xTaskGetTickCount();
        lost    = false;
        continue;
      }

      TickType_t silence = xTaskGetTickCount() - lastPvt;

      if (silence > pdMS_TO_TICKS(GNSS_LOST_MS) && !lost) {
        /* Liaison coupée : on ne laisse pas une vieille position passer pour valide */
        if (Gnss_GetFix(&fix)) {
          fix.fixType = 0;
          fix.flags   = 0;
          fix.numSV   = 0;
          xQueueOverwrite(xGnssFixQueue, &fix);
        }
        lost = true;
      }

      if (silence > pdMS_TO_TICKS(GNSS_RECONFIG_MS)) {
        break;   /* module redémarré ou débit perdu : on refait toute la configuration */
      }
    }
  }
}
