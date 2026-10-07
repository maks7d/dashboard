/**
  ******************************************************************************
  * @file    app_power.c
  * @brief   Bouton marche/arrêt (SW2_PWR sur PG7) et maintien d'alimentation (PE3).
  *
  * Fonctionnement matériel : appuyer sur S2 met la carte sous tension (via le PMOS Q1).
  * Le firmware doit alors mettre PE3 (POWER_HOLD) à 1 pour "tenir" l'alimentation même
  * quand on relâche le bouton (c'est fait au tout début de main()). Pour éteindre, on
  * remet PE3 à 0.
  *
  * Fonctionnement logiciel :
  *   - l'ISR (Power_ButtonIrq) ne fait que dire "appui" ou "relâchement" dans une queue ;
  *   - la tâche mesure la durée : relâché avant 500 ms = appui court,
  *     toujours appuyé après 5 s = appui long = extinction.
  ******************************************************************************
  */
#include "app.h"

#define POWER_DEBOUNCE_MS        20     /* on ignore les fronts plus rapprochés que ça (rebonds) */
#define POWER_SHORT_PRESS_MAX_MS 500
#define POWER_LONG_PRESS_MS      5000

typedef enum {
  POWER_EVT_PRESSED = 1,
  POWER_EVT_RELEASED
} PowerEvent_t;

static QueueHandle_t xPowerEventQueue = NULL;

void Power_CreateObjects(void)
{
  xPowerEventQueue = xQueueCreate(8, sizeof(uint8_t));
  if (xPowerEventQueue == NULL) { Error_Handler(); }
}

/* Appelée depuis HAL_GPIO_EXTI_Callback (contexte INTERRUPTION). */
void Power_ButtonIrq(BaseType_t *pxHigherPriorityTaskWoken)
{
  static TickType_t lastEdgeTick = 0;
  static uint8_t    haveLastEdge = 0;

  /* Variante "FromISR" obligatoire dans une interruption (xTaskGetTickCount() est interdit) */
  TickType_t now = xTaskGetTickCountFromISR();
  if (haveLastEdge && (now - lastEdgeTick) < pdMS_TO_TICKS(POWER_DEBOUNCE_MS)) {
    return;  /* rebond */
  }
  haveLastEdge = 1;
  lastEdgeTick = now;

  /* On lit le NIVEAU de la broche plutôt que de deviner le sens du front :
     bouton vers GND + pull-up => appuyé = niveau bas. */
  uint8_t evt = (HAL_GPIO_ReadPin(POWER_BTN_PORT, POWER_BTN_PIN) == GPIO_PIN_RESET)
                  ? POWER_EVT_PRESSED : POWER_EVT_RELEASED;
  xQueueSendFromISR(xPowerEventQueue, &evt, pxHigherPriorityTaskWoken);
}

static void Power_OnShortPress(void)
{
  /* TODO : action d'appui court (ex: changer de page, régler la luminosité...) */
}

static void Power_Off(void)
{
  /* TODO : avant de couper, sauvegarder les logs / éteindre l'écran proprement */
  HAL_GPIO_WritePin(POWER_HOLD_PORT, POWER_HOLD_PIN, GPIO_PIN_RESET);
}

void vTaskPowerButton(void *pvParameters)
{
  (void)pvParameters;
  uint8_t evt;

  for (;;) {
    /* Bloquée (0 % CPU) jusqu'à un événement : c'est tout l'intérêt d'une queue. */
    xQueueReceive(xPowerEventQueue, &evt, portMAX_DELAY);
    if (evt != POWER_EVT_PRESSED) { continue; }

    TickType_t pressTick = xTaskGetTickCount();

    /* On attend le relâchement, mais au maximum 5 s : le timeout EST la détection d'appui long. */
    if (xQueueReceive(xPowerEventQueue, &evt, pdMS_TO_TICKS(POWER_LONG_PRESS_MS)) == pdPASS) {
      if (evt == POWER_EVT_RELEASED) {
        TickType_t held = xTaskGetTickCount() - pressTick;
        if (held < pdMS_TO_TICKS(POWER_SHORT_PRESS_MAX_MS)) {
          Power_OnShortPress();
        }
        /* entre 500 ms et 5 s : appui "moyen", ignoré */
      }
    } else {
      /* Timeout : on revérifie la broche, car l'anti-rebond a pu "manger" un front de
         relâchement. On n'éteint que si le bouton est VRAIMENT toujours enfoncé. */
      if (HAL_GPIO_ReadPin(POWER_BTN_PORT, POWER_BTN_PIN) == GPIO_PIN_RESET) {
        Power_Off();
      }
    }
  }
}
