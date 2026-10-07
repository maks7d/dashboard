/**
  ******************************************************************************
  * @file    app_buttons.c
  * @brief   Boutons de navigation SW1 (PG8), SW3 (PG6), SW4 (PG5).
  *
  * Chaque appui (front descendant : bouton vers GND + pull-up) est envoyé dans
  * xButtonQueue sous la forme du GPIO_PIN_x concerné. Pour l'instant seule la tâche
  * d'affichage les lit (elle affiche le dernier bouton pressé) ; plus tard, une tâche
  * "interface" pourra gérer les menus.
  ******************************************************************************
  */
#include "app.h"

#define BUTTON_DEBOUNCE_MS 30

QueueHandle_t xButtonQueue = NULL;

void Buttons_CreateObjects(void)
{
  xButtonQueue = xQueueCreate(8, sizeof(uint16_t));
  if (xButtonQueue == NULL) { Error_Handler(); }
}

/* Appelée depuis HAL_GPIO_EXTI_Callback (contexte INTERRUPTION). */
void Buttons_Irq(uint16_t GPIO_Pin, BaseType_t *pxHigherPriorityTaskWoken)
{
  static TickType_t lastTick = 0;
  static uint8_t    haveLast = 0;

  /* Les boutons sont configurés sur les deux fronts : on ne garde que l'appui */
  if (HAL_GPIO_ReadPin(POWER_BTN_PORT, GPIO_Pin) != GPIO_PIN_RESET) {
    return;
  }

  TickType_t now = xTaskGetTickCountFromISR();
  if (haveLast && (now - lastTick) < pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
    return;  /* rebond */
  }
  haveLast = 1;
  lastTick = now;

  xQueueSendFromISR(xButtonQueue, &GPIO_Pin, pxHigherPriorityTaskWoken);
}

const char *Buttons_Name(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin) {
    case GPIO_PIN_8: return "SW1";
    case GPIO_PIN_6: return "SW3";
    case GPIO_PIN_5: return "SW4";
    default:         return "?";
  }
}
