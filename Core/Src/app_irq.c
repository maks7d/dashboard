/**
  ******************************************************************************
  * @file    app_irq.c
  * @brief   Callback HAL unique pour toutes les interruptions GPIO (EXTI).
  *
  * La HAL appelle HAL_GPIO_EXTI_Callback() pour TOUTES les broches EXTI ; on ne peut en
  * définir qu'une seule dans le projet. Elle aiguille donc selon la broche vers le module
  * concerné. Tout ce qui est appelé ici s'exécute en contexte INTERRUPTION : rapide,
  * sans attente, et uniquement des fonctions FreeRTOS "...FromISR".
  *
  * Le chemin complet d'une interruption :
  *   broche -> EXTIx_IRQHandler (stm32h7xx_it.c) -> HAL_GPIO_EXTI_IRQHandler (HAL)
  *          -> HAL_GPIO_EXTI_Callback (ici) -> module (Power_ButtonIrq, Buttons_Irq...)
  ******************************************************************************
  */
#include "app.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  switch (GPIO_Pin) {
    case POWER_BTN_PIN:            /* PG7 : bouton power */
      Power_ButtonIrq(&xHigherPriorityTaskWoken);
      break;

    case GPIO_PIN_5:               /* PG5 : SW4 */
    case GPIO_PIN_6:               /* PG6 : SW3 */
    case GPIO_PIN_8:               /* PG8 : SW1 */
      Buttons_Irq(GPIO_Pin, &xHigherPriorityTaskWoken);
      break;

    case GPIO_PIN_0:               /* PE0 : IMU_INT1 */
      /* TODO : notifier la future tâche IMU (xTaskNotifyFromISR / queue) */
      break;

    case GPIO_PIN_4:               /* PI4 : INT_LIGHT_SENSOR */
      /* TODO : seuils d'interruption du VEML6030 (non utilisés pour l'instant) */
      break;

    default:
      break;
  }

  /* Si une tâche plus prioritaire vient d'être débloquée, on lui donne la main dès la
     sortie de l'interruption plutôt que d'attendre le prochain tick. */
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
