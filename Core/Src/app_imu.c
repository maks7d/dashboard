/**
  ******************************************************************************
  * @file    app_imu.c
  * @brief   IMU BMI270 (SPI1) : initialisation au démarrage puis tâche de lecture du FIFO.
  *
  * La tâche dort jusqu'à INT1 (PE0), levée par le capteur quand son FIFO contient
  * BMI270_FIFO_WATERMARK_MS de données. Elle lit alors tout le FIFO par DMA, publie le dernier
  * échantillon dans xImuQueue et enregistre chaque échantillon dans les logs.
  * Les autres modules lisent avec Imu_GetSample() : jamais bloquant.
  *
  * Pilote (SPI, FIFO, décodage) : bmi270.c.
  ******************************************************************************
  */
#include "app.h"

#define IMU_RETRY_MS         2000    /* capteur absent ou init ratée : nouvel essai                      */
#define IMU_POLL_TIMEOUT_MS  (BMI270_FIFO_WATERMARK_MS * 2u)   /* garde-fou si INT1 ne vient pas : on lit quand même */
#define IMU_LOST_MS          1000    /* sans aucun échantillon depuis 1 s : on réinitialise le capteur   */

/* Dernier échantillon. Queue de taille 1 + xQueueOverwrite = "boîte aux lettres" (comme la lumière) */
QueueHandle_t xImuQueue = NULL;

static TaskHandle_t s_task;

/* Compteurs lisibles au debugger : INT1 doit être proche du nombre de lectures. S'il vaut 0 alors
   que les lectures progressent, la tâche tourne sur son garde-fou : INT1 / PE0 est mal câblée. */
static volatile uint32_t s_irqCount;
static volatile uint32_t s_readCount;
static volatile uint32_t s_timeoutCount;

/* Échantillons décodés : en DTCM, ce n'est pas un tampon DMA */
static Bmi270_Sample_t s_samples[BMI270_FIFO_MAX_FRAMES];

void Imu_CreateObjects(void)
{
  xImuQueue = xQueueCreate(1, sizeof(Bmi270_Sample_t));
  if (xImuQueue == NULL) { Error_Handler(); }
}

bool Imu_GetSample(Bmi270_Sample_t *sample)
{
  return xQueuePeek(xImuQueue, sample, 0) == pdTRUE;
}

/* Appelée depuis l'interruption EXTI de PE0 (app_irq.c) */
void Imu_Irq(BaseType_t *pxHigherPriorityTaskWoken)
{
  s_irqCount++;
  if (s_task != NULL) {
    vTaskNotifyGiveFromISR(s_task, pxHigherPriorityTaskWoken);
  }
}

void vTaskIMU(void *pvParameters)
{
  (void)pvParameters;

  s_task = xTaskGetCurrentTaskHandle();
  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  for (;;) {
    /* Initialisation faite ici et non dans la tâche d'init : si le capteur est absent ou lent,
       ça ne retarde pas l'affichage. On réessaie jusqu'à ce qu'il réponde. */
    if (!BMI270_Init()) {
      vTaskDelay(pdMS_TO_TICKS(IMU_RETRY_MS));
      continue;
    }

    (void)ulTaskNotifyTake(pdTRUE, 0);          /* oublie les INT1 survenues pendant l'init */
    TickType_t lastData = xTaskGetTickCount();

    for (;;) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_POLL_TIMEOUT_MS)) == 0u) {
        s_timeoutCount++;
      }

      uint32_t n;
      do {
        Bmi270_FifoInfo_t info;
        n = BMI270_ReadFifo(s_samples, BMI270_FIFO_MAX_FRAMES, &info);
        s_readCount++;

        if (n > 0u) {
          for (uint32_t i = 0; i < n; i++) {
            Log_ImuSample((const int16_t *)&s_samples[i]);
          }
          xQueueOverwrite(xImuQueue, &s_samples[n - 1u]);
          lastData = xTaskGetTickCount();
        }
        /* INT1 encore haute = le FIFO est resté au-dessus du seuil : pas de nouveau front à
           attendre, on relit tout de suite (INT1 n'est pas verrouillée, voir INT_LATCH). */
      } while (n > 0u && HAL_GPIO_ReadPin(IMU_INT_PORT, IMU_INT_PIN) == GPIO_PIN_SET);

      if ((xTaskGetTickCount() - lastData) > pdMS_TO_TICKS(IMU_LOST_MS)) {
        break;                                  /* capteur muet : on refait toute l'initialisation */
      }
    }
  }
}
