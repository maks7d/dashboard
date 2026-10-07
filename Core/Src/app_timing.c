/**
  ******************************************************************************
  * @file    app_timing.c
  * @brief   Chronométrage précis : temps au tour (PA0) et fréquence RPM (PH11) avec TIM5.
  *
  * Principe (zéro interruption, zéro CPU par impulsion) :
  *
  *   front sur la broche
  *     -> TIM5 copie son compteur 32 bits (200 MHz, 1 tick = 5 ns) dans CCRx  [matériel]
  *     -> le DMA copie CCRx dans un tampon circulaire en RAM                  [matériel]
  *     -> toutes les 10 ms, vTaskTiming lit les nouveaux horodatages et calcule.
  *
  * L'horodatage est pris par le silicium au front exact : la précision (5 ns) ne dépend
  * NI de la latence d'interruption, NI de FreeRTOS, NI de la charge du CPU. Rien n'est
  * bloquant : la tâche ne fait que lire de la RAM puis se rendort ; un tour peut être
  * capturé même si le CPU est occupé ailleurs, il sera lu au prochain passage de la tâche.
  *
  * Compteur 32 bits = débordement toutes les 21,47 s : on l'étend en 64 bits dans la tâche
  * (qui passe bien plus souvent que 21 s), ce qui permet de chronométrer des tours de
  * plusieurs minutes sans perdre la résolution de 5 ns.
  *
  * Précision absolue = celle du quartz (Y1) qui cadence le PLL.
  ******************************************************************************
  */
#include "app.h"
#include "tim.h"
#include <string.h>

#define CAP_BUF_LEN       64u     /* entrées par canal, DOIT être une puissance de 2 */
#define TIMING_POLL_MS    10      /* période de lecture des tampons                  */
#define LAP_MIN_TIME_MS   5000    /* impulsions plus rapprochées = parasite/double déclenchement */
#define RPM_TIMEOUT_MS    1000    /* sans impulsion depuis 1 s => régime = 0         */

QueueHandle_t xLapTimeQueue = NULL;

/* Tampons écrits par le DMA : doivent être en AXI SRAM (.dma_buffers), pas en DTCM (voir ws2812.c).
   Alignés sur 32 octets (ligne de cache) pour pouvoir invalider le cache D si on l'active un jour. */
__attribute__((section(".dma_buffers"), aligned(32))) static uint32_t s_lapBuf[CAP_BUF_LEN];
__attribute__((section(".dma_buffers"), aligned(32))) static uint32_t s_rpmBuf[CAP_BUF_LEN];

typedef struct {
  uint32_t          *buf;
  DMA_HandleTypeDef *hdma;
  uint32_t           rd;     /* prochain index à lire */
} CapChannel_t;

static CapChannel_t s_lap = { s_lapBuf, NULL, 0 };
static CapChannel_t s_rpm = { s_rpmBuf, NULL, 0 };

static uint32_t s_tickHz;               /* fréquence du compteur TIM5 */
static uint64_t s_now64;                /* compteur étendu en 64 bits à la dernière lecture */
static uint32_t s_lastCnt32;

/* Tour */
static uint8_t  s_haveLap;
static uint64_t s_lastLapTs;

/* RPM : écrits par vTaskTiming, lus par n'importe quelle tâche (accès 32 bits atomiques) */
static uint8_t           s_haveRpm;
static uint64_t          s_prevRpmTs;
static volatile uint32_t s_rpmPeriodTicks;
static volatile uint32_t s_rpmLastTs32;

void Timing_CreateObjects(void)
{
  /* Durées de tour en MICROSECONDES (uint32 => jusqu'à 71 minutes) */
  xLapTimeQueue = xQueueCreate(10, sizeof(uint32_t));
  if (xLapTimeQueue == NULL) { Error_Handler(); }
}

void Timing_HwInit(void)
{
  /* APB1 prescaler != 1 => horloge des timers = 2 x PCLK1 (200 MHz ici, voir SystemClock_Config) */
  s_tickHz = (HAL_RCC_GetPCLK1Freq() * 2u) / (htim5.Init.Prescaler + 1u);

  s_lap.hdma = htim5.hdma[TIM_DMA_ID_CC1];
  s_rpm.hdma = htim5.hdma[TIM_DMA_ID_CC2];

  /* .dma_buffers n'est pas initialisée au démarrage */
  memset(s_lapBuf, 0, sizeof(s_lapBuf));
  memset(s_rpmBuf, 0, sizeof(s_rpmBuf));

  s_lastCnt32 = __HAL_TIM_GET_COUNTER(&htim5);
  s_now64     = s_lastCnt32;

  if (HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_1, s_lapBuf, CAP_BUF_LEN) != HAL_OK) { Error_Handler(); }
  if (HAL_TIM_IC_Start_DMA(&htim5, TIM_CHANNEL_2, s_rpmBuf, CAP_BUF_LEN) != HAL_OK) { Error_Handler(); }
}

/* Index du prochain emplacement que le DMA va écrire (NDTR décompte de CAP_BUF_LEN à 1) */
static uint32_t Cap_WritePos(const CapChannel_t *c)
{
  return (CAP_BUF_LEN - __HAL_DMA_GET_COUNTER(c->hdma)) & (CAP_BUF_LEN - 1u);
}

static void Cap_InvalidateCache(const CapChannel_t *c)
{
  /* Sans effet tant que le cache D est désactivé (cas actuel) ; protège si on l'active */
  if (SCB->CCR & SCB_CCR_DC_Msk) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)c->buf, CAP_BUF_LEN * sizeof(uint32_t));
  }
}

static void Lap_OnPulse(uint64_t ts)
{
  if (s_haveLap) {
    uint64_t dt = ts - s_lastLapTs;
    if (dt < ((uint64_t)(s_tickHz / 1000u) * LAP_MIN_TIME_MS)) {
      return;  /* trop proche du précédent : parasite, on garde l'ancienne référence */
    }
    uint64_t us = (dt * 1000000u) / s_tickHz;
    if (us <= UINT32_MAX) {
      uint32_t lapUs = (uint32_t)us;
      xQueueSend(xLapTimeQueue, &lapUs, 0);   /* timeout 0 : jamais bloquant (queue pleine = perdu) */
    }
  }
  s_lastLapTs = ts;
  s_haveLap   = 1;
}

static void Rpm_OnPulse(uint64_t ts)
{
  if (s_haveRpm && (ts - s_prevRpmTs) < s_tickHz) {   /* période < 1 s, sinon repart de zéro */
    s_rpmPeriodTicks = (uint32_t)(ts - s_prevRpmTs);
  }
  s_prevRpmTs   = ts;
  s_rpmLastTs32 = (uint32_t)ts;
  s_haveRpm     = 1;
}

static void Timing_Poll(void)
{
  /* ORDRE IMPORTANT : d'abord les positions d'écriture du DMA, ENSUITE le compteur.
     Ainsi tout horodatage lu est forcément antérieur au compteur lu (sinon la différence
     modulaire "compteur - horodatage" donnerait ~21 s). */
  uint32_t wrLap = Cap_WritePos(&s_lap);
  uint32_t wrRpm = Cap_WritePos(&s_rpm);
  uint32_t cnt   = __HAL_TIM_GET_COUNTER(&htim5);

  s_now64    += (uint32_t)(cnt - s_lastCnt32);   /* étend le compteur 32 -> 64 bits */
  s_lastCnt32 = cnt;

  Cap_InvalidateCache(&s_lap);
  Cap_InvalidateCache(&s_rpm);

  while (s_lap.rd != wrLap) {
    uint32_t ts32 = s_lapBuf[s_lap.rd];
    s_lap.rd = (s_lap.rd + 1u) & (CAP_BUF_LEN - 1u);
    Lap_OnPulse(s_now64 - (uint32_t)(cnt - ts32));
  }

  while (s_rpm.rd != wrRpm) {
    uint32_t ts32 = s_rpmBuf[s_rpm.rd];
    s_rpm.rd = (s_rpm.rd + 1u) & (CAP_BUF_LEN - 1u);
    Rpm_OnPulse(s_now64 - (uint32_t)(cnt - ts32));
  }
}

void vTaskTiming(void *pvParameters)
{
  (void)pvParameters;

  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    Timing_Poll();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TIMING_POLL_MS));
  }
}

/* Impulsions par minute (à diviser par le nombre d'impulsions par tour du capteur) */
uint32_t Rpm_GetPulsesPerMinute(void)
{
  uint32_t period = s_rpmPeriodTicks;
  uint32_t age    = __HAL_TIM_GET_COUNTER(&htim5) - s_rpmLastTs32;   /* modulo 2^32 */

  if (period == 0 || age > (s_tickHz / 1000u) * RPM_TIMEOUT_MS) {
    return 0;
  }
  return (uint32_t)(((uint64_t)s_tickHz * 60u) / period);
}
