/**
  ******************************************************************************
  * @file    app_light.c
  * @brief   Capteur de lumière VEML6030 (I2C1) : fonctions de bus + tâche de lecture.
  *
  * Le pilote veml6030.c ne connaît pas le microcontrôleur : il appelle les fonctions
  * VEML6030_I2C_xxx ci-dessous (la "glue" entre le pilote et la HAL STM32).
  ******************************************************************************
  */
#include "app.h"
#include "i2c.h"
#include "veml6030.h"

extern I2C_HandleTypeDef hi2c1;

/* Dernière valeur de lux. Queue de taille 1 + xQueueOverwrite = "boîte aux lettres" :
   l'écrivain remplace toujours la valeur, le lecteur (xQueuePeek) voit la plus récente. */
QueueHandle_t xLightSensorQueue = NULL;

/* Mutex = verrou pour ne pas avoir deux tâches qui parlent en même temps sur I2C1 */
static SemaphoreHandle_t xI2C1Mutex = NULL;
static uint8_t s_lightOk = 0;

/* ===== Fonctions de bus demandées par le pilote ===== */
static int32_t VEML6030_I2C_Init(void)
{
  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    MX_I2C1_Init(); /* réinitialiser le bus I2C si nécessaire */
  }
  return VEML6030_OK;
}

static int32_t VEML6030_I2C_DeInit(void)
{
  HAL_I2C_DeInit(&hi2c1);
  return VEML6030_OK;
}

static int32_t VEML6030_I2C_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (HAL_I2C_Mem_Write(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}

static int32_t VEML6030_I2C_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length)
{
  if (HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}

static int32_t VEML6030_I2C_IsReady(uint16_t Addr, uint32_t Trials)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, Addr, Trials, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}

/* HAL_GetTick() (et non xTaskGetTickCount) : le pilote l'utilise pour ses attentes, et
   HAL_GetTick() avance toujours (SysTick), même avant le démarrage du scheduler. */
static int32_t VEML6030_GetTick(void)
{
  return (int32_t)HAL_GetTick();
}

static VEML6030_Object_t veml6030;
static VEML6030_IO_t veml6030_io = {
  .Init         = VEML6030_I2C_Init,
  .DeInit       = VEML6030_I2C_DeInit,
  .ReadAddress  = VEML6030_I2C_READ_ADD,  /* 0x21 */
  .WriteAddress = VEML6030_I2C_WRITE_ADD, /* 0x20 */
  .IsReady      = VEML6030_I2C_IsReady,
  .WriteReg     = VEML6030_I2C_WriteReg,
  .ReadReg      = VEML6030_I2C_ReadReg,
  .GetTick      = VEML6030_GetTick
};

void Light_CreateObjects(void)
{
  xLightSensorQueue = xQueueCreate(1, sizeof(uint16_t));
  if (xLightSensorQueue == NULL) { Error_Handler(); }

  xI2C1Mutex = xSemaphoreCreateMutex();
  if (xI2C1Mutex == NULL) { Error_Handler(); }
}

uint8_t Light_HwInit(void)
{
  s_lightOk = 0;
  VEML6030_RegisterBusIO(&veml6030, &veml6030_io);
  if (VEML6030_Init(&veml6030) != VEML6030_OK) {
    return 0;  /* capteur absent / ADDR flottante : le dashboard fonctionne sans lui */
  }
  VEML6030_SetExposureTime(&veml6030, VEML6030_CONF_IT100);                 /* 100 ms */
  VEML6030_SetGain(&veml6030, VEML6030_ALS_CHANNEL, VEML6030_CONF_GAIN_1);  /* gain 1x */
  VEML6030_Start(&veml6030, VEML6030_MODE_CONTINUOUS);
  s_lightOk = 1;
  return 1;
}

void vTaskLightSensor(void *pvParameters)
{
  (void)pvParameters;

  /* Attendre que la tâche d'init ait terminé (bit posé dans le groupe d'événements) */
  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  if (!s_lightOk) {
    vTaskDelete(NULL);  /* pas de capteur : inutile de tourner pour rien */
  }

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(200);  /* lecture toutes les 200 ms */

  for (;;) {
    if (xSemaphoreTake(xI2C1Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      uint32_t als_value = 0;
      if (VEML6030_GetValues(&veml6030, &als_value) == VEML6030_OK) {
        uint16_t lux = (uint16_t)als_value;
        xQueueOverwrite(xLightSensorQueue, &lux);
      }
      xSemaphoreGive(xI2C1Mutex);
    }

    /* vTaskDelayUntil : période FIXE (200 ms entre deux débuts de lecture),
       contrairement à vTaskDelay qui attendrait 200 ms APRÈS la fin du travail. */
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}
