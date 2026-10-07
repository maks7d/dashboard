/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   Couche "disque" de FatFS au-dessus de la HAL SD (SDMMC1, 4 bits, DMA interne).
  *
  * Aucune attente active : le CPU lance un transfert DMA puis la tâche dort sur un
  * sémaphore ; l'interruption SDMMC1 la réveille à la fin du transfert. La programmation
  * interne de la carte (qui peut durer des dizaines de ms) est attendue avec vTaskDelay(1).
  *
  * Les données passent par un tampon intermédiaire en AXI SRAM (.dma_buffers) : le DMA de
  * SDMMC1 ne sait pas lire la DTCM, où se trouvent la plupart des variables.
  *
  * ATTENTION : FatFS n'est pas réentrant ici (FF_FS_REENTRANT = 0). Seule la tâche de log
  * (vTaskLog) doit appeler FatFS / ce pilote.
  ******************************************************************************
  */
#include "main.h"
#include "sdmmc.h"
#include "sd_card.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "ff.h"        /* types BYTE, LBA_t, UINT... nécessaires à diskio.h */
#include "diskio.h"
#include <string.h>

#define SD_SECTOR_SIZE       512u
#define SD_BOUNCE_SECTORS    16u                 /* 8 Ko par transfert DMA */
#define SD_XFER_TIMEOUT_MS   2000u
#define SD_READY_TIMEOUT_MS  2000u

extern SD_HandleTypeDef hsd1;

__attribute__((section(".dma_buffers"), aligned(32)))
static uint8_t s_bounce[SD_BOUNCE_SECTORS * SD_SECTOR_SIZE];

static SemaphoreHandle_t s_xferDone = NULL;
static volatile uint8_t  s_xferError = 0;
static uint8_t           s_cardReady = 0;

/* ===== Détection / ouverture de la carte ===== */
bool SDCard_IsInserted(void)
{
  return HAL_GPIO_ReadPin(CD_SD_PORT, CD_SD_PIN) == CD_SD_ACTIVE_LEVEL;
}

bool SDCard_Open(void)
{
  if (s_xferDone == NULL) {
    s_xferDone = xSemaphoreCreateBinary();
    if (s_xferDone == NULL) { return false; }
  }

  HAL_SD_DeInit(&hsd1);          /* repart d'un état propre (sans effet si jamais initialisée) */
  s_cardReady = 0;
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
    HAL_SD_DeInit(&hsd1);
    return false;
  }
  s_cardReady = 1;
  return true;
}

void SDCard_Close(void)
{
  s_cardReady = 0;
  HAL_SD_DeInit(&hsd1);
}

/* ===== Callbacks HAL (contexte interruption) ===== */
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_xferDone, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_xferDone, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
  (void)hsd;
  BaseType_t woken = pdFALSE;
  s_xferError = 1;
  xSemaphoreGiveFromISR(s_xferDone, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  HAL_SD_ErrorCallback(hsd);
}

/* ===== Attente de fin de programmation interne de la carte ===== */
static int sd_wait_card_ready(void)
{
  TickType_t t0 = xTaskGetTickCount();
  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER) {
    if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(SD_READY_TIMEOUT_MS)) { return 0; }
    vTaskDelay(1);   /* la carte programme sa mémoire : on rend le CPU */
  }
  return 1;
}

/* ===== Attente de fin de transfert DMA, puis de la carte ===== */
static int sd_wait_transfer(void)
{
  if (xSemaphoreTake(s_xferDone, pdMS_TO_TICKS(SD_XFER_TIMEOUT_MS)) != pdTRUE) {
    HAL_SD_Abort(&hsd1);
    return 0;
  }
  if (s_xferError) { return 0; }
  return sd_wait_card_ready();
}

/* ===== Horodatage des fichiers (FF_FS_NORTC = 0 dans ffconf.h) =====
 * Pas d'horloge temps réel sur la carte : date fixe pour l'instant. Quand le GNSS aura un fix,
 * renvoyer ici la date UTC (format FAT : bits 31-25 année-1980, 24-21 mois, 20-16 jour,
 * 15-11 heure, 10-5 minute, 4-0 secondes/2). */
DWORD get_fattime(void)
{
  return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);   /* 2026-01-01 00:00:00 */
}

/* ===== Interface FatFS ===== */
DSTATUS disk_status(BYTE pdrv)
{
  (void)pdrv;
  return s_cardReady ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
  (void)pdrv;
  return s_cardReady ? 0 : STA_NOINIT;   /* la carte est initialisée par SDCard_Open() */
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
  (void)pdrv;
  if (!s_cardReady) { return RES_NOTRDY; }

  while (count > 0) {
    UINT n = (count > SD_BOUNCE_SECTORS) ? SD_BOUNCE_SECTORS : count;
    s_xferError = 0;
    xSemaphoreTake(s_xferDone, 0);   /* purge un éventuel signal périmé */
    if (HAL_SD_ReadBlocks_DMA(&hsd1, s_bounce, (uint32_t)sector, n) != HAL_OK) { return RES_ERROR; }
    if (!sd_wait_transfer()) { return RES_ERROR; }
    memcpy(buff, s_bounce, n * SD_SECTOR_SIZE);
    buff += n * SD_SECTOR_SIZE; sector += n; count -= n;
  }
  return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
  (void)pdrv;
  if (!s_cardReady) { return RES_NOTRDY; }

  while (count > 0) {
    UINT n = (count > SD_BOUNCE_SECTORS) ? SD_BOUNCE_SECTORS : count;
    memcpy(s_bounce, buff, n * SD_SECTOR_SIZE);
    s_xferError = 0;
    xSemaphoreTake(s_xferDone, 0);
    if (HAL_SD_WriteBlocks_DMA(&hsd1, s_bounce, (uint32_t)sector, n) != HAL_OK) { return RES_ERROR; }
    if (!sd_wait_transfer()) { return RES_ERROR; }
    buff += n * SD_SECTOR_SIZE; sector += n; count -= n;
  }
  return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
  (void)pdrv;
  if (!s_cardReady) { return RES_NOTRDY; }

  switch (cmd) {
    case CTRL_SYNC:
      return sd_wait_card_ready() ? RES_OK : RES_ERROR;
    case GET_SECTOR_COUNT:
      *(LBA_t *)buff = hsd1.SdCard.LogBlockNbr;
      return RES_OK;
    case GET_SECTOR_SIZE:
      *(WORD *)buff = (WORD)hsd1.SdCard.LogBlockSize;
      return RES_OK;
    case GET_BLOCK_SIZE:
      *(DWORD *)buff = 1;
      return RES_OK;
    default:
      return RES_PARERR;
  }
}
