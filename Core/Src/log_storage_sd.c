/**
  ******************************************************************************
  * @file    log_storage_sd.c
  * @brief   Support "carte microSD" : FatFS (FAT32 / exFAT), lisible directement sur un PC.
  *
  * Règles de sécurité vis-à-vis de la carte de l'utilisateur :
  *   - JAMAIS de formatage (f_mkfs n'est même pas compilé : FF_USE_MKFS = 0). Une carte
  *     illisible ou sans système de fichiers reconnu est simplement ignorée.
  *   - On ne crée et ne modifie que le dossier DASH/ ; les autres fichiers ne sont jamais
  *     touchés ni supprimés.
  *   - Les fichiers sont créés en "création exclusive" : aucun fichier existant n'est écrasé.
  ******************************************************************************
  */
#include "log_storage.h"
#include "log_config.h"
#include "sd_card.h"
#include "ff.h"
#include <stdio.h>

static FATFS s_fs;
static FIL   s_fil[LOG_STREAM_COUNT];
static bool  s_mounted = false;
static bool  s_opened[LOG_STREAM_COUNT];

static bool sd_mount(LogStorageReason_t *why)
{
  if (!SDCard_Open()) { *why = LOG_REASON_SD_ERROR; return false; }

  /* mount immédiat (opt = 1) : lit le secteur de boot. FR_NO_FILESYSTEM = carte non
     formatée ou format inconnu (ex: ext4) : on n'y touche pas. */
  if (f_mount(&s_fs, "", 1) != FR_OK) {
    SDCard_Close();
    *why = LOG_REASON_SD_ERROR;
    return false;
  }
  s_mounted = true;
  return true;
}

static void sd_unmount(void)
{
  for (int i = 0; i < LOG_STREAM_COUNT; i++) { s_opened[i] = false; }
  if (s_mounted) { f_mount(NULL, "", 0); s_mounted = false; }  /* désenregistre, sans écrire */
  SDCard_Close();
}

static uint32_t sd_free_kb(void)
{
  FATFS *fs;
  DWORD free_clusters;
  if (f_getfree("", &free_clusters, &fs) != FR_OK) { return 0; }
  uint64_t kb = ((uint64_t)free_clusters * fs->csize) / 2u;   /* secteurs de 512 octets */
  return (kb > UINT32_MAX) ? UINT32_MAX : (uint32_t)kb;
}

static int32_t sd_max_session_id(void)
{
  DIR dir;
  FILINFO fno;
  int32_t max_id = 0;

  if (f_opendir(&dir, LOG_SD_ROOT_DIR) != FR_OK) { return 0; }
  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
    if (fno.fattrib & AM_DIR) {
      int32_t id = LogStorage_ParseSessionDir(fno.fname);
      if (id > max_id) { max_id = id; }
    }
  }
  f_closedir(&dir);
  return max_id;
}

static bool sd_prepare_session(uint32_t *session_id)
{
  char path[32];

  FRESULT r = f_mkdir(LOG_SD_ROOT_DIR);
  if (r != FR_OK && r != FR_EXIST) { return false; }

  /* On ne réutilise jamais un numéro existant : une carte déjà utilisée (dans ce dashboard
     ou un autre) garde ses anciennes sessions intactes. */
  uint32_t max_id = (uint32_t)sd_max_session_id();
  uint32_t id = (*session_id > max_id) ? *session_id : (max_id + 1u);
  if (id > 9999u) { return false; }

  snprintf(path, sizeof(path), "%s/S%04lu", LOG_SD_ROOT_DIR, (unsigned long)id);
  if (f_mkdir(path) != FR_OK) { return false; }
  *session_id = id;
  return true;
}

static bool sd_supports(LogStream_t stream)
{
  (void)stream;
  return true;   /* la carte accepte les trois flux, brut compris */
}

static bool sd_open(LogStream_t stream, uint32_t session_id, uint16_t segment)
{
  char path[64];
  snprintf(path, sizeof(path), "%s/S%04lu/%s_%02u.bin", LOG_SD_ROOT_DIR,
           (unsigned long)session_id, LogStorage_StreamName(stream), (unsigned)segment);
  if (f_open(&s_fil[stream], path, FA_CREATE_NEW | FA_WRITE) != FR_OK) { return false; }
  s_opened[stream] = true;
  return true;
}

static bool sd_write(LogStream_t stream, const void *data, uint32_t len)
{
  UINT written = 0;
  if (!s_opened[stream]) { return false; }
  return (f_write(&s_fil[stream], data, len, &written) == FR_OK) && (written == len);
}

static bool sd_sync(LogStream_t stream)
{
  return s_opened[stream] && (f_sync(&s_fil[stream]) == FR_OK);
}

static void sd_close(LogStream_t stream)
{
  if (s_opened[stream]) { f_close(&s_fil[stream]); s_opened[stream] = false; }
}

const LogStorageOps_t g_logStorageSd = {
  .mount = sd_mount, .unmount = sd_unmount, .free_kb = sd_free_kb,
  .prepare_session = sd_prepare_session, .supports = sd_supports,
  .open = sd_open, .write = sd_write, .sync = sd_sync, .close = sd_close,
  .make_room = NULL            /* on ne supprime jamais rien sur la carte de l'utilisateur */
};
