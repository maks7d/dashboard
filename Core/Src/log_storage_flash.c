/**
  ******************************************************************************
  * @file    log_storage_flash.c
  * @brief   Support "flash interne W25N01" : littlefs. Utilisé quand il n'y a pas de carte SD
  *          exploitable. Accepte seulement les flux session et système (le flux brut est trop
  *          volumineux pour 128 Mo).
  *
  * littlefs recopie le bloc en cours (128 Ko) à chaque sync fait en milieu de bloc : c'est
  * pourquoi LOG_FLASH_SYNC_PERIOD_MS est long.
  ******************************************************************************
  */
#include "log_storage.h"
#include "log_config.h"
#include "w25n01g.h"
#include "lfs.h"
#include <stdio.h>

#define FLASH_SLOTS  2   /* session + sys */

static lfs_t       s_lfs;
static lfs_file_t  s_file[FLASH_SLOTS];
static bool        s_opened[FLASH_SLOTS];
static bool        s_mounted = false;

/* Tampons de fichier statiques : évite tout malloc (littlefs en réclamerait un par fichier ouvert) */
static uint8_t s_fileBuf[FLASH_SLOTS][LFS_CACHE_SIZE_BYTES] __attribute__((aligned(4)));
static struct lfs_file_config s_fileCfg[FLASH_SLOTS];

static int slot_of(LogStream_t stream)
{
  return (stream == LOG_STREAM_SESSION) ? 0 : (stream == LOG_STREAM_SYS) ? 1 : -1;
}

static bool fl_mount(LogStorageReason_t *why)
{
  if (!w25n_init()) { *why = LOG_REASON_FLASH_ERROR; return false; }

  int err = lfs_mount(&s_lfs, &cfg_lfs);
  if (err) {
    /* Flash interne dédiée aux logs : si elle n'est pas (ou plus) un système de fichiers
       valide, on la recrée. Contrairement à la carte de l'utilisateur, rien d'autre n'y vit. */
    err = lfs_format(&s_lfs, &cfg_lfs);
    if (!err) { err = lfs_mount(&s_lfs, &cfg_lfs); }
    if (err) { *why = LOG_REASON_FLASH_ERROR; return false; }
    *why = LOG_REASON_FLASH_FORMATTED;
  }
  s_mounted = true;
  return true;
}

static void fl_unmount(void)
{
  for (int i = 0; i < FLASH_SLOTS; i++) { s_opened[i] = false; }
  if (s_mounted) { lfs_unmount(&s_lfs); s_mounted = false; }
}

static uint32_t fl_free_blocks(void)
{
  lfs_ssize_t used = lfs_fs_size(&s_lfs);
  if (used < 0 || (lfs_size_t)used > cfg_lfs.block_count) { return 0; }
  return cfg_lfs.block_count - (lfs_size_t)used;
}

static uint32_t fl_free_kb(void)
{
  return fl_free_blocks() * (cfg_lfs.block_size / 1024u);
}

/* Parcourt la racine et renvoie le plus grand (max) ou le plus petit (min) numéro de session */
static int32_t fl_scan_sessions(bool want_max, uint32_t skip_id)
{
  lfs_dir_t dir;
  struct lfs_info info;
  int32_t best = -1;

  if (lfs_dir_open(&s_lfs, &dir, "/") < 0) { return -1; }
  while (lfs_dir_read(&s_lfs, &dir, &info) > 0) {
    if (info.type != LFS_TYPE_DIR) { continue; }
    int32_t id = LogStorage_ParseSessionDir(info.name);
    if (id < 0 || (uint32_t)id == skip_id) { continue; }
    if (best < 0 || (want_max ? id > best : id < best)) { best = id; }
  }
  lfs_dir_close(&s_lfs, &dir);
  return best;
}

static bool fl_prepare_session(uint32_t *session_id)
{
  char path[16];
  int32_t max_id = fl_scan_sessions(true, 0);
  uint32_t max_u = (max_id < 0) ? 0u : (uint32_t)max_id;
  uint32_t id = (*session_id > max_u) ? *session_id : (max_u + 1u);
  if (id > 9999u) { return false; }

  snprintf(path, sizeof(path), "/S%04lu", (unsigned long)id);
  if (lfs_mkdir(&s_lfs, path) < 0) { return false; }
  *session_id = id;
  return true;
}

static bool fl_supports(LogStream_t stream)
{
  return slot_of(stream) >= 0;
}

static bool fl_open(LogStream_t stream, uint32_t session_id, uint16_t segment)
{
  char path[48];
  int slot = slot_of(stream);
  if (slot < 0) { return false; }

  snprintf(path, sizeof(path), "/S%04lu/%s_%02u.bin", (unsigned long)session_id,
           LogStorage_StreamName(stream), (unsigned)segment);
  s_fileCfg[slot].buffer = s_fileBuf[slot];
  if (lfs_file_opencfg(&s_lfs, &s_file[slot], path,
                       LFS_O_WRONLY | LFS_O_CREAT | LFS_O_EXCL, &s_fileCfg[slot]) < 0) { return false; }
  s_opened[slot] = true;
  return true;
}

static bool fl_write(LogStream_t stream, const void *data, uint32_t len)
{
  int slot = slot_of(stream);
  if (slot < 0 || !s_opened[slot]) { return false; }
  return lfs_file_write(&s_lfs, &s_file[slot], data, len) == (lfs_ssize_t)len;
}

static bool fl_sync(LogStream_t stream)
{
  int slot = slot_of(stream);
  return slot >= 0 && s_opened[slot] && lfs_file_sync(&s_lfs, &s_file[slot]) == 0;
}

static void fl_close(LogStream_t stream)
{
  int slot = slot_of(stream);
  if (slot >= 0 && s_opened[slot]) { lfs_file_close(&s_lfs, &s_file[slot]); s_opened[slot] = false; }
}

/* Supprime la plus ancienne session (fichiers puis dossier) sauf celle en cours */
static bool fl_make_room(uint32_t keep_session_id)
{
  int32_t oldest = fl_scan_sessions(false, keep_session_id);
  if (oldest < 0) { return false; }

  char dirpath[16], filepath[64];
  lfs_dir_t dir;
  struct lfs_info info;
  snprintf(dirpath, sizeof(dirpath), "/S%04ld", (long)oldest);

  if (lfs_dir_open(&s_lfs, &dir, dirpath) < 0) { return false; }
  /* On supprime en relisant le dossier depuis le début à chaque fois : plus simple et sûr */
  for (;;) {
    bool found = false;
    lfs_dir_rewind(&s_lfs, &dir);
    while (lfs_dir_read(&s_lfs, &dir, &info) > 0) {
      if (info.type == LFS_TYPE_REG) {
        snprintf(filepath, sizeof(filepath), "%s/%.40s", dirpath, info.name);
        found = true;
        break;
      }
    }
    if (!found) { break; }
    lfs_dir_close(&s_lfs, &dir);
    if (lfs_remove(&s_lfs, filepath) < 0) { return false; }
    if (lfs_dir_open(&s_lfs, &dir, dirpath) < 0) { return false; }
  }
  lfs_dir_close(&s_lfs, &dir);
  return lfs_remove(&s_lfs, dirpath) == 0;
}

const LogStorageOps_t g_logStorageFlash = {
  .mount = fl_mount, .unmount = fl_unmount, .free_kb = fl_free_kb,
  .prepare_session = fl_prepare_session, .supports = fl_supports,
  .open = fl_open, .write = fl_write, .sync = fl_sync, .close = fl_close,
  .make_room = fl_make_room
};

/* Utilisé par app_log.c pour savoir s'il faut libérer de la place */
uint32_t LogStorageFlash_FreeBlocks(void) { return fl_free_blocks(); }
