/**
  ******************************************************************************
  * @file    app_log.c
  * @brief   Moteur de logs : queues, tâche d'écriture, choix du support (SD / flash).
  *
  * Chemin d'un record :
  *   Log_Xxx() [tâche/ISR, ~1 µs]  ->  queue du flux  ->  vTaskLog (toutes les 200 ms)
  *   ->  tampon RAM de 2 Ko du flux  ->  support (carte SD si présente et non pleine, sinon flash)
  *
  * Choix du support :
  *   carte SD détectée (CD_SD) et utilisable ........ SD : flux session + système + brut
  *   pas de carte / carte illisible ou pleine ....... flash : session + système (brut abandonné)
  *   carte retirée en cours de session .............. bascule sur la flash (nouveau segment)
  *   carte insérée en cours de session .............. bascule sur la SD (nouveau segment)
  * Chaque bascule est tracée par un record LOG_ID_STORAGE.
  ******************************************************************************
  */
#include "app.h"
#include "log_storage.h"
#include "sd_card.h"
#include <string.h>

#define LOG_EVT_FLUSH_REQ   (1u << 0)
#define LOG_EVT_CLOSED      (1u << 1)

extern uint32_t LogStorageFlash_FreeBlocks(void);

/* ===== État ===== */
static QueueHandle_t      s_q[LOG_STREAM_COUNT];
static const uint8_t      s_recSize[LOG_STREAM_COUNT] = { sizeof(LogRecord_t), sizeof(LogRawRecord_t), sizeof(LogRecord_t) };
static volatile uint32_t  s_drop[LOG_STREAM_COUNT];          /* records perdus (queue pleine) */
static EventGroupHandle_t s_ev;

static uint8_t   s_stage[LOG_STREAM_COUNT][LOG_STAGE_SIZE];
static uint16_t  s_stageLen[LOG_STREAM_COUNT];
static uint32_t  s_streamBytes[LOG_STREAM_COUNT];
static bool      s_streamOpen[LOG_STREAM_COUNT];

static const LogStorageOps_t *s_be = NULL;     /* support courant */
static LogBackend_t s_backend = LOG_BACKEND_NONE;
static uint32_t s_bootSession = 0;             /* n° attribué au 1er support de cette mise sous tension */
static uint32_t s_sessionId[3];                /* n° de session sur chaque support (index LogBackend_t) */
static uint16_t s_segment = 0;                 /* prochain numéro de segment */
static bool     s_closed = false;              /* Log_FlushAndClose() effectué : on n'écrit plus */

static bool     s_cdLast = false;
static uint8_t  s_cdStable = 0;
static bool     s_sdRetryAllowed = true;       /* évite de re-tenter en boucle une carte illisible */

static uint32_t s_uid;

_Static_assert(LOG_IMU_SAMPLES_PER_RECORD == 2, "LogImu_t contient 2 échantillons : adapter log_types.h");
_Static_assert((LOG_STAGE_SIZE % 16u) == 0 && (LOG_STAGE_SIZE % 32u) == 0, "LOG_STAGE_SIZE : multiple de 16 et 32");

/* ============================================================================
 *  PRODUCTEURS : appelés par les tâches / ISR. Jamais bloquants.
 * ========================================================================== */
static bool Log_Push(LogStream_t st, uint8_t id, const void *payload, uint8_t len, bool fromIsr)
{
  if (s_q[st] == NULL) { return false; }

  union { LogRecord_t r16; LogRawRecord_t r32; } u;     /* r16 est au début : la queue ne copie que sa taille */
  memset(&u, 0, sizeof(u));
  uint8_t *dst; uint8_t maxlen;
  uint32_t tick = fromIsr ? xTaskGetTickCountFromISR() : xTaskGetTickCount();

  if (st == LOG_STREAM_RAW) { u.r32.tick_ms = tick; u.r32.id = id; dst = u.r32.payload; maxlen = sizeof(u.r32.payload); }
  else                      { u.r16.tick_ms = tick; u.r16.id = id; dst = u.r16.payload; maxlen = sizeof(u.r16.payload); }
  if (len > maxlen) { len = maxlen; }
  if (payload != NULL && len > 0) { memcpy(dst, payload, len); }

  BaseType_t ok = fromIsr ? xQueueSendFromISR(s_q[st], &u, NULL)
                          : xQueueSend(s_q[st], &u, 0);          /* 0 = ne JAMAIS attendre */
  if (ok != pdPASS) {
    __atomic_fetch_add(&s_drop[st], 1u, __ATOMIC_RELAXED);
    return false;
  }
  return true;
}

bool Log_Session(uint8_t id, const void *p, uint8_t len)         { return Log_Push(LOG_STREAM_SESSION, id, p, len, false); }
bool Log_Sys(uint8_t id, const void *p, uint8_t len)             { return Log_Push(LOG_STREAM_SYS,     id, p, len, false); }
bool Log_Raw(uint8_t id, const void *p, uint8_t len)             { return Log_Push(LOG_STREAM_RAW,     id, p, len, false); }
bool Log_SessionFromISR(uint8_t id, const void *p, uint8_t len)  { return Log_Push(LOG_STREAM_SESSION, id, p, len, true);  }

void Log_ImuSample(const int16_t sample[6])
{
  static LogImu_t acc;
  static uint8_t  n = 0;

  memcpy(acc.sample[n], sample, 6 * sizeof(int16_t));
  if (++n == LOG_IMU_SAMPLES_PER_RECORD) {
    acc.odr_hz = (uint16_t)LOG_IMU_RATE_HZ;
    Log_Raw(LOG_ID_IMU, &acc, sizeof(acc));
    n = 0;
  }
}

void Log_CreateObjects(void)
{
  s_q[LOG_STREAM_SESSION] = xQueueCreate(LOG_SESSION_QUEUE_DEPTH, sizeof(LogRecord_t));
  s_q[LOG_STREAM_RAW]     = xQueueCreate(LOG_RAW_QUEUE_DEPTH,     sizeof(LogRawRecord_t));
  s_q[LOG_STREAM_SYS]     = xQueueCreate(LOG_SYS_QUEUE_DEPTH,     sizeof(LogRecord_t));
  s_ev = xEventGroupCreate();
  for (int i = 0; i < LOG_STREAM_COUNT; i++) { if (s_q[i] == NULL) { Error_Handler(); } }
  if (s_ev == NULL) { Error_Handler(); }

  s_uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

  /* Premier record du flux système : version + cause du reset (RCC->RSR), puis on efface les flags */
  LogBoot_t boot = { LOG_FW_VERSION, RCC->RSR };
  __HAL_RCC_CLEAR_RESET_FLAGS();
  Log_Sys(LOG_ID_BOOT, &boot, sizeof(boot));
}

bool Log_FlushAndClose(uint32_t timeout_ms)
{
  if (s_ev == NULL) { return false; }
  xEventGroupClearBits(s_ev, LOG_EVT_CLOSED);
  xEventGroupSetBits(s_ev, LOG_EVT_FLUSH_REQ);
  EventBits_t b = xEventGroupWaitBits(s_ev, LOG_EVT_CLOSED, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms));
  return (b & LOG_EVT_CLOSED) != 0;
}

/* ============================================================================
 *  MOTEUR : exécuté uniquement par vTaskLog
 * ========================================================================== */
static bool Log_WriteHeader(LogStream_t st)
{
  LogFileHeader_t h = {0};
  h.magic = LOG_MAGIC;
  h.version = LOG_FORMAT_VERSION;
  h.stream = st;
  h.record_size = s_recSize[st];
  h.session_id = s_sessionId[s_backend];
  h.boot_session_id = s_bootSession;
  h.segment = (uint16_t)(s_segment - 1u);          /* s_segment = prochain numéro */
  h.imu_rate_hz = (uint16_t)LOG_IMU_RATE_HZ;
  h.fw_version = LOG_FW_VERSION;
  h.device_uid = s_uid;
  h.tick_at_open = xTaskGetTickCount();
  return s_be->write(st, &h, sizeof(h));
}

static void Log_CloseAll(bool graceful)
{
  for (int st = 0; st < LOG_STREAM_COUNT; st++) {
    if (!s_streamOpen[st]) { continue; }
    if (graceful) {
      if (s_stageLen[st] > 0) { s_be->write((LogStream_t)st, s_stage[st], s_stageLen[st]); }
      s_be->sync((LogStream_t)st);
      s_be->close((LogStream_t)st);
    }
    s_streamOpen[st] = false;
    s_stageLen[st] = 0;
  }
}

static void Log_StopBackend(bool graceful)
{
  if (s_be == NULL) { return; }
  Log_CloseAll(graceful);
  s_be->unmount();
  s_be = NULL;
  s_backend = LOG_BACKEND_NONE;
}

static void Log_EmitStorage(LogStorageReason_t reason)
{
  LogStorage_t rec = { (uint8_t)s_backend, (uint8_t)reason, s_be ? s_be->free_kb() : 0u };
  Log_Sys(LOG_ID_STORAGE, &rec, sizeof(rec));
}

/* Ouvre un nouveau segment (un fichier par flux accepté) sur le support déjà monté */
static bool Log_OpenSegment(void)
{
  uint16_t seg = s_segment++;
  for (int st = 0; st < LOG_STREAM_COUNT; st++) {
    s_streamOpen[st] = false; s_stageLen[st] = 0; s_streamBytes[st] = 0;
    if (!s_be->supports((LogStream_t)st)) { continue; }
    if (!s_be->open((LogStream_t)st, s_sessionId[s_backend], seg) || !Log_WriteHeader((LogStream_t)st)) {
      Log_CloseAll(false);
      return false;
    }
    s_streamOpen[st] = true;
    s_streamBytes[st] = sizeof(LogFileHeader_t);
  }
  return true;
}

/* Tente de démarrer sur ce support. false = inutilisable (*why dit pourquoi) */
static bool Log_TryBackend(const LogStorageOps_t *be, LogBackend_t id, LogStorageReason_t *why)
{
  if (!be->mount(why)) { return false; }

  if (id == LOG_BACKEND_SD) {
    if (be->free_kb() < (LOG_SD_MIN_FREE_MB * 1024u)) { be->unmount(); *why = LOG_REASON_SD_FULL; return false; }
  } else {
    /* flash : on supprime les plus anciennes sessions tant qu'il manque de place */
    while (LogStorageFlash_FreeBlocks() < LOG_FLASH_MIN_FREE_BLOCKS && be->make_room != NULL &&
           be->make_room(s_sessionId[id])) { }
  }

  if (s_sessionId[id] == 0) {                       /* première fois sur ce support depuis le boot */
    uint32_t sid = s_bootSession;                   /* on essaie de garder le même numéro partout */
    if (!be->prepare_session(&sid)) { be->unmount(); *why = (id == LOG_BACKEND_SD) ? LOG_REASON_SD_ERROR : LOG_REASON_FLASH_ERROR; return false; }
    s_sessionId[id] = sid;
    if (s_bootSession == 0) { s_bootSession = sid; }
  }

  s_be = be;
  s_backend = id;
  if (!Log_OpenSegment()) {
    be->unmount(); s_be = NULL; s_backend = LOG_BACKEND_NONE;
    *why = (id == LOG_BACKEND_SD) ? LOG_REASON_SD_ERROR : LOG_REASON_FLASH_ERROR;
    return false;
  }
  return true;
}

/* Choisit le meilleur support disponible : SD si une carte est présente, sinon flash */
static void Log_SelectBackend(LogStorageReason_t reasonIfSd)
{
  LogStorageReason_t why = reasonIfSd;

  if (SDCard_IsInserted() && s_sdRetryAllowed) {
    if (Log_TryBackend(&g_logStorageSd, LOG_BACKEND_SD, &why)) { Log_EmitStorage(reasonIfSd); return; }
    s_sdRetryAllowed = false;                       /* inutilisable : on attend que l'utilisateur la retire */
  }
  LogStorageReason_t flashWhy = why;
  if (Log_TryBackend(&g_logStorageFlash, LOG_BACKEND_FLASH, &flashWhy)) {
    Log_EmitStorage((flashWhy == LOG_REASON_FLASH_FORMATTED) ? flashWhy : why);
    return;
  }
  /* aucun support : les records seront jetés (comptés dans STATS) */
  s_backend = LOG_BACKEND_NONE;
  LogStorage_t rec = { LOG_BACKEND_NONE, (uint8_t)LOG_REASON_FLASH_ERROR, 0 };
  Log_Sys(LOG_ID_STORAGE, &rec, sizeof(rec));
}

/* Un support est tombé en erreur / a été retiré / est plein : on le quitte et on rebascule */
static void Log_Failover(LogStorageReason_t reason, bool graceful)
{
  Log_StopBackend(graceful);
  Log_SelectBackend(reason);
}

static void Log_FlushStage(LogStream_t st)
{
  if (s_stageLen[st] == 0 || !s_streamOpen[st]) { s_stageLen[st] = 0; return; }

  uint16_t len = s_stageLen[st];
  s_stageLen[st] = 0;
  if (!s_be->write(st, s_stage[st], len)) {
    Log_Failover((s_backend == LOG_BACKEND_SD) ? LOG_REASON_SD_ERROR : LOG_REASON_FLASH_ERROR, false);
    return;
  }
  s_streamBytes[st] += len;

  /* Fichier brut trop gros (FAT32 : 4 Go max) : nouveau segment */
  if (st == LOG_STREAM_RAW && s_streamBytes[st] >= LOG_RAW_FILE_MAX_BYTES) {
    Log_CloseAll(true);
    if (!Log_OpenSegment()) { Log_Failover(LOG_REASON_FILE_FULL, false); } else { Log_EmitStorage(LOG_REASON_FILE_FULL); }
  }
}

static void Log_Append(LogStream_t st, const uint8_t *rec)
{
  if (!s_streamOpen[st]) { return; }               /* pas de support pour ce flux : record jeté */
  memcpy(&s_stage[st][s_stageLen[st]], rec, s_recSize[st]);
  s_stageLen[st] += s_recSize[st];
  if (s_stageLen[st] + s_recSize[st] > LOG_STAGE_SIZE) { Log_FlushStage(st); }
}

/* Vide les trois queues dans les tampons. Borné pour ne pas tourner indéfiniment si un
   producteur remplit aussi vite qu'on vide. */
static void Log_Drain(void)
{
  uint8_t rec[sizeof(LogRawRecord_t)];
  for (int st = 0; st < LOG_STREAM_COUNT; st++) {
    uint32_t budget = 2u * uxQueueSpacesAvailable(s_q[st]) + 2u * uxQueueMessagesWaiting(s_q[st]);
    while (budget-- > 0 && xQueueReceive(s_q[st], rec, 0) == pdPASS) {
      if (s_closed) { continue; }                  /* après Log_FlushAndClose : on jette */
      Log_Append((LogStream_t)st, rec);
    }
  }
}

static void Log_SyncAll(void)
{
  for (int st = 0; st < LOG_STREAM_COUNT; st++) {
    if (!s_streamOpen[st]) { continue; }
    Log_FlushStage((LogStream_t)st);
    if (s_streamOpen[st] && !s_be->sync((LogStream_t)st)) {
      Log_Failover((s_backend == LOG_BACKEND_SD) ? LOG_REASON_SD_ERROR : LOG_REASON_FLASH_ERROR, false);
      return;
    }
  }
}

static uint16_t sat16(uint32_t v) { return (v > 0xFFFFu) ? 0xFFFFu : (uint16_t)v; }

static void Log_EmitStats(void)
{
  LogStats_t s;
  s.drop_session = sat16(s_drop[LOG_STREAM_SESSION]);
  s.drop_raw = sat16(s_drop[LOG_STREAM_RAW]);
  s.drop_sys = sat16(s_drop[LOG_STREAM_SYS]);
  s.min_free_heap_kb = sat16((uint32_t)(xPortGetMinimumEverFreeHeapSize() / 1024u));
  s.backend = (uint8_t)s_backend;
  s.sd_present = SDCard_IsInserted() ? 1u : 0u;
  Log_Sys(LOG_ID_STATS, &s, sizeof(s));
}

/* Détecte insertion / retrait de la carte (anti-rebond : 2 lectures identiques = 400 ms) */
static void Log_CheckCard(void)
{
  bool present = SDCard_IsInserted();
  if (present == s_cdLast) { if (s_cdStable < 255) { s_cdStable++; } } else { s_cdLast = present; s_cdStable = 0; }
  if (s_cdStable < 2) { return; }

  if (!present) {
    s_sdRetryAllowed = true;                       /* carte retirée : on pourra réessayer à la prochaine insertion */
    s_sessionId[LOG_BACKEND_SD] = 0;               /* la prochaine carte aura son propre dossier de session */
    if (s_backend == LOG_BACKEND_SD) { Log_Failover(LOG_REASON_SD_REMOVED, false); }
  } else if (s_backend != LOG_BACKEND_SD && s_sdRetryAllowed && !s_closed) {
    Log_StopBackend(true);
    Log_SelectBackend(LOG_REASON_SD_INSERTED);
  }
}

static void Log_CheckSpace(void)
{
  if (s_backend == LOG_BACKEND_SD) {
    if (g_logStorageSd.free_kb() < (LOG_SD_MIN_FREE_MB * 1024u)) { Log_Failover(LOG_REASON_SD_FULL, true); }
  } else if (s_backend == LOG_BACKEND_FLASH) {
    while (LogStorageFlash_FreeBlocks() < LOG_FLASH_MIN_FREE_BLOCKS && g_logStorageFlash.make_room(s_sessionId[LOG_BACKEND_FLASH])) { }
  }
}

void vTaskLog(void *pvParameters)
{
  (void)pvParameters;
  TickType_t lastSync = 0, lastStats = 0;

  xEventGroupWaitBits(xSystemEvents, EVT_INIT_DONE, pdFALSE, pdTRUE, portMAX_DELAY);

  /* Choix initial du support */
  s_cdLast = SDCard_IsInserted();
  s_cdStable = 2;
  Log_SelectBackend(LOG_REASON_BOOT);
  lastSync = lastStats = xTaskGetTickCount();

  for (;;) {
    /* Dort 200 ms, ou se réveille tout de suite sur Log_FlushAndClose() */
    EventBits_t ev = xEventGroupWaitBits(s_ev, LOG_EVT_FLUSH_REQ, pdFALSE, pdFALSE, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));

    if (ev & LOG_EVT_FLUSH_REQ) {
      if (!s_closed) {
        Log_Drain();
        Log_Drain();                                 /* 2e passage : records arrivés pendant l'écriture */
        Log_StopBackend(true);                       /* écrit les tampons, sync, ferme, démonte */
        s_closed = true;
      }
      xEventGroupClearBits(s_ev, LOG_EVT_FLUSH_REQ);
      xEventGroupSetBits(s_ev, LOG_EVT_CLOSED);
      continue;
    }

    if (!s_closed) { Log_CheckCard(); }
    Log_Drain();

    TickType_t now = xTaskGetTickCount();
    if (!s_closed && s_backend != LOG_BACKEND_NONE) {
      uint32_t syncPeriod = (s_backend == LOG_BACKEND_SD) ? LOG_SD_SYNC_PERIOD_MS : LOG_FLASH_SYNC_PERIOD_MS;
      if ((now - lastSync) >= pdMS_TO_TICKS(syncPeriod)) { Log_SyncAll(); lastSync = now; }
    }
    if ((now - lastStats) >= pdMS_TO_TICKS(LOG_STATS_PERIOD_MS)) {
      lastStats = now;
      if (!s_closed) { Log_CheckSpace(); }
      Log_EmitStats();
    }
  }
}
