/**
  ******************************************************************************
  * @file    log_storage.h
  * @brief   Interface commune des supports de stockage des logs (carte SD / flash).
  *          Le moteur de logs (app_log.c) ne connaît que cette interface.
  *
  * Toutes ces fonctions sont appelées par la SEULE tâche vTaskLog.
  ******************************************************************************
  */
#ifndef LOG_STORAGE_H
#define LOG_STORAGE_H

#include <stdbool.h>
#include <stdint.h>
#include "log_types.h"

typedef struct {
  /* Monte le support. En cas d'échec, *why indique la raison. */
  bool     (*mount)(LogStorageReason_t *why);
  void     (*unmount)(void);                      /* ne fait AUCUNE écriture */
  uint32_t (*free_kb)(void);
  /* Crée le dossier de la session. *session_id = numéro souhaité (0 = n'importe lequel) ;
     renvoie le numéro réellement utilisé (jamais un dossier déjà existant). */
  bool     (*prepare_session)(uint32_t *session_id);
  bool     (*supports)(LogStream_t stream);       /* ce support accepte-t-il ce flux ? */
  bool     (*open)(LogStream_t stream, uint32_t session_id, uint16_t segment);
  bool     (*write)(LogStream_t stream, const void *data, uint32_t len);
  bool     (*sync)(LogStream_t stream);
  void     (*close)(LogStream_t stream);
  /* Libère de la place en supprimant la plus ancienne session (hors keep_session_id).
     NULL si le support ne supprime jamais rien (carte SD de l'utilisateur). */
  bool     (*make_room)(uint32_t keep_session_id);
} LogStorageOps_t;

extern const LogStorageOps_t g_logStorageSd;
extern const LogStorageOps_t g_logStorageFlash;

/* Nom de fichier : "ses", "raw" ou "sys". Courts exprès : la config FatFs du projet n'accepte que
   les noms 8.3 (FF_USE_LFN = 0) ; "ses_00.bin" devient SES_00.BIN sur la carte. */
static inline const char *LogStorage_StreamName(LogStream_t s)
{
  return (s == LOG_STREAM_SESSION) ? "ses" : (s == LOG_STREAM_RAW) ? "raw" : "sys";
}

/* Analyse un nom de dossier "S0042" -> 42 ; renvoie -1 si le nom n'est pas de ce format */
static inline int32_t LogStorage_ParseSessionDir(const char *name)
{
  if (name[0] != 'S') { return -1; }
  int32_t v = 0;
  for (int i = 1; i <= 4; i++) {
    if (name[i] < '0' || name[i] > '9') { return -1; }
    v = v * 10 + (name[i] - '0');
  }
  return (name[5] == '\0') ? v : -1;
}

#endif /* LOG_STORAGE_H */
