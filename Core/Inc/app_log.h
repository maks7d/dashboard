/**
  ******************************************************************************
  * @file    app_log.h
  * @brief   API du système de logs : les tâches appellent Log_xxx(), c'est tout.
  *
  * Chaque appel copie un record de 16 ou 32 octets dans une queue FreeRTOS (~1 µs) et
  * revient immédiatement : jamais d'accès disque, jamais de formatage de texte, jamais
  * d'attente (timeout 0). Si la queue est pleine le record est perdu et compté (voir STATS).
  * La tâche vTaskLog (basse priorité) écrit ensuite par gros blocs sur la carte SD, ou à
  * défaut sur la flash. Voir log_types.h pour le format et log_config.h pour les réglages.
  ******************************************************************************
  */
#ifndef APP_LOG_H
#define APP_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "log_types.h"
#include "log_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cycle de vie (appelées par app.c) */
void Log_CreateObjects(void);                 /* queues + record BOOT : avant le scheduler */
void vTaskLog(void *pvParameters);

/* Vide les queues, écrit les tampons, ferme les fichiers. À appeler avant de couper
   l'alimentation. Attend au plus timeout_ms ; renvoie true si tout a été écrit. */
bool Log_FlushAndClose(uint32_t timeout_ms);

/* ===== API générique (depuis une tâche) ===== */
bool Log_Session(uint8_t id, const void *payload, uint8_t len);   /* flux utilisateur, ne perd pas */
bool Log_Sys(uint8_t id, const void *payload, uint8_t len);       /* flux système                  */
bool Log_Raw(uint8_t id, const void *payload, uint8_t len);       /* flux brut (peut être perdu)   */
/* Depuis une interruption */
bool Log_SessionFromISR(uint8_t id, const void *payload, uint8_t len);

/* ===== Raccourcis typés : c'est ce qu'on appelle dans le code applicatif ===== */
static inline bool Log_Lap(uint32_t lap_us, uint16_t lap_number)
{ LogLap_t p = { lap_us, lap_number }; return Log_Session(LOG_ID_LAP, &p, sizeof(p)); }

static inline bool Log_Rpm(uint32_t pulses_per_min)
{ LogRpm_t p = { pulses_per_min }; return Log_Session(LOG_ID_RPM, &p, sizeof(p)); }

static inline bool Log_Lux(uint16_t lux)
{ LogLux_t p = { lux }; return Log_Session(LOG_ID_LUX, &p, sizeof(p)); }

static inline bool Log_ButtonFromISR(uint8_t button)
{ LogButton_t p = { button }; return Log_SessionFromISR(LOG_ID_BUTTON, &p, sizeof(p)); }

static inline bool Log_Power(uint8_t event)
{ LogPower_t p = { event }; return Log_Sys(LOG_ID_POWER, &p, sizeof(p)); }

static inline bool Log_CanRx(uint32_t can_id, uint8_t dlc, const uint8_t *data)
{
  LogCanRx_t p = { can_id, dlc, {0} };
  for (uint8_t i = 0; i < dlc && i < 8; i++) { p.data[i] = data[i]; }
  return Log_Raw(LOG_ID_CAN_RX, &p, sizeof(p));
}

/* Un échantillon IMU brut (ax ay az gx gy gz). Regroupe LOG_IMU_SAMPLES_PER_RECORD échantillons
   par record. À appeler depuis UNE SEULE tâche (la future tâche IMU). */
void Log_ImuSample(const int16_t sample[6]);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
