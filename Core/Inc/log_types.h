/**
  ******************************************************************************
  * @file    log_types.h
  * @brief   FORMAT BINAIRE des fichiers de logs. Source unique de vérité : le script
  *          tools/decode_log.py reprend exactement ces structures (little-endian).
  *
  * Trois flux, trois fichiers par segment de session :
  *   ses_NN.bin     : données utilisateur (tours, RPM, lux, boutons...)   records de 16 octets
  *   sys_NN.bin     : système (boot, stockage, statistiques, erreurs)     records de 16 octets
  *   raw_NN.bin     : brut développeur (IMU, trames CAN)                  records de 32 octets
  *
  * Chaque fichier = un en-tête de 32 octets (LogFileHeader_t) suivi de records de taille fixe.
  * Un record dont id == 0 est du remplissage et doit être ignoré.
  * Si tu modifies une structure : incrémente LOG_FORMAT_VERSION.
  ******************************************************************************
  */
#ifndef LOG_TYPES_H
#define LOG_TYPES_H

#include <stdint.h>

#define LOG_MAGIC           0x474F4C44u   /* "DLOG" en little-endian */
#define LOG_FORMAT_VERSION  1u

typedef enum {
  LOG_STREAM_SESSION = 0,
  LOG_STREAM_RAW     = 1,
  LOG_STREAM_SYS     = 2,
  LOG_STREAM_COUNT
} LogStream_t;

/* ===== En-tête de fichier (32 octets) ===== */
typedef struct __attribute__((packed)) {
  uint32_t magic;            /* LOG_MAGIC                                              */
  uint8_t  version;          /* LOG_FORMAT_VERSION                                     */
  uint8_t  stream;           /* LogStream_t                                            */
  uint16_t record_size;      /* 16 ou 32                                               */
  uint32_t session_id;       /* numéro de session sur CE support (dossier S0042)       */
  uint32_t boot_session_id;  /* numéro attribué au démarrage : identique pour tous les
                                segments/supports d'une même mise sous tension          */
  uint16_t segment;          /* 0, 1, 2... incrémenté à chaque changement de fichier   */
  uint16_t imu_rate_hz;      /* fréquence d'échantillonnage IMU configurée             */
  uint32_t fw_version;       /* LOG_FW_VERSION                                         */
  uint32_t device_uid;       /* identifiant unique du microcontrôleur                  */
  uint32_t tick_at_open;     /* tick FreeRTOS (ms depuis le boot) à l'ouverture        */
} LogFileHeader_t;

/* ===== Records ===== */
/* Flux session et système : 16 octets */
typedef struct __attribute__((packed)) {
  uint32_t tick_ms;          /* xTaskGetTickCount() : ms depuis le démarrage */
  uint8_t  id;               /* LogId_t                                      */
  uint8_t  payload[11];
} LogRecord_t;

/* Flux brut : 32 octets */
typedef struct __attribute__((packed)) {
  uint32_t tick_ms;
  uint8_t  id;
  uint8_t  flags;            /* réservé */
  uint8_t  payload[26];
} LogRawRecord_t;

/* ===== Identifiants d'événements ===== */
typedef enum {
  LOG_ID_PADDING     = 0x00,

  /* --- flux SESSION (utilisateur) --- */
  LOG_ID_LAP         = 0x01,  /* LogLap_t      */
  LOG_ID_RPM         = 0x02,  /* LogRpm_t      */
  LOG_ID_LUX         = 0x03,  /* LogLux_t      */
  LOG_ID_BUTTON      = 0x04,  /* LogButton_t   */
  LOG_ID_WATER_TEMP  = 0x05,  /* réservé : LogWaterTemp_t (ADC non branché pour l'instant) */
  LOG_ID_GNSS        = 0x06,  /* réservé */
  LOG_ID_SYNC_UTC    = 0x07,  /* réservé : correspondance tick <-> date UTC GNSS */

  /* --- flux SYSTEME --- */
  LOG_ID_BOOT        = 0x20,  /* LogBoot_t     */
  LOG_ID_STATS       = 0x21,  /* LogStats_t    */
  LOG_ID_POWER       = 0x22,  /* LogPower_t    */
  LOG_ID_STORAGE     = 0x23,  /* LogStorage_t  */

  /* --- flux BRUT (développeur) --- */
  LOG_ID_IMU         = 0x40,  /* LogImu_t      */
  LOG_ID_CAN_RX      = 0x41   /* LogCanRx_t    */
} LogId_t;

/* ===== Charges utiles (payload) ===== */
/* Session */
typedef struct __attribute__((packed)) { uint32_t lap_us; uint16_t lap_number; } LogLap_t;       /* lap_number 0 = passage de départ */
typedef struct __attribute__((packed)) { uint32_t pulses_per_min; } LogRpm_t;
typedef struct __attribute__((packed)) { uint16_t lux; } LogLux_t;
typedef struct __attribute__((packed)) { uint8_t button; } LogButton_t;                           /* 1 = SW1, 3 = SW3, 4 = SW4 */
typedef struct __attribute__((packed)) { int16_t centi_celsius; } LogWaterTemp_t;

/* Système */
typedef struct __attribute__((packed)) { uint32_t fw_version; uint32_t reset_flags; } LogBoot_t;  /* reset_flags = RCC->RSR */
typedef struct __attribute__((packed)) {
  uint16_t drop_session;     /* records perdus depuis le boot (queue pleine), saturé à 65535 */
  uint16_t drop_raw;
  uint16_t drop_sys;
  uint16_t min_free_heap_kb; /* plus bas niveau de heap FreeRTOS libre */
  uint8_t  backend;          /* LogBackend_t */
  uint8_t  sd_present;
} LogStats_t;
typedef struct __attribute__((packed)) { uint8_t event; } LogPower_t;                              /* 1 = appui court, 2 = extinction */
typedef struct __attribute__((packed)) {
  uint8_t  backend;          /* LogBackend_t */
  uint8_t  reason;           /* LogStorageReason_t */
  uint32_t free_kb;
} LogStorage_t;

typedef enum { LOG_BACKEND_NONE = 0, LOG_BACKEND_SD = 1, LOG_BACKEND_FLASH = 2 } LogBackend_t;
typedef enum {
  LOG_REASON_BOOT = 0,          /* choix initial                     */
  LOG_REASON_SD_REMOVED,        /* carte retirée                     */
  LOG_REASON_SD_INSERTED,       /* carte insérée en cours de session */
  LOG_REASON_SD_FULL,           /* carte pleine                      */
  LOG_REASON_SD_ERROR,          /* erreur d'écriture / carte illisible */
  LOG_REASON_FLASH_ERROR,
  LOG_REASON_FLASH_FORMATTED,   /* système de fichiers de la flash recréé */
  LOG_REASON_FILE_FULL          /* taille max du fichier brut atteinte    */
} LogStorageReason_t;

/* Brut */
typedef struct __attribute__((packed)) {
  uint16_t odr_hz;           /* fréquence d'échantillonnage */
  int16_t  sample[2][6];     /* 2 échantillons : ax ay az gx gy gz (valeurs brutes du capteur) */
} LogImu_t;                  /* 26 octets */
typedef struct __attribute__((packed)) { uint32_t can_id; uint8_t dlc; uint8_t data[8]; } LogCanRx_t;

_Static_assert(sizeof(LogFileHeader_t) == 32, "LogFileHeader_t doit faire 32 octets");
_Static_assert(sizeof(LogRecord_t)     == 16, "LogRecord_t doit faire 16 octets");
_Static_assert(sizeof(LogRawRecord_t)  == 32, "LogRawRecord_t doit faire 32 octets");
_Static_assert(sizeof(LogLap_t)   <= 11 && sizeof(LogRpm_t)  <= 11 && sizeof(LogLux_t)     <= 11 &&
               sizeof(LogButton_t) <= 11 && sizeof(LogBoot_t) <= 11 && sizeof(LogStats_t)  <= 11 &&
               sizeof(LogPower_t) <= 11 && sizeof(LogStorage_t) <= 11 && sizeof(LogWaterTemp_t) <= 11,
               "payload > 11 octets");
_Static_assert(sizeof(LogImu_t) <= 26 && sizeof(LogCanRx_t) <= 26, "payload brut > 26 octets");

#endif /* LOG_TYPES_H */
