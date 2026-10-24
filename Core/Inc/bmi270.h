/**
  ******************************************************************************
  * @file    bmi270.h
  * @brief   Pilote Bosch BMI270 (accéléromètre + gyroscope) sur SPI1 : initialisation avec le
  *          fichier de configuration, acquisition par FIFO, décodage des trames.
  *
  * Référence : BMI270 datasheet BST-BMI270-DS000-08 (hardware/bst-bmi270-ds000.pdf). Les numéros
  * de page cités dans bmi270.c renvoient à ce document.
  *
  * Toutes les fonctions doivent être appelées depuis UNE SEULE tâche (celle de app_imu.c),
  * scheduler démarré : le pilote utilise vTaskDelay et n'a ni mutex ni file d'attente.
  ******************************************************************************
  */
#pragma once
#include "stm32h7xx_hal.h"
#include "log_config.h"
#include <stdbool.h>
#include <stdint.h>

/* ===== Réglages (le reste du pilote s'en déduit) ===== */

/* Fréquence d'échantillonnage de l'accéléromètre ET du gyroscope. Une seule source de vérité :
   LOG_IMU_RATE_HZ (log_config.h), aussi inscrite dans l'en-tête des fichiers de log.
   Valeurs valides : 25, 50, 100, 200, 400, 800, 1600. */
#define BMI270_ODR_HZ                LOG_IMU_RATE_HZ

/* Plage de mesure. Plus la plage est petite, meilleure est la résolution ; trop petite, le signal
   sature (vibrations moteur, chocs sur les bordures). Valeurs valides :
   accéléromètre 2, 4, 8, 16 (g) ; gyroscope 125, 250, 500, 1000, 2000 (dps). */
#define BMI270_ACC_RANGE_G           8
#define BMI270_GYR_RANGE_DPS         1000

/* Durée de données que le FIFO accumule avant de lever INT1 (interruption "watermark").
   Plus court = latence plus faible mais plus de réveils de la tâche. */
#define BMI270_FIFO_WATERMARK_MS     50

/* ===== Conversions (valeurs brutes -> unités physiques) ===== */
#define BMI270_ACC_LSB_PER_G         (32768.0f / (float)BMI270_ACC_RANGE_G)      /* 8 g -> 4096 LSB/g   */
#define BMI270_GYR_LSB_PER_DPS       (32768.0f / (float)BMI270_GYR_RANGE_DPS)    /* 1000 dps -> 32,768  */

/* ===== FIFO ===== */
#define BMI270_FIFO_FRAME_BYTES      13u     /* 1 octet d'en-tête + 6 (gyro) + 6 (accéléro) */
#define BMI270_FIFO_BYTES            2048u   /* taille du FIFO (datasheet 4.7)              */
#define BMI270_FIFO_MAX_FRAMES       160u    /* >= BMI270_FIFO_BYTES / BMI270_FIFO_FRAME_BYTES */

/* Un échantillon : valeurs brutes signées sur 16 bits, dans les axes du CAPTEUR (pas ceux du
   véhicule). Même ordre que Log_ImuSample() : ax ay az gx gy gz. Le gyroscope est corrigé de la
   sensibilité croisée (CAS) comme l'exige la datasheet 4.6.10. */
typedef struct {
  int16_t acc[3];
  int16_t gyr[3];
} Bmi270_Sample_t;

_Static_assert(sizeof(Bmi270_Sample_t) == 6 * sizeof(int16_t), "Bmi270_Sample_t doit tenir sur 6 int16");

/* Informations sur une lecture du FIFO */
typedef struct {
  uint16_t frames;        /* échantillons décodés (= valeur de retour de BMI270_ReadFifo)         */
  uint16_t skipped;       /* échantillons PERDUS signalés par le capteur (débordement du FIFO)    */
  bool     hasTime;       /* sensorTime valide                                                   */
  uint32_t sensorTime;    /* compteur 24 bits, 39,0625 us par LSB, instant du DERNIER échantillon */
} Bmi270_FifoInfo_t;

typedef enum {
  BMI270_ERR_NONE = 0,
  BMI270_ERR_OS,          /* objet FreeRTOS non créé                                  */
  BMI270_ERR_SPI,         /* transfert SPI en échec                                   */
  BMI270_ERR_CHIP_ID,     /* CHIP_ID != 0x24 : capteur absent, SPI ou câblage         */
  BMI270_ERR_INIT,        /* INTERNAL_STATUS jamais "init_ok" après le fichier de config */
  BMI270_ERR_CONFIG,      /* le capteur a refusé ACC_CONF / GYR_CONF (registre EVENT)   */
  BMI270_ERR_FATAL        /* ERR_REG.fatal_err                                         */
} Bmi270_Error_t;

typedef struct {
  uint32_t samples;       /* échantillons décodés depuis le démarrage                             */
  uint32_t skipped;       /* échantillons perdus (débordements du FIFO)                           */
  uint32_t badFrames;     /* trames FIFO incompréhensibles (lecture interrompue)                  */
  uint32_t spiErrors;     /* erreurs de transfert SPI / DMA                                       */
  uint32_t inits;         /* initialisations réussies                                             */
  uint8_t  lastError;     /* Bmi270_Error_t de la dernière initialisation ratée                   */
  uint8_t  chipId;        /* dernier CHIP_ID lu (0x24 attendu)                                    */
  int8_t   gyrCas;        /* facteur de correction CAS du gyroscope (GYR_CAS.factor_zx)           */
} Bmi270_Stats_t;

/* Initialise le capteur : identification, soft reset, envoi du fichier de configuration (8 Ko),
   réglage des capteurs, du FIFO et de l'interruption INT1, puis vide le FIFO. Bloquante
   (environ 100 ms). Peut être rappelée à tout moment : elle commence toujours par un soft reset,
   ce qui est obligatoire car le fichier de configuration ne doit être envoyé qu'une fois après
   une mise sous tension ou un soft reset (datasheet 4.4). */
bool BMI270_Init(void);

/* Lit tout le FIFO et décode les trames dans out[] (capacité maxOut >= BMI270_FIFO_MAX_FRAMES).
   Renvoie le nombre d'échantillons, 0 s'il n'y en a pas ou en cas d'erreur (voir BMI270_GetStats).
   Bloque la tâche pendant le transfert DMA (les autres tâches continuent). info peut être NULL. */
uint32_t BMI270_ReadFifo(Bmi270_Sample_t *out, uint32_t maxOut, Bmi270_FifoInfo_t *info);

void BMI270_GetStats(Bmi270_Stats_t *stats);
