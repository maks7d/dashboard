/**
  ******************************************************************************
  * @file    gnss.h
  * @brief   Pilote u-blox NEO-M9N (firmware SPG 4.04) sur USART1, protocole UBX.
  *
  * Références : "u-blox M9 SPG 4.04 Interface description" (UBX-21022436) et datasheet
  * NEO-M9N-00B (UBX-19014285), dossier hardware/.
  *
  * Toutes les fonctions GNSS_Wait/Poll/Configure doivent être appelées depuis UNE SEULE
  * tâche (celle de app_gnss.c) : le pilote n'a ni mutex ni file d'attente.
  ******************************************************************************
  */
#pragma once
#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ===== Réglages ===== */
#define GNSS_BAUDRATE        460800u   /* NAV-PVT à 25 Hz = ~25 kbit/s : large marge. Diviseur exact à 100 MHz (0 % d'erreur) */
#define GNSS_RATE_HZ         25u       /* maximum du NEO-M9N en PVT, toutes constellations (datasheet 1.2)  */
#define GNSS_DYNMODEL        4u        /* CFG-NAVSPG-DYNMODEL : 4 = AUTOMOT (voir Interface description 5.9.13) */

/* ===== Solution de navigation (extrait de UBX-NAV-PVT) ===== */
typedef struct {
  uint32_t iTOW_ms;       /* heure GPS de la semaine de l'époque de navigation          */
  uint16_t year;          /* date et heure UTC, valides selon "valid"                   */
  uint8_t  month, day, hour, min, sec;
  uint8_t  valid;         /* bit0 validDate, bit1 validTime, bit2 fullyResolved         */
  int32_t  nano_ns;       /* fraction de seconde UTC, -1e9..1e9                         */
  uint32_t tAcc_ns;       /* précision de l'heure UTC                                   */
  uint8_t  fixType;       /* 0 aucun, 1 estime, 2 2D, 3 3D, 4 GNSS+estime, 5 heure seule */
  uint8_t  flags;         /* bit0 gnssFixOK, bit1 diffSoln, bits7..6 carrSoln           */
  uint8_t  numSV;         /* satellites utilisés                                        */
  int32_t  lon_e7;        /* degrés x 1e-7                                              */
  int32_t  lat_e7;        /* degrés x 1e-7                                              */
  int32_t  hMSL_mm;       /* altitude au-dessus du niveau moyen de la mer               */
  uint32_t hAcc_mm;       /* précision horizontale estimée                              */
  int32_t  gSpeed_mms;    /* vitesse sol 2D                                             */
  uint32_t sAcc_mms;      /* précision de vitesse estimée                               */
  int32_t  headMot_e5;    /* cap du mouvement (2D), degrés x 1e-5                       */
  uint32_t headAcc_e5;    /* précision du cap, degrés x 1e-5                            */
  uint16_t pDOP_e2;       /* PDOP x 0,01                                                */
} Gnss_Fix_t;

/* Position exploitable : fix 2D/3D/estimé valide (le fix "heure seule" n'a pas de position) */
static inline bool GNSS_FixIsValid(const Gnss_Fix_t *f)
{
  return (f->flags & 0x01u) && f->fixType >= 2u && f->fixType <= 4u;
}

typedef struct {
  uint32_t frames;        /* trames UBX valides reçues                                  */
  uint32_t pvt;           /* dont UBX-NAV-PVT                                           */
  uint32_t badChecksum;   /* trames rejetées (checksum) : liaison ou débit à revoir     */
  uint32_t uartErrors;    /* rafales avec erreur de trame/bruit (mauvais débit, parasites)  */
  uint32_t nak;           /* UBX-ACK-NAK reçus (configuration refusée)                  */
} GNSS_Stats_t;

/* Configure le DMA circulaire + l'interruption IDLE de USART1 et lance la réception.
   Ne parle pas encore au module. À appeler depuis la tâche d'init (scheduler démarré). */
void GNSS_Init(void);

/* À appeler depuis USART1_IRQHandler (stm32h7xx_it.c) : fin de trame et erreurs de ligne */
void GNSS_UartIrq(void);

/* Trouve le débit du module, le passe à GNSS_BAUDRATE, puis configure : UBX seul en sortie,
   NAV-PVT à GNSS_RATE_HZ, modèle dynamique. Configuration en RAM du module uniquement
   (rien en flash) : elle est refaite à chaque démarrage. Bloquante (~100 ms à 1 s) ;
   false si le module ne répond pas ou refuse (NAK) : on peut rappeler. */
bool GNSS_Configure(void);

/* Bloque la tâche appelante jusqu'à de nouvelles données reçues (fin de rafale = interruption
   IDLE) ou timeout. Renvoie true si réveillée par des données. */
bool GNSS_Wait(uint32_t timeout_ms);

/* Lit les octets reçus et décode les trames. Renvoie le nombre de UBX-NAV-PVT décodés ;
   s'il est > 0, *fix reçoit le plus récent (fix peut être NULL). Ne bloque jamais. */
uint32_t GNSS_Poll(Gnss_Fix_t *fix);

void GNSS_GetStats(GNSS_Stats_t *stats);
