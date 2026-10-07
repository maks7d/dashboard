/**
  ******************************************************************************
  * @file    sd_card.h
  * @brief   Carte microSD : détection (PB8) et initialisation à l'exécution.
  *          Le pilote bas niveau pour FatFS est dans sd_diskio.c.
  ******************************************************************************
  */
#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>

bool SDCard_IsInserted(void);   /* lit CD_SD (PB8) : true si une carte est dans le connecteur */
bool SDCard_Open(void);         /* initialise la carte (SDMMC + HAL) ; false si illisible      */
void SDCard_Close(void);        /* arrête le périphérique SDMMC (carte retirée ou erreur)      */

#endif /* SD_CARD_H */
