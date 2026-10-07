/**
  ******************************************************************************
  * @file    log_config.h
  * @brief   Réglages du système de logs. Tout ce qui est ajustable est ici.
  ******************************************************************************
  */
#ifndef LOG_CONFIG_H
#define LOG_CONFIG_H

/* ===== Fréquence d'échantillonnage IMU (modifiable) =====
 * Utilisée pour : (1) configurer le futur pilote BMI270 (ODR), (2) dimensionner la queue du
 * flux brut, (3) être inscrite dans l'en-tête des fichiers brut pour que le script Python sache
 * à quelle cadence relire les échantillons. Valeurs usuelles : 100, 200, 400, 800, 1600. */
#define LOG_IMU_RATE_HZ              100u

#define LOG_IMU_SAMPLES_PER_RECORD   2u      /* échantillons regroupés dans un record brut (LogImu_t) */
#define LOG_CAN_RAW_EST_FPS          200u    /* trames CAN/s attendues, pour dimensionner la queue brute */
#define LOG_RAW_BUFFER_SECONDS       1u      /* durée de données brutes que la queue doit absorber    */

/* ===== Version affichée dans l'en-tête des fichiers et le record BOOT : 0xMMmmpppp ===== */
#define LOG_FW_VERSION               0x00010000u

/* ===== Queues (en nombre de records) ===== */
#define LOG_SESSION_QUEUE_DEPTH      256u    /* 256 x 16 o = 4 Ko  : ne doit JAMAIS déborder   */
#define LOG_SYS_QUEUE_DEPTH          32u     /*  32 x 16 o         */
#define LOG_RAW_QUEUE_DEPTH_MIN      64u
#define LOG_RAW_QUEUE_DEPTH_CALC     (((LOG_IMU_RATE_HZ / LOG_IMU_SAMPLES_PER_RECORD) + LOG_CAN_RAW_EST_FPS) * LOG_RAW_BUFFER_SECONDS)
#define LOG_RAW_QUEUE_DEPTH          (LOG_RAW_QUEUE_DEPTH_CALC > LOG_RAW_QUEUE_DEPTH_MIN ? LOG_RAW_QUEUE_DEPTH_CALC : LOG_RAW_QUEUE_DEPTH_MIN)

/* ===== Tâche de log ===== */
#define LOG_TASK_PERIOD_MS           200u    /* réveil de la tâche : vide les queues d'un coup */
#define LOG_STATS_PERIOD_MS          10000u  /* record STATS + contrôle de l'espace libre       */
#define LOG_STAGE_SIZE               2048u   /* tampon RAM par flux (multiple de 16 et 32)      */

/* ===== Politique de synchronisation (écriture réelle des métadonnées) ===== */
#define LOG_SD_SYNC_PERIOD_MS        2000u   /* FAT : peu coûteux. Perte max en cas de coupure ~2 s */
#define LOG_FLASH_SYNC_PERIOD_MS     30000u  /* littlefs sur NAND (blocs de 128 Ko) : un sync au milieu d'un
                                                bloc recopie ce bloc => à espacer. Perte max ~30 s.        */

/* ===== Espace disque ===== */
#define LOG_SD_MIN_FREE_MB           64u     /* en dessous : carte considérée pleine -> bascule flash   */
#define LOG_FLASH_MIN_FREE_BLOCKS    8u      /* 8 x 128 Ko = 1 Mo : on supprime la plus ancienne session flash */
#define LOG_RAW_FILE_MAX_BYTES       (512u * 1024u * 1024u)  /* nouveau segment brut au-delà (FAT32 : 4 Go max) */

/* ===== Noms ===== */
#define LOG_SD_ROOT_DIR              "DASH"  /* seul dossier créé/modifié sur la carte SD de l'utilisateur */

#endif /* LOG_CONFIG_H */
