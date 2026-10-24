/**
  ******************************************************************************
  * @file    bmi270.c
  * @brief   Pilote Bosch BMI270 sur SPI1 (datasheet BST-BMI270-DS000-08).
  *
  * Principe :
  *
  *   - Accès aux registres (rares, 1 à 2 octets) : transferts SPI directs, ~3 us chacun.
  *   - Lecture des données : le capteur écrit chaque échantillon (accéléromètre + gyroscope) dans
  *     son FIFO de 2 Ko. Quand il contient BMI270_FIFO_WATERMARK_MS de données, il lève INT1
  *     (PE0) ; la tâche se réveille, lit TOUT le FIFO en une seule transaction SPI par DMA
  *     (le CPU ne fait rien pendant le transfert), puis décode les trames.
  *   - Aucun échantillon n'est perdu si la tâche est retardée jusqu'à ~157 échantillons ; un
  *     débordement est signalé par le capteur (trame "skip") et compté.
  *
  * Protocole SPI (datasheet 6.4, p. 123-126), mode 0, 10 MHz max :
  *   écriture : [adresse & 0x7F] [donnée] [donnée]...      (adresse auto-incrémentée)
  *   lecture  : [adresse | 0x80] [octet factice] [donnée] [donnée]...
  *   Un octet factice sépare toujours l'adresse des données en lecture SPI (p. 17 et 126).
  *   Le chip select est piloté par GPIO (PA4) : voir main.h, gpio.c et spi.c.
  ******************************************************************************
  */
#include "bmi270.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

/* Fichier de configuration Bosch (lib/bmi270_config/), exactement 8192 octets */
extern const uint8_t bmi270_config_file[];
#define BMI270_CONFIG_SIZE        8192u

/* ===== Registres (datasheet 5.2, p. 71-120) ===== */
#define REG_CHIP_ID               0x00u   /* 0x24                                                    */
#define REG_ERR_REG               0x02u   /* bit0 fatal_err ; effacé à la lecture                    */
#define REG_EVENT                 0x1Bu   /* bits 4..2 error_code (ACC_CONF / GYR_CONF refusés)       */
#define REG_INTERNAL_STATUS       0x21u   /* bits 3..0 message : 0x01 = init_ok                      */
#define REG_FIFO_LENGTH_0         0x24u   /* octets dans le FIFO : _0 = bits 7..0                    */
#define REG_FIFO_LENGTH_1         0x25u   /*                       _1 = bits 13..8 (6 bits utiles)   */
#define REG_FIFO_DATA             0x26u   /* lecture en rafale, l'adresse ne s'incrémente pas        */
#define REG_FEAT_PAGE             0x2Fu
#define REG_FEATURES_GYR_CAS      0x3Cu   /* page 0 : GYR_CAS, bits 6..0 = factor_zx (p. 89)         */
#define REG_ACC_CONF              0x40u
#define REG_ACC_RANGE             0x41u
#define REG_GYR_CONF              0x42u
#define REG_GYR_RANGE             0x43u
#define REG_FIFO_DOWNS            0x45u
#define REG_FIFO_WTM_0            0x46u   /* seuil en octets : _0 = bits 7..0                        */
#define REG_FIFO_WTM_1            0x47u   /*                   _1 = bits 12..8                       */
#define REG_FIFO_CONFIG_0         0x48u
#define REG_FIFO_CONFIG_1         0x49u
#define REG_INT1_IO_CTRL          0x53u
#define REG_INT_LATCH             0x55u
#define REG_INT_MAP_DATA          0x58u
#define REG_INIT_CTRL             0x59u
#define REG_INIT_DATA             0x5Eu   /* écriture en rafale, l'adresse ne s'incrémente pas       */
#define REG_PWR_CONF              0x7Cu
#define REG_PWR_CTRL              0x7Du
#define REG_CMD                   0x7Eu

#define CHIP_ID_BMI270            0x24u
#define CMD_FIFO_FLUSH            0xB0u
#define CMD_SOFTRESET             0xB6u

/* ===== Valeurs de configuration, déduites des réglages de bmi270.h ===== */

/* ODR : même codage pour ACC_CONF.acc_odr et GYR_CONF.gyr_odr (p. 100-102) */
#if   BMI270_ODR_HZ == 25
#define ODR_CODE                  0x06u
#elif BMI270_ODR_HZ == 50
#define ODR_CODE                  0x07u
#elif BMI270_ODR_HZ == 100
#define ODR_CODE                  0x08u
#elif BMI270_ODR_HZ == 200
#define ODR_CODE                  0x09u
#elif BMI270_ODR_HZ == 400
#define ODR_CODE                  0x0Au
#elif BMI270_ODR_HZ == 800
#define ODR_CODE                  0x0Bu
#elif BMI270_ODR_HZ == 1600
#define ODR_CODE                  0x0Cu
#else
#error "BMI270_ODR_HZ invalide : 25, 50, 100, 200, 400, 800 ou 1600"
#endif

/* ACC_RANGE.acc_range (p. 102) */
#if   BMI270_ACC_RANGE_G == 2
#define ACC_RANGE_CODE            0x00u
#elif BMI270_ACC_RANGE_G == 4
#define ACC_RANGE_CODE            0x01u
#elif BMI270_ACC_RANGE_G == 8
#define ACC_RANGE_CODE            0x02u
#elif BMI270_ACC_RANGE_G == 16
#define ACC_RANGE_CODE            0x03u
#else
#error "BMI270_ACC_RANGE_G invalide : 2, 4, 8 ou 16"
#endif

/* GYR_RANGE.gyr_range (p. 103) */
#if   BMI270_GYR_RANGE_DPS == 2000
#define GYR_RANGE_CODE            0x00u
#elif BMI270_GYR_RANGE_DPS == 1000
#define GYR_RANGE_CODE            0x01u
#elif BMI270_GYR_RANGE_DPS == 500
#define GYR_RANGE_CODE            0x02u
#elif BMI270_GYR_RANGE_DPS == 250
#define GYR_RANGE_CODE            0x03u
#elif BMI270_GYR_RANGE_DPS == 125
#define GYR_RANGE_CODE            0x04u
#else
#error "BMI270_GYR_RANGE_DPS invalide : 125, 250, 500, 1000 ou 2000"
#endif

/* Mode "performance" de la datasheet (p. 22) :
   ACC_CONF : acc_filter_perf=1 (bit7), acc_bwp=0b010 "normal" (bits 6..4), acc_odr (bits 3..0)
   GYR_CONF : gyr_filter_perf=1 (bit7), gyr_noise_perf=1 (bit6), gyr_bwp=0b10 (bits 5..4), gyr_odr */
#define ACC_CONF_VALUE            (0x80u | (0x02u << 4) | ODR_CODE)
#define GYR_CONF_VALUE            (0x80u | 0x40u | (0x02u << 4) | ODR_CODE)

#define FIFO_CONFIG_0_VALUE       0x02u   /* fifo_time_en=1 (trame sensortime), stop_on_full=0 (écrase le plus ancien) */
#define FIFO_CONFIG_1_VALUE       0xD0u   /* fifo_gyr_en | fifo_acc_en | fifo_header_en (p. 106)                       */
#define FIFO_DOWNS_VALUE          0x88u   /* données FILTRÉES, sans sous-échantillonnage (p. 104)                      */
#define INT1_IO_CTRL_VALUE        0x0Au   /* output_en=1, push-pull, actif haut : front montant sur PE0 (p. 109)        */
#define INT_LATCH_VALUE           0x00u   /* non verrouillée : la broche retombe quand le FIFO repasse sous le seuil    */
#define INT_MAP_DATA_VALUE        0x02u   /* fwm_int1 : FIFO watermark -> INT1 (p. 112)                                 */
#define PWR_CTRL_VALUE            0x0Eu   /* temp_en | acc_en | gyr_en (p. 120)                                         */
#define PWR_CONF_VALUE            0x02u   /* adv_power_save=0, fifo_self_wake_up=1 (p. 119)                             */

/* Seuil FIFO en octets : nombre entier de trames, au moins une */
#define FIFO_WM_FRAMES_RAW        ((BMI270_ODR_HZ * BMI270_FIFO_WATERMARK_MS) / 1000u)
#define FIFO_WM_FRAMES            (FIFO_WM_FRAMES_RAW > 0u ? FIFO_WM_FRAMES_RAW : 1u)
#define FIFO_WM_BYTES             (FIFO_WM_FRAMES * BMI270_FIFO_FRAME_BYTES)
_Static_assert(FIFO_WM_BYTES <= (BMI270_FIFO_BYTES - 4u * BMI270_FIFO_FRAME_BYTES),
               "BMI270_FIFO_WATERMARK_MS trop grand : le FIFO déborderait avant la lecture");
_Static_assert(BMI270_FIFO_MAX_FRAMES * BMI270_FIFO_FRAME_BYTES >= BMI270_FIFO_BYTES,
               "BMI270_FIFO_MAX_FRAMES trop petit pour le FIFO");

/* ===== Délais (datasheet p. 25 et 123) ===== */
#define SPI_ACTIVATION_US         450u    /* après le front montant de CSB, l'interface passe en SPI (>= 200 us) */
#define IDLE_AFTER_READ_US        2u      /* tIDLE_rd : après une lecture, tous modes                             */
#define IDLE_AFTER_WRITE_US       2u      /* tIDLE_wr_act : après une écriture, mode normal                       */
#define IDLE_AFTER_WRITE_APS_US   450u    /* tIDLE_wacc_sum : après une écriture, mode suspend / basse conso      */
#define RESET_WAIT_MS             3u      /* après soft reset : tPO = 2 ms max (p. 11), +1 tick de marge          */
#define INIT_POLL_STEP_MS         5u      /* INTERNAL_STATUS : "au plus 20 ms" (p. 25)                            */
#define INIT_POLL_TRIES           10u
#define GYR_STARTUP_MS            80u     /* démarrage du gyroscope : 45 ms typ. (p. 13)                          */
#define XFER_TIMEOUT_MS           20u     /* 2 Ko à 8 MHz = 2 ms                                                  */

/* Octets lus en plus de FIFO_LENGTH : de quoi recevoir la trame "sensortime" (4 octets) que le
   capteur ajoute quand le FIFO se vide pendant la lecture (p. 36). La trame sensortime n'est pas
   comptée dans FIFO_LENGTH. */
#define FIFO_EXTRA_BYTES          4u

/* Un transfert = 1 octet d'adresse + 1 octet factice + les données */
#define SPI_READ_OVERHEAD         2u
#define POLL_MAX_BYTES            8u      /* accès registres directs : 8 octets de données au plus */

/* Tampon DMA en AXI SRAM (le DMA2 n'accède pas à la DTCM, voir ws2812.c). Il sert à la fois à
   l'envoi du fichier de configuration (1 octet d'adresse + 8192) et à la lecture du FIFO. */
#define XFER_BUF_SIZE             ((1u + BMI270_CONFIG_SIZE + 31u) & ~31u)
_Static_assert(XFER_BUF_SIZE >= (SPI_READ_OVERHEAD + BMI270_FIFO_BYTES + FIFO_EXTRA_BYTES),
               "tampon trop petit pour lire tout le FIFO");

__attribute__((section(".dma_buffers"), aligned(32))) static uint8_t s_buf[XFER_BUF_SIZE];
/* Octet envoyé sur MOSI pendant toute la lecture du FIFO : adresse de FIFO_DATA avec le bit de
   lecture. Après l'adresse le capteur ignore MOSI (p. 126), répéter cet octet est sans effet. */
__attribute__((section(".dma_buffers"), aligned(32))) static uint8_t s_txRead[32];

static SemaphoreHandle_t   s_xferDone;
static volatile uint8_t    s_xferFailed;
static bool                s_ready;
static bool                s_aps;            /* "advanced power save" actif : écritures plus lentes */
static int8_t              s_gyrCas;
static Bmi270_Stats_t      s_stats;

/* ===== Temporisation fine : compteur de cycles du coeur (DWT) ===== */
static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR          = 0xC5ACCE55u;              /* déverrouillage (Cortex-M7), sans effet sinon */
  DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
  const uint32_t start  = DWT->CYCCNT;
  const uint32_t cycles = us * (SystemCoreClock / 1000000u);
  while ((DWT->CYCCNT - start) < cycles) { }
}

static void delay_ms(uint32_t ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms));
}

/* ===== Bus ===== */
static inline void cs_low(void)  { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_high(void) { HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);   }

/* Lecture de 1 à POLL_MAX_BYTES octets consécutifs, en une seule transaction */
static bool reg_read(uint8_t reg, uint8_t *dst, uint8_t n)
{
  uint8_t tx[SPI_READ_OVERHEAD + POLL_MAX_BYTES] = { (uint8_t)(reg | 0x80u) };
  uint8_t rx[SPI_READ_OVERHEAD + POLL_MAX_BYTES];

  cs_low();
  const HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(SPI_READ_OVERHEAD + n), 10);
  cs_high();
  delay_us(IDLE_AFTER_READ_US);

  if (st != HAL_OK) { s_stats.spiErrors++; return false; }
  memcpy(dst, &rx[SPI_READ_OVERHEAD], n);
  return true;
}

static bool reg_write(uint8_t reg, uint8_t value)
{
  const uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

  cs_low();
  const HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, sizeof(tx), 10);
  cs_high();
  delay_us(s_aps ? IDLE_AFTER_WRITE_APS_US : IDLE_AFTER_WRITE_US);

  if (st != HAL_OK) { s_stats.spiErrors++; return false; }
  return true;
}

/* Envoi du fichier de configuration : UNE transaction (adresse + 8192 octets) pour ne jamais
   interrompre l'horloge SCK pendant que CS est bas. */
static bool upload_config(void)
{
  s_buf[0] = REG_INIT_DATA;                              /* écriture, l'adresse ne s'incrémente pas */
  memcpy(&s_buf[1], bmi270_config_file, BMI270_CONFIG_SIZE);

  cs_low();
  const HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, s_buf, (uint16_t)(1u + BMI270_CONFIG_SIZE), 100);
  cs_high();
  delay_us(IDLE_AFTER_WRITE_US);

  if (st != HAL_OK) { s_stats.spiErrors++; return false; }
  return true;
}

/* Fin de transfert DMA (interruption) : réveille la tâche qui attend dans fifo_read_dma() */
static void xfer_done_from_isr(void)
{
  if (s_xferDone != NULL) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_xferDone, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) { xfer_done_from_isr(); }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    s_xferFailed = 1;
    xfer_done_from_isr();
  }
}

/* Lit `n` octets de FIFO_DATA dans &s_buf[SPI_READ_OVERHEAD] par DMA ; la tâche dort pendant
   le transfert. */
static bool fifo_read_dma(uint16_t n)
{
  const uint16_t total = (uint16_t)(SPI_READ_OVERHEAD + n);

  s_txRead[0]  = (uint8_t)(REG_FIFO_DATA | 0x80u);
  s_xferFailed = 0;
  (void)xSemaphoreTake(s_xferDone, 0);                   /* purge un éventuel signal en retard */

  if (SCB->CCR & SCB_CCR_DC_Msk) {                       /* sans effet tant que le cache D est désactivé */
    SCB_CleanDCache_by_Addr((uint32_t *)s_txRead, sizeof(s_txRead));
  }

  cs_low();
  bool ok = (HAL_SPI_TransmitReceive_DMA(&hspi1, s_txRead, s_buf, total) == HAL_OK);
  if (ok) {
    ok = (xSemaphoreTake(s_xferDone, pdMS_TO_TICKS(XFER_TIMEOUT_MS)) == pdTRUE) && !s_xferFailed;
    if (!ok && hspi1.State != HAL_SPI_STATE_READY) {
      (void)HAL_SPI_Abort(&hspi1);
    }
  }
  cs_high();
  delay_us(IDLE_AFTER_READ_US);

  if (!ok) { s_stats.spiErrors++; return false; }

  if (SCB->CCR & SCB_CCR_DC_Msk) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_buf, (int32_t)((total + 31u) & ~31u));
  }
  return true;
}

/* ===== Décodage du FIFO en mode "header" (datasheet 4.7.1, p. 35-36) =====
 *
 * En-tête (1 octet) : bits 7..6 = fh_mode, bits 5..2 = fh_parm, bits 1..0 = fh_ext (étiquettes INT)
 *   fh_mode = 0b10 : trame régulière ; fh_parm : bit0 accéléromètre, bit1 gyroscope, bit2 auxiliaire.
 *                    Données dans l'ordre : [auxiliaire] [gyro X Y Z] [accéléro X Y Z], 16 bits
 *                    little-endian. Accéléro + gyro = 1 + 12 = 13 octets.
 *   fh_mode = 0b01 : trame de contrôle ; fh_parm = code :
 *                    0 skip (1 octet : nombre de trames perdues), 1 sensortime (3 octets,
 *                    little-endian), 2 changement de configuration (4 octets).
 *   0x80           : "trame non initialisée", renvoyée quand on lit plus que le contenu du FIFO.
 */
#define FH_MODE_MASK              0xC0u
#define FH_MODE_REGULAR           0x80u
#define FH_MODE_CONTROL           0x40u
#define FH_PARM_ACC               0x01u
#define FH_PARM_GYR               0x02u
#define FH_PARM_AUX               0x04u
#define FH_CTRL_SKIP              0x0u
#define FH_CTRL_SENSORTIME        0x1u
#define FH_CTRL_INPUT_CONFIG      0x2u

static inline int16_t le16(const uint8_t *p) { return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }

static int16_t sat16(int32_t v)
{
  if (v >  32767) { return  32767; }
  if (v < -32768) { return -32768; }
  return (int16_t)v;
}

static uint32_t decode_fifo(const uint8_t *p, uint32_t len, Bmi270_Sample_t *out, uint32_t maxOut,
                            Bmi270_FifoInfo_t *info)
{
  uint32_t i = 0;
  uint32_t n = 0;
  bool     stop = false;

  while (i < len && !stop) {
    const uint8_t h = p[i];

    if ((h & FH_MODE_MASK) == FH_MODE_REGULAR) {
      const uint8_t parm = (uint8_t)((h >> 2) & 0x0Fu);

      if (parm == 0u)            { break; }                          /* 0x80 : fin des données            */
      if (parm & FH_PARM_AUX)    { s_stats.badFrames++; break; }     /* l'interface auxiliaire est inutilisée */

      const uint32_t size = 1u + ((parm & FH_PARM_ACC) ? 6u : 0u) + ((parm & FH_PARM_GYR) ? 6u : 0u);
      if (i + size > len)        { break; }                          /* trame coupée : le capteur la renverra */

      if (parm == (FH_PARM_ACC | FH_PARM_GYR)) {
        if (n >= maxOut) { break; }
        const uint8_t *d = &p[i + 1];
        Bmi270_Sample_t *s = &out[n++];

        /* Correction de sensibilité croisée du gyroscope (datasheet 4.6.10, p. 29) :
           Rate_x = gyr_x - GYR_CAS.factor_zx * gyr_z / 2^9 ; y et z inchangés */
        const int16_t gx = le16(&d[0]);
        const int16_t gy = le16(&d[2]);
        const int16_t gz = le16(&d[4]);
        s->gyr[0] = sat16((int32_t)gx - ((int32_t)s_gyrCas * gz) / 512);
        s->gyr[1] = gy;
        s->gyr[2] = gz;
        s->acc[0] = le16(&d[6]);
        s->acc[1] = le16(&d[8]);
        s->acc[2] = le16(&d[10]);
      } else {
        s_stats.badFrames++;       /* un seul capteur dans la trame : transitoire, on ne le garde pas */
      }
      i += size;

    } else if ((h & FH_MODE_MASK) == FH_MODE_CONTROL) {
      switch ((h >> 2) & 0x0Fu) {
        case FH_CTRL_SKIP:
          if (i + 2u > len) { stop = true; break; }
          info->skipped = (uint16_t)(info->skipped + p[i + 1]);
          i += 2u;
          break;
        case FH_CTRL_SENSORTIME:
          if (i + 4u > len) { stop = true; break; }
          info->sensorTime = (uint32_t)p[i + 1] | ((uint32_t)p[i + 2] << 8) | ((uint32_t)p[i + 3] << 16);
          info->hasTime    = true;
          i += 4u;
          break;
        case FH_CTRL_INPUT_CONFIG:
          if (i + 5u > len) { stop = true; break; }
          i += 5u;
          break;
        default:
          s_stats.badFrames++;
          stop = true;
          break;
      }
    } else {
      s_stats.badFrames++;
      break;
    }
  }
  return n;
}

uint32_t BMI270_ReadFifo(Bmi270_Sample_t *out, uint32_t maxOut, Bmi270_FifoInfo_t *info)
{
  Bmi270_FifoInfo_t local = { 0 };
  if (info == NULL) { info = &local; }
  memset(info, 0, sizeof(*info));

  if (!s_ready || out == NULL) { return 0; }

  /* FIFO_LENGTH_0 et _1 sont lus ensemble : leurs valeurs restent cohérentes (p. 70) */
  uint8_t lenReg[2];
  if (!reg_read(REG_FIFO_LENGTH_0, lenReg, 2)) { return 0; }
  uint32_t len = (uint32_t)lenReg[0] | ((uint32_t)(lenReg[1] & 0x3Fu) << 8);
  if (len == 0u) { return 0; }

  if (len > BMI270_FIFO_BYTES) { len = BMI270_FIFO_BYTES; }
  const uint32_t readLen = len + FIFO_EXTRA_BYTES;
  if (!fifo_read_dma((uint16_t)readLen)) { return 0; }

  /* s_buf[0] = octet reçu pendant l'adresse, s_buf[1] = octet factice : les données commencent à [2] */
  const uint32_t n = decode_fifo(&s_buf[SPI_READ_OVERHEAD], readLen, out, maxOut, info);

  info->frames     = (uint16_t)n;
  s_stats.samples += n;
  s_stats.skipped += info->skipped;
  return n;
}

/* ===== Initialisation (datasheet 4.4 p. 25, 4.6.10, 4.7, 4.9) ===== */

/* Lecture "à blanc" : le front montant de CSB fait passer le capteur de I2C à SPI (p. 123) */
static void select_spi_interface(void)
{
  uint8_t dummy;
  (void)reg_read(REG_CHIP_ID, &dummy, 1);
  delay_us(SPI_ACTIVATION_US);
}

static bool check_chip_id(void)
{
  uint8_t id = 0;
  if (!reg_read(REG_CHIP_ID, &id, 1)) { s_stats.lastError = BMI270_ERR_SPI; return false; }
  s_stats.chipId = id;
  if (id != CHIP_ID_BMI270)           { s_stats.lastError = BMI270_ERR_CHIP_ID; return false; }
  return true;
}

static bool wait_init_ok(void)
{
  for (uint32_t t = 0; t < INIT_POLL_TRIES; t++) {
    delay_ms(INIT_POLL_STEP_MS);
    uint8_t status = 0;
    if (!reg_read(REG_INTERNAL_STATUS, &status, 1)) { s_stats.lastError = BMI270_ERR_SPI; return false; }
    if ((status & 0x0Fu) == 0x01u) { return true; }              /* message = init_ok */
  }
  s_stats.lastError = BMI270_ERR_INIT;
  return false;
}

static bool write_list(const uint8_t (*list)[2], uint32_t count)
{
  for (uint32_t i = 0; i < count; i++) {
    if (!reg_write(list[i][0], list[i][1])) { s_stats.lastError = BMI270_ERR_SPI; return false; }
  }
  return true;
}

bool BMI270_Init(void)
{
  s_ready = false;
  s_stats.lastError = BMI270_ERR_NONE;

  if (s_xferDone == NULL) { s_xferDone = xSemaphoreCreateBinary(); }
  if (s_xferDone == NULL) { s_stats.lastError = BMI270_ERR_OS; return false; }

  dwt_init();
  cs_high();

  /* 1) Interface SPI + identification */
  select_spi_interface();
  if (!check_chip_id()) { return false; }

  /* 2) Soft reset : ramène le capteur à l'état de mise sous tension (suspend, advanced power save
        actif), ce qui permet d'envoyer le fichier de configuration à chaque démarrage du STM32
        même si le capteur, lui, est resté alimenté. Il repasse en mode I2C : on refait la
        sélection SPI (p. 69, 123). */
  s_aps = true;
  if (!reg_write(REG_CMD, CMD_SOFTRESET)) { s_stats.lastError = BMI270_ERR_SPI; return false; }
  delay_ms(RESET_WAIT_MS);
  select_spi_interface();
  if (!check_chip_id()) { return false; }

  /* 3) Séquence d'initialisation de la datasheet (p. 18 et 25) */
  if (!reg_write(REG_PWR_CONF, 0x00u)) { s_stats.lastError = BMI270_ERR_SPI; return false; }  /* adv_power_save = 0 ; 450 us ensuite */
  s_aps = false;
  if (!reg_write(REG_INIT_CTRL, 0x00u)) { s_stats.lastError = BMI270_ERR_SPI; return false; } /* prépare le chargement */
  if (!upload_config())                 { s_stats.lastError = BMI270_ERR_SPI; return false; } /* 8192 octets dans INIT_DATA */
  if (!reg_write(REG_INIT_CTRL, 0x01u)) { s_stats.lastError = BMI270_ERR_SPI; return false; } /* termine le chargement */
  if (!wait_init_ok()) { return false; }

  /* 4) Capteurs, FIFO, interruption. Les capteurs sont encore éteints (mode configuration). */
  static const uint8_t cfg[][2] = {
    { REG_ACC_CONF,     ACC_CONF_VALUE     },
    { REG_ACC_RANGE,    ACC_RANGE_CODE     },
    { REG_GYR_CONF,     GYR_CONF_VALUE     },
    { REG_GYR_RANGE,    GYR_RANGE_CODE     },
    { REG_FIFO_CONFIG_0, FIFO_CONFIG_0_VALUE },
    { REG_FIFO_DOWNS,   FIFO_DOWNS_VALUE   },
    { REG_FIFO_WTM_0,   (uint8_t)(FIFO_WM_BYTES & 0xFFu) },
    { REG_FIFO_WTM_1,   (uint8_t)((FIFO_WM_BYTES >> 8) & 0x1Fu) },
    { REG_FIFO_CONFIG_1, FIFO_CONFIG_1_VALUE },
    { REG_INT1_IO_CTRL, INT1_IO_CTRL_VALUE },
    { REG_INT_LATCH,    INT_LATCH_VALUE    },
    { REG_INT_MAP_DATA, INT_MAP_DATA_VALUE },
    /* Allumage : accéléromètre + gyroscope + température, puis PWR_CONF comme dans l'exemple
       "performance mode" de la datasheet (p. 22) */
    { REG_PWR_CTRL,     PWR_CTRL_VALUE     },
    { REG_PWR_CONF,     PWR_CONF_VALUE     },
  };
  if (!write_list(cfg, sizeof(cfg) / sizeof(cfg[0]))) { return false; }

  delay_ms(GYR_STARTUP_MS);

  /* 5) Contrôles : réglages acceptés, pas d'erreur fatale */
  uint8_t ev = 0, err = 0;
  if (!reg_read(REG_EVENT, &ev, 1) || !reg_read(REG_ERR_REG, &err, 1)) { s_stats.lastError = BMI270_ERR_SPI; return false; }
  if (((ev >> 2) & 0x07u) != 0u) { s_stats.lastError = BMI270_ERR_CONFIG; return false; }
  if (err & 0x01u)               { s_stats.lastError = BMI270_ERR_FATAL;  return false; }

  /* 6) Facteur CAS du gyroscope : registre FEATURES, page 0, adresse 0x3C, 7 bits signés (p. 29, 89) */
  uint8_t cas[2];
  if (!reg_write(REG_FEAT_PAGE, 0x00u) || !reg_read(REG_FEATURES_GYR_CAS, cas, 2)) {
    s_stats.lastError = BMI270_ERR_SPI;
    return false;
  }
  s_gyrCas = (int8_t)((cas[0] & 0x40u) ? (cas[0] | 0x80u) : (cas[0] & 0x7Fu));  /* extension de signe 7 -> 8 bits */
  s_stats.gyrCas = s_gyrCas;

  /* 7) On repart d'un FIFO vide : les premières trames pendant le démarrage du gyroscope sont inutiles */
  if (!reg_write(REG_CMD, CMD_FIFO_FLUSH)) { s_stats.lastError = BMI270_ERR_SPI; return false; }

  s_ready = true;
  s_stats.inits++;
  return true;
}

void BMI270_GetStats(Bmi270_Stats_t *stats)
{
  *stats = s_stats;
}
