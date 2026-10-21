/**
  ******************************************************************************
  * @file    gnss.c
  * @brief   Pilote u-blox NEO-M9N (UBX) : réception DMA circulaire, parseur de trames,
  *          configuration par UBX-CFG-VALSET, décodage de UBX-NAV-PVT.
  *
  * Réception (zéro interruption par octet) :
  *
  *   octet reçu sur PB15 -> le DMA le copie dans s_rx[] (tampon circulaire, AXI SRAM)
  *   fin de rafale (ligne silencieuse 1 octet) -> interruption IDLE -> réveille la tâche
  *   la tâche appelle GNSS_Poll() : lit les nouveaux octets, valide et décode les trames.
  *
  * L'API de réception de la HAL (HAL_UARTEx_ReceiveToIdle_DMA) n'est volontairement PAS
  * utilisée : à la moindre erreur UART (très fréquente si le module émet à un autre débit que
  * le STM32, ex. au démarrage après un reset du STM32 seul) elle arrête le DMA et peut refuser
  * de démarrer. Ici le DMA circulaire ne s'arrête jamais : l'overrun est désactivé (OVRDIS), les
  * erreurs de trame/bruit n'ont aucun effet sur le DMA, et le parseur rejette les trames
  * corrompues par leur checksum.
  *
  * Une trame NAV-PVT de 100 octets dure 2,2 ms à 460800 bauds : la solution est décodée
  * environ 2,3 ms après la fin de son émission par le module.
  *
  * Protocole UBX (Interface description 3.2) :
  *   0xB5 0x62 | classe | id | longueur (U2 little-endian) | payload | CK_A CK_B
  *   checksum de Fletcher 8 bits sur classe..fin du payload.
  ******************************************************************************
  */
#include "gnss.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define GNSS_RX_BUF_SIZE        1024u   /* DOIT être une puissance de 2 (2,5 Ko/s reçus : ~400 ms de marge) */
#define GNSS_MAX_PAYLOAD        128u    /* NAV-PVT = 92 ; les trames plus longues sont ignorées            */
#define GNSS_CFG_LAYER_RAM      0x01u   /* UBX-CFG-VALSET.layers : bit0 = RAM (pas de flash : pas d'usure) */
#define GNSS_CFG_MAX_ITEMS      8u
#define GNSS_PROBE_TIMEOUT_MS   100u
#define GNSS_ACK_TIMEOUT_MS     1000u   /* ACK/NAK "au plus tard dans la seconde" (Interface description 3.9.1) */

_Static_assert((GNSS_RX_BUF_SIZE & (GNSS_RX_BUF_SIZE - 1u)) == 0u, "GNSS_RX_BUF_SIZE : puissance de 2");
_Static_assert((1000u % GNSS_RATE_HZ) == 0u && (1000u / GNSS_RATE_HZ) >= 25u,
               "GNSS_RATE_HZ : 1000/rate entier et >= 25 ms (CFG-RATE-MEAS, minimum 25)");

/* ===== Classes / identifiants UBX ===== */
#define UBX_CLS_NAV         0x01u
#define UBX_ID_NAV_PVT      0x07u   /* 92 octets (Interface description 3.15.11) */
#define UBX_CLS_ACK         0x05u
#define UBX_ID_ACK_NAK      0x00u
#define UBX_ID_ACK_ACK      0x01u
#define UBX_CLS_CFG         0x06u
#define UBX_ID_CFG_VALSET   0x8Au
#define UBX_ID_CFG_VALGET   0x8Bu
#define UBX_NAV_PVT_LEN     92u

/* ===== Clés de configuration (Interface description chap. 5.9) =====
   Bits 30..28 de la clé = taille de la valeur : 1 et 2 = 1 octet, 3 = 2 octets, 4 = 4 octets. */
#define CFG_UART1_BAUDRATE              0x40520001u   /* U4                       */
#define CFG_UART1INPROT_UBX             0x10730001u   /* L                        */
#define CFG_UART1OUTPROT_UBX            0x10740001u   /* L                        */
#define CFG_UART1OUTPROT_NMEA           0x10740002u   /* L                        */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1    0x20910007u   /* U1 : messages par époque */
#define CFG_NAVSPG_DYNMODEL             0x20110021u   /* E1                       */
#define CFG_RATE_MEAS                   0x30210001u   /* U2 : ms entre mesures    */
#define CFG_RATE_NAV                    0x30210002u   /* U2 : mesures par solution */

typedef struct {
  uint32_t key;
  uint32_t val;
} GnssCfg_t;

/* Groupe 1 : flux de sortie. Groupe 2 : cadence (séparé : un NAK indique alors lequel pose problème) */
static const GnssCfg_t cfg_stream[] = {
  { CFG_UART1INPROT_UBX,           1u },
  { CFG_UART1OUTPROT_UBX,          1u },
  { CFG_UART1OUTPROT_NMEA,         0u },   /* plus de NMEA : toute la bande passante pour NAV-PVT */
  { CFG_MSGOUT_UBX_NAV_PVT_UART1,  1u },   /* un NAV-PVT par époque de navigation                 */
  { CFG_NAVSPG_DYNMODEL,           GNSS_DYNMODEL },
};

static const GnssCfg_t cfg_rate[] = {
  { CFG_RATE_NAV,                  1u },                          /* une solution par mesure */
  { CFG_RATE_MEAS,                 1000u / GNSS_RATE_HZ },        /* 40 ms = 25 Hz           */
};

/* Débits essayés pour retrouver le module : le nôtre (déjà configuré, ex. reset du STM32
   seul), puis le défaut usine (38400, datasheet 5.5), puis les autres courants. */
static const uint32_t probe_bauds[] = { GNSS_BAUDRATE, 38400u, 115200u, 9600u };

/* ===== État ===== */
/* Tampon écrit par le DMA : AXI SRAM (.dma_buffers), pas la DTCM inaccessible au DMA1 (voir ws2812.c) */
__attribute__((section(".dma_buffers"), aligned(32))) static uint8_t s_rx[GNSS_RX_BUF_SIZE];
static DMA_HandleTypeDef hdma_usart1_rx;
static uint32_t          s_rd;                   /* prochain index à lire dans s_rx */

static TaskHandle_t      s_waiter;               /* tâche réveillée par l'interruption IDLE */

typedef enum {
  P_SYNC1, P_SYNC2, P_CLASS, P_ID, P_LEN1, P_LEN2, P_PAYLOAD, P_CKA, P_CKB
} ParseState_t;

static struct {
  ParseState_t state;
  uint8_t  cls, id;
  uint16_t len, idx;
  uint8_t  ckA, ckB, rxCkA;
  uint8_t  payload[GNSS_MAX_PAYLOAD];
} s_p;

static Gnss_Fix_t   s_fix;
static uint32_t     s_newPvt;                    /* NAV-PVT décodés pendant le GNSS_Poll en cours */
static GNSS_Stats_t s_stats;
static struct { uint8_t cls, id, result; } s_ack;   /* result : 0 rien, 1 ACK, 2 NAK */

/* ===== Lecture little-endian ===== */
static inline uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline int32_t rd_i32(const uint8_t *p) { return (int32_t)rd_u32(p); }

static void put_le(uint8_t *dst, uint32_t v, uint8_t n)
{
  for (uint8_t i = 0; i < n; i++) { dst[i] = (uint8_t)(v >> (8u * i)); }
}

/* ===== Décodage UBX-NAV-PVT (offsets : Interface description 3.15.11.1) ===== */
static void decode_pvt(const uint8_t *p, Gnss_Fix_t *f)
{
  f->iTOW_ms    = rd_u32(p + 0);
  f->year       = rd_u16(p + 4);
  f->month      = p[6];
  f->day        = p[7];
  f->hour       = p[8];
  f->min        = p[9];
  f->sec        = p[10];
  f->valid      = p[11];
  f->tAcc_ns    = rd_u32(p + 12);
  f->nano_ns    = rd_i32(p + 16);
  f->fixType    = p[20];
  f->flags      = p[21];
  f->numSV      = p[23];
  f->lon_e7     = rd_i32(p + 24);
  f->lat_e7     = rd_i32(p + 28);
  f->hMSL_mm    = rd_i32(p + 36);
  f->hAcc_mm    = rd_u32(p + 40);
  f->gSpeed_mms = rd_i32(p + 60);
  f->headMot_e5 = rd_i32(p + 64);
  f->sAcc_mms   = rd_u32(p + 68);
  f->headAcc_e5 = rd_u32(p + 72);
  f->pDOP_e2    = rd_u16(p + 76);
}

/* ===== Parseur de trames ===== */
static void parser_reset(void)
{
  s_p.state = P_SYNC1;
}

static void on_frame(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
  s_stats.frames++;

  if (cls == UBX_CLS_NAV && id == UBX_ID_NAV_PVT) {
    if (len >= UBX_NAV_PVT_LEN) {
      decode_pvt(pl, &s_fix);
      s_newPvt++;
      s_stats.pvt++;
    }
  } else if (cls == UBX_CLS_ACK && len == 2u) {
    s_ack.cls    = pl[0];
    s_ack.id     = pl[1];
    s_ack.result = (id == UBX_ID_ACK_ACK) ? 1u : 2u;
    if (id == UBX_ID_ACK_NAK) { s_stats.nak++; }
  }
}

static inline void ck_add(uint8_t b)
{
  s_p.ckA = (uint8_t)(s_p.ckA + b);
  s_p.ckB = (uint8_t)(s_p.ckB + s_p.ckA);
}

static void parse_byte(uint8_t b)
{
  switch (s_p.state) {
  case P_SYNC1:
    if (b == 0xB5u) { s_p.state = P_SYNC2; }
    break;
  case P_SYNC2:
    if (b == 0x62u)      { s_p.state = P_CLASS; }
    else if (b != 0xB5u) { s_p.state = P_SYNC1; }
    break;
  case P_CLASS:
    s_p.cls = b; s_p.ckA = 0; s_p.ckB = 0; ck_add(b);
    s_p.state = P_ID;
    break;
  case P_ID:
    s_p.id = b; ck_add(b);
    s_p.state = P_LEN1;
    break;
  case P_LEN1:
    s_p.len = b; ck_add(b);
    s_p.state = P_LEN2;
    break;
  case P_LEN2:
    s_p.len |= (uint16_t)(b << 8); ck_add(b);
    s_p.idx = 0;
    /* Longueur absurde = fausse synchro (bruit au démarrage) : on repart tout de suite au lieu
       d'avaler des centaines d'octets (donc de vraies trames) pour rien. */
    if (s_p.len > GNSS_MAX_PAYLOAD) { s_p.state = P_SYNC1; }
    else                            { s_p.state = s_p.len ? P_PAYLOAD : P_CKA; }
    break;
  case P_PAYLOAD:
    s_p.payload[s_p.idx++] = b; ck_add(b);
    if (s_p.idx == s_p.len) { s_p.state = P_CKA; }
    break;
  case P_CKA:
    s_p.rxCkA = b;
    s_p.state = P_CKB;
    break;
  case P_CKB:
    if (s_p.rxCkA == s_p.ckA && b == s_p.ckB) { on_frame(s_p.cls, s_p.id, s_p.payload, s_p.len); }
    else                                      { s_stats.badChecksum++; }
    s_p.state = P_SYNC1;
    break;
  }
}

/* ===== Réception DMA ===== */
static bool uart_set_baud(uint32_t baud);

static void rx_stop(void)
{
  USART1->CR1 &= ~USART_CR1_IDLEIE;
  USART1->CR3 &= ~USART_CR3_DMAR;
  HAL_DMA_Abort(&hdma_usart1_rx);   /* erreur sans conséquence s'il n'y a rien à arrêter */
}

static bool rx_start(void)
{
  /* Purge ce que le récepteur a vu avant : drapeaux d'erreur, IDLE, octet en attente dans RDR */
  USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF | USART_ICR_IDLECF;
  USART1->RQR = USART_RQR_RXFRQ;

  if (HAL_DMA_Start(&hdma_usart1_rx, (uint32_t)&USART1->RDR, (uint32_t)s_rx, GNSS_RX_BUF_SIZE) != HAL_OK) {
    return false;
  }
  s_rd = 0;            /* le DMA repart de s_rx[0] */
  parser_reset();

  USART1->CR3 |= USART_CR3_DMAR;      /* chaque octet reçu déclenche une requête DMA            */
  USART1->CR1 |= USART_CR1_IDLEIE;    /* seule interruption UART : ligne silencieuse (fin de trame) */
  return true;
}

void GNSS_Init(void)
{
  memset(s_rx, 0, sizeof(s_rx));   /* .dma_buffers n'est pas initialisée au démarrage */

  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA1 Stream3 : USART1_RX -> mémoire, circulaire, octets. (Stream0 = TIM16/WS2812, 1-2 = TIM5) */
  hdma_usart1_rx.Instance                 = DMA1_Stream3;
  hdma_usart1_rx.Init.Request             = DMA_REQUEST_USART1_RX;
  hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
  hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
  hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
  hdma_usart1_rx.Init.Mode                = DMA_CIRCULAR;
  hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_HIGH;
  hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK) { Error_Handler(); }

  /* Overrun désactivé (CR3.OVRDIS) : un octet perdu ne bloque jamais le récepteur. Appliqué
     par HAL_UART_Init dans uart_set_baud(). */
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

  /* Priorité 5 = la plus haute autorisée à appeler FreeRTOS (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY).
     Le handler est dans stm32h7xx_it.c. Pas d'interruption DMA : seule la fin de trame (IDLE) compte. */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Directement au débit voulu : c'est celui du module s'il a déjà été configuré */
  if (!uart_set_baud(GNSS_BAUDRATE)) { Error_Handler(); }
}

/* Appelée depuis USART1_IRQHandler. Ne fait que réveiller la tâche ; la position d'écriture du
   DMA est relue dans GNSS_Poll(). Les erreurs de ligne sont comptées et acquittées ici, à la fin
   de chaque rafale (elles ne déclenchent pas d'interruption elles-mêmes). */
static void notify_from_isr(void)
{
  if (s_waiter != NULL) {
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_waiter, &woken);
    portYIELD_FROM_ISR(woken);
  }
}

void GNSS_UartIrq(void)
{
  uint32_t isr = USART1->ISR;

  if (isr & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) {
    USART1->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
    s_stats.uartErrors++;
  }
  if (isr & USART_ISR_IDLE) {
    USART1->ICR = USART_ICR_IDLECF;
    notify_from_isr();
  }
}

bool GNSS_Wait(uint32_t timeout_ms)
{
  s_waiter = xTaskGetCurrentTaskHandle();
  return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) != 0;
}

uint32_t GNSS_Poll(Gnss_Fix_t *fix)
{
  /* Position d'écriture du DMA : NDTR décompte de GNSS_RX_BUF_SIZE à 1 puis recharge */
  uint32_t wr = (GNSS_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx)) & (GNSS_RX_BUF_SIZE - 1u);

  /* Sans effet tant que le cache D est désactivé (cas actuel) ; protège si on l'active */
  if (SCB->CCR & SCB_CCR_DC_Msk) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_rx, GNSS_RX_BUF_SIZE);
  }

  s_newPvt = 0;
  while (s_rd != wr) {
    parse_byte(s_rx[s_rd]);
    s_rd = (s_rd + 1u) & (GNSS_RX_BUF_SIZE - 1u);
  }

  if (s_newPvt != 0u && fix != NULL) { *fix = s_fix; }
  return s_newPvt;
}

void GNSS_GetStats(GNSS_Stats_t *stats)
{
  *stats = s_stats;
}

/* ===== Émission ===== */
static bool send_frame(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
  uint8_t frame[8u + 4u + GNSS_CFG_MAX_ITEMS * 12u];
  if (len > sizeof(frame) - 8u) { return false; }

  frame[0] = 0xB5u;
  frame[1] = 0x62u;
  frame[2] = cls;
  frame[3] = id;
  frame[4] = (uint8_t)(len & 0xFFu);
  frame[5] = (uint8_t)(len >> 8);
  memcpy(&frame[6], pl, len);

  uint8_t a = 0, b = 0;
  for (uint16_t i = 2; i < 6u + len; i++) {   /* de classe à la fin du payload */
    a = (uint8_t)(a + frame[i]);
    b = (uint8_t)(b + a);
  }
  frame[6u + len] = a;
  frame[7u + len] = b;

  return HAL_UART_Transmit(&huart1, frame, (uint16_t)(len + 8u), 100) == HAL_OK;
}

static uint8_t cfg_value_size(uint32_t key)
{
  switch ((key >> 28) & 7u) {
    case 1: case 2: return 1;
    case 3:         return 2;
    case 4:         return 4;
    default:        return 8;
  }
}

/* UBX-CFG-VALSET version 0 (Interface description 3.10.28.1) : version, layers, 2 réservés,
   puis des paires clé (U4) + valeur (1, 2, 4 ou 8 octets) sans alignement. */
static bool send_valset(const GnssCfg_t *items, uint8_t n)
{
  uint8_t  pl[4u + GNSS_CFG_MAX_ITEMS * 12u];
  uint16_t len = 0;

  if (n > GNSS_CFG_MAX_ITEMS) { return false; }

  pl[len++] = 0x00u;                 /* version  */
  pl[len++] = GNSS_CFG_LAYER_RAM;    /* layers   */
  pl[len++] = 0x00u;                 /* reserved */
  pl[len++] = 0x00u;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t sz = cfg_value_size(items[i].key);
    put_le(&pl[len], items[i].key, 4);
    len += 4u;
    put_le(&pl[len], items[i].val, sz);
    len += sz;
  }
  return send_frame(UBX_CLS_CFG, UBX_ID_CFG_VALSET, pl, len);
}

/* UBX-CFG-VALGET (3.10.27.1) : demande d'une clé en RAM. Sert de "ping" : le module répond
   par un VALGET ou un NAK, dans les deux cas c'est une trame UBX valide donc le débit est bon. */
static bool send_valget_ping(void)
{
  uint8_t pl[8];
  pl[0] = 0x00u;                     /* version  */
  pl[1] = 0x00u;                     /* layer 0 = RAM */
  pl[2] = 0x00u;                     /* position (U2) */
  pl[3] = 0x00u;
  put_le(&pl[4], CFG_UART1_BAUDRATE, 4);
  return send_frame(UBX_CLS_CFG, UBX_ID_CFG_VALGET, pl, sizeof(pl));
}

/* ===== Configuration ===== */
static bool uart_set_baud(uint32_t baud)
{
  rx_stop();
  huart1.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart1) != HAL_OK) { return false; }   /* UART déjà initialisé : pas de MspInit */
  return rx_start();
}

/* Vrai si le module répond (n'importe quelle trame UBX valide) au débit courant */
static bool probe(void)
{
  for (uint8_t attempt = 0; attempt < 2u; attempt++) {
    uint32_t before = s_stats.frames;
    if (!send_valget_ping()) { return false; }

    uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < GNSS_PROBE_TIMEOUT_MS) {
      GNSS_Wait(5);
      GNSS_Poll(NULL);
      if (s_stats.frames != before) { return true; }
    }
  }
  return false;
}

static bool wait_ack(uint8_t cls, uint8_t id)
{
  uint32_t t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < GNSS_ACK_TIMEOUT_MS) {
    GNSS_Poll(NULL);
    if (s_ack.result != 0u && s_ack.cls == cls && s_ack.id == id) { return s_ack.result == 1u; }
    GNSS_Wait(5);
  }
  return false;
}

static bool valset_acked(const GnssCfg_t *items, uint8_t n)
{
  s_ack.result = 0;
  if (!send_valset(items, n)) { return false; }
  return wait_ack(UBX_CLS_CFG, UBX_ID_CFG_VALSET);
}

bool GNSS_Configure(void)
{
  /* 1) retrouver le débit actuel du module */
  uint32_t found = 0;
  for (uint8_t i = 0; i < (sizeof(probe_bauds) / sizeof(probe_bauds[0])); i++) {
    if (uart_set_baud(probe_bauds[i]) && probe()) {
      found = probe_bauds[i];
      break;
    }
  }
  if (found == 0u) { return false; }   /* module absent, non alimenté ou débranché */

  /* 2) le passer au débit voulu. On ne compte pas sur l'ACK de ce message (envoyé avant ou
        après le changement de débit selon le firmware) : on bascule le STM32 puis on vérifie. */
  if (found != GNSS_BAUDRATE) {
    const GnssCfg_t baud = { CFG_UART1_BAUDRATE, GNSS_BAUDRATE };
    if (!send_valset(&baud, 1)) { return false; }
    vTaskDelay(pdMS_TO_TICKS(20));
    if (!uart_set_baud(GNSS_BAUDRATE) || !probe()) { return false; }
  }

  /* 3) flux et cadence, chacun acquitté */
  if (!valset_acked(cfg_stream, sizeof(cfg_stream) / sizeof(cfg_stream[0]))) { return false; }
  if (!valset_acked(cfg_rate,   sizeof(cfg_rate)   / sizeof(cfg_rate[0])))   { return false; }
  return true;
}
