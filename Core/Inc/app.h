/**
  ******************************************************************************
  * @file    app.h
  * @brief   Application du dashboard : objets FreeRTOS partagés et interface
  *          des modules (power, boutons, CAN, lumière, chronométrage, affichage, LEDs).
  *
  * Organisation du firmware :
  *
  *   main.c        -> horloges + init des périphériques HAL (CubeMX), puis App_Start()
  *   app.c         -> heap FreeRTOS, création des objets/tâches, tâche d'init, hooks
  *   app_xxx.c     -> un module par fonction (une tâche + ses ISR + son matériel)
  *   app_irq.c     -> point d'entrée unique des interruptions GPIO (EXTI)
  *
  * Convention de nommage des fonctions d'un module "Xxx" :
  *   Xxx_CreateObjects() : crée queues/mutex.  Appelé AVANT le scheduler (App_Start).
  *   Xxx_HwInit()        : initialise le composant externe. Appelé par la tâche d'init,
  *                         scheduler démarré (donc on peut utiliser HAL_Delay, I2C...).
  *   vTaskXxx()          : la tâche FreeRTOS du module.
  *   Xxx_...Irq()        : appelée DEPUIS une interruption : ne doit appeler que des
  *                         fonctions FreeRTOS en "...FromISR", ne jamais bloquer.
  ******************************************************************************
  */
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

/* ===== Priorités des tâches (0 = idle, configMAX_PRIORITIES-1 = 4 = la plus haute) =====
 * Règle : plus une tâche doit réagir vite et travaille peu, plus sa priorité est haute.
 * L'affichage travaille longtemps -> priorité basse pour ne pas bloquer le reste. */
#define PRIO_INIT      4   /* tâche d'init : tourne une fois puis se supprime */
#define PRIO_POWER     3   /* bouton power : doit réagir tout de suite       */
#define PRIO_CAN       3   /* réception CAN : ne pas perdre de trames        */
#define PRIO_TIMING    3   /* lit les horodatages TIM5 (court, toutes les 10 ms) */
#define PRIO_LIGHT     2
#define PRIO_DISPLAY   2
#define PRIO_LED       1   /* cosmétique                                     */

/* ===== Groupe d'événements système ===== */
#define EVT_INIT_DONE  (1u << 0)   /* posé par la tâche d'init quand le matériel est prêt */
extern EventGroupHandle_t xSystemEvents;

/* ===== Queues partagées entre modules ===== */
extern QueueHandle_t xLapTimeQueue;      /* uint32_t : durée d'un tour en µs      (timing -> display) */
extern QueueHandle_t xLightSensorQueue;  /* uint16_t : dernière valeur de lux     (light -> display) */
extern QueueHandle_t xButtonQueue;       /* uint16_t : GPIO_PIN_x du bouton pressé (ISR -> display)  */
extern QueueHandle_t xCanRxQueue;        /* CAN_Frame_t : trames reçues           (ISR -> tâche CAN) */

typedef struct {
  uint32_t Identifier;
  uint8_t  DataLength;  /* nombre d'octets (0-8 en Classic CAN) */
  uint8_t  Data[8];
} CAN_Frame_t;

/* ===== Point d'entrée de l'application (ne retourne jamais) ===== */
void App_Start(void);

/* ===== Module power : bouton marche/arrêt + maintien d'alimentation ===== */
void Power_CreateObjects(void);
void vTaskPowerButton(void *pvParameters);
void Power_ButtonIrq(BaseType_t *pxHigherPriorityTaskWoken);

/* ===== Module boutons de navigation (SW1, SW3, SW4) ===== */
void Buttons_CreateObjects(void);
void Buttons_Irq(uint16_t GPIO_Pin, BaseType_t *pxHigherPriorityTaskWoken);
const char *Buttons_Name(uint16_t GPIO_Pin);

/* ===== Module CAN (FDCAN1) ===== */
void Can_CreateObjects(void);
void Can_HwInit(void);
void vTaskCAN(void *pvParameters);
HAL_StatusTypeDef CAN_SendMessage(uint32_t Identifier, uint8_t *pData, uint8_t DataLength);

/* ===== Module capteur de lumière (VEML6030 sur I2C1) ===== */
void Light_CreateObjects(void);
uint8_t Light_HwInit(void);              /* 1 = capteur OK, 0 = absent/en erreur */
void vTaskLightSensor(void *pvParameters);

/* ===== Module chronométrage (TIM5 : tour sur PA0 = CH1, RPM sur PH11 = CH2) ===== */
void Timing_CreateObjects(void);
void Timing_HwInit(void);
void vTaskTiming(void *pvParameters);
uint32_t Rpm_GetPulsesPerMinute(void);   /* impulsions/min ; 0 si moteur arrêté */

/* ===== Module affichage ===== */
void Display_HwInit(void);
void vTaskDisplay(void *pvParameters);

/* ===== Module LEDs WS2812B ===== */
void Led_HwInit(void);
void vTaskLED(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
