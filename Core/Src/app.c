/**
  ******************************************************************************
  * @file    app.c
  * @brief   Démarrage de l'application FreeRTOS : heap, objets, tâches, hooks.
  ******************************************************************************
  */
#include "app.h"
#include "gpio.h"   /* MX_GPIO_EXTI_Init() */

/* ===== Heap FreeRTOS (heap_5) =====
 * heap_5 ne sait rien de la RAM au départ : il faut lui donner une ou plusieurs zones
 * avec vPortDefineHeapRegions() AVANT le premier xQueueCreate/xTaskCreate. Sans cet appel,
 * toute allocation échoue. Ici : un seul tableau, placé en DTCM par le linker (.bss).
 * Tout ce que FreeRTOS alloue dynamiquement (piles des tâches, queues, mutex) vient d'ici. */
static uint8_t app_heap[configTOTAL_HEAP_SIZE] __attribute__((aligned(8)));

EventGroupHandle_t xSystemEvents = NULL;

/* Nom de la dernière tâche ayant débordé de sa pile : à lire au debugger si on tombe
   dans Error_Handler() depuis vApplicationStackOverflowHook(). */
static volatile char g_overflow_task_name[configMAX_TASK_NAME_LEN];

/* Crée une tâche ou s'arrête sur Error_Handler() si la RAM du heap est insuffisante.
   Les piles sont exprimées en MOTS de 4 octets (1024 mots = 4 Ko). */
#define APP_CREATE_TASK(fn, name, stack_words, prio) \
  do { if (xTaskCreate((fn), (name), (stack_words), NULL, (prio), NULL) != pdPASS) { Error_Handler(); } } while (0)

/* Tâche d'initialisation : elle tourne UNE fois, scheduler démarré, puis se supprime.
 * Pourquoi ne pas tout initialiser dans main() ? Parce que certains pilotes (VEML6030,
 * affichage) attendent avec un temps ("timeout") : cela ne marche qu'une fois le temps
 * système lancé. Les tâches qui dépendent du matériel attendent le bit EVT_INIT_DONE. */
static void vTaskInit(void *pvParameters)
{
  (void)pvParameters;

  Display_HwInit();   /* écran + rétroéclairage (PWM sur TIM2 CH4) */
  Led_HwInit();       /* buffer DMA des WS2812B */
  Light_HwInit();     /* VEML6030 : un échec n'est pas fatal, la tâche se supprimera */
  Can_HwInit();       /* filtre + démarrage FDCAN1 + interruption de réception */
  Timing_HwInit();    /* démarre la capture TIM5 + DMA (tour sur PA0, RPM sur PH11) */

  /* On autorise les interruptions EXTI seulement maintenant : leurs queues existent
     (créées avant le scheduler) et le matériel est prêt. */
  MX_GPIO_EXTI_Init();

  xEventGroupSetBits(xSystemEvents, EVT_INIT_DONE);
  vTaskDelete(NULL);  /* NULL = "moi-même" ; la RAM de la tâche est libérée par la tâche idle */
}

void App_Start(void)
{
#ifdef DEBUG
  /* Garde le debugger connecté quand le CPU dort (WFI dans la tâche idle) */
  HAL_DBGMCU_EnableDBGSleepMode();
  HAL_DBGMCU_EnableDBGStopMode();
  HAL_DBGMCU_EnableDBGStandbyMode();
#endif

  /* 1) Heap : à faire avant TOUT appel FreeRTOS qui alloue */
  const HeapRegion_t xHeapRegions[] = {
    { app_heap, sizeof(app_heap) },
    { NULL, 0 }                       /* terminateur obligatoire */
  };
  vPortDefineHeapRegions(xHeapRegions);

  /* 2) Objets de synchronisation (queues, mutex, event group) */
  xSystemEvents = xEventGroupCreate();
  if (xSystemEvents == NULL) { Error_Handler(); }

  Log_CreateObjects();      /* en premier : les autres modules peuvent loguer dès leur init */
  Power_CreateObjects();
  Buttons_CreateObjects();
  Can_CreateObjects();
  Light_CreateObjects();
  Timing_CreateObjects();

  /* 3) Tâches */
  APP_CREATE_TASK(vTaskInit,        "Init",        1024, PRIO_INIT);
  APP_CREATE_TASK(vTaskPowerButton, "PowerBtn",     512, PRIO_POWER);
  APP_CREATE_TASK(vTaskCAN,         "CAN",         1024, PRIO_CAN);
  APP_CREATE_TASK(vTaskTiming,      "Timing",       512, PRIO_TIMING);
  APP_CREATE_TASK(vTaskLightSensor, "LightSensor", 1024, PRIO_LIGHT);
  APP_CREATE_TASK(vTaskDisplay,     "Display",     2048, PRIO_DISPLAY);
  APP_CREATE_TASK(vTaskLED,         "LED",          512, PRIO_LED);
  APP_CREATE_TASK(vTaskLog,         "Log",         2048, PRIO_LOG);

  /* 4) Le scheduler prend la main : cette fonction ne revient normalement jamais.
        Si on arrive à la ligne suivante, il n'y avait pas assez de RAM pour la tâche idle. */
  vTaskStartScheduler();
  Error_Handler();
}

/* ===== Hooks FreeRTOS (activés dans FreeRTOSConfig.h) ===== */

/* Appelé en boucle quand AUCUNE tâche n'est prête : on endort le CPU jusqu'à la prochaine
   interruption (le tick 1 ms suffit à le réveiller). */
void vApplicationIdleHook(void)
{
  __WFI();
}

/* Un xTaskCreate/xQueueCreate/pvPortMalloc a échoué : heap (configTOTAL_HEAP_SIZE) trop petit */
void vApplicationMallocFailedHook(void)
{
  Error_Handler();
}

/* Une tâche a écrit au-delà de sa pile : agrandir la pile dans APP_CREATE_TASK ci-dessus */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  for (uint32_t i = 0; i < configMAX_TASK_NAME_LEN; i++) {
    g_overflow_task_name[i] = pcTaskName[i];
    if (pcTaskName[i] == '\0') { break; }
  }
  Error_Handler();
}
