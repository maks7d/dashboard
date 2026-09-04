#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* === Architecture Configuration === */
#define configCPU_CLOCK_HZ                  ( SystemCoreClock )
#define configTICK_RATE_HZ                  ( 1000 )
#define configMAX_PRIORITIES                ( 5 )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 256 )

/* === Priority Configuration (CRITIQUE POUR CORTEX-M7) === */
#define configPRIO_BITS                      ( 4 )   /* STM32H7 a 4 bits de priorité */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  ( 0x0F )
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY ( 5 )
#define configKERNEL_INTERRUPT_PRIORITY        ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* === Memory Management === */
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 32 * 1024 ) )
#define configAPPLICATION_ALLOCATED_HEAP    ( 1 )
#define configSUPPORT_STATIC_ALLOCATION      ( 0 )  // Désactivé pour simplifier
#define configSUPPORT_DYNAMIC_ALLOCATION     ( 1 )

/* === Scheduler Configuration === */
#define configUSE_PREEMPTION                ( 1 )
#define configUSE_TIME_SLICING              ( 1 )
#define configUSE_IDLE_HOOK                 ( 0 )
#define configUSE_TICK_HOOK                 ( 0 )
#define configGENERATE_RUN_TIME_STATS        ( 0 )
#define configUSE_TRACE_FACILITY            ( 0 )  // Désactivé pour simplifier
#define configIDLE_SHOULD_YIELD              ( 1 )
#define configUSE_MUTEXES                   ( 1 )
#define configCHECK_FOR_STACK_OVERFLOW      ( 0 )  // Désactivé pour simplifier

/* === FreeRTOS API Includes === */
#define INCLUDE_vTaskDelay                   ( 1 )
#define INCLUDE_xTaskGetSchedulerState      ( 1 )
#define INCLUDE_xQueueGetMutexHolder         ( 1 )

/* === Task Configuration === */
#define configMAX_TASK_NAME_LEN             ( 16 )
#define configQUEUE_REGISTRY_SIZE           ( 8 )

/* === Timer Configuration === */
#define configUSE_TIMERS                    ( 0 )  // Désactivé pour simplifier
#define configTIMER_TASK_PRIORITY           ( 3 )
#define configTIMER_QUEUE_LENGTH            ( 10 )
#define configTIMER_TASK_STACK_DEPTH        ( configMINIMAL_STACK_SIZE * 2 )

/* === Event Groups === */
#define configUSE_COUNTING_SEMAPHORES       ( 1 )
#define configMAX_COUNTING_SEMAPHORES       ( 10 )

/* === Cortex-M7 Specific === */
#define configENABLE_FPU                    ( 1 )
#define configENABLE_MPU                    ( 0 )
#define configUSE_CO_ROUTINES               ( 0 )

/* Tick configuration - REQUIRED */
#define configUSE_16_BIT_TICKS               ( 0 )

/* === Includes === */
/* Note: stm32h7xx_hal.h is NOT included here to avoid dependency issues.
   SystemCoreClock is declared as extern in the STM32 code. */
extern uint32_t SystemCoreClock;

#endif /* FREERTOS_CONFIG_H */