/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "ltdc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ws2812.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "display.h"
#include "veml6030.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
//Queue for VEML6030 light sensor data
static QueueHandle_t xLightSensorQueue = NULL;

//Semaphore for protecting access to the VEML6030 light sensor (in case many tasks want to access it)
static SemaphoreHandle_t xVEML6030Mutex = NULL;

// Task Handles (for )
static TaskHandle_t xTaskLightSensorHandle = NULL;
static TaskHandle_t xTaskDisplayHandle = NULL;
static TaskHandle_t xTaskPowerButtonHandle = NULL;

static volatile uint32_t uLapTimerTicks = 0;
static EventGroupHandle_t xPowerButtonEventGroup = NULL;
#define POWER_BUTTON_SHORT_PRESS (0x01) //button shortly pressed
#define POWER_BUTTON_LONG_PRESS  (0x02) //button long pressed (>5s)

/* USER CODE BEGIN PV */
static volatile uint32_t power_btn_press_tick = 0;
static volatile uint8_t  power_btn_held = 0;
static QueueHandle_t xLapTimeQueue = NULL; // Queue for lap timer ticks
static volatile uint32_t uLastCaptureTime = 0; // Last capture time for lap timer (in FreeRTOS ticks, needs conversion according to FreeRTOSconfig.h hz rate)
static QueueHandle_t xPowerButtonQueue = NULL;  // Queue pour les événements du bouton
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
// ===== FreeRTOS Tasks =====
void vTaskLightSensor(void *pvParameters);    // Lecture du VEML6030
void vTaskDisplay(void *pvParameters);         // Gestion de l'affichage
void vTaskPowerButton(void *pvParameters);    // Gestion du bouton power

// ===== Callbacks HAL modifiés =====
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
// ===== existantes =====
int32_t VEML6030_I2C_Init(void);
int32_t VEML6030_I2C_DeInit(void);
int32_t VEML6030_I2C_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t VEML6030_I2C_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length);
int32_t VEML6030_I2C_IsReady(uint16_t Addr, uint32_t Trials);
int32_t VEML6030_GetTick(void);
void vTaskLED(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// fonctions de bus i2c pour le VEML6030
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim16;
int32_t VEML6030_I2C_Init(void){
  if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
    MX_I2C1_Init(); // réinitialiser le bus I2C si nécessaire
  }
  return VEML6030_OK;
}

int32_t VEML6030_I2C_DeInit(void){
  HAL_I2C_DeInit(&hi2c1);
  return VEML6030_OK;
}
int32_t VEML6030_I2C_WriteReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length){
  //Addr est l'adresse I2C du périphérique, Reg est le registre à écrire, pData est le pointeur vers les données à écrire, Length est la longueur des données
  if (HAL_I2C_Mem_Write(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}
int32_t VEML6030_I2C_ReadReg(uint16_t Addr, uint16_t Reg, uint8_t *pData, uint16_t Length){
  if (HAL_I2C_Mem_Read(&hi2c1, Addr, Reg, I2C_MEMADD_SIZE_8BIT, pData, Length, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}

int32_t VEML6030_I2C_IsReady(uint16_t Addr, uint32_t Trials){
  if (HAL_I2C_IsDeviceReady(&hi2c1, Addr, Trials, 100) != HAL_OK) {
    return VEML6030_ERROR;
  }
  return VEML6030_OK;
}

int32_t VEML6030_GetTick(void){
  return xTaskGetTickCount();
}

VEML6030_Object_t veml6030;
VEML6030_IO_t veml6030_io = {
  .Init = VEML6030_I2C_Init,
  .DeInit = VEML6030_I2C_DeInit,
  .ReadAddress = VEML6030_I2C_READ_ADD, // 0x21
  .WriteAddress = VEML6030_I2C_WRITE_ADD, // 0x20
  .IsReady = VEML6030_I2C_IsReady,
  .WriteReg = VEML6030_I2C_WriteReg,
  .ReadReg = VEML6030_I2C_ReadReg,
  .GetTick = VEML6030_GetTick
};

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  GPIOE->MODER = (GPIOE->MODER & ~GPIO_MODER_MODE3_Msk) | GPIO_MODER_MODE3_0;
  GPIOE->BSRR = GPIO_PIN_3;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LTDC_Init();
  MX_SDMMC1_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(POWER_HOLD_PORT, POWER_HOLD_PIN, GPIO_PIN_SET);
  MX_GPIO_EXTI_Init();
  xPowerButtonQueue = xQueueCreate(5, sizeof(uint32_t));
  if (xPowerButtonQueue == NULL) { Error_Handler(); }
  // ===== Initialisation FreeRTOS Objects =====
  // Créer une queue pour les temps de tour (10 éléments max)
  xLapTimeQueue = xQueueCreate(10, sizeof(uint32_t));
  if (xLapTimeQueue == NULL) { Error_Handler(); }
  // Créer une queue pour les données du capteur (taille 1, élément = uint16_t pour le lux)
  xLightSensorQueue = xQueueCreate(1, sizeof(uint16_t));
  if (xLightSensorQueue == NULL) { Error_Handler(); }

  // Créer un mutex pour le VEML6030 (si accès partagé)
  xVEML6030Mutex = xSemaphoreCreateMutex();
  if (xVEML6030Mutex == NULL) { Error_Handler(); }

  // Créer un groupe d'événements pour le bouton power
  xPowerButtonEventGroup = xEventGroupCreate();
  if (xPowerButtonEventGroup == NULL) { Error_Handler(); }

  // ===== Initialisation VEML6030 (dans une tâche ou ici) =====
  VEML6030_RegisterBusIO(&veml6030, &veml6030_io);
  if (VEML6030_Init(&veml6030) != VEML6030_OK) {
    Error_Handler();
  }
  VEML6030_SetExposureTime(&veml6030, VEML6030_CONF_IT100); // 100ms
  VEML6030_SetGain(&veml6030, VEML6030_ALS_CHANNEL, VEML6030_CONF_GAIN_1); // Gain 1x
  VEML6030_Start(&veml6030, VEML6030_MODE_CONTINUOUS);

  // ===== Initialisation Display =====
  Display_Init();

  // ===== Démarrer le timer TIM2 (mode IT) =====
  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  // ===== Création des tâches FreeRTOS =====
  // Tâche Capteur de Lumière (priorité élevée, car données critiques)
  xTaskCreate(vTaskLightSensor, "LightSensor", configMINIMAL_STACK_SIZE * 4, NULL, 3, &xTaskLightSensorHandle);

  // Tâche Affichage (priorité max pour fluidez)
  xTaskCreate(vTaskDisplay, "Display", configMINIMAL_STACK_SIZE * 8, NULL, 4, &xTaskDisplayHandle);

  // Tâche Bouton Power (priorité moyenne)
  xTaskCreate(vTaskPowerButton, "PowerBtn", configMINIMAL_STACK_SIZE * 2, NULL, 2, &xTaskPowerButtonHandle);

  // Tâche LED (priorité basse, car cosmétique)
  xTaskCreate(vTaskLED, "TaskLED", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  /* USER CODE END 2 */

  vTaskStartScheduler(); // Start FreeRTOS scheduler
  
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Should never be reached because the scheduler is running
  while (1)
  {
  }
  /* USER CODE END WHILE */
}

void vTaskLED(void *pvParameters) {
  uint8_t color = 0;
  while (1) {
    // Cycle de couleurs RGB
    switch (color % 3) {
      case 0: WS2812B_SetAll(255, 0, 0); break; // Rouge
      case 1: WS2812B_SetAll(0, 255, 0); break; // Vert
      case 2: WS2812B_SetAll(0, 0, 255); break; // Bleu
    }
    color++;
    vTaskDelay(pdMS_TO_TICKS(1000));
    WS2812B_Clear();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}


void vApplicationIdleHook(void) {
  // This function is called when the system is idle
  __WFI(); // Enter low-power mode until the next interrupt
  // You can put low-priority tasks here, like power-saving operations
}

void vApplicationTickHook(void) {
  // Appelé à chaque tick du scheduler (ex: toutes les 1ms)
  // Utile pour des actions périodiques légères (ex: clignotement LED)
}

void vApplicationMallocFailedHook(void) {
  Error_Handler(); // FreeRTOS n'a pas pu allouer de mémoire
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName) {
  (void)pcTaskName;
  (void)pxTask;
  Error_Handler(); // Une tâche a débordé de sa stack
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (GPIO_Pin == POWER_BTN_PIN) {
    if (HAL_GPIO_ReadPin(POWER_BTN_PORT, POWER_BTN_PIN) == GPIO_PIN_RESET) {
      power_btn_press_tick = xTaskGetTickCount();
      power_btn_held = 1;
    } else {
      if (power_btn_held) {
        uint32_t press_duration = xTaskGetTickCount() - power_btn_press_tick;
        uint32_t event = (press_duration < 500) ? POWER_BUTTON_SHORT_PRESS : 0;
        power_btn_held = 0;
        if (event != 0) {
          xQueueSendFromISR(xPowerButtonQueue, &event, &xHigherPriorityTaskWoken);
        }
      }
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t ulCurrentCaptureTime = xTaskGetTickCount();
  uint32_t ulLapTimeMs = 0;

  if (htim == &htim2) {
    if (uLastCaptureTime != 0) {
      ulLapTimeMs = (ulCurrentCaptureTime - uLastCaptureTime) * portTICK_PERIOD_MS;
      xQueueSendFromISR(xLapTimeQueue, &ulLapTimeMs, &xHigherPriorityTaskWoken);
    }
    uLastCaptureTime = ulCurrentCaptureTime;
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_SYSTICK_Callback(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (power_btn_held && (xTaskGetTickCount() - power_btn_press_tick >= pdMS_TO_TICKS(5000))) {
    uint32_t event = POWER_BUTTON_LONG_PRESS;
    power_btn_held = 0;
    xQueueSendFromISR(xPowerButtonQueue, &event, &xHigherPriorityTaskWoken);
    HAL_GPIO_WritePin(POWER_HOLD_PORT, POWER_HOLD_PIN, GPIO_PIN_RESET);
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

void vTaskLightSensor(void *pvParameters) {
  uint16_t lux = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(200); // Lecture toutes les 200ms

  while (1) {
    // Prendre le mutex avant d'accéder au VEML6030
    if (xSemaphoreTake(xVEML6030Mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      // Lire la valeur du capteur (mode non-bloquant si possible)
      uint32_t als_value = 0;
      if (VEML6030_GetValues(&veml6030, &als_value) == VEML6030_OK) {
        // Convertir les valeurs en lux
        lux = (uint16_t)als_value;
        // Envoyer la valeur à la queue (si espace disponible)
        xQueueOverwrite(xLightSensorQueue, &lux);
          // Queue pleine, ignorer ou gérer l'erreur
        
      }
      // Libérer le mutex
      xSemaphoreGive(xVEML6030Mutex);
    }

    // Attendre la prochaine lecture
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

void vTaskDisplay(void *pvParameters) {
  uint16_t lux = 0;
  uint32_t ulLapTimeMs = 0;
  char buffer[64];

  while (1) {
    // Display the lux value if available
    if (xQueuePeek(xLightSensorQueue, &lux, 0) == pdPASS) {
      sprintf(buffer, "Lux: %u", lux);
      Display_ShowText(10, 10, buffer, WHITE, BLACK);
    }

    // Display the lap time if available
    if (xQueueReceive(xLapTimeQueue, &ulLapTimeMs, 0) == pdPASS) {
      sprintf(buffer, "Lap Time: %lu ms", ulLapTimeMs);
      Display_ShowText(10, 30, buffer, WHITE, BLACK);
    }

    vTaskDelay(pdMS_TO_TICKS(100));  // Refresh at 10 Hz
  }
}

void vTaskPowerButton(void *pvParameters) {
  uint32_t event;
  while (1) {
    if (xQueueReceive(xPowerButtonQueue, &event, portMAX_DELAY) == pdPASS) {
      if (event == POWER_BUTTON_SHORT_PRESS) {
        // Action appui court
      } else if (event == POWER_BUTTON_LONG_PRESS) {
        HAL_GPIO_WritePin(POWER_HOLD_PORT, POWER_HOLD_PIN, GPIO_PIN_RESET);
      }
    }
  }
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
