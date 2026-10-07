/**
  ******************************************************************************
  * @file    app_can.c
  * @brief   FDCAN1 en mode Classic CAN : réception par interruption + queue, envoi.
  *
  * Chaîne de réception :
  *   trame sur le bus -> FDCAN1 -> ISR HAL_FDCAN_RxFifo0Callback -> xCanRxQueue -> vTaskCAN
  * L'ISR fait le strict minimum (copier la trame dans la queue) ; le décodage se fait
  * dans la tâche, où l'on peut prendre son temps.
  ******************************************************************************
  */
#include "app.h"
#include "fdcan.h"

QueueHandle_t xCanRxQueue = NULL;

void Can_CreateObjects(void)
{
  xCanRxQueue = xQueueCreate(16, sizeof(CAN_Frame_t));
  if (xCanRxQueue == NULL) { Error_Handler(); }
}

void Can_HwInit(void)
{
  /* Filtre standard : accepte tous les ID (mask = 0) vers la RX FIFO0 */
  FDCAN_FilterTypeDef sCanFilter = {0};
  sCanFilter.IdType       = FDCAN_STANDARD_ID;
  sCanFilter.FilterIndex  = 0;
  sCanFilter.FilterType   = FDCAN_FILTER_MASK;
  sCanFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sCanFilter.FilterID1    = 0x000;
  sCanFilter.FilterID2    = 0x000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sCanFilter) != HAL_OK) { Error_Handler(); }

  /* Trames non filtrées / étendues / remote : rejetées */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) { Error_Handler(); }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) { Error_Handler(); }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) { Error_Handler(); }
}

void vTaskCAN(void *pvParameters)
{
  (void)pvParameters;
  CAN_Frame_t frame;

  for (;;) {
    if (xQueueReceive(xCanRxQueue, &frame, portMAX_DELAY) == pdPASS) {
      Log_CanRx(frame.Identifier, frame.DataLength, frame.Data);   /* flux brut */
      /* TODO : décoder la trame (frame.Identifier, frame.Data, frame.DataLength) */
    }
  }
}

HAL_StatusTypeDef CAN_SendMessage(uint32_t Identifier, uint8_t *pData, uint8_t DataLength)
{
  FDCAN_TxHeaderTypeDef TxHeader = {0};
  TxHeader.Identifier          = Identifier;
  TxHeader.IdType              = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
  TxHeader.DataLength          = ((uint32_t)DataLength) << 16; /* DLC encodé dans les bits [19:16] */
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker       = 0;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, pData);
}

/* Callback HAL appelé DEPUIS l'interruption FDCAN1 (priorité 5, voir fdcan.c) */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0) {
    FDCAN_RxHeaderTypeDef RxHeader;
    CAN_Frame_t frame;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, frame.Data) == HAL_OK) {
      frame.Identifier = RxHeader.Identifier;
      frame.DataLength = (uint8_t)(RxHeader.DataLength >> 16); /* DLC (0-8) dans les bits [19:16] */
      xQueueSendFromISR(xCanRxQueue, &frame, &xHigherPriorityTaskWoken);
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
