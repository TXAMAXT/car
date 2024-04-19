/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"

/* USER CODE BEGIN 0 */
    //uint8_t TxDATA[8]={0};
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_User_Init(CAN_HandleTypeDef* hcan )   //用户初始化函数
{
  CAN_FilterTypeDef  sFilterConfig;
  HAL_StatusTypeDef  HAL_Status;
  sFilterConfig.FilterActivation = ENABLE;   //激活过滤器
  sFilterConfig.FilterBank = 1;                       //过滤器1
  sFilterConfig.FilterMode =  CAN_FILTERMODE_IDMASK;  //设为掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;    //设为32位
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;    //接收到的报文放入到FIFO0中
  sFilterConfig.FilterIdHigh = 0;   //基本ID放入到STID中
  sFilterConfig.FilterIdLow  = 0;
  sFilterConfig.FilterMaskIdHigh =0;
  sFilterConfig.FilterMaskIdLow  =0;
  sFilterConfig.SlaveStartFilterBank  = 0;
 
  HAL_Status=HAL_CAN_ConfigFilter(hcan, &sFilterConfig);
  HAL_Status=HAL_CAN_Start(hcan);  //开启CAN
 
  if(HAL_Status!=HAL_OK){
// printf("开启CAN失败\r\n");
 }
 HAL_Status=HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);
 if(HAL_Status!=HAL_OK){
 //printf("开启挂起中段允许失败\r\n");
  }
}
/*接收设置函数*/
int receiveOrder(uint32_t StdId,uint8_t IDE,uint8_t  RTR, uint8_t DLC)
{
uint8_t aData[8];    // 缓存接收到的信息
 
Can_Rx.StdId = StdId; // 接收ID（此处无用，can接收有的ID号）
Can_Rx.ExtId = 0;
Can_Rx.IDE = IDE; // 接收标准
Can_Rx.DLC = DLC;  // 接收8bit数据
Can_Rx.RTR = RTR; // 接收数据
Can_Rx.FilterMatchIndex = 0; // 使用0号过滤器
Can_Rx.Timestamp = 0;
 
 if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &Can_Rx, aData) != HAL_OK)
 {
  return Rx_Error;
 }
 else
 {
  // 取出接收到的信息
  for(uint8_t i = 0; i<Can_Rx.DLC; i++)
  {
   Rxdata[i] = aData[i];
  }
  return Rx_OK;
 }
}


/*
 发送命令函数
 StdId   标准帧ID
 ExtId   扩展帧ID  当标志位 IDE为CAN_ID_STD时 扩展帧无效
 IDE     扩展帧标志位  CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
 RTR     0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
 DLC     数据长度
*/

void sendOrder(uint32_t StdId,uint8_t IDE,uint8_t  RTR, uint8_t DLC)
{
 Can_Tx.StdId = StdId;//标准ID
 Can_Tx.ExtId = 0;//扩展ID
 Can_Tx.IDE = IDE;//CAN_ID_STD为标准ID CAN_ID_EXT为使用扩展ID
 Can_Tx.RTR = RTR;     //0(CAN_RTR_DATA)为数据帧 1(CAN_RTR_REMOTE)为远程帧
 Can_Tx.DLC = DLC;     //数据长度
 
}
void sendmessage(short int motor1,short int motor2,short int motor3,short int motor4)
{
 uint32_t pTxMailbox = 0;

  Txdata[0]=motor1>>8;
  Txdata[1]=motor1;
  Txdata[2]=motor2>>8;
  Txdata[3]=motor2;
  Txdata[4]=motor3>>8;
  Txdata[5]=motor3;
  Txdata[6]=motor4>>8;
  Txdata[7]=motor4;
  printf("TX ID:0x%003X\r\n",Can_Tx.StdId);
 printf("TX DATA:%02X%02X%02X%02X%02X%02X%02X%02X\r\n",Txdata[0],Txdata[1],Txdata[2],Txdata[3],Txdata[4],Txdata[5],Txdata[6],Txdata[7]);
 HAL_CAN_AddTxMessage(&hcan,&Can_Tx,Txdata,&pTxMailbox);
}

/* USER CODE END 1 */
