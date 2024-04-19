/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "usart.h"


/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#define Tx_Error 3
#define Rx_Error 2
#define Rx_OK 0
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

extern CAN_TxHeaderTypeDef Can_Tx;
extern CAN_RxHeaderTypeDef Can_Rx;
extern uint8_t Txdata[8];
extern uint8_t Rxdata[8];
void CAN_User_Init(CAN_HandleTypeDef* hcan );
void sendmessage(short int motor1,short int motor2,short int motor3,short int motor4);
void sendOrder(uint32_t StdId,uint8_t IDE,uint8_t  RTR, uint8_t DLC);
int receiveOrder(uint32_t StdId,uint8_t IDE,uint8_t  RTR, uint8_t DLC);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

