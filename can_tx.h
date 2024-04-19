#ifndef __CAN_TX_H
#define __CAN_TX_H
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

extern uint8_t TxDATA[8];
extern void set_can_tx(CAN_TxHeaderTypeDef CAN1_TxHander,short int motor1,short int motor2,short int motor3,short int motor4);
#endif
