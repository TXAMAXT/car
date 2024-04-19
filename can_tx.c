#include "can_tx.h"



void set_can_tx(CAN_TxHeaderTypeDef CAN1_TxHander,short int motor1,short int motor2,short int motor3,short int motor4){
    CAN1_TxHander.IDE=CAN_ID_STD;
    CAN1_TxHander.RTR=CAN_RTR_DATA;
    CAN1_TxHander.DLC=0x08;
    CAN1_TxHander.StdId=0x200;
    TxDATA[0]=motor1>>8;
    TxDATA[1]=motor1;
    TxDATA[2]=motor2>>8;
    TxDATA[3]=motor2;
    TxDATA[4]=motor3>>8;
    TxDATA[5]=motor3;
    TxDATA[6]=motor4>>8;
    TxDATA[7]=motor4;
    
    HAL_CAN_AddTxMessage(&hcan,&CAN1_TxHander,TxDATA,0);
};
