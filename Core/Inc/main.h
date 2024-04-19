/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern int returnRece;
extern uint32_t cap_buf_ch1[3];//记录3次捕获的时刻
extern uint32_t cap_buf_ch2[3];
extern uint32_t cap_buf_ch3[3];
extern uint8_t cap_flag;//捕获状态
extern uint8_t ch1;//通道捕获标志
extern uint8_t ch2;
extern uint8_t ch3;
extern double cap_htime_ch1;//记录高电平时间
extern double cap_alltime_ch1;//记录一个周期的时间
extern double cap_htime_ch2;
extern double cap_alltime_ch2;
extern double cap_htime_ch3;
extern double cap_alltime_ch3;
extern uint32_t tim_over;//记录溢出次数
extern double pwm_ch1;//记录ch1的pwm占空比
extern double pwm_ch2;//记录ch2的pwm占空比
extern double pwm_ch3;//记录ch3的pwm占空比
extern short int motor1;//记录电机1转速值
extern short int motor2;//记录电机2转速值
extern short int motor3;//记录电机3转速值
extern short int motor4;//记录电机4转速值
extern short int maximum;
extern short int rc_ch1;//控制值
extern short int rc_ch2;
extern short int rc_ch3;
extern short int diff;
extern double coe;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
