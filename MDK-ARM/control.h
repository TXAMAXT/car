#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

extern double pwm_ch1;//记录ch1的pwm占空比
extern double pwm_ch2;//记录ch2的pwm占空比
extern double pwm_ch3;//记录ch3的pwm占空比
extern short int motor1;//记录电机1转速值
extern short int motor2;//记录电机2转速值
extern short int motor3;//记录电机3转速值
extern short int motor4;//记录电机4转速值
extern short int maximum;
extern short int diff;
extern double coe;

extern void control(double pwm_ch1,double pwm_ch2,double pwm_ch3);


#endif
