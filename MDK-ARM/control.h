#ifndef __CONTROL_H
#define __CONTROL_H
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

extern double pwm_ch1;//��¼ch1��pwmռ�ձ�
extern double pwm_ch2;//��¼ch2��pwmռ�ձ�
extern double pwm_ch3;//��¼ch3��pwmռ�ձ�
extern short int motor1;//��¼���1ת��ֵ
extern short int motor2;//��¼���2ת��ֵ
extern short int motor3;//��¼���3ת��ֵ
extern short int motor4;//��¼���4ת��ֵ
extern short int maximum;
extern short int diff;
extern double coe;

extern void control(double pwm_ch1,double pwm_ch2,double pwm_ch3);


#endif
