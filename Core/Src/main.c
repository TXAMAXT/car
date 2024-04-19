/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "control.h"

 #define RXBUFFERSIZE  256
char RxBuffer[RXBUFFERSIZE]; 
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t cap_buf_ch1[3]={0};//��¼3�β����ʱ��
uint32_t cap_buf_ch2[3]={0};
uint32_t cap_buf_ch3[3]={0};
uint8_t cap_flag=0;//����״̬
uint8_t ch1=0;//ͨ�������־
uint8_t ch2=0;
uint8_t ch3=0;
double cap_htime_ch1=0;//��¼�ߵ�ƽʱ��
double cap_alltime_ch1=0;//��¼һ�����ڵ�ʱ��
double cap_htime_ch2=0;
double cap_alltime_ch2=0;
double cap_htime_ch3=0;
double cap_alltime_ch3=0;
uint32_t tim_over=0;//��¼�������
double pwm_ch1=0;//��¼ch1��pwmռ�ձ�
double pwm_ch2=0;//��¼ch2��pwmռ�ձ�
double pwm_ch3=0;//��¼ch3��pwmռ�ձ�
short int motor1=0;//��¼���1ת��ֵ
short int motor2=0;//��¼���2ת��ֵ
short int motor3=0;//��¼���3ת��ֵ
short int motor4=0;//��¼���4ת��ֵ
short int maximum=20000;
short int diff=0;
double coe=0;
CAN_RxHeaderTypeDef CAN1_RxHander;//����can���ܽṹ��
CAN_TxHeaderTypeDef CAN1_TxHander;//����can���ͽṹ��

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define IR_IN1 HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_12)
#define IR_IN2 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)
#define IR_IN3 HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)
double time_dowmch1_num=0;//�½��ؼ���
double time_dowmch2_num=0;
double time_dowmch3_num=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef Can_Tx;
CAN_RxHeaderTypeDef Can_Rx;
int returnRece=0;
uint8_t Rxdata[8];//CAN���ջ�����
uint8_t Txdata[8] = {0};//CAN���ͻ�����
extern uint8_t can_rx_finish_flag;//������ɱ�־λ
// �����жϻص�������ÿ�β����źžͻ��������ص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
 // �ж��Ƿ��Ƕ�ʱ��4���ⲿ�����2
      
 if(htim->Instance == TIM4)
 {
  if(IR_IN1)//��һ������
  {
   __HAL_TIM_SET_COUNTER(&htim4, 0); // �������㣬��ͷ��ʼ��
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
  }
  else if(IR_IN1==0)//�½�
  {
   time_dowmch1_num = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); // ��ȡ������������ʱ�伴Ϊ�����س�����ʱ��
   HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);//��������ʧ�ܶ�ʱ��ĳһͨ�������벶���ܺ���Ӧ���ж�
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim4, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // �ı䲶����Ϊ�������ز���
  }
 }
 
 if(htim->Instance == TIM5)
 {
  if(IR_IN2)//��һ������
  {
   __HAL_TIM_SET_COUNTER(&htim5, 0); // �������㣬��ͷ��ʼ��
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
  }
  else if(IR_IN2==0)//�½�
  {
   time_dowmch2_num = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); // ��ȡ������������ʱ�伴Ϊ�����س�����ʱ��
   HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_1);//��������ʧ�ܶ�ʱ��ĳһͨ�������벶���ܺ���Ӧ���ж�
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // �ı䲶����Ϊ�������ز���
  }
 }
 
  if(htim->Instance == TIM3)
 {
  if(IR_IN3)//��һ������
  {
   __HAL_TIM_SET_COUNTER(&htim3, 0); // �������㣬��ͷ��ʼ��
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING); // �ı䲶����Ϊ�½��ز���
  }
  else if(IR_IN3==0)//�½�
  {
   time_dowmch3_num = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1); // ��ȡ������������ʱ�伴Ϊ�����س�����ʱ��
   HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);//��������ʧ�ܶ�ʱ��ĳһͨ�������벶���ܺ���Ӧ���ж�
   __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING); // �ı䲶����Ϊ�������ز���
  }
 }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

   CAN_User_Init(&hcan);
sendOrder(0x200,CAN_ID_STD,CAN_RTR_DATA,0x08);
   HAL_TIM_IC_Stop_IT(&htim4,TIM_CHANNEL_1);
   HAL_TIM_IC_Stop_IT(&htim5,TIM_CHANNEL_1);
   HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);

/*���ղ���*/
receiveOrder(0x201,CAN_ID_STD,0,8);
returnRece=receiveOrder(0x201,CAN_ID_STD,0,8);
         HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
          HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch1=(time_dowmch1_num-1246)/800;
      HAL_Delay(200);
      printf("ch1:%lf\n",pwm_ch1);
          HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch2=(time_dowmch2_num-1098)/800;
      HAL_Delay(200);
      printf("ch2:%lf\n",pwm_ch2);
         HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch3=(time_dowmch3_num-1098)/800;
      HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
          HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch1=(time_dowmch1_num-1246)/800;
          HAL_Delay(10);
          HAL_TIM_IC_Start_IT(&htim5,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch2=(time_dowmch2_num-1098)/800;
          HAL_Delay(10);
      printf("ch2:%lf\n",pwm_ch2);
          HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);//��������ʹ�ܶ�ʱ��ĳһͨ�������벶����,��ʹ����Ӧ���ж�
          pwm_ch3=(time_dowmch3_num-1098)/800;
          HAL_Delay(10);

          control(pwm_ch1,pwm_ch2,pwm_ch3);
      sendmessage(motor1,motor2,motor3,motor4);
  };
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM4){
      tim_over++;//ÿ�ν����ж����ֵ+1
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
