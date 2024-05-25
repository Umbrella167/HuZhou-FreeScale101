#ifndef __HEADFILE_H_
#define __HEADFILE_H_

#include "isr.h"
#include <string.h>
#include <stdio.h>
#include "intrins.h"
//------STC32G SDK��
#include "STC32Gxx.h"
#include "board.h"
#include "common.h"

//------��ɿƼ���Ƭ����������ͷ�ļ�
#include "zf_uart.h"
#include "zf_gpio.h"
#include "zf_iic.h"
#include "zf_adc.h"
#include "zf_spi.h"
#include "zf_tim.h"
#include "zf_pwm.h"
#include "zf_nvic.h"
#include "zf_exti.h"
#include "zf_delay.h"
#include "zf_eeprom.h"

//------��ɿƼ���Ʒ����ͷ�ļ�
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_18TFT.h"

#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_TSL1401.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "SEEKFREE_MPU6050.h"
#include "SEEKFREE_OLED.h"
#include "SEEKFREE_ABSOLUTE_ENCODER.h"
#include "SEEKFREE_WIRELESS.h"
#include "SEEKFREE_PRINTF.h"
#include "SEEKFREE_AT24C02.h"
#include "SEEKFREE_BLUETOOTH_CH9141.h"
#include "SEEKFREE_WIRELESS_CH573.h"
#include "SEEKFREE_CONFIG.h"
#include "SEEKFREE_IMU660RA.h"
#include "SEEKFREE_IMU963RA.h"
#include "SEEKFREE_DL1A.h"

//�û��Զ���ͷ�ļ�
#include "fuse.h"
#include "ADC.h"
#include "PID.h"
#include  "speed.h"
#include "debug.h"
#include "park.h"
/*****************ʹ�õ�Ӳ�����Ŷ���*********
Ϊ����ǿ����ļ����ԣ��󲿷ֳ�������ʹ�ú궨�壬���Ӳ���в���޸�����궨�弴�ɣ�
ע���޸�ʱ����go to������Ӧģ�����ճ�����ƣ���Ҫ�Լ���Ŷ��
*********************************************/
//����������� 
//MOTOR_MODE  0�Ļ�����·ģʽMOS������1�Ļ���һ·ģʽDRV����
#define MOTOR_MODE_SELECT  1
//=====һ�������Ҫ��·PWMģʽ=====MOS/btn��������=====
#define Left_Z_Pin     PWMA_CH1P_P60   //PWMA_CH1P_P60 PWMB_CH2_P75
#define Left_F_Pin     PWMA_CH2P_P62  //PWMA_CH2P_P62  PWMB_CH1_P74
#define Right_Z_Pin    PWMA_CH3P_P64  //PWMA_CH3P_P64  PWMB_CH3_P76
#define Right_F_Pin    PWMA_CH4P_P66  //PWMA_CH4P_P66  PWMB_CH4_P77
//=====һ�����ֻҪһ·PWMģʽ������һ�����ſ��Ʒ���=====DRV����==
#define Left_PWM_Pin   PWMA_CH4P_P66    //PWMA_CH4P_P66
#define Left_DIR_Pin   P64              //P62  P75
#define Right_PWM_Pin  PWMA_CH2P_P62    //PWMA_CH2P_P62
#define Right_DIR_Pin  P60            //P66 P77
//�������   
#define Steer_Pin   PWMB_CH1_P74    //
//����������  ռ���˶�ʱ��TIM0��TIM3��TIM4  
#define Left_Ecoder_Pin1     CTIM0_P34  //CTIM3_P04   CTIM0_P34    //LSB����
#define Left_Ecoder_Pin2     P35        //P05         P30           //Dir��������
#define Right_Ecoder_Pin1    CTIM3_P04   //CTIM4_P06  CTIM3_P04     //LSB����
#define Right_Ecoder_Pin2    P53          //Dir                     ��������P07
//��������     ռ�ö�ʱ��TIM4
#define UART_User    UART_4          //
#define UART_TX_Pin  UART4_TX_P03   //UART4_TX_P03 
#define UART_RX_Pin  UART4_RX_P02  //UART4_RX_P02 
//����������
//#define BUZZPin     P34
//ADC����ź�����5����У�ʵ���϶����ԣ���У����������޸ļ��ɣ�
#define Left_ADC_Pin      ADC_P00  
#define LeftXie_ADC_Pin   ADC_P01
#define RightXie_ADC_Pin  ADC_P05 
#define Right_ADC_Pin     ADC_P06
//#define Mid_ADC_Pin       ADC_P05 
//�ɻɹ����� (Ĭ��ʹ����ͨIO��ȡ���ⲿ�жϺ���ͨ��Ч����࣬ͳһʹ����ͨIO��ȡ)
#define Reed_Switch_Pin P2_6  //
#endif