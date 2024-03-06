#ifndef __ADC_H__
#define __ADC_H__
#include "headfile.h"
//�����������ֵ
#define IN_ANNULUS_H_LIMIT        160
//�����������ֵ  
#define OUT_ANNULUS_S_LIMIT       160
//�������־���
#define DISTANCE_ANNULUS_S        400    
//������ǻ���
#define DISTANCE_ANNULUS_Z        250
//������ʱ����
#define DISTANCE_ANNULUS_T        500    
//�����ж�
#define OUTSIDE                   10
//����ģ���趨����(����ʵ������޸�)
#define SET_DLLA_DISTANCE         400
//������ֵ
#define Steer_Duty_Max            990
#define Steer_Duty_Midle          850 //�����ֵ
#define Steer_Duty_Min            710
//���������ж�
struct ROAD_TYPE
{
     int8 straight;               //ֱ��
	   int8 bend;                   //���
     int8 annulus;                //����
     int8 in_annulus_left;        //���󻷵�
	   int8 in_annulus_right;       //���һ���
     int8 on_annulus_left;        //���󻷵�
	   int8 on_annulus_right;       //���һ���
     int8 out_annulus;            //������
	   int8 in_park;                //���
};
extern struct ROAD_TYPE road_type;
//��������
extern uint8 adc_value[4];  
extern int16 AD_V[4]; 
extern uint8 adc_max[4];  
extern uint8 adc_min[4];  
extern uint8 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc;
extern int8 NM;          
extern uint16 annulus_s;         
extern uint16 annulus_t;
extern uint16 annulus_z;     
extern float Current_Dir;         
extern uint8 flag_obstacle;
extern uint16 obstacle_time;
extern int16 ADC_PWM;            
extern uint8 temp;
//��������
void ADC_int(void);                   
void ADC_Collect(void);               
void Data_current_analyze(void);      
float Cha_bi_he(int16 data1, int16 data2,int16 x); 
void Road_type_judge(void);    
void Annulus_handle(void);      
int16 Direction_error(void);    
void init_Steer_PWM(void);      
void Steering_Control_Out(int16 duty);  
void Out_protect(void);                  
void Get_deviation(void);
void Annulus_assist(void);
void obstacle_avoidance(void);
#endif