#ifndef __speed_H_
#define __speed_H_

#include "headfile.h"

#define Limit_Min_Max(data,min,max) (((data)>(max)) ? (max) : (((data) < (min)) ? (min) : (data)))

//��������
extern int16  aim_speed;         
extern int16 real_speed;        
extern int16 left_speed;       
extern int16 right_speed;       
extern int16 All_PWM_left;     
extern int16 All_PWM_right;    
extern int16 Speed_pwm_left;      
extern int16 Speed_pwm_right;
extern int16 Speed_pwm_all;      
extern int16 Steer_pwm;
extern uint16 Open_pack_time;

//��������
void init_PWM(unsigned char Motor_Set);
void encoder_init(void);
void speed_measure(void);
void go_motor (int16 left_PWM,int16 right_PWM);
void timed_task(void);

#endif
