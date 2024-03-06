#ifndef _park_h
#define _park_h

#include "headfile.h"

#define START_T          5000
#define T_OUT_PARK1      1000
#define T_OUT_PARK2      1400

//��������
extern int8   flag_start;          //�������
extern int8   flag_open_reed;      //���θɻɹ�
extern int8   reed_state;          //�ɻɹ�״̬
extern int8   flag_end;            //��ʼͣ����־λ
extern uint16 T_outku;

//��������
extern void Handle_Barn_Out(uint8 type);
extern void Reed(void);
extern void In_park(uint8 type);

#endif