#ifndef __fuse_H__
#define __fuse_H__
#include "headfile.h"
//�ṹ�嶨��
typedef struct
{
      uint8 Run;                 
      uint8 Timer;                
      uint8 ItvTime;              
      void (*TaskHook)(void);   
}TASK_COMPONENTS;
// �����嵥
typedef enum _TASK_LIST
{
    TAST_Motor_output_control,            
    TAST_Trailing_control,                         
    TASKS_MAX         
}TASK_LIST;
//��������
int16 range_protect(int16 duty, int16 min, int16 max);
void TaskRemarks(void);
void TaskProcess(void);
void Motor_output_control(void);
void Trailing_control(void);
void Speed_control(void);
void PID_int(void);
#endif
