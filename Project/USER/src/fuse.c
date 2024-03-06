#include "fuse.h"
PID SpeedPID = {0};
PID TurnPID ={0};
PID Turn_NeiPID ={0};
/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/
void PID_int(void)
{
	SpeedPID.Kp=3.8;     //速度环PID参数
	SpeedPID.Ki=0.8;
	SpeedPID.Kd=0;
	
	TurnPID.Kp=80;       //转向外环PID参数 
	TurnPID.Ki=0;
	TurnPID.Kd=5;
	
	Turn_NeiPID.Kp=2.8;  //角速度内环PID参数
	Turn_NeiPID.Ki=0;
	Turn_NeiPID.Kd=0.5;
}
static TASK_COMPONENTS TaskComps[] =
{
    {0,  2,  2, Motor_output_control},         //角速度内环和速度环2ms
    {0, 10, 10, Trailing_control},           //转向外环10ms
};
/**************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理
* EntryParameter : None
* ReturnValue    : None
* attention      : ***在定时器中断中调用此函数即可***
**************************************************************************************/
void TaskRemarks(void)
{
    uint8 i;
    for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
    {
        if (TaskComps[i].Timer)          // 时间不为0
        {
           TaskComps[i].Timer--;         // 减去一个节拍
           if (TaskComps[i].Timer == 0)       // 时间减完了
           {
             TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，重新下一次
             TaskComps[i].Run = 1;           // 任务可以运行
           }
        }
   }
}

/**************************************************************************************
* FunctionName   : TaskProcess()
* Description    : 任务处理|判断什么时候该执行那一个任务
* EntryParameter : None
* ReturnValue    : None
* * attention      : ***放在main的while(1)***
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i; 
	  for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
    {
        if (TaskComps[i].Run)           
        {
            TaskComps[i].TaskHook();       // 运行任务
            TaskComps[i].Run = 0;          // 标志清0
        }
    }
}
/****************************角速度内环**************************************
函数：  void Motor_output_control()
参数：  void
说明：  角速度内环和D车速度环
返回值：void
***************************************************************************************/
void Motor_output_control()
{
    imu660ra_get_gyro();   //获取陀螺仪角速度值
		speed_measure();       //编码器测量
	  aim_speed = 400;       //目标速度
	  //Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed,real_speed);     //速度环（位置式）
	  Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);      //速度环（增量式）
	  Steer_pwm=LocP_DCalc(&Turn_NeiPID,imu660ra_gyro_z,ADC_PWM);      //转向内环PWM	
    Steer_pwm=range_protect(Steer_pwm,-5000,2000);                  //转向内环PWM限幅
		All_PWM_left=Speed_pwm_all+Steer_pwm ;                          //左电机所有PWM输出 
		All_PWM_right=Speed_pwm_all -Steer_pwm ;                        //右电机所有PWM输出
    go_motor(All_PWM_left,All_PWM_right);                             //动力输出
}
/****************************转向环（转向外环）**************************************
函数：  void Trailing_control()
参数：  void
说明：  转向环（转向外环）
返回值：void
***************************************************************************************/
void Trailing_control()
{
	  Get_deviation();  //电磁采集并获取赛道偏差
	  Annulus_assist(); //环岛辅助函数
		ADC_PWM = LocP_DCalc(&TurnPID,Current_Dir,0);//位置式PD控制转向
}

/***************************************************************************************
函数名：int16 range_protect(int16 duty, int16 min, int16 max)
功  能：限幅保护 
参  数：
返回值：duty
**************************************************************************************/
int16 range_protect(int16 duty, int16 min, int16 max)//限幅保护
{
  if (duty >= max)
  {
    return max;
  }
  if (duty <= min)
  {
    return min;
  }
  else
  {
    return duty;
  }
}
