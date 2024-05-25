#include "fuse.h"

PID SpeedPID = {0};
PID L_SpeedPID ={0};
PID R_SpeedPID ={0};
PID TurnPID ={0};
PID Turn_NeiPID ={0};
PID TurnPID_wan ={0};
PID Turn_NeiPID_wan ={0};
PID TurnPID_RTwan ={0};
PID Turn_NeiPID_RTwan ={0};

/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/
void PID_int(void)
{
	L_SpeedPID.Kp=0.5;     //速度环PID参数（D车用，速度环2ms）
	L_SpeedPID.Ki=0.05;
	L_SpeedPID.Kd=0;
	
	R_SpeedPID.Kp=0.1;       //速度环PID参数（D车用，速度环2ms） 
	R_SpeedPID.Ki=0.05;
	R_SpeedPID.Kd=0;
	
	TurnPID.Kp=20.0;       //转向外环PID参数 
	TurnPID.Ki=1;
	TurnPID.Kd=20;
	TurnPID.Kc=1;
	
//	Turn_NeiPID.Kp=0.125;  //角速度内环PID参数
//	Turn_NeiPID.Ki=0;
//	Turn_NeiPID.Kd=0.5;
//	
//	TurnPID_wan.Kp=100.0;       //弯道转向外环PID参数 
//	TurnPID_wan.Ki=0;
//	TurnPID_wan.Kd=8;
//	
//	Turn_NeiPID_wan.Kp=0.13;  //弯道角速度内环PID参数
//	Turn_NeiPID_wan.Ki=0.3;
//	Turn_NeiPID_wan.Kd=0.5;
//	
//	TurnPID_RTwan.Kp=80.0;       //直角弯道转向外环PID参数 
//	TurnPID_RTwan.Ki=0;
//	TurnPID_RTwan.Kd=8;
//	
//	Turn_NeiPID_RTwan.Kp=0.4;  //直角弯道角速度内环PID参数
//	Turn_NeiPID_RTwan.Ki=0;
//	Turn_NeiPID_RTwan.Kd=0.5;
}
static TASK_COMPONENTS TaskComps[] =
{
    {0,  1,  1, Motor_output_control},         //角速度内环和D车速度环2ms
    {0,  5,  5, Trailing_control},           //转向外环5ms
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
             TaskComps[i].Timer = TaskComps[i].ItvTime; // 恢复计时器值，从新下一次
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
* * attention      : ***放在mian的while(1)即可***
**************************************************************************************/
void TaskProcess(void)
{
    uint8 i; 
	  for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
    {
        if (TaskComps[i].Run)           // 时间不为0
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
			//imu660ra_get_gyro();   //获取陀螺仪角速度值
			speed_measure();       //编码器测量
			
			//Speed_pwm_all = LocP_DCalc(&SpeedPID,aim_speed,real_speed); //D车速度环（位置式）

			aim_speed = 800;       //目标速度
			//Speed_pwm_all += IncPIDCalc(&SpeedPID,aim_speed,real_speed);//D车速度环（增量式）
	
			//Steer_pwm=LocP_DCalc(&TurnPID,imu660ra_gyro_z,ADC_PWM);   //转向内环PWM	
			//Steer_pwm=range_protect(Steer_pwm,-5000,5000);   							//转向内环PWM限幅

			All_PWM_left = aim_speed + ADC_PWM;                         //左电机所有PWM输出 
			All_PWM_right= aim_speed - 0.6 * ADC_PWM;                   //右电机所有PWM输出
	
		
//			All_PWM_left = IncPIDCalc(&L_SpeedPID,All_PWM_left,left_speed*2);//D车速度环（增量式）
//			All_PWM_right = IncPIDCalc(&R_SpeedPID,All_PWM_right,right_speed*2);//D车速度环（增量式）
	
	
	
			go_motor(All_PWM_left,All_PWM_right);											 //动力输出
		
		
		  ips114_showint16(122,6,All_PWM_left);
			ips114_showint16(122,7,All_PWM_right);
			ips114_showint16(122,8,Steer_pwm);
//		printf("left_pwm=%d \n",All_PWM_left);
//		printf("right_pwm=%d \n",All_PWM_right);
			     
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
	//  Annulus_assist(); //环岛辅助函数

		ADC_PWM = LocP_DCalc(&TurnPID,0,Current_Dir);//位置式PD控制转向

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
