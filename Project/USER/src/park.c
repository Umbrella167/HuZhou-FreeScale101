#include "park.h"

int8   flag_start           = 0;    //出库完成标志位
int8   flag_open_reed       = 0;    //打开干簧管
int8   reed_state           = 0;    //干簧管状态
int8   flag_end             = 0;    //开始停车标志位
uint16 T_outku              = 0;    //出库直走和打角定时

/*****************************************出库函数***************************************
函数：  void Handle_Barn_Out(uint8 type) 
参数：  type-----1为左出库，2为右出库
说明：  出库函数

*注意：调用此函数后执行出库操作，直走的时间和打角时间及占空比按需自己修改调试
返回值：无   
******************************************************************************************/
void Handle_Barn_Out(uint8 type)
{//1为左出库，2为右出库
    if(type ==1)
    {
		 if(!flag_start)
		 {
			 //pwm_duty(PWMB_CH1_P74,STEER_MID);              //（C车用）
			 go_motor(2000, 2000);
			 if (T_outku >= T_OUT_PARK1) 
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM);	//（C车用）
				 go_motor(0, 2500);
			 }
			 if(T_outku > T_OUT_PARK2)
			 {
				  flag_start = 1;
				  //T_outku=0;
			 }		
		  }	
	  }
		if(type ==2)
    {
			if(!flag_start)
			{
				//pwm_duty(PWMB_CH1_P74,STEER_MID);            //（C车用）
			 go_motor(2000, 2000);
			 if (T_outku >= T_OUT_PARK1) 
			 {
				 //pwm_duty(PWMB_CH1_P74,STEER_MID+STEER_LIM); //（C车用）
				 go_motor(2500, 0);
			 }
			 if(T_outku > T_OUT_PARK2)
			 {
				  flag_start = 1;
				  //T_outku=0;
			 }															
			}
    }
}
/*****************************************干簧管检测停车***************************************
函数：  void Reed(void) 
参数：  void
说明：  干簧管检测停车

*注意： 干簧管使用方法就和按键类似，通过读取IO口电平即可
返回值：void  
*********************************************************************************************/
void Reed(void)
{
	if(flag_start)//开始时不开启干簧管检测，防止出库时误测
	{
		//走过一段距离后开启干簧管检测
		if(Open_pack_time > START_T)
		{
			flag_open_reed = 1;
			Open_pack_time=0;
		}
	}
	if(flag_open_reed)             //干簧管检测标志位成立后才开始检测
	{
		reed_state = Reed_Switch_Pin;//干簧管状态
		if(reed_state==0)
		{
			flag_end = 1;              //识别到停车标志位开启
		}
	 }
}
/*****************************************入库函数***************************************
函数：  void Reed(void) 
参数：  void
说明：  入库函数

*注意： 目前只是检测到斑马线进行停车操作，还没有写倒车入库，后续马上调试更新
返回值：void  
*********************************************************************************************/
void In_park(uint8 type)
{
	if(type ==1)
  {
		if(flag_end)
		{
			 aim_speed = 0;
			 go_motor(0,0);
		}
 }
	if(type ==2)
  {
		if(flag_end==1)
		{
			 aim_speed = 0;
			 go_motor(0,0);
		}
 }
}