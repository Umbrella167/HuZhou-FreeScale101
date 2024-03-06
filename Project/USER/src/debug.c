#include "debug.h"
/*************************使用说明****************************************
本协议与“Visual Scope”软件协议兼容，用过的可以直接用原来的下位机协议即可
首次使用时：
1.将“outputdata.c”和“outputdata.h”添加到你的工程中
2.在“outputdata.c”中包含你原程序的串口发送函数头文件
3.将uart_putchar(databuf[i]);语句替换为你的串口字节发送函数，如send_char(databuf[i]);
4.在你的程序需要发送波形数据的.c文件中添加包含：#include "outputdata.h"，并在本文件中调用函数OutPut_Data(x,y,z,w);
  其中形参x，y，z，w就是传入四个short int 16位数据，分别对应通道1,2,3,4
************************************************************************/
//此处添加你的串口头文件包含！！！！！！！！！！！！
#include "zf_uart.h"
//****************************移植**************************//

void Data_Send(UARTN_enum uratn,signed short int *pst)
{
		unsigned char _cnt=0; unsigned char sum = 0;
    unsigned char data_to_send[23] = {0};         //发送缓存
    unsigned char i;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0x02;
    data_to_send[_cnt++]=0;
    data_to_send[_cnt++]=(unsigned char)(pst[0]>>8);  //高8位
    data_to_send[_cnt++]=(unsigned char)pst[0];  //低8位
    data_to_send[_cnt++]=(unsigned char)(pst[1]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[1];
    data_to_send[_cnt++]=(unsigned char)(pst[2]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[2];
    data_to_send[_cnt++]=(unsigned char)(pst[3]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[3];
    data_to_send[_cnt++]=(unsigned char)(pst[4]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[4];
    data_to_send[_cnt++]=(unsigned char)(pst[5]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[5];
    data_to_send[_cnt++]=(unsigned char)(pst[6]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[6];
    data_to_send[_cnt++]=(unsigned char)(pst[7]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[7];
    data_to_send[_cnt++]=(unsigned char)(pst[8]>>8);
    data_to_send[_cnt++]=(unsigned char)pst[8];


    data_to_send[3] = _cnt-4;

    sum = 0;
    for(i=0;i<_cnt;i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

        for(i=0;i<_cnt;i++)
    uart_putchar(uratn,data_to_send[i]);
}

//===================================================上位机相关的==========================================================
//=============================================上位机使用匿名科创地面站======================================================
/******************************************数据传输****************************************************
函数：void datasend()
参数：  无
说明： 可以同时传输6个数  icm_gyro_x   icm_acc_x ICM_Real.gyro.y  ICM_Real.acc.z   
常看的变量：icm_acc_x  icm_gyro_y  Angle  adc_date[0] Left_Adc

返回值：无
日期：
作者： 
*****************************************************************************************************/
void datasend()
{  
   short send_data[6];                      

   send_data[0]= left_speed; ////////ICM_Start.acc.x 
   send_data[1]= 400; //////////////////    MpuStart.gyro.x   Angle  
   send_data[2]= right_speed; //////////
   send_data[3]= 400; //
   send_data[4]= 0;
   send_data[5]= 0;
	 //Data_Send(UART_4,send_data);
   Data_Send(UART_User,send_data);
}

//====================================================屏幕相关的=(已删除)=============================================================
//============================================================================================================================
//sprintf(temp," date20=%d",date);
//TFTSPI_P8X8Str(0,19,temp,BLACK,WHITE);break;

//==========================================================拨码开关及按键相关=========================================================
//====================================================================================================================================

//拨码开关引脚宏定义
#define Switch_Pin_1       P75
#define Switch_Pin_2       P76
#define Switch_Pin_3       P33
#define Switch_Pin_4       P33
#define Switch_Pin_5       P33
#define Switch_Pin_6       P33
//定义按键引脚
#define KEY1    P70      
#define KEY2    P71      
#define KEY3    P72        
#define KEY4    P73     
#define KEY5    P33 

//***************函数宏定义****(下面这些函数请修改宏定义为对应的GPIO库函数操作)***********
#define KEY_INT(key_x)           gpio_mode(key_x,GPO_PP)//配置为GPO_PP:推挽输出 
#define SWITCH_INT(switch_x)     gpio_mode(switch_x,GPO_PP)//配置为GPO_PP:推挽输出
#define READ_GPIO(Pin_X)         Pin_X
#define TiaoCan_DelayMs(M_S)     delay_ms(M_S)   //延时函数

unsigned char TiaoCan=0;////////////////////////调参标志位
unsigned char TFT_SHOW=0;///////////////////////屏幕开关
unsigned char Switch1=0,Switch2=0,Switch3=0,Switch4=0,Switch5=0,Switch6=0;//拨码开关
char parameter=0;//参数选择

//开关状态变量
uint8 key1_status = 1,key2_status = 1,key3_status = 1, key4_status = 1,key5_status = 1;
//上一次开关状态变量
uint8 key1_last_status, key2_last_status, key3_last_status, key4_last_status,key5_last_status;
//开关标志位
uint8 key1_flag=0,key2_flag=0,key3_flag=0, key4_flag=0,key5_flag=0;
/*****************拨码开关及按键初始化*****************
函数：void Switch_Key_init()
功能：初始化IO
参数：  无
说明： 初始化IO口   gpio_init(D1, GPI, GPIO_HIGH, GPI_PULL_UP); GPO_PUSH_PULL
返回值：无*/
void Switch_Key_init()
{

  //拨码开关初始化  （无需修改，请勿修改）
	SWITCH_INT(Switch_Pin_1) ;
	SWITCH_INT(Switch_Pin_2) ;
	//SWITCH_INT(Switch_Pin_3) ;
	//SWITCH_INT(Switch_Pin_4) ;
	//SWITCH_INT(Switch_Pin_5) ;
	//SWITCH_INT(Switch_Pin_6) ;
    
  //按键初始化 （无需修改，请勿修改）
  KEY_INT(KEY1);
	KEY_INT(KEY2);
	KEY_INT(KEY3);
	KEY_INT(KEY4);
	//KEY_INT(KEY5);
}

/*****************拨码开关策略选择*****************
函数：void Strategy_Slect()
功能：通过拨码开关调整策略
参数：  无
说明：  6位拨码开关，如果有增加或者减少可对照修改,如果不足6个也不要删除多余的，多余的你随便引脚改个没用的即可
        使用你定义的就好了，其他没有用到的无需关心
返回值：无*/
void Strategy_Slect()
{
  //读取拨码开关状态
      if(!READ_GPIO(Switch_Pin_1))//用
      {
       Switch1=1;
       //TFT_SHOW = 1;
			 //显示电感原始值
			 /*ips114_showuint8(0,0,adc_value[0]);
	     ips114_showuint8(0,1,adc_value[1]);
	     ips114_showuint8(0,2,adc_value[2]);
	     ips114_showuint8(0,3,adc_value[3]);*/
      }
      if(!READ_GPIO (Switch_Pin_2))//用
      {
       Switch2=1;
			 //显示电感归一化值
			 /*ips114_showuint8(0,0,Left_Adc);
	     ips114_showuint8(0,1,Left_Shu_Adc);
	     ips114_showuint8(0,2,Right_Shu_Adc);
	     ips114_showuint8(0,3,Right_Adc);*/
       //LEFT_RIGHT=0;//左出库左入库
      }
      if(!READ_GPIO (Switch_Pin_3))
      {
       Switch3=1;
      }
      if(!READ_GPIO (Switch_Pin_4))
      {
       Switch4=1;

      }
      if(!READ_GPIO (Switch_Pin_5))
      {
       Switch5=1;

      }
      if(!READ_GPIO (Switch_Pin_6))
      {
       Switch6=1;
 
      }

    if(Switch1||Switch2||Switch3||Switch4||Switch5||Switch6)//开启拨码开关
      {

      }
}

/*****************按键扫描读取*****************
函数：void  Key_Scan_Deal ()
功能：读取按键并执行对应操作
参数：  无
说明： 参考逐飞例程 ，5位按键，如果有增加或者减少可对照修改
      // 1号为左移键，2号为上键，3号为右移键，4号为中键盘，5号为下键
     //本次程序没有使用调参，stc单片机下载程序也快，改了烧就可以，如果要加的话自己根据下面的自己加就可以
返回值：无     */
uint8 gogo=0;
void  Key_Scan_Deal ()
{
    //使用此方法优点在于，不需要使用while(1) 等待，避免处理器资源浪费
    //保存按键状态
    key1_last_status = key1_status;
    key2_last_status = key2_status;
    key3_last_status = key3_status;
    key4_last_status = key4_status;
    key5_last_status = key5_status;
    //读取当前按键状态
    key1_status = READ_GPIO(KEY1);
    key2_status = READ_GPIO(KEY2);
    key3_status = READ_GPIO(KEY3);
    key4_status = READ_GPIO(KEY4);
    key5_status = READ_GPIO(KEY5);
    //检测到按键按下之后  并放开置位标志位
    if(key1_status && !key1_last_status)    key1_flag = 1;
    if(key2_status && !key2_last_status)    key2_flag = 1;
    if(key3_status && !key3_last_status)    key3_flag = 1;
    if(key4_status && !key4_last_status)    key4_flag = 1;
    if(key5_status && !key5_last_status)    key5_flag = 1;
    //标志位置位之后，可以使用标志位执行自己想要做的事件
 
           if(key1_flag&&(gogo==1||gogo==2)) //S1键
           {
               key1_flag = 0;//使用按键之后，应该清除标志位
               /*以下为用户任务  */
               switch(parameter)
               {
                 //-----------------------调参请修改下面--（注意修改对应的显示）----------------------------------------------------------------
                 //第一页显示的东西
                 case 0:  break;
                 case 1:  break;
                 case 2:  break;
                 case 3:  break;
                 case 4:  break;
                 case 5:  break;

                 /// case 6:  ; break;//这个不能加任何操作在这里了，翻页使用啦

                 //第二页显示的东西
                 case 7:  break;
                 case 8:  break;
                 case 9:  break;
                 case 10: break;
                 case 11: break;
                 case 12: break;
                //--------------------调参请修改上面------------------------------------------------------------------
               }
                 /*  以上为用户任务  */
          }
          if(key2_flag&&(gogo==1||gogo==2))//S2键
          {
             key2_flag = 0;//使用按键之后，应该清除标志位
             /*  以下为用户任务  */
             switch(parameter)
             {
                //----------------------调参请修改下面--（注意修改对应的显示）--------------------------------------------------------------
                //第一页显示的东西
                case 0: break;
                case 1: break;
                case 2: break;
                case 3: break;
                case 4: break;
                case 5: break;

                /// case 6:  ; break;//这个不能加任何操作在这里了，翻页使用啦

                //第二页显示的东西
                case 7: break;
                case 8: break;
                case 9: break;
                case 10: break;
                case 11: break;
                case 12: break;
                //--------------------调参请修改上面------------------------------------------------------------------
               }
               /*  以上为用户任务  */  
           }
           if(key3_flag&&(gogo==1||gogo==2))//S3键
           {
               key3_flag = 0;//使用按键之后，应该清除标志位
               /*  以下为用户任务  */
                      parameter--;
               /*  以上为用户任务  */

           }

           if(key4_flag&&(gogo==1||gogo==2))//S4键
           {
               key4_flag = 0;//使用按键之后，应该清除标志位
               /*  以下为用户任务  */
                      parameter++;
               /*  以上为用户任务  */

           }
            if(key5_flag) //S5键
           {
              key5_flag = 0;//使用按键之后，应该清除标志位
            /*  以下为用户任务  */
						

						/*  以上为用户任务  */

           }
    if(gogo==6)//告辞啦，调参结束！！！！！！！！！！！！
		{

			 TiaoCan = 1;      //调参结束标志位

		}
    //*******************************屏幕显示第一页***********************

    if(parameter<6&&gogo>=1)//显示参数0到5，实际显示1到6
    {
          
    }
    //*******************************屏幕显示第二页**************************************************
    if(parameter>6&&parameter<13)//这里行号从4到9   一页调6个参数  //显示参数7到5，实际显示7到12
    {
          
    }
    //*******************************屏幕显示第三页**************************************************
    if(parameter>13&&parameter<20)
    {
                  
    }
    //###########还需要更多页仿照着写就可以咯######################这里就不写了 结束
    if(parameter==6||parameter==13||parameter==20)  //翻页准备
     {
                     
     }//清屏
}

/**********************************************蜂鸣器滴滴滴******************************************
函数：void BUZZ_DiDiDi()
功能：蜂鸣器滴滴滴
参数：  无
说明：
返回值：无
***************************************************************************************************/
void BUZZ_DiDiDi(uint16 PinLV)
{
  BUZZ_ON;
  TiaoCan_DelayMs(PinLV);
  BUZZ_OFF;
}
/****************************测试完毕*********************************************
 *  函数名称：Test_Motor_Hardware(void)
 *  功能说明：测试标定输出PWM控制电机
 *  参数说明：无
 *  函数返回：无
 *  修改时间：
 *  备    注：驱动2个电机
 【注意事项】注意，一定要对电机输出进行限制
 1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
 2.接好母板到驱动板的信号线及电源线；
 3.接好驱动板到电机的导线；
 4.烧写程序并运行，确定电机能正常转动后，开启驱动板电源开关；
 5.按键K0/K1确定电机转动速度及方向；
 6.如果出现疯转，按下K1键返回低速模式，或者直接关闭驱动板电源！
 ******************************************************************************/
void Test_Motor_Hardware (void)
{
    int16 duty = 2000;
    ips114_clear(YELLOW);  //初始清屏
	  ips114_showstr(2, 0, "Test_Motor_Hardware:");
    init_PWM(1);
	
    while (1)
    {
        if (!READ_GPIO(KEY1))                       //按下KEY1键   左轮单独正转
        {
             go_motor (duty,0);
					   ips114_showstr(0,4,"Left  Front");     //字符串显示
        } 
        if (!READ_GPIO(KEY2))                       //按下KEY2键  右轮单独正转
        {
           	 go_motor (0,duty);
						 ips114_showstr(0,4,"Right Front");   //字符串显示
        }
				if (!READ_GPIO(KEY3))                       //按下KEY3键，左右轮同时反转
        {
             go_motor (-duty,-duty);
					   ips114_showstr(0,4,"All  Black");      //字符串显示
           	  	 
        }
				if((READ_GPIO(KEY1))&&(READ_GPIO(KEY2))&&(READ_GPIO(KEY3)))
        go_motor (0,0);
      	TiaoCan_DelayMs(100);  
    }
}

/****************************测试完毕*********************************************
 *  函数名称：void Test_Electric_Hardware (void)
 *  功能说明：测试电磁电感硬件
 *  参数说明：无
 *  函数返回：无
 *  修改时间：2021-5-30
 *  备    注：
 *************************************************************************/
void Test_Electric_Hardware (void)
{
	char txt[16];
	ips114_clear(YELLOW);  //初始清屏
	ips114_showstr(2, 0, "Test_Electric_Hardware:");
	ADC_int();
	while(1)
	{
			  //datasend();
		    if (!READ_GPIO(KEY1)) //按下KEY1键
        {
					ips114_showstr(2, 1, "Normalize_Deal....");   //字符串显示
					ADC_Collect();  //电感采值
					
					sprintf(txt,"adc0= %02f",adc_value[0]);
					ips114_showstr(1, 2, txt); //显示
					sprintf(txt,"adc1= %02f",adc_value[1]);
					ips114_showstr(1, 3, txt); //显示
					sprintf(txt,"adc2= %02f",adc_value[2]);
					ips114_showstr(1, 4, txt); //显示
					sprintf(txt,"adc3= %02f",adc_value[3]);
					ips114_showstr(1, 5, txt); //显示				  	  	 
        }
				if(!READ_GPIO(KEY2)) //按下KEY2键
				{
					ips114_showstr(2, 1, "GYH_Normalize_Deal....");   //字符串显示
					ADC_Collect();  //电感采值
	        Data_current_analyze();  //电感值归一化函数
					Current_Dir=Cha_bi_he(Left_Adc,Right_Adc,100);
					
					sprintf(txt,"adc0= %05d",Left_Adc);
					ips114_showstr(1, 2, txt); //显示
					sprintf(txt,"adc1= %05d",Left_Shu_Adc);
					ips114_showstr(1, 3, txt); //显示
					sprintf(txt,"adc2= %05d",Right_Shu_Adc);
					ips114_showstr(1, 4, txt); //显示
					sprintf(txt,"adc3= %05d",Right_Adc);
					ips114_showstr(1, 5, txt); //显示
					sprintf(txt,"Current_Dir= %05d",Current_Dir);
					ips114_showstr(1, 6, txt); //显示
        }
  }	
}

/*
			         	encoder_init();						
	   while(1)
		 {
			 		 speed_measure();
		delay_ms(50);
			 	sprintf(txt,"Left_Speed  = %05d",left_speed);
		ips114_showstr(1, 3, txt); //显示
			sprintf(txt,"Right_Speed = %05d",right_speed);
		ips114_showstr(1, 4, txt); //显示
		  sprintf(txt,"Speed     = %05d",real_speed);
		ips114_showstr(1, 5, txt); //显示
		 }
*/



