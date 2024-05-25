#include "ADC.h"
#include "math.h"
#define tickLenth 5
uint8 adc_value[4];                 //储存电感采集值原始值    4个电感
uint8 adc_Tick[4][tickLenth] = {0};
int16 AD_V[4];                      //储存电感采集值归一化值中间变量 （无需关心，请勿删除）
uint8 adc_max[4]={232,235,235,234}; //电感采值最大值 需要自己采集
uint8 adc_min[4]={4,1,0,4};        //电感采值最小值(1,2,3,4)左到右
uint8 Left_Adc,Right_Adc,Left_Shu_Adc,Right_Shu_Adc,Left_value,Right_value;//电感值
int8 NM=4;                          //电感个数

//环道参数
uint16 annulus_s     = 0;           //环岛积分距离
uint16 annulus_z     = 0;           //环岛第积分打角
uint16 annulus_t = 0;

struct ROAD_TYPE road_type = {0};
int16 obstacle_Current_Dir[]={
	                            30,31,32,33,34,35,36,37,38,39,
	                            40,41,42,43,44,45,46,47,48,49,
	                            -69,-68,-67,-66,-65,-64,-63,-62,-61,-60,
	                            -59,-58,-57,-56,-55,-54,-53,-52,-51,-50,
	                            -49,-48,-47,-46,-45,-44,-43,-42,-41,-40,
                              -39,-38,-37,-36,-35,-34,-33,-32,-31,-30,
                             };
/***当前位置*************/
float Current_Dir = 0;

int16 ADC_PWM=0;
uint8 flag_obstacle=0;
uint16 obstacle_time=0;
uint8 temp=0;				 
/***************************电感采集通道初始化****************************
函数：  void ADC_init(void)  
功能：  电感采值进行初始化
参数：  void
说明：  电感采集初始化
返回值；无
************************************************************************/
void ADC_int(void)
{
	adc_init(Left_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.0为ADC功能
	adc_init(LeftXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.1为ADC功能
	adc_init(RightXie_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.5为ADC功能
	adc_init(Right_ADC_Pin,ADC_SYSclk_DIV_2);//初始化P0.6为ADC功能 
}

/***************************电感采值************************************
函数：  void ADC_Collect()   
功能：  电感采值
参数：  void
说明：  8位ADC输出，0~255（2的8次方），5v电压平均分成255份，分辨率为5/255=0.196
返回值；void
***********************************************************************/
void ADC_Collect()
{
	uint8 i;
	
	adc_Tick[0][tickLenth-1] = adc_once(Left_ADC_Pin,ADC_8BIT);
	adc_Tick[1][tickLenth-1] = adc_once(LeftXie_ADC_Pin,ADC_8BIT);
	adc_Tick[2][tickLenth-1] = adc_once(RightXie_ADC_Pin,ADC_8BIT);
	adc_Tick[3][tickLenth-1] = adc_once(Right_ADC_Pin,ADC_8BIT);

	for(i = 0;i < tickLenth;i++)
	{
		adc_value[0] += adc_Tick[0][i];
		adc_value[1] += adc_Tick[1][i];
		adc_value[2] += adc_Tick[2][i];
		adc_value[3] += adc_Tick[3][i];
		// 更新电感数据 后入前出
		if(i < tickLenth - 1)
		{
			adc_Tick[0][i] = adc_Tick[0][i + 1];
			adc_Tick[1][i] = adc_Tick[1][i + 1];
			adc_Tick[2][i] = adc_Tick[2][i + 1];
			adc_Tick[3][i] = adc_Tick[3][i + 1];
		}
	}
	// 均值滤波
	adc_value[0]=adc_value[0] / tickLenth; //左横电感
	adc_value[1]=adc_value[1] / tickLenth; //左竖电感
	adc_value[2]=adc_value[2] / tickLenth; //右竖电感
	adc_value[3]=adc_value[3] / tickLenth; //右横电感
	
//	adc_value[0] = adc_once(Left_ADC_Pin,ADC_8BIT);
//	adc_value[1] = adc_once(LeftXie_ADC_Pin,ADC_8BIT);
//	adc_value[2] = adc_once(RightXie_ADC_Pin,ADC_8BIT);
//	adc_value[3] = adc_once(Right_ADC_Pin,ADC_8BIT);

}

/*********************************电感采值********************************
函数：  void Data_current_analyze()   
功能：  电感采值原始值归一化（0~100）
参数：  void
说明：  归一化处理
返回值；void       
*************************************************************************/
void Data_current_analyze()
{
	uint8 i;
  for(i=0;i < NM; i++)              
  {
   AD_V[i] = ((adc_value[i]-adc_min[i])*100)/adc_max[i];         
   if( AD_V[i]<=0)
   {
      AD_V[i]=0;
   }
   else if( AD_V[i]>=100)
   {
      AD_V[i]=100;
   }
  }
  Left_Adc = AD_V[0];       //左电感最终值
  Left_Shu_Adc = AD_V[1];   //左竖电感最终值
  Right_Shu_Adc = AD_V[2];  //右竖电感最终值
  Right_Adc = AD_V[3];	    //右电感最终值
	
	Left_value=func_limit_ab(sqrt(Left_Adc*Left_Adc+Left_Shu_Adc*Left_Shu_Adc),0,100);
	Right_value=func_limit_ab(sqrt(Right_Shu_Adc*Right_Shu_Adc+Right_Adc*Right_Adc),0,100);
}

/*********************************差比和函数**********************************
函数：  float Cha_bi_he(int16 data1, int16 data2,int16 x)
功能：  差比和求赛道偏差
参数：  int16 data1, int16 data2,int16 x
说明：  差比和求赛道偏差
返回值；result         
****************************************************************************/
float Cha_bi_he(int16 data1, int16 data2,int16 x)
{
    int16 cha;
    int16 he;
    float result;

    cha = data1-data2;
    he = data1+data2+1;
    result = (cha*x)/(1.0*he);

    return result;
}
/*****************************************出界保护函数*************************************
函数：  void Out_protect() 
参数：  无
说明：  防止车冲出赛道后撞坏东西,检测出赛道后中断失能，电机停转，放回赛道中断使能继续跑

*注意：！！！平时调试时可以打开，加了避障处理后需要关闭此函数，不然有可能无法实现避障功能！！！
返回值：无  
******************************************************************************************/
void Out_protect(void)
{
	if(Left_Adc<OUTSIDE&&Right_Adc<OUTSIDE)
	{
		DisableGlobalIRQ();//关闭总中断
		go_motor(0,0);
	}
	else
	{
		EnableGlobalIRQ();
	}
}
/*****************************************判断赛道类型*************************************
函数：  void Road_type_judge(void)
参数：  无
说明：  赛道类型判断--环岛--弯道--直道--
返回值：无  
******************************************************************************************/
void Road_type_judge(void)
{	 
	  //环岛判断
//	  if((Left_Adc+Right_Adc)>IN_ANNULUS_H_LIMIT)
//	  {
//			road_type.annulus        = 1;
//			road_type.straight       = 0;
//			road_type.bend           = 0;  
//		}
//		//十字路口判断	
//		  if(Right_Shu_Adc >114&&Left_Shu_Adc>80)//改范围
//			{     
//				road_type.annulus        = 0;
//				road_type.straight       = 0;
//				road_type.bend           = 0;
//				road_type.straightbend   = 0;
//				road_type.crossroad   	 = 1;
//			}
//		//直角弯判断	
//		  if((Left_Adc < 0 Left_Shu_Adc > 14 && Right_Adc < 0 Right_Shu_Adc > 80)//改范围
//			{     
//				road_type.annulus        = 0;
//				road_type.straight       = 0;
//				road_type.bend           = 0;
//				road_type.straightbend   = 1;
//				road_type.crossroad   	 = 0;
//			}
//		//弯道判断（不准确暂时不用）
	  if((Left_Adc > 50 && Right_Adc < 30 && Left_Shu_Adc>22)||(Left_Adc <30 && Right_Adc > 50 && Right_Shu_Adc>20))
		{     
			road_type.annulus        = 0;
			road_type.straight       = 0;
			road_type.bend           = 1;
			road_type.straightbend   = 0;
			road_type.crossroad   	 = 0;
		}
//		//直道判断：以上类型均不满足则为直道
		else
		{   
			road_type.annulus        = 0;
		  road_type.straight       = 1;
		  road_type.bend           = 0;	
			road_type.straightbend   = 0;
			road_type.crossroad   	 = 0;
		}
}
/*****************************************环岛处理***************************************
函数：  void Annulus_handle(void)
参数：  无
说明：  环岛处理函数

*注意：用两个竖电感引导进环
返回值：无  
******************************************************************************************/
void Annulus_handle(void)
{
	  //左环判断
		if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_left==0&&(Left_Shu_Adc>30))	 
		{		  
			road_type.in_annulus_left = 1;
			BUZZ_ON;
			P52                      = 0;
		}
		//右环判断
	  else if(annulus_s > DISTANCE_ANNULUS_S&&road_type.annulus==1&&road_type.in_annulus_right==0&&(Right_Shu_Adc>30))
		{
			road_type.in_annulus_right = 1;
			BUZZ_ON;
			P52                      = 0;
		}
		//左环处理
		if(road_type.in_annulus_left == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_left==1&&road_type.on_annulus_left==0)
		  {
			  road_type.on_annulus_left = 1;
				BUZZ_ON;
			  P52                      = 1;
		  }
		  if(road_type.on_annulus_left==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)
		  {			
				road_type.out_annulus = 1;
        BUZZ_ON;				
		  }
		}
		//右环处理
	  else if(road_type.in_annulus_right == 1)
		{
			if(annulus_z > DISTANCE_ANNULUS_Z&&road_type.in_annulus_right==1&&road_type.on_annulus_right==0)
		  {
			  road_type.on_annulus_right = 1;
			  P52                      = 1;
				BUZZ_ON;
		  }
		  if(road_type.on_annulus_right==1&&road_type.out_annulus==0&& Left_Adc+Right_Adc>OUT_ANNULUS_S_LIMIT)  
		  {			
				road_type.out_annulus = 1;		
				BUZZ_ON;
		  }
		}
		//出环处理
		if(annulus_t>DISTANCE_ANNULUS_T)
		{				
				road_type.annulus          = 0;
				road_type.in_annulus_left  = 0;
			  road_type.in_annulus_right = 0;
				road_type.on_annulus_left  = 0;
			  road_type.on_annulus_right = 0;
				road_type.out_annulus      = 0;
				annulus_s             		 = 0;
			  annulus_z                  = 0;
			  annulus_t                  = 0;
			  BUZZ_OFF;	
		}
}
/*************************************环岛辅助函数*************************************
函数：  void Annulus_assist(void)
参数：  无
说明：  过环岛三角区积分，进环积分，出环积分等

*注意： 积分值会会随采样时间的不同而改变，需要自己用手推车去测量用屏幕显示看并记录去修改
返回值：无  
******************************************************************************************/
void Annulus_assist(void)
{
	 if(road_type.annulus==1&&road_type.in_annulus_left==0&&road_type.in_annulus_right==0)
   {
        annulus_s += fabs(real_speed*0.1);  
   }
	 if((road_type.in_annulus_left==1 ||road_type.in_annulus_right==1) && road_type.on_annulus_left==0)
   {
        annulus_z += fabs(icm20602_gyro_z*0.01);
   }
	 if((road_type.on_annulus_left==1 ||road_type.on_annulus_right==1) && road_type.out_annulus==1)
   {
        annulus_t=annulus_t+10;
   }
}
/*************************************避障检测函数*************************************
函数：  void obstacle_avoidance(void)
参数：  无
说明：  TFO避障模块检测，使用软件模拟IIC通信，理论上任何引脚都可以使用，但是要注意不能引脚
        复用。
*注意： TOF模块离障碍物越远数值越大，越近数值越小
返回值：无  
******************************************************************************************/
void obstacle_avoidance(void)
{
	dl1a_get_distance();                                       //距离测量
	if(dl1a_finsh_flag==1&&dl1a_distance_mm<SET_DLLA_DISTANCE) //测量距离小于设定值标志位成立
	{
		dl1a_finsh_flag=0;
		flag_obstacle=1;
	}
	ips114_showint16(0,2,dl1a_distance_mm);
	printf("%d\n",dl1a_distance_mm);
}
/*************************根据赛道类型选择不同的方向偏差计算方法*************************
函数：  int16 Direction_error(void)
功能：  根据赛道类型选择不同的方向偏差
参数：  无
说明：  根据赛道类型选择不同的方向偏差--直道--弯道--环岛处理切换不同差比和
返回值：error--返回赛道偏差
****************************************************************************************/
int16 Direction_error(void)
{
    int16 error = 0;
    //直道方向偏差计算
    if(road_type.straight==1)
    {
			error = Cha_bi_he(Left_value,Right_value,80); 
			printf("%d\n",error);
    }
    //弯道方向偏差计算
    else if(road_type.bend==1)
    {
			error = Cha_bi_he(Left_value,Right_value,100); 
			printf("%d\n",error);			
    }
		//直角弯道方向偏差计算
		else if(road_type.straightbend==1)
    {
			error = Cha_bi_he(Left_value,Right_value,140);  
			//printf("%d\n",error);
    }
//    //环岛方向偏差计算
//    else if(road_type.annulus==1)
//    {
//			  //准备入环岛方向偏差计算
//        if(road_type.in_annulus_left==0 && road_type.in_annulus_right==0 && road_type.on_annulus_left==0 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
//				{
//            error = Cha_bi_he(Left_value,Right_value,10);  
//				}
//        //入左环岛方向偏差计算
//        if(road_type.in_annulus_left ==1 && road_type.on_annulus_left==0 && road_type.out_annulus==0)
//				{
//				    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,60);
//				}
//				//入右环岛方向偏差计算
//			  if(road_type.in_annulus_right ==1 && road_type.on_annulus_right==0 && road_type.out_annulus==0)
//				{
//				    error = Cha_bi_he(Left_Shu_Adc, Right_Shu_Adc,90);
//					
//				}
//        //出环岛方向偏差计算
//        //if((road_type.in_annulus_left||road_type.in_annulus_right) && (road_type.on_annulus_left||road_type.on_annulus_right)&&road_type.out_annulus)
//				    //error = Cha_bi_he(Left_Adc,Right_Adc,30);
//    }
//		//避障误差处理
	  else if(flag_obstacle==1)
		{
				 BUZZ_ON;
				 aim_speed=100;
				 error = obstacle_Current_Dir[temp];
				 temp++;
				 if(temp==40)
				 {
					 temp=0;
					 flag_obstacle=0;
					 obstacle_time=0;
				   aim_speed=400;
				   BUZZ_OFF;
				 }
		}
    return error;
}

/**********************************电磁所有总处理***************************************
函数：  void Get_deviation(void)
功能：  电磁所有总处理
参数：  无
说明：  放中断调用此函数即可
****************************************************************************************/
void Get_deviation(void)
{
	ADC_Collect();           //电感原始值采值
	Data_current_analyze();  //电感值归一化函数
	Road_type_judge();       //赛道类型判断---直道---弯道---环岛
	//Annulus_handle();        //环岛处理
	//obstacle_avoidance();    //障碍物检测
	Current_Dir=Direction_error(); //获得赛道偏差
	ips114_showfloat(122,4,Current_Dir);

	Out_protect();         //出界保护
}