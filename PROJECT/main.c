#include "sys.h"
#include "u8g2.h"


//在此定义全局变量 在参数.h声明的 在sys.h里面extern的
PS2 Ps2_Number;							//手柄数据
SBUS Sbus_Number;
DroneRTInfo RT_Info;                   //飞行器实时数据
PIDOut OriginalPitch, OriginalRoll, OriginalYaw, OriginalPosX, OriginalPosY, OriginalPosZ,
       OriginalPitchRate, OriginalRollRate, OriginalYawRate, OriginalVelX, OriginalVelY, OriginalVelZ,
       OriginalFlowX, OriginalFlowY, OriginalFlowVelX, OriginalFlowVelY,
       OriginalAccZ;
PIDPara PID_ParaInfo;
KalmanFilter XAxis, YAxis, ZAxis, Barometer;
DroneTargetInfo Target_Info;           //飞行器目标的全局变量
Data_Combine DataCombineflag;      		 //飞行模式0123
DroneFlightControl FlightControl;      //飞行器状态变量
Thrust UAVThrust;											 //飞行器扭力计算
DroneErrangle Errangle_Info;           //飞行器平地校准数据
OffsetInfo OffsetData;                 //磁校准数据全局变量
Throttle Throttle_Info;								 //电调油门百分比
float TOFHeight, RTbattery; //激光高度,电池电压全局变量

//任务标志位
u8 IMU_FLAG, KAERMAN_FLAG, PS2_FLAG, POSITION_FLAG, ATTITUDE_FLAG, SAFE_FLAG = 0, DAYIN_FLAG, LAND_FLAG, AADC_FLAG;

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置中断分组2
	delay_init(168);
    uart_init(115200);//串口1打印数据 
    Adc_Init();
    Pid_Init();//PID值初始化
    TIM4_Init();//1ms中断一次
    General_Gpioinit();
    PWM_Init();
    SPI3_Init();
	delay_ms(30);
    IMU_HardwareInit();// IMU各个传感器寄存器配置
    Usart2_Init(115200); //激光模块
	delay_ms(30);
	OLED_Init();				//初始化OLED
	OLED_Clear();
    Usart5_Init(100000);//航模遥控器
    KalmanFilter_Init(&XAxis, &YAxis, &ZAxis, &Barometer);	// KalmanFilter参数初始化
    //	PS2_Init(); PS2_SetInit();
	LED1_ON;
    Beep_RingsOnce();	
	

	
    //主函数局部变量
    unsigned int fly_Pretime = 0;
    float PreparationPitch = 0 ;
    float PreparationRoll = 0;
    u8 PreparationFlag = 1;
    FlightControl.ControlStart = false;
    while(1)
    {
		OLED_ShowCHinese(0,0,0);//
		OLED_ShowCHinese(18,0,1);//
		OLED_ShowCHinese(36,0,2);//
		OLED_ShowCHinese(54,0,3);//
		OLED_ShowCHinese(72,0,4);//
		OLED_ShowCHinese(90,0,5);//		
		OLED_ShowString(0,3,"1.3' OLED TEST");

        //标志位记得清零
//        if(DAYIN_FLAG == 1)
//        {
//            DAYIN_FLAG = 0;
////            printf("Height:%f\n", TOFHeight);
////            LED1 = ~LED1;
////            if(TOFHeight >= 0.70f)LED2_ON;
//        }
        //1111.PS2 50ms一次
        if(PS2_FLAG == 1)
        {
            PS2_FLAG = 0;
			Sbus_Data_Count();
			Sbus_Number.SBUS_RX=(CH[0]);//+-40  YAW  
			Sbus_Number.SBUS_RY=(CH[1]);//+-100  HEIGHT
			Sbus_Number.SBUS_LX=(CH[3]);//+-16  ROLL
			Sbus_Number.SBUS_LY=(CH[2]);//+-16  PITCH
			Sbus_Number.SBUS_V1=(float)CH[5];//V1
			Sbus_Number.SBUS_V2=(float)CH[6];//V2
			Sbus_Number.SBUS_SB=(float)CH[4];//SB
	
            if(Sbus_Number.SBUS_SB==1024)	//扳到中间
            {SAFE_FLAG = 1; ALLLED_ON;}                          
            else if(Sbus_Number.SBUS_SB==353)//扳到上方
            {ALLLED_OFF;SAFE_FLAG = 0;}   
			TIM2->CCR3=Sbus_Number.SBUS_RY;
			TIM2->CCR4=Sbus_Number.SBUS_RY;
//			if(Sbus_Number.SBUS_RY>=1100&&SAFE_FLAG==1)
//				{
//					for(int i=0;i<=Sbus_Number.SBUS_RY;i+=10)
//						{TIM1->CCR3=i;
//						TIM1->CCR4=i;
//						LED1_OFF;
//						}
//				}
//			else if(Sbus_Number.SBUS_RY<=1000&&SAFE_FLAG==1)
//				{
//				for(int i=0;i>=Sbus_Number.SBUS_RY;i-=10)
//						{TIM1->CCR3=i;
//						TIM1->CCR4=i;
//						LED1_OFF;
//						}
//				}

          
        }
//        //2222.卡尔曼滤波 5ms一次
//        if(KAERMAN_FLAG == 1)
//        {
//            KAERMAN_FLAG = 0;
//            POS_KalmanFilter(&ZAxis, TOFHeight, RT_Info.accZaxis);
//            RT_Info.Height = ZAxis.Axis_Pos;
//            RT_Info.Height_V = ZAxis.Axis_Vel;
//        }
//        //3333.位置控制 ms一次									//200*0.007
//        if(POSITION_FLAG == 1)
//        {
//            POSITION_FLAG = 0;
//            float Climbing = 0.007f;
//            float Declining = 0.002f;
//            Target_Info.Height = 1.0f;
//            if(FlightControl.DroneMode == Drone_Mode_4Axis && FlightControl.OnOff == Drone_On && FlightControl.ControlStart == true)
//            {
//                Position_control(0, Climbing, Declining);   //Data_Headmode = 0,	Data_Headfree = 1
//            }
//        }
//        //4444.姿态控制 ms一次
//        if(ATTITUDE_FLAG == 1)
//        {
//            ATTITUDE_FLAG = 0;
//            //判断是不是ON并且没有高度的话不起飞
//            if(FlightControl.OnOff == Drone_On  && RT_Info.Height >= 0.01f)
//            {
//                /*判断是否为四轴起飞模式*/
//                if(FlightControl.DroneMode == Drone_Mode_4Axis)
//                {
//                    /*预飞程序*/
//                    if(fly_Pretime < 500) //只执行一次
//                    {
//                        fly_Pretime++;
//                        PreparationPitch += (RT_Info.Pitch - Errangle_Info.fixedErroPitch);
//                        PreparationRoll += (RT_Info.Roll - Errangle_Info.fixedErroRoll);
//                        PWM_OUTPUT(250, 250, 250, 250); //先以低油门旋转保持4电机转速一致
//                    }
//                    else
//                    {
//                        if(PreparationFlag) //只执行一次
//                        {
//                            PreparationPitch /= 500;//500次平均后再传进姿态控制
//                            PreparationRoll /= 500;
//                            PreparationFlag = false;
//                            Target_Info.Yaw = RT_Info.Yaw;//现在的角度就是目标角度，防止乱转*
//                            RT_Info.FlowX = 0 ;
//                            RT_Info.FlowY = 0 ;
//                            FlightControl.ControlStart = true;//姿态控制到这里改变标志位后 位置控制开始
//                        }
//                        if(FlightControl.ControlStart)
//                        {
//                            Attitude_control(PreparationPitch, PreparationRoll);
//                            /*保护程序*/
//                            Safety_Protection();
//                        }
//                    }
//                }//有高度但不是四轴模式 就不在考虑平地误差校准
//                else
//                {
//                    Attitude_control(0, 0);
//                }
//            }
//            else //判断是不是ON或者没有高度 也可以说是降落之后为第二次起飞做准备
//            {
//                PreparationFlag = true;
//                PreparationPitch = 0;
//                PreparationRoll = 0;
//                fly_Pretime = 0;
//                OriginalPitchRate.iOut = 0;
//                OriginalRollRate.iOut = 0;
//                OriginalYaw.iOut = 0;
//                OriginalVelZ.iOut = 0;
//                OriginalAccZ.iOut = 0;
//                OriginalVelX.iOut = 0;
//                OriginalVelY.iOut = 0;
//                OriginalFlowVelX.iOut = 0;
//                OriginalFlowVelY.iOut = 0;
//                Target_Info.Height = 1.0f; //恢复初始的默认目标高度
//                Target_Info.Pitch = 0.0f; //恢复初始的默认目标俯仰
//                Target_Info.Roll = 0.0f; //恢复初始的默认目标翻滚
//                FlightControl.LaunchFlag = true; //第一次起飞标志位 位置控制里面会用到
//                FlightControl.ControlStart = false;
//                PWM_OUTPUT(0, 0, 0, 0);
//            }
//        }

        //5555.姿态检测 2ms一次
        if(IMU_FLAG == 1)
        {
            IMU_FLAG = 0;
            IMU_getInfo();
        }
//        //6666.电池电压检测 1s一次
//        if(AADC_FLAG == 1)
//        {
//            AADC_FLAG = 0;
//            RTbattery = Get_battery_volt() ;
//            if(RTbattery <= 10.5f&&RTbattery >= 9.5f)
//            {
//                TIM14 -> CCR1 = 2000;
//            }           
//        }


    }//WHILE
}//MAIN
