#include "position.h"

void Position_control(unsigned char Data_flag,float Climb,float Decline)
{
	static unsigned char hover = 0 ;
	static unsigned char controlCnt = 0;
	static float TgtHeight = 0 ;
//	static Vector3f_t DesiredAcceleration;
	UAVThrust.Gravity_Acceleration = 9.8f ;
	
	controlCnt++;
	if(FlightControl.landFlag==1){//进行位置控制前先判断是否要求降落
        //缓慢降落
        TgtHeight = TgtHeight - Decline;
		//三环控制 高度环PID后输出给速度环，再输出给加速度环
		if(controlCnt ==2)
			{
		  //外环单P控制器
			float heightErro = TgtHeight - RT_Info.Height;
			OriginalPosZ.value = Limits_data( PID_ParaInfo.PosZ.Kp * heightErro , 1.0f ,-1.0f);
			}
        //速度环PID控制器
        OriginalVelZ.value = Limits_data(  PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,OriginalPosZ.value,RT_Info.Height_V,0.002,4) ,7,-7);
		//加速度环PID控制器
		OriginalAccZ.value =  Limits_data(PID_Control(&PID_ParaInfo.AccZ,&OriginalAccZ,OriginalVelZ.value,RT_Info.accZaxis,0.002,7),10,-10);				
		//降落到6CM以下的时候
        if(RT_Info.Height<0.06f){
            FlightControl.OnOff = Drone_Land;//着陆模式
            FlightControl.landFlag = 0;		 //清除降落标志位
            TgtHeight = 0;
			FlightControl.ControlStart = false;
            Target_Info.Height = 1.0f; //恢复初始的默认目标高度
		
        }
    }
	    else{//正常起飞		
				/* 手柄遥控高度控制器 */
        if(Sbus_Number.SBUS_RY==0)
        {
            /* 第一次回到悬停状态，将现在的高度设置为目标高度 */
            if(hover ==1)
            {	//Target_Info.Height 用来设置0.9m
                Target_Info.Height = RT_Info.Height ;
                hover=0 ;
            }
            /************************高度悬停控制 ************************/
		/* 第一次起飞缓慢上升 */
		if(TgtHeight < Target_Info.Height && FlightControl.LaunchFlag == true)
		{
			if(TgtHeight < 0.6f)
					 TgtHeight = TgtHeight + Climb ;
			 else
					 TgtHeight = TgtHeight + Climb/2 ;
		}
		else{
					TgtHeight = Target_Info.Height;
					FlightControl.LaunchFlag = false ;
			}
	if(controlCnt ==2)
		{//三环控制
//外环单P控制器
float heightErro = TgtHeight - RT_Info.Height;
OriginalPosZ.value = Limits_data( PID_ParaInfo.PosZ.Kp * heightErro ,1.0f ,-1.0f);  //±1m/s的目标速度
		}
//速度环PID控制器
OriginalVelZ.value = Limits_data(  PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,OriginalPosZ.value,RT_Info.Height_V,0.002,4) ,7.0f,-7.0f);
//加速度环PID控制器
OriginalAccZ.value =  Limits_data(PID_Control(&PID_ParaInfo.AccZ,&OriginalAccZ,OriginalVelZ.value,RT_Info.accZaxis,0.002,7),10.0f,-10.0f);
						
        }//如果动遥控器的LY
        else
        {																						
OriginalVelZ.value = Limits_data(  PID_Control(&PID_ParaInfo.VelZ,&OriginalVelZ,Sbus_Number.SBUS_RY/100,RT_Info.Height_V,0.002,4) ,7.0f,-7.0f);
//加速度环PID控制器
OriginalAccZ.value =  Limits_data(PID_Control(&PID_ParaInfo.AccZ,&OriginalAccZ,OriginalVelZ.value,RT_Info.accZaxis,0.002,7),10.f,-10.f);
          hover =1;
        }
    }
UAVThrust.HeightThrust =  OriginalAccZ.value + UAVThrust.Gravity_Acceleration;
	
/////////////************************ 位置环速度控制器  ************************/
    //只有飞行器的高度大于10cm才开始进行位置控制
    if(RT_Info.Height != 0.0f)
	{
        switch (Data_flag){
            /************************ 有头模式（无定位）  ************************/
            case 0://///////////
			Target_Info.Pitch = Sbus_Number.SBUS_LY;
			Target_Info.Roll =  Sbus_Number.SBUS_LX;
                break;
            /************************ 无头模式（无定位）  ************************/
            case 1:

                break;
           }

	}
	if(controlCnt == 2)
	controlCnt =0;
}


