#ifndef _PARAMETER_H
#define _PARAMETER_H
#include <stdbool.h>
#include <stdint.h>
/* 选择匹配的电机型号 */
//#define Tmotor_2213
#define Tmotor_2306


/* 选择定高传感器 */
#define Tof
//#define Ultrasonic
//#define Barometer


//定义轴距机型
//#define Model330
#ifdef Model330
    #define ARM_Length 0.165f
    #define Drone_Mass 1.02f
    #define Drag_Coeff 0.016f
    //转动惯量设置  转动惯量 = M*R^2  对于质点而言。这里无法测得转动惯量大小，变为一个可调参数设置
    #define Inertia_Wx    0.002f
    #define Inertia_Wy    0.002f
    #define Inertia_Wz    0.004f
#else
    #define ARM_Length 0.125f
    #define Drone_Mass 0.86f
    #define Drag_Coeff 0.016f
    //转动惯量设置  转动惯量 = M*R^2  对于质点而言。这里无法测得转动惯量大小，变为一个可调参数设置
    #define Inertia_Wx    0.001f
    #define Inertia_Wy    0.001f
    #define Inertia_Wz    0.002f
#endif

typedef struct{//PS2遥控相关
	double PS2_LX;
	double PS2_LY;
	double PS2_RX;
	double PS2_RY;
	int PS2_KEY;
}PS2;

typedef struct{//PS2遥控相关
	float SBUS_LX;
	float SBUS_LY;
	float SBUS_RX;
	float SBUS_RY;
	float SBUS_KEYL;//左边摇杆
	float SBUS_KEYR;//右边摇杆
	float SBUS_SB;
	float SBUS_V1;
	float SBUS_V2;
}SBUS;
/*无人机实时信息*/
typedef struct
{
	float Pitch; 		  //实时Pitch角度
	float Roll;			  //实时Roll角度
    float Yaw;			  //实时Yaw角度
	float HomeYaw;		  //起飞时yaw角度
	float HeadfreeYaw;    //无头模式下yaw角度
	float Headfreezeta;   //无头模式旋转Zeta角
	float ratePitch;
	float rateRoll;
	float rateYaw;
	float accXaxis;
	float accYaxis;
	float accZaxis;
	float Height;					//US100超声波高度
	float Height_V;				//US100超声波速度
	float Barometer;      //气压计数据
	float Barometer_V;    //气压计速度
	float FlowX;					//光流位置X
	float FlowY;          //光流位置Y
	float FlowX_V;				//光流速度X
	float FlowY_V;				//光流速度Y
	float PointX;					//点数据X位移
	float PointY;					//点数据Y位移
	float PointX_V;				//点数据X的速度
	float PointY_V;				//点数据Y的速度
	float batteryVoltage; //电池电量
	float AccX;
	float AccY;
	float AccZ;
	float GyroX;
	float GyroY;
	float GyroZ;
	float MagX;
	float MagY;
	float MagZ;
	int lowPowerFlag;			//低电压标志位
	float CpuTick;        //系统时间计时
	float LPFTest1;        //滤波前数据测试
	float LPFTest2;        //滤波前数据测试
	unsigned char AllowLanding;  // 遥控器控制降落标志
//	Remote_Control_Status  controlStatus;  //专业遥控器控制状态
//	Height_Data_Switching  heightDataSwitching;  //高度数据来源切换
}DroneRTInfo;

typedef struct
{
	float error;
	float lasterror;
	float differential;
	float differentialFliter;
	float pOut;
    float iOut;
	float dOut;
	float value;//PID控制输出
	
}PIDOut;

typedef struct
{
	float Kp;
	float Ki;
	float Kd;
}PID;

typedef struct
{
    PID Pitch;
    PID Roll;
    PID Yaw;

    PID PitchRate;
    PID RollRate;
    PID YawRate;

    PID PosX;
    PID PosY;
    PID PosZ;

    PID VelX;
    PID VelY;
    PID VelZ;

    PID AccZ;

    PID FlowX;
    PID FlowY;
    PID FlowVelX;
    PID FlowVelY;
}PIDPara;



typedef struct{
    //滤波时间
    float Merge_t;

    //参数
    float Q_Position;
    float Q_Velocity;
    float Q_Bias;
    float R_Position;

    //状态
    float Axis_Pos;
    float Axis_Vel;
    float Axis_Bias;
    float Axis_Err;
    float AxisPCt_0;
    float AxisPCt_1;
    float AxisPCt_2;
    float AxisE;
    char  AxisC_0;
    float AxisK_0;
    float AxisK_1;
    float AxisK_2;
    float Axist_0;
    float Axist_1;
    float Axist_2;
    float AxisPdot[9];
    float AxisPP[3][3];
}KalmanFilter;


typedef struct
{
	float Pitch;
	float Roll;
	float Yaw;
	float RateRoll;
	float RatePitch;
	float RateYaw;
	float Height;
	float VelHeight;
	float AccHeight;
	float RemotePitch;
	float RemoteRoll;
	float BlackLineV;
    float BlackLineYaw;
	float DesiredAccelerationX;
	float DesiredAccelerationY;
	float DesiredAccelerationZ;
}DroneTargetInfo;

typedef enum
{
	Data_Headmode = 0,
	Data_Headfree = 1,
	Data_Point = 2,
	Data_Flow = 3,
	Data_Follow = 4,
}Data_Combine;

typedef enum
{  
	Drone_Off  = 0x00,//起飞或者调试打开电机
  Drone_On   = 0x01,//关闭电机
  Drone_Land = 0x02,//降落	
}DroneFlightOnOff_TypeDef;

typedef enum{
  Drone_Mode_None=0,
  Drone_Mode_RatePitch, //姿态内环
  Drone_Mode_RateRoll,
  Drone_Mode_Pitch,     //姿态外环
  Drone_Mode_Roll,
  Drone_Mode_4Axis,     //四轴飞行
}DroneFlightMode_TypeDef;

typedef enum
{  
	Report_SET      = 0x01,
  Report_RESET    = 0x00, 		 	
}DroneReportSW_TypeDef;
typedef struct
{
	DroneFlightOnOff_TypeDef OnOff;
	DroneFlightMode_TypeDef DroneMode;
	DroneReportSW_TypeDef ReportSW;
	int landFlag;
	bool LaunchFlag;
	bool ControlStart;
}DroneFlightControl;

typedef struct{
    float PitchThrust;
    float RollThrust;
    float YawThrust;
    float HeightThrust;
    float Gravity_Acceleration;
	float f1;
	float f2;
	float f3;
	float f4;
	float t1;
	float t2;
	float t3;
	float t4;
	float PaddleEffect;
}Thrust;

/* 平地误差数据 */
typedef struct
{
	float fixedErroPitch;
	float fixedErroRoll;
}DroneErrangle;

/*校准数据*/
typedef struct
{
	bool MagOffseting;
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
	float GyroX;
	float GyroY;
	float GyroZ;
	float AccX;
	float AccY;
	float AccZ;
	float AccXScale;
	float AccYScale;
	float AccZScale;
}OffsetInfo;

typedef struct {
    float x;
    float y;
    float z;
} Vector3f_t;

typedef struct
{
	int M1;
	int M2;
	int M3;
	int M4;
}Throttle;

#endif
