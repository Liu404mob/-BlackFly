#include "pid.h"

float PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID, float PIDtime, float Integrallimiter){
	
	PIDstatus->error = expect_PID - feedback_PID;
	PIDstatus->differential = (PIDstatus->error - PIDstatus->lasterror)/PIDtime;
	
	PIDstatus->differential = PIDstatus->differentialFliter + (PIDtime / (lowpass_filter + PIDtime)) * (PIDstatus->differential - PIDstatus->differentialFliter);

    PIDstatus->differentialFliter = PIDstatus->differential;
	
	PIDstatus->lasterror = PIDstatus->error;
	
	PIDstatus->pOut = PIDpara->Kp * PIDstatus->error;
	PIDstatus->iOut += PIDpara->Ki * PIDstatus->error;
	PIDstatus->dOut = PIDpara->Kd * PIDstatus->differential;
	PIDstatus->iOut = Limits_data(PIDstatus->iOut,Integrallimiter,-Integrallimiter);
	
	PIDstatus->value = PIDstatus->pOut + PIDstatus->iOut + PIDstatus->dOut;
	
	return PIDstatus->value;
}

void Pid_Init(void)
{
	
	PID_ParaInfo.Pitch.Kp = -0.01f;
	PID_ParaInfo.Pitch.Ki= 0;
	PID_ParaInfo.Pitch.Kd= 0;

	PID_ParaInfo.Roll.Kp= 0.01;
	PID_ParaInfo.Roll.Ki= 0;
	PID_ParaInfo.Roll.Kd= 0;

	PID_ParaInfo.Yaw.Kp= 0.01f;
	PID_ParaInfo.Yaw.Ki= 0;
	PID_ParaInfo.Yaw.Kd= 0;	

//	PID_ParaInfo.PitchRate.Kp= 130.0f;
//	PID_ParaInfo.PitchRate.Ki= 1.2f;
//	PID_ParaInfo.PitchRate.Kd= 4.0f;

//	PID_ParaInfo.RollRate.Kp= 130.0f;
//	PID_ParaInfo.RollRate.Ki= 1.2f;
//	PID_ParaInfo.RollRate.Kd= 4.0f;

//	PID_ParaInfo.YawRate.Kp= 70.0f;
//	PID_ParaInfo.YawRate.Ki= 0.4f;
//	PID_ParaInfo.YawRate.Kd= 1.5f;	
	
	
	PID_ParaInfo.PitchRate.Kp=120.0f;
	PID_ParaInfo.PitchRate.Ki= 2.35f;
	PID_ParaInfo.PitchRate.Kd= 2.5f;//0.8

	PID_ParaInfo.RollRate.Kp= 120.0f;
	PID_ParaInfo.RollRate.Ki= 2.35f;
	PID_ParaInfo.RollRate.Kd= 2.5f;//0.8

	PID_ParaInfo.YawRate.Kp= 60.0f;
	PID_ParaInfo.YawRate.Ki= 0.0f;
	PID_ParaInfo.YawRate.Kd= 0.1f;	


	PID_ParaInfo.PosZ.Kp=0.8f ;
	PID_ParaInfo.PosZ.Ki=0;
	PID_ParaInfo.PosZ.Kd=0;	

	PID_ParaInfo.VelZ.Kp=6.5f;
	PID_ParaInfo.VelZ.Ki=0.05f;
	PID_ParaInfo.VelZ.Kd=0;	

	PID_ParaInfo.PosX.Kp=-0.5f;
	PID_ParaInfo.PosX.Ki=0;
	PID_ParaInfo.PosX.Kd=0;	

	PID_ParaInfo.VelX.Kp=17.0f;
	PID_ParaInfo.VelX.Ki=0;
	PID_ParaInfo.VelX.Kd=0.6f;	

	PID_ParaInfo.PosY.Kp=-0.5f;
	PID_ParaInfo.PosY.Ki=0;
	PID_ParaInfo.PosY.Kd=0;	

	PID_ParaInfo.VelY.Kp=17.0f;
	PID_ParaInfo.VelY.Ki=0;
	PID_ParaInfo.VelY.Kd=0.6f;

	PID_ParaInfo.AccZ.Kp=0.55f;
	PID_ParaInfo.AccZ.Ki=0.04f;
	PID_ParaInfo.AccZ.Kd=0;	

	PID_ParaInfo.FlowX.Kp=0;
	PID_ParaInfo.FlowX.Ki=0;
	PID_ParaInfo.FlowX.Kd=0;

	PID_ParaInfo.FlowVelX.Kp=0;
	PID_ParaInfo.FlowVelX.Ki=0;
	PID_ParaInfo.FlowVelX.Kd=0;

	PID_ParaInfo.FlowY.Kp=0;
	PID_ParaInfo.FlowY.Ki=0;
	PID_ParaInfo.FlowY.Kd=0;

	PID_ParaInfo.FlowVelY.Kp=0;
	PID_ParaInfo.FlowVelY.Ki=0;
	PID_ParaInfo.FlowVelY.Kd=0;

	Errangle_Info.fixedErroPitch= 0;
	Errangle_Info.fixedErroRoll= 0;

	OffsetData.MagX =0;
	OffsetData.MagY =0;
	OffsetData.MagZ =0;

	OffsetData.AccX =0;
	OffsetData.AccY =0;
	OffsetData.AccZ =0;	

	OffsetData.AccXScale =1.0f ;
	OffsetData.AccYScale =1.0f ;
	OffsetData.AccZScale =1.0f ;
}



