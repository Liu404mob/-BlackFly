#ifndef __PID_H
#define __PID_H

#include "sys.h"


#define lowpass_filter  7.9577e-3f

float PID_Control(PID *PIDpara, PIDOut *PIDstatus, float expect_PID, float feedback_PID, float PIDtime,float Integrallimiter);
void Pid_Init(void);


#endif




