#ifndef __POSITIONESTIMATION_H
#define __POSITIONESTIMATION_H

#include "sys.h"


//extern Butter_BufferData  flow_filter_buf[3];

void KalmanFilter_Init(KalmanFilter *XAXIS,KalmanFilter *YAXIS,KalmanFilter *ZAXIS , KalmanFilter *BAROMETER);
void POS_KalmanFilter(KalmanFilter *KalmanFilter_Input,float Position,float Acceleration);
//void OpticalFlow_Estimation(float flow_x,float flow_y,float Accx,float Accy);
#endif 

