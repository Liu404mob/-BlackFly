#ifndef _IMU_AHRS_
#define _IMU_AHRS_
#include "sys.h"

float safe_asin(float v);
void IMU_HardwareInit(void);
void IMU_getValues(float *values) ;
void IMU_getInfo(void);
#endif

