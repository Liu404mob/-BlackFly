#include "Limits.h"
/*ÏÞ·ùº¯Êý*/
float Limits_data(float LimitsData,float LimitsUp,float LimitsDown)
{
	if(LimitsData > LimitsUp)
	{
		LimitsData = LimitsUp;
	}
	if(LimitsData < LimitsDown)
	{
		LimitsData = LimitsDown;
	}
	return LimitsData;
}
/*¸³Áãº¯Êý*/
float Limits_zero(float LimitsData,float LimitsUp,float LimitsDown)
{
	if(LimitsData	< LimitsUp && LimitsData > LimitsDown)
	{
		LimitsData = 0;
	}
	else
	{
		LimitsData = LimitsData;
	}
	return LimitsData;
}


