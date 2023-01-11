#include "MahonyAHRS.h"

#define sampleFreq	 500.0f			// 频率为500HZ
#define twoKpDef	(2.0f * 0.42f)	// KP=1
#define twoKiDef	(2.0f * 0.2f)	// KI=0
//四元数以及PI参数的定义
volatile float twoKp = twoKpDef;																		
volatile float twoKi = twoKiDef;																		
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	//四元数的初始化				
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	//积分变量初始化
///////////////////////////////1
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
	{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	//如果磁力计测量无效，请使用IMU算法（避免在磁力计归一化中使用NaN）
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;//通过归一化处理后， ax， ay， az 的数值范围变成-1 到+1 之间。
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrt(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
		//加速度计的坐标的转换
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		//磁力计的坐标转换
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
		// 磁力计和加速度计的总误差
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		// KI=0所以不积分
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);	
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	
			integralFBy = 0.0f;
			integralFBz = 0.0f;
			}
		// 对KP累加上 加速度计和磁力计总的
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	gx *= (0.5f * (1.0f / sampleFreq));	// 累加了500次 所以除以500
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);//得到加速度计和磁力计共同修正后的四元数
	//归一化
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
////////////////////////////////////2
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	/*如果加速计各轴的数均是 0，那么忽略该加速度数据。否则在加速计数据归一化处理
的时候，会导致除以 0 的错误*/
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;//ax， ay， az 的数值范围变成-1 到+1 之间
		//ax,ay,az是实际的加速度值 halfvx，vy,vz,是理论的加速度值
		//将向量(0,0,1)转换到载体坐标上的向量，也即是理论的加速度
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
		// 将 a 向量与 b 向量做叉积的公式如下，得到 e 向量（误差向量）。
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);
		// 这里Ki等于0 不进行微分
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (1.0f / sampleFreq);
			integralFBy += twoKi * halfey * (1.0f / sampleFreq);
			integralFBz += twoKi * halfez * (1.0f / sampleFreq);
			gx += integralFBx;	
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}
		// 将误差进行P调节，然后累加到陀螺仪的数据上，修正陀螺仪
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// 0.5f是因为TWOk *1/500因为频率为500HZ，一秒累加计算了500次
	gx *= (0.5f * (1.0f / sampleFreq));		
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	qa = q0;
	qb = q1;
	qc = q2;
	//一阶龙格-库塔法 通过修正陀螺仪来修正四元数 来修正预测的加速度计
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
   //数值范围变成-1 到+1 之间
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}
/**************************实现函数*********************************************************************
函  数：static float invSqrt(float x) 
功　能: 快速计算 1/Sqrt(x) 	
参  数：要计算的值
返回值：结果
备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*********************************************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
