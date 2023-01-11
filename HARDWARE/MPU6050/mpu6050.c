#include "mpu6050.h"

u8 Gyro_Off_started = 0;
int16_t lastAx, lastAy, lastAz,
        lastGx, lastGy, lastGz;
static int16_t Gx_offset = 0, Gy_offset = 0, Gz_offset = 0;

static void MPU6500_writeReg(u8 reg, u8 data)
{
    MPU6500_CSL();//陀螺仪的片选！！！
    SPI3_ReadWrite_Byte(reg);
    SPI3_ReadWrite_Byte(data);
    MPU6500_CSH();
}

void MPU6500_initialize(void)
{
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1, 0x80);          // 复位
    delay_ms(20);//电源管理器的第7位reset位置为1就会复位
    MPU6500_writeReg(MPU6500_RA_SIGNAL_PATH_RESET, 0x07);//重置陀螺仪数字信号路径。
    delay_ms(20);
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1,0x03);          // 使用内部20M时钟
    delay_ms(20);//00000011【2：0】位如果锁相环准备好了自动选择最好的时钟，否则使用内部20M时钟
    //未禁止中断 中断引脚/旁路使能配置寄存器 全部设置为0就可禁止所有中断
    MPU6500_writeReg(MPU6500_RA_SMPLRT_DIV, 0x00);          // sample rate.  Fsample= 1Khz/(<this value>+1) = 1000Hz
    delay_ms(20);//分频数=0
    MPU6500_writeReg(MPU6500_RA_ACCEL_CONFIG, 0x10);        // +-8G accel
    delay_ms(20);//00010000 4:3位为10 代表+-8g 65536/16=4096LSB/g
    MPU6500_writeReg(MPU6500_RA_FF_THR, 0x04);              // 加计低通滤波器
    delay_ms(20);//00000100第三位为0 2:0为100 即可设置输出滤波为20HZ
    MPU6500_writeReg(MPU6500_RA_GYRO_CONFIG, 0x18); //00011000 4:3为11代表+-2000 gyro 0:1 为00 代表F_choice为11
    delay_ms(20);//陀螺仪的16位AD输出范围是-32768到+32768 灵敏度为65536/4000=16.384LSB/°/S
    MPU6500_writeReg(MPU6500_RA_CONFIG, 0x03);              // 内部低通巴特沃斯滤波器  截止频率 41Hz
    delay_ms(20);//00000011
    MPU6500_writeReg(MPU6500_RA_PWR_MGMT_2, 0x00);
    delay_ms(20);//开启电源管理2寄存器的5:0位 全部设置为0就开启3轴 陀螺仪和加速度
//	  MPU6500_writeReg(MPU6500_RA_PWR_MGMT_1, 0x00);	//解除休眠状态
//	MPU6500_writeReg(MPU6500_RA_SMPLRT_DIV , 0x07);
//	MPU6500_writeReg(MPU6500_RA_CONFIG, 0x06);
//	MPU6500_writeReg(MPU6500_RA_GYRO_CONFIG, 0x18);
//	MPU6500_writeReg(MPU6500_RA_ACCEL_CONFIG , 0x10);
}

void MPU6500_readGyro_Acc(int16_t *gyro, int16_t *acc)
{
    static u8 buf[14];
    static int16_t gx, gy, gz;
    static int16_t ax, ay, az;
    MPU6500_CSL();//每个轴的数据分高8低8分别存在 MPU6500_RA_ACCEL_XOUT_H   MPU6500_RA_ACCEL_XOUT_L 寄存器中
    SPI3_readRegs(MPU6500_RA_ACCEL_XOUT_H, 14, buf); //陀螺仪3轴用6个寄存器 温度计用2个寄存器 加速度计用6个寄存器
    MPU6500_CSH();//一共14个 都储存在buf数组里面
    ax = (int16_t)(((int16_t)buf[0]) << 8 | buf[1]);
    ay = (int16_t)(((int16_t)buf[2]) << 8 | buf[3]);//加速度计测出来是载体坐标系下的
    az = (int16_t)(((int16_t)buf[4]) << 8 | buf[5]);
    gx = (int16_t)(((int16_t)buf[8]) << 8 | buf[9]);
    gy = (int16_t)(((int16_t)buf[10]) << 8 | buf[11]);
    gz = (int16_t)(((int16_t)buf[12]) << 8 | buf[13]);
    gyro[0] = gx - Gx_offset;	//gyro陀螺仪
    gyro[1] = gy - Gy_offset;
    gyro[2] = gz - Gz_offset;
    acc[0] = ax; //acc加速度计
    acc[1] = ay;
    acc[2] = az;
}

void MPU6500_Init_Offset(void)//6500初始化校准 求个平均值 简单滤个波
{
    unsigned int i;
    int16_t temp[3], temp2[3]; //temp 就是储存加速度计的3轴数组 temp2是储存陀螺仪3轴加速度计数组
    int32_t tempgx = 0, tempgy = 0, tempgz = 0;
    Gx_offset = 0;
    Gy_offset = 0;
    Gz_offset = 0;

    for(i = 0; i < 500; i++)
    {
        delay_us(200);
        MPU6500_readGyro_Acc(temp, temp2); //temp 就是储存陀螺仪的3轴数组 temp2是储存加速度计3轴加速度计数组
        tempgx += temp[0];//数据5000次累加
        tempgy += temp[1];
        tempgz += temp[2];
    }
    Gx_offset = tempgx / 500; //只求了陀螺仪的平均值
    Gy_offset = tempgy / 500;
    Gz_offset = tempgz / 500;
}


//------------------End of File----------------------------

