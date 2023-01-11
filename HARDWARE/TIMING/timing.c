#include "timing.h"

void TIM4_Init(void)//1MS中断一次
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    //TIM4来自APB1  42*2=84M 开发手册上说的频率*2
    TIM_TimeBaseStructure.TIM_Period = 999; //自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = 83; //预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //一秒钟计数1M次，计数到999+1，需要0.001s
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    //程序时基TIM4定时器中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢断优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; //子优选级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM4, ENABLE);
}
void TIM4_IRQHandler(void)
{

    static uint16_t  KAERMAN = 0, PS2 = 0, POSITION = 0, ATTITUDE = 0, IMU = 0, DAYIN = 0, LAND = 0, AADC = 0; //分频系数
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        KAERMAN++;
        PS2++, POSITION++, ATTITUDE++, IMU++, DAYIN++, AADC++;
        if(SAFE_FLAG == 1)LAND++;
        if(PS2 >= 50)
        {
            PS2_FLAG = 1;
            PS2 = 0;
        }
        if(KAERMAN >= 5)
        {
            KAERMAN_FLAG = 1;
            KAERMAN = 0;
        }
        if(POSITION >= 5)
        {
            POSITION_FLAG = 1;
            POSITION = 0;
        }
        if(ATTITUDE >= 2)
        {
            ATTITUDE_FLAG = 1;
            ATTITUDE = 0;
        }
        if(IMU >= 2)
        {
            IMU_FLAG = 1;
            IMU = 0;
        }
        if(DAYIN >= 1000)
        {
            DAYIN_FLAG = 1;
            DAYIN = 0;
        }
        if(AADC >= 500)
        {
            AADC_FLAG = 1;
            AADC = 0;
        }
        //		if(LAND>=10000)	{LAND_FLAG=1;LAND=0;SAFE_FLAG=0;}
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
