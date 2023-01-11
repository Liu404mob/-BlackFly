#include "motor.h"

void PWM_Init(void)
{
    //结构体
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct ;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    //时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    //复用功能函数
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
    //引脚基本设置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //定时器的基本设置
    TIM_TimeBaseInitStruct.TIM_Period = 2000;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;						//配置为PWM模式1
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//计数值小于该值时为高电平

    TIM_OCInitStructure.TIM_Pulse = 1024;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_Pulse = 1024;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);	 //使能通道2
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_Pulse = 1024;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OCInitStructure.TIM_Pulse = 1024;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);	 //使能通道4
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);			 // 使能TIM2重载寄存器ARR
    TIM_Cmd(TIM2, ENABLE);                   //使能定时器2

}

