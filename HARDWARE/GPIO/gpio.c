#include "gpio.h"


void General_Gpioinit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);
    //LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //BEEP
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
	GPIO_SetBits(GPIOB, GPIO_Pin_2); 
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//普通输出模式
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM14);
//    //结构体
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//    TIM_OCInitTypeDef  TIM_OCInitStructure;
//    //时钟
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
//    //定时器基本设置
//    TIM_TimeBaseStructure.TIM_Period = 3000;      							//PWM周期
//    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;    						//设置预分频
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;			//设置时钟分频系数：不分频
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
//    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
//    //通道一设置
//    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;						//配置为PWM模式2
//    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//计数值小于该值时为低电平
//    TIM_OCInitStructure.TIM_Pulse = 0;
//    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//    TIM_OC1Init(TIM14, &TIM_OCInitStructure);	 //使能通道1
//    TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
//    //使能
//    TIM_ARRPreloadConfig(TIM14, ENABLE);			 // 使能TIM14重载寄存器ARR
//    TIM_Cmd(TIM14, ENABLE);                   //使能定时器14

    //KEY
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//SW3
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//SW2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO

    GPIO_SetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);
    GPIO_SetBits(GPIOC, GPIO_Pin_2 | GPIO_Pin_3);
    GPIO_SetBits(GPIOC, GPIO_Pin_1);
}


void Beep_RingsOnce(void)
{
//    TIM14 -> CCR1 = 1500;
//    delay_ms(500);
//    TIM14 -> CCR1 = 0;
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);  //蜂鸣器对应引脚GPIOF8拉低，
	delay_ms(500);
	GPIO_SetBits(GPIOB, GPIO_Pin_2); 
}

void SelfChecking_By(void)
{
    TIM14 -> CCR1 = 1000;
    delay_ms(500);
    TIM14 -> CCR1 = 0;
    delay_ms(100);
    TIM14 -> CCR1 = 2000;
    delay_ms(500);
    TIM14 -> CCR1 = 0;
    delay_ms(500);
}

