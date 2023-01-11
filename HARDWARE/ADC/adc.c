#include "adc.h"

//初始化ADC
void  Adc_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef       ADC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);	 //ADC1复位
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);	//复位结束

    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;//两个采样阶段之间的延迟5个时钟
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz
    ADC_CommonInit(&ADC_CommonInitStructure);//初始化

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//连续转换
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐
    ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1
    ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化

    ADC_Cmd(ADC1, ENABLE);//开启AD转换器
}
//获得ADC值
//ch: @ref ADC_channels
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)
{
    //设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度

    ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能

    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

    return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

float Get_battery_volt(void)
{
    float Volt;//电池电压
    Volt = Get_Adc(ADC_Channel_8) * 3.3 * 4.25 / 4096;	//电阻分压
    //	Volt = 4.25f*3.3f*((float)Get_Adc(ADC_Channel_8)/4096);
    return Volt;
}

u16 Get_Adc_Average(void)
{
    u32 temp_val = 0;
    u8 t;
    for(t = 0; t < 5; t++) //取5次平均值
    {
        temp_val += Get_Adc(ADC_Channel_8) * 3.3 * 4.35 / 4096;
        delay_ms(5);
    }
    return temp_val / 5;
}

