#include "sbus.h"

void Usart5_Init(u32 bound)// RX PD2
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉

    GPIO_Init(GPIOD, &GPIO_InitStructure);
    //USART5 初始化设置
    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;//！字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_2;//2个停止位
    USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx| USART_Mode_Tx ;	//收发模式
    USART_Init(UART5, &USART_InitStructure); //初始化串口5

    //Usart5 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//串口5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; //抢占优先级4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

    USART_Cmd(UART5, ENABLE);  //使能串口5

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//接收到8位数据就中断一次
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
}

uint8_t buf[26];
void UART5_IRQHandler(void)
{
	uint8_t res;
	uint8_t clear = 0;
	static uint8_t Rx_Sta = 1;

if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
{
	res =UART5->DR;
	buf[Rx_Sta++] = res;
}
	//一包数据发送完成 准备调整一下后，去接收下一包数据 中间有4MS/14MS延迟供你来检测
	else if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
	{//检测到空闲线路时，该位由硬件置 1，该位由软件序列清零（读入 USART_SR 寄存器，然后读入 USART_DR 寄存器）。
		clear = UART5->SR;
		clear = UART5->DR;
		buf[0] = Rx_Sta - 1;//在Xms内接收到的数据数量
		Rx_Sta = 1;
	}

}

uint16_t CH[18]; // 通道值
uint8_t rc_flag = 0;
void Sbus_Data_Count(void)
{
	CH[0] = ((int16_t)buf[ 2] >> 0 | ((int16_t)buf[ 3] << 8 )) & 0x07FF;//右边遥杆（左右） 上353-1024-1695下
	CH[1] = ((int16_t)buf[ 3] >> 3 | ((int16_t)buf[ 4] << 5 )) & 0x07FF;//右边摇杆（上下） 左353-1024-1695右
	CH[2] = ((int16_t)buf[ 4] >> 6 | ((int16_t)buf[ 5] << 2 ) | (int16_t)buf[ 6] << 10 ) & 0x07FF;//左边摇杆 （上下）
	CH[3] = ((int16_t)buf[ 6] >> 1 | ((int16_t)buf[ 7] << 7 )) & 0x07FF;//左边摇杆 （左右）
	CH[4] = ((int16_t)buf[ 7] >> 4 | ((int16_t)buf[ 8] << 4 )) & 0x07FF;//SB 上353、中1024 下1695
	CH[5] = ((int16_t)buf[ 8] >> 7 | ((int16_t)buf[ 9] << 1 ) | (int16_t)buf[10] << 9 ) & 0x07FF;//V1中间旋钮 左-右：353-1695
	CH[6] = ((int16_t)buf[10] >> 2 | ((int16_t)buf[11] << 6 )) & 0x07FF;//V2侧旋转轮 往上到353 中间1024 下面1695

//没有用到的通道
//CH[ 7] = ((int16_t)buf[11] >> 5 | ((int16_t)buf[12] << 3 )) & 0x07FF;
//CH[ 8] = ((int16_t)buf[13] << 0 | ((int16_t)buf[14] << 8 )) & 0x07FF;
//CH[ 9] = ((int16_t)buf[14] >> 3 | ((int16_t)buf[15] << 5 )) & 0x07FF;
//CH[10] = ((int16_t)buf[15] >> 6 | ((int16_t)buf[16] << 2 ) | (int16_t)buf[17] << 10 ) & 0x07FF;
//CH[11] = ((int16_t)buf[17] >> 1 | ((int16_t)buf[18] << 7 )) & 0x07FF;
//CH[12] = ((int16_t)buf[18] >> 4 | ((int16_t)buf[19] << 4 )) & 0x07FF;
//CH[13] = ((int16_t)buf[19] >> 7 | ((int16_t)buf[20] << 1 ) | (int16_t)buf[21] << 9 ) & 0x07FF;
//CH[14] = ((int16_t)buf[21] >> 2 | ((int16_t)buf[22] << 6 )) & 0x07FF;
//CH[15] = ((int16_t)buf[22] >> 5 | ((int16_t)buf[23] << 3 )) & 0x07FF;
}
