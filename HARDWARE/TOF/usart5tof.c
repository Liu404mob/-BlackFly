#include "usart5tof.h"//其实是串口2

//#define USART5_REC_LEN 200
//#define Data_Head      0x59
//#define Data_Length     9

u8 USART2_RX_BUF[200];
u8 ccc=0;
void Usart2_Init(u32 bound)//TX PD5 RX PD6
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource6, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //USART2 初始化设置

    USART_InitStructure.USART_BaudRate = bound;//波特率设置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
    USART_Init(USART2, &USART_InitStructure); //初始化串口5

    //Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口5中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4; //抢占优先级4
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

    USART_Cmd(USART2, ENABLE);  //使能串口2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//接收到8位数据就中断一次


}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
    //printf("b\r\n");
    static u8 flag_data = 0;
    static u8 index = 0;
    u16 CheckSum = 0;
    u8 i;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
    {
        //printf("a\r\n");
        if(USART_ReceiveData(USART2) == 0x59) //读取接收到的数据
        {
            flag_data = 1;
        }
        if(flag_data == 1)
        {
            USART2_RX_BUF[index++] = USART_ReceiveData(USART2);
            if(index == 9)
            {
                if((USART2_RX_BUF[0] == 0x59) && (USART2_RX_BUF[1] == 0x59))
                {
                    for(i = 0; i < 8; i++)
                    {
                        CheckSum += USART2_RX_BUF[i];
                    }
                    if((CheckSum & 0x00ff) == USART2_RX_BUF[8])
                    {
                        uint16_t Distance = (((uint16_t)USART2_RX_BUF[3]) << 8) + USART2_RX_BUF[2];
                        uint16_t Strength = (((uint16_t)USART2_RX_BUF[5]) << 8) + USART2_RX_BUF[4];
                        //根据数据手册强度在这个范围内有效
                        if(Strength > 500 && Strength < 60000)
                        {
                            TOFHeight = ((float)Distance) * 0.01f;   // 米制单位
                        }
                        index = 0;
                        flag_data = 0;
                    }
                }
            }
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

}
