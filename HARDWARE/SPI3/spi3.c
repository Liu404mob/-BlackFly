#include "spi3.h"

void SPI3_Init(void)
{
    //定义结构体
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;
    //使能时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    //GPIO基本配置
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //GPIO复用成SPI3的函数/* SCK, MISO and MOSI  PB3=CLK,PB4=MISO,PB5=MOSI*/
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
    //两个片选引脚的配置
    GPIO_SetBits(GPIOC, GPIO_Pin_8);//陀螺仪片选
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB, GPIO_Pin_15);//磁力计片选
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //SPI3的基本配置
    SPI_Cmd(SPI3, DISABLE);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;       //CSK空闲时为低电平
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; //第一个沿采样
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;  //128分频
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;//高位先行
    SPI_InitStructure.SPI_CRCPolynomial = 7; //CRC检验
    //SPI初始化
    SPI_Init(SPI3, &SPI_InitStructure);
    //SPI3使能
    SPI_Cmd(SPI3, ENABLE);
}

uint8_t SPI3_ReadWrite_Byte(u8 byte)
{
    while((SPI3->SR & SPI_I2S_FLAG_TXE) == RESET);	//等待发送缓存区为空
    SPI3->DR = byte;/* 发送数据 Write in the DR register the data to be sent */

    while((SPI3->SR & SPI_I2S_FLAG_RXNE) == RESET); //等待接收缓存区非空
    return(SPI3->DR);//返回接收到的数据/* Return the data in the DR register */
}

void SPI3_writeReg(u8 reg, u8 data)
{
    SPI3_ReadWrite_Byte(reg);
    SPI3_ReadWrite_Byte(data);
}


u8 SPI3_readReg(u8 reg)
{
    SPI3_ReadWrite_Byte(reg | 0x80);
    return SPI3_ReadWrite_Byte(0xff);
}


void SPI3_readRegs(u8 reg, u8 length, u8 *data)
{
    u8 count = 0;
    SPI3_ReadWrite_Byte(reg | 0x80);
    for(count = 0; count < length; count++)
    {
        data[count] = SPI3_ReadWrite_Byte(0xff);
    }
}
