#ifndef _GPIO_H
#define _GPIO_H

#include "sys.h"

void General_Gpioinit(void);
void Beep_RingsOnce(void);
void SelfChecking_By(void);

//LED
#define LED1 PBout(9)
#define LED2 PBout(8)
#define LED3 PCout(2)
#define LED4 PCout(3)

#define KEY1 GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)

#define LED1_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_9)
#define LED2_ON  GPIO_ResetBits(GPIOB,GPIO_Pin_8)
#define LED3_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED4_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_3)

#define ALLLED_ON LED2_ON;LED3_ON;LED4_ON

#define LED1_OFF  GPIO_SetBits(GPIOB,GPIO_Pin_9)
#define LED2_OFF  GPIO_SetBits(GPIOB,GPIO_Pin_8)
#define LED3_OFF  GPIO_SetBits(GPIOC,GPIO_Pin_2)
#define LED4_OFF  GPIO_SetBits(GPIOC,GPIO_Pin_3)
#define ALLLED_OFF LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF

#define BluetoothLED_ON   GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define BluetoothLED_OFF  GPIO_SetBits(GPIOC,GPIO_Pin_1)

//串口模式选择
#define Usart_mode GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0?1:0
#define Take_Off GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==0?1:0

//BEEP
//直接操作定时器14的寄存器
#endif
