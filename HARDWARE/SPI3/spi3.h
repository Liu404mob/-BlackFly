#ifndef _SPI3_
#define _SPI3_

#include "sys.h"
void SPI3_Init(void);
uint8_t SPI3_ReadWrite_Byte(u8 byte);
void SPI3_writeReg(u8 reg, u8 data);
u8 SPI3_readReg(u8 reg);
void SPI3_readRegs(u8 reg, u8 length, u8 *data);
#endif
