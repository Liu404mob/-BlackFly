#ifndef _ADC_H
#define _ADC_H

#include "sys.h"

void  Adc_Init(void);
u16 Get_Adc(u8 ch) ;
float Get_battery_volt(void) ;
u16 Get_Adc_Average(void);

#endif
