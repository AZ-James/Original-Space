#ifndef     __delay_H
#define     __delay_H
 
#include "stm32f10x.h"  
 
void Delay_Init(void);
void Delay_us(uint32_t nus);
void Delay_ms(uint32_t nms);
#endif

