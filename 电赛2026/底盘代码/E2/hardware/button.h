#ifndef __BUTTON_H
#define __BUTTON_H

#include "stm32f10x.h"

void button_init(void);
uint8_t button_pressed(void);

#endif
