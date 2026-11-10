#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

// 定时器初始化（1ms中断）
void timer_init(void);
// 定时器开关（1=开，0=关）
void timer_control(uint8_t enable);
// timer.h 中添加
extern volatile uint8_t g_finish_detected;
#endif
