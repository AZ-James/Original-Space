// motor.h
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void motor_init(void);
void motor_forward(uint8_t left_speed, uint8_t right_speed);
void motor_stop(void);
void motor_set_speed(uint8_t left_speed, uint8_t right_speed);

#endif
