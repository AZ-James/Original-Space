#include "motor.h"
#include "stm32f10x_tim.h"
#include "stm32f10x.h"

void motor_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
    
    // 电机方向控制引脚：PB8和PB9用于左电机方向控制，PB10和PB11用于右电机方向控制
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 左电机PWM引脚 PA6 (TIM3_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 重要修改：右电机PWM引脚改为 PA8 (TIM1_CH1)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;  // PA8
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // TIM3配置（左电机PWM）
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // TIM1配置（右电机PWM） - 使用相同的时基配置
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
    // PWM通道配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    // 左电机 - TIM3_CH1 (PA6)
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    
    // 重要修改：右电机 - TIM1_CH1 (PA8)
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);  // TIM1通道1
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    // 使能自动重载预装载
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    
    // 对于高级定时器TIM1，需要使能主输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    
    // 使能定时器
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
    
    // 停止电机
    motor_stop();
}

void motor_set_speed(uint8_t left_speed, uint8_t right_speed)
{
    if(left_speed > 100) left_speed = 100;
    if(right_speed > 100) right_speed = 100;
    
    TIM_SetCompare1(TIM3, left_speed);
    TIM_SetCompare1(TIM1, right_speed);  // 改为TIM1的比较寄存器
}

void motor_forward(uint8_t left_speed, uint8_t right_speed)
{
    // 左电机方向控制：PB8和PB9
    GPIO_SetBits(GPIOB, GPIO_Pin_8);
    GPIO_ResetBits(GPIOB, GPIO_Pin_9);
    
    // 右电机方向控制：PB10和PB11
    GPIO_SetBits(GPIOB, GPIO_Pin_10);
    GPIO_ResetBits(GPIOB, GPIO_Pin_11);
    
    motor_set_speed(left_speed, right_speed);
}

void motor_stop(void)
{
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare1(TIM1, 0);  // 改为TIM1的比较寄存器
    
    // 停止所有电机方向引脚
    GPIO_ResetBits(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
}
