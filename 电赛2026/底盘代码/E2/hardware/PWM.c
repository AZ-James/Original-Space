// PWM.c - 修正版本
#include "stm32f10x.h"
#include "pwm.h"

// 注意：PA5对应TIM2_CH1，使用通道1

void PWM_Init(void)
{
    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 2. 配置PA5为复用推挽输出（PWM输出）
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      // 复用推挽输出
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;           // 重要：改为PA5
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 3. 配置TIM2时基
    TIM_InternalClockConfig(TIM2);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;      // ARR = 100，PWM分辨率100
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1;   // PSC = 720，72MHz/720=100kHz，100kHz/100=1kHz PWM频率
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
    
    // 4. 配置PWM输出通道（通道1对应PA5）
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);  // 先填充默认值
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比为0
    
    // 重要：使用OC1Init，因为PA5是TIM2_CH1
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    
    // 5. 使能预装载和定时器
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

// 重要：改为SetCompare1，对应通道1
void PWM_SetCompare1(uint16_t Compare)
{
    TIM_SetCompare1(TIM2, Compare);
}
