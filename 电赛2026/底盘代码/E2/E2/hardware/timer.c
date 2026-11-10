#include "stm32f10x.h"
#include "timer.h"
#include "linefollowing.h"
#include "motor.h"
// 定时器初始化（1ms中断）
volatile uint8_t g_finish_detected = 0; 

void timer_init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能TIM4时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 定时器配置: 72MHz / 720 = 100kHz, 100kHz / 100 = 1kHz (1ms)
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 720 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 清除更新中断标志
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    
    // 配置NVIC
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能定时器更新中断
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}


void timer_control(uint8_t enable)
{
    if(enable)
    {
        TIM_Cmd(TIM4, ENABLE);
    }
    else
    {
        TIM_Cmd(TIM4, DISABLE);
    }
}


void TIM4_IRQHandler(void)
{
    static uint8_t counter = 0;
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
        
        // 如果已停车，禁用定时器
        if(g_finish_detected) {
            TIM_Cmd(TIM4, DISABLE);
            return;
        }
        
        counter++;
        if(counter >= 10)  // 10ms一次循迹（100Hz）
        {
            counter = 0;
            
            // 调用循迹函数（包含停车判断）
            line_following_task();
        }
    }
}
