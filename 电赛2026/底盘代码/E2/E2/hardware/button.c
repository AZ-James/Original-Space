#include "button.h"
#include "delay.h"
#include "stm32f10x.h"
void button_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t button_pressed(void)
{
    static uint8_t last_state = 1;
    uint8_t current = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
    
    if(last_state == 1 && current == 0)  // 下降沿检测
    {
        Delay_ms(20);  // 消抖
        if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12) == 0)
        {
            last_state = current;
            return 1;
        }
    }
    
	
	last_state = current;  // 必须更新状态！
    return 0; 
}
