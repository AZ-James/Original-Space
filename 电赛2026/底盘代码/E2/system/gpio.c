#include "stm32f10x.h"
#include "gpio.h"

void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	
  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_WriteBit(GPIOB,GPIO_Pin_0|GPIO_Pin_1,Bit_RESET);
	GPIO_WriteBit(GPIOA,GPIO_Pin_7,Bit_RESET);
    
    // 电机方向控制引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 电机驱动使能（STBY）- PB5
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_SetBits(GPIOB, GPIO_Pin_5);
    
    // 蜂鸣器
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 启动按键
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 初始化状态
    GPIO_ResetBits(GPIOB, GPIO_Pin_10| GPIO_Pin_11 | GPIO_Pin_8 | GPIO_Pin_9);
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
}
