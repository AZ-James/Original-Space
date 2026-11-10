#include "stm32f10x.h"
#include "button.h"
#include "motor.h"
#include "adc.h"
#include "gpio.h"
#include "timer.h"
#include "Delay.h"
#include "linefollowing.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "buzzer.h"

No_MCU_Sensor sensor;
unsigned short black[8] = {75,80,133,126,502,198,465,141};
unsigned short white[8] = {790,737,827,829,1056,897,1084,815};
unsigned char Digtal;
extern volatile uint8_t g_finish_detected;
extern volatile uint8_t g_line_follow_flag;

int main(void)
{
  
    SystemInit();
	gpio_init();
    button_init();
    motor_init();  
	buzzer_init();
    adc_init();
    timer_init();
	Delay_Init();
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    // TIM4初始化

    // 开启TIM4定时器
    

    while(1)
    {

        // 主循环保持空闲
        // 循迹由TIM4中断10ms调用
       timer_control(1);
        if(g_finish_detected)
        {
            motor_stop();
        }

    }

}