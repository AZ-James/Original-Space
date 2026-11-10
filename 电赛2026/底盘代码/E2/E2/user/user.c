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



    while(1)
    {
        
//		if(button_pressed())
//        {
            buzzer_on();
			Delay_ms(1000);
			buzzer_off();
			
			g_finish_detected = 0;
            

			timer_control(1);
            // 进入循迹循环
            
        
		 
		while(g_finish_detected == 0)
			{
                Delay_ms(10);  // 小延时避免CPU过载
            }
            
            // 停车后处理
            timer_control(0);  // 停止定时器
            motor_stop();      // 确保电机停止
            
            buzzer_on();
            Delay_ms(1000);
            buzzer_off();
            
            // 停车后死循环
            while(1);
         }
	}
//}
