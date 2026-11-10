#include "stm32f10x.h"
#include "linefollowing.h"
#include "motor.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "adc.h"
extern No_MCU_Sensor sensor;
extern volatile uint8_t g_finish_detected;
// PID参数
float Kp = 15.0;
float Kd = 2.0;

float last_error = 0;
void line_following_task(void)
{

    unsigned short Anolog[8];
    unsigned char Digtal;
    //读取灰度
    No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
    Digtal = Get_Digtal_For_User(&sensor);
    Get_Anolog_Value(&sensor, Anolog);
    static uint8_t finish_counter = 0;
    int black_count = 0;
    for(int i=0;i<8;i++)
    {
        if(((Digtal>>i)&0x01)==0)
        {
            black_count++;
        }
    }

    // 6个以上检测黑线认为终点
    if(black_count >= 6)
    {
        finish_counter++;
        if(finish_counter >= 3)
        {
            motor_stop();
            g_finish_detected = 1;
            return;
        }

    }
    else
    {
        finish_counter = 0;
    }
    if(g_finish_detected)
    {
        motor_stop();
        return;
    }

    /***************PID循迹****************/
    float error = 0;
    int count = 0;
    for(int i=0;i<8;i++)
    {
        //黑线
        if(((Digtal>>i)&0x01)==0)
        {
            error += (i - 3.5);
            count++;
        }
    }
    //正常检测到线
    if(count > 0)
    {
        error = error / count;
    }
    else
    {
        //丢线保持方向
        error = last_error;
    }
    //PID计算
    float output;
    output = Kp * error + 
             Kd * (error-last_error);
    last_error = error;
    //限制PID输出
    if(output > 50)
        output = 50;
    if(output < -50)
        output = -50;
    /***************速度计算****************/
    int16_t base_speed = 15
	;
    int16_t left_speed;
    int16_t right_speed;
    /*
    error <0:
    黑线在左边
    error >0:
    黑线在右边
    */
    left_speed  = base_speed + output;
    right_speed = base_speed - output;
    //速度限制
    if(left_speed > 60)
        left_speed = 60;
    if(left_speed < 0)
        left_speed = 0;
    if(right_speed > 60)
        right_speed = 60;
    if(right_speed < 0)
        right_speed = 0;
    motor_forward(left_speed,right_speed);

}