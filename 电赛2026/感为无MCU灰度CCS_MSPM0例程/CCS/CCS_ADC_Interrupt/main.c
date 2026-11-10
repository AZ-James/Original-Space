#include "ti_msp_dl_config.h"
#include "delay.h"
#include "stdio.h"
#include "Uart.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
unsigned short Anolog[8]={0};
unsigned short white[8]={1800,1800,1800,1800,1800,1800,1800,1800};
unsigned short black[8]={300,300,300,300,300,300,300,300};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
/********************************************No_Mcu_Demo*******************************************/
/*****************芯片型号 MSPM0G3507 主频80Mhz ***************************************************/
/*****************引脚 AD0:PB0 AD1:PB1 AD2:PB2  !!!严格按照该顺序接线，接反或接错都会导致数据错误*****/
/*****************OUT PA27*************************************************************************/
/*****************串口 Tx PA10 Rx PA11 ************************************************************/
/*****************传感器供电需要5V电压稳定供电，否则可能无法正常使用***************************/
/*****************保证单片机和传感器共地，如果不共地无法正常使用***********************************/
/********************************************No_Mcu_Demo*******************************************/

//初始化
No_MCU_Sensor sensor;
unsigned char Digtal;

int main(void)
{
    SYSCFG_DL_init();
	
		//根据黑白校准值初始化传感器
		No_MCU_Ganv_Sensor_Init(&sensor,white,black);
		//开启ADC中断
 	    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
	
		Tick_delay(100);
	
		while (1) {
			//无时基传感器常规任务，包含模拟量，数字量，归一化量
			No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
			//获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
			Digtal=Get_Digtal_For_User(&sensor);
			sprintf((char *)rx_buff,"Digtal %d-%d-%d-%d-%d-%d-%d-%d\r\n",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			
			//获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
			if(Get_Anolog_Value(&sensor,Anolog)){
			sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			}
			
			//获取传感器归一化结果(只有当有黑白值传入进去了之后才会有这个值！！有黑白值初始化后返回1 没有返回 0)
			if(Get_Normalize_For_User(&sensor,Normal)){
			sprintf((char *)rx_buff,"Normalize %d-%d-%d-%d-%d-%d-%d-%d\r\n",Normal[0],Normal[1],Normal[2],Normal[3],Normal[4],Normal[5],Normal[6],Normal[7]);
			uart0_send_string((char *)rx_buff);
			memset(rx_buff,0,256);
			}
			Tick_delay(1);
		}
}