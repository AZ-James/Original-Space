#ifndef _SENSOR_H
#define _SENSOR_H

//告诉主函数有这个数组在Sensor.h中
extern uint16_t AD_Value[8];

void Sensor_AD_Init(void);
void AD_ALL_GetValue(uint16_t *AD_Value);
void Get_All_Sensor_Digital(float *Digital_Value);
float Caculate_Line_Deviation(float *Digital_Value,uint16_t *Digital_State);


#endif
