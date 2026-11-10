#include "ADC.h"
#include "string.h"
uint16_t ADC_VALUE[40];
//读取ADC的数据
unsigned int adc_getValue(unsigned int number)
{
	//等待DMA搬运完成新通道的ADC数据
	memset((uint16_t*)ADC_VALUE, 0, sizeof(ADC_VALUE));
	__WFI(); 
	while(ADC_VALUE[number-1]==0){
		 __WFI(); 
	};
        unsigned int gAdcResult = 0;
        unsigned char i = 0;
        //采集多次累加
        for( i = 0; i < number; i++ )
        {
                gAdcResult += ADC_VALUE[i];
        }
        //均值滤波
        gAdcResult /= number;

        return gAdcResult;
}
