#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint16_t AD_Value[8];




void Sensor_AD_Init(void)
{
	//1.打开时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);  //72MHZ/6=12MHZ
	
	
	//2.配置PA3
	GPIO_InitTypeDef GPIO_InitStruture;
	//模拟输入模式
	GPIO_InitStruture.GPIO_Mode=GPIO_Mode_AIN;
	GPIO_InitStruture.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruture.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruture);
	
	//配置PA4-PA6，要输出高低电平
	GPIO_InitStruture.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStruture.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStruture.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruture);
	
	
	//3.选择规则组的输入通道
	//最后一个数据越大越稳定，越小越快（选了PA0）
	ADC_RegularChannelConfig(ADC1,ADC_Channel_3,1,ADC_SampleTime_239Cycles5);
	
	
	//4.初始化ADC
	ADC_InitTypeDef ADC_InitStructure;
	//工作模式（独立模式，ADC1和ADC2互不干扰）
	ADC_InitStructure.ADC_Mode=ADC_Mode_Independent;
	//数据对齐（右对齐）
	ADC_InitStructure.ADC_DataAlign=ADC_DataAlign_Right;
	//外部触发转换选择（控制的触发源）
	//因为这里软件触发，所以选择不需要
	ADC_InitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
	//连续转换模式（单次转换）
	ADC_InitStructure.ADC_ContinuousConvMode=DISABLE;	
	//扫描转换模式（非扫描）
	ADC_InitStructure.ADC_ScanConvMode=DISABLE;
	//通道数目(只有一个通道）
	ADC_InitStructure.ADC_NbrOfChannel=1;
	ADC_Init(ADC1,&ADC_InitStructure);
	
	
	//5.中断和看门狗
	
	
	//6.启动ADC
	ADC_Cmd(ADC1,ENABLE);
	
	//7.校准（建议）
	//开始复位校准
	ADC_ResetCalibration(ADC1);
	//等待复位校准完成再继续（1就是在校准过程中）
	while (ADC_GetResetCalibrationStatus(ADC1)==SET);
	//启动校准
	ADC_StartCalibration(ADC1);
	//等待校准完成
	while (ADC_GetCalibrationStatus(ADC1)==SET);
	
}


/////////////////////////////////////////////

void Reset_Analog_Mux(void)
{
//    // 将所有地址线短暂置高再置低
//    GPIO_SetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
//    Delay_us(100);
//    GPIO_ResetBits(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3);
//    Delay_us(500);
}

void AD_ALL_GetValue(uint16_t *AD_Value)
{
	//000
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[0]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	
	//001
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[1]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//010
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[2]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//011
	GPIO_ResetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[3]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//100
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	
	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[4]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//101
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);
 
	
	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[5]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//110
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[6]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
	
	//111
	GPIO_SetBits(GPIOA,GPIO_Pin_6);
	GPIO_SetBits(GPIOA,GPIO_Pin_5);
	GPIO_SetBits(GPIOA,GPIO_Pin_4);

	
	//软件触发转换
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	//获取标志位状态
	//规则组转换完成（未完成则一直循环）
	while (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
	//获取转换值
	AD_Value[7]=ADC_GetConversionValue(ADC1);
	Reset_Analog_Mux();
}






/////////////////////////////////////////////

//接下来是模拟量转数字量

// 每个传感器的校准范围（你需要自己在比赛前标定）
// 红色时最小，白色时最大
uint16_t Sensor_Min[8] = {800,800,800,800,800,800,800,800};
uint16_t Sensor_Max[8] = {3000,3000,3000,3000,3000,3000,3000,3000};

//获取归一化之后的数字量
void Get_All_Sensor_Digital(float *Digital_Value)
{
    AD_ALL_GetValue(AD_Value);  // 获取 8 路 AD

    for(int i = 0; i < 8; i++)
    {
        float norm = (float)(Sensor_Max[i] - AD_Value[i]) / 
                     (float)(Sensor_Max[i] - Sensor_Min[i]);

        // 限幅到 0~1
        if(norm < 0) norm = 0;
        if(norm > 1) norm = 1;

        Digital_Value[i] = norm;
    }
}




////////////////////////////////////////////////////////

//接下来的函数是计算偏差值
//用于后续PID计算

float Caculate_Line_Deviation(float *Digital_Value,uint16_t *Digital_State)
{
    static float last_deviation = 0;


    // 传感器权重
    const int32_t Position_Weight[8] = {-3500, -2500, -1500, -500, 500,  1500,  2500,  3500};

    int32_t numerator = 0;
    int32_t denominator = 0;

    for(int i = 0; i < 8; i++)
    {
        float val = Digital_Value[i];
        Digital_State[i]= 2;   // 2 表示忽略

        if(val > 0.5)
        {
            Digital_State[i]=1;   // 红线（有效）
        }
        else if(val < 0.5)
        {
            Digital_State[i]=0;   // 白底（无效，不计入）
        }
        else
        {
            Digital_State[i]=2;   // 噪声区，不参与
        }

        // 只对有效线（红线=1）参与计算
        if(Digital_State[i] == 1)
        {
            numerator += Position_Weight[i];
            denominator += 1;
        }
    }

    // ● 正常情况：能看到线
    if(denominator != 0)
    {
        float deviation = numerator / (float)denominator;
        last_deviation = deviation;
        return deviation/3500.0f;
    }
	else{return last_deviation;}
}





