#include "adc.h"
#include "delay.h"		 
	   
//正点原子@ALIENTEK


//单位Ω
const u16 T_R_Table[111]=
	   {51815,49283,46890,44625,42481,40451,38526,36702,34971,33329,
		31770,30253,28816,27453,26161,24935,23772,22668,21621,20626,
		19682,18785,17933,17123,16353,15621,14925,14264,13634,13035,
		12465,11923,11406,10915,10446,10000, 9575, 9170, 8784, 8416,
		 8064, 7730, 7410, 7106, 6815, 6538, 6273, 6020, 5778, 5548,
		 5327, 5117, 4915, 4723, 4539, 4363, 4195, 4034, 3880, 3733,
		 3592, 3457, 3328, 3204, 3086, 2972, 2863, 2759, 2659, 2564,
		 2472, 2384, 2299, 2218, 2141, 2066, 1994, 1926, 1860, 1796,
		 1735, 1677, 1621, 1567, 1515, 1465, 1417, 1371, 1326, 1284,
		 1243, 1203, 1165, 1128, 1093, 1059, 1027,  996,  965,  936,
		  908,  881,  855,  830,  805,  782,  759,  737,  715,  695,
		  674};	

//初始化ADC															   
void  Adc_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

	//先初始化ADC1通道0IO口
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PA0 通道0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//初始化

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化

	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
}				  
//获得ADC值
//ch: ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
 	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
	 
u16 res_Now=0;
u16 temp_adc=0;
u16 Get_Resistance()
{
	temp_adc = Get_Adc_Average(ADC_Channel_0,10);//得到电压数字量（0-4095）
//	res_Now=10*((float)temp/4096.0f*3000.0f);//
	res_Now = 10000.0f*((float)temp_adc/(4096.0f-(float)temp_adc));//根据电压得到电阻值，10000.0f代表10KΩ
	return res_Now;
}

//查表法找到电阻值对应的温度
float temper_Now=0.0;
float Get_Temperature(u16 resistance)
{
	u16 i=0;
	while(i<111)
	{
		if( (resistance<=T_R_Table[i]) && (resistance>=T_R_Table[i+1]) )
		{
			break;
		}
		i++;
	}
	temper_Now=((float)i - 10.0f + 0.6f ) + (1.0f - ((float)(resistance - T_R_Table[i+1]) / (float)(T_R_Table[i] - T_R_Table[i+1])) );
	return temper_Now;
}

