#include "timer.h"
#include "led.h"
	   
//正点原子@ALIENTEK

//TIM3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz，此处是84MHz，对APB1自动2倍频，即42Mhz*2=84Mhz
//这里使用的是定时器3!

//定时时间1ms
void TIM3_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟

	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern int Heat_PWM;//控制量u，加热占空比
uint16_t Timer_cnt;
u16 intNumber=1;
void MY_HEAT()
{

	intNumber++;  //每10ms，intNumber++
	if(intNumber>200)//2000ms一次控制周期
	{
		intNumber=0;
	}
	//2S内，Heat_PWM%的时间在加热，其余不加热
	if((Heat_PWM>0) && (Heat_PWM<=100)&& (intNumber <= (2*Heat_PWM)))
	{
		FAN1=1;//内循环风扇一直开启
		HEAT=0;//加热
		HEAT_LED = 0;//加热指示灯亮的时间=加热的时间
	}
	else
	{
		FAN1=1;//内循环风扇一直开启
		HEAT=1;//不加热
		HEAT_LED = 1;//加热指示灯灭的时间=不加热的时间
	}
}

uint16_t timer_send;
uint16_t Timer_run;
//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{	
		Timer_cnt++;
		timer_send++;
		Timer_run++;
		if(Timer_cnt>=10)  //每10ms，加热丝控制
		{
			Timer_cnt = 0;
			MY_HEAT();							
		}
		if(Timer_run>=500) //运行指示灯每500ms亮灭翻转
		{
			Timer_run=0;
			RUN_LED=!RUN_LED;					
		}
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
