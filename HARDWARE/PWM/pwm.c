#include "pwm.h"

////TIM9 PWM部分初始化 
////PWM输出初始化
////arr：自动重装值
////psc：时钟预分频数
void TIM9_PWM_Init(void)
{		 					 
	//此部分需手动修改IO口设置
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  		//TIM9时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE); 		//使能GPIOE时钟	
	
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9); 		//PE6复用为定时器9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;        //PE6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        		//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      		//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;       		//不带上下拉
	GPIO_Init(GPIOE,&GPIO_InitStructure);             		    //初始化PE6
	  
	TIM_TimeBaseStructure.TIM_Prescaler=840-1;  				//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 	//向上计数模式
	TIM_TimeBaseStructure.TIM_Period=100-1;   					//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseStructure);				//初始化定时器9
	
	//初始化TIM9 Channel1  Channel2  PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; 			//选择定时器模式:TIM脉冲宽度调制模式2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; 	//输出极性:TIM输出比较极性低
//	TIM_OCInitStructure.TIM_Pulse =0; 							// 初始占空比
//	TIM_OC1Init(TIM9, &TIM_OCInitStructure);  					//根据T指定的参数初始化外设TIM1 4OC1

//	TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable); 			 //使能TIM14在CCR1上的预装载寄存器
	
	TIM_OCInitStructure.TIM_Pulse =0; 							// 初始占空比
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM9,ENABLE);//ARPE使能 

	TIM_Cmd(TIM9, ENABLE);  //使能TIM9	

//	TIM_SetCompare1(TIM9,100);						//设置通道1占空比
//	TIM_SetCompare2(TIM9,100);
}  




