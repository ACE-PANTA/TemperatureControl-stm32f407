#include "hmi.h"
#include "app_config.h"
#include "delay.h"
#include "ds18b20.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t u3_sendbuf[15];
uint8_t Tempbuf_data[5];

//串口屏对应串口3初始化
void Uart_HMI_Init(uint32_t bound)
{
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //使能GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟

	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8复用为USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD9复用为USART3

	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9; //GPIOD8，9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure); //初始化PD9，PD8

	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//发送模式
	USART_Init(USART3, &USART_InitStructure); //初始化串口3

	USART_Cmd(USART3, ENABLE);  //使能串口3 

}
/*
	* @name   usart3_send
	* @brief  串口3发送数据函数
	* @param  data -> 内存首地址
			  len  -> 内存长度   
	* @retval None      
*/
void usart3_send(unsigned char *data,unsigned char len)
{
	unsigned char t;
	for(t=0;t<len;t++)
	{	
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
		USART_SendData(USART3,data[t]); 				
	}
}

/*
	* @name   Memory_Set
	* @brief  内存清除函数
	* @param  pucBuffer -> 内存首地址
			  LEN       -> 内存长度   
	* @retval None      
*/
void Memory_Clr(uint8_t* pucBuffer,uint16_t LEN)
{
	uint16_t i;
	
	for(i=0;i<LEN;i++)
	{
		*(pucBuffer+i) = (uint8_t)0;
	}
}
/*
	* @name   HMI_Send_txt
	* @brief  向串口屏文本控件发送函数
	* @param  ch		-> 文本控件对应的标号
			  data      -> 文本控件对应的字符串Step->1 5 10 Mode-> A M   
	* @retval None      
*/
void HMI_Send_txt(uint8_t ch,uint8_t data)
{
	uint8_t send_len;

	u3_sendbuf[0]=0x74;
	if(ch==0)
		u3_sendbuf[1]=0x37;
	else if(ch==1)
		u3_sendbuf[1]=0x38;
	
	u3_sendbuf[2]=0x2E;
	u3_sendbuf[3]=0x74;
	u3_sendbuf[4]=0x78;
	u3_sendbuf[5]=0x74;
	u3_sendbuf[6]=0x3D;
	u3_sendbuf[7]=0x22;
	if(ch==0)
	{
		if(data<10)
		{
			u3_sendbuf[8]=data+0x30;
			u3_sendbuf[9]=0x22;
			u3_sendbuf[10]=0xFF;
			u3_sendbuf[11]=0xFF;
			u3_sendbuf[12]=0xFF;
			send_len = 13;
		}
		else
		{
			u3_sendbuf[8] = 0x31;
			u3_sendbuf[9] = 0x30;
			u3_sendbuf[10]=0x22;
			u3_sendbuf[11]=0xFF;
			u3_sendbuf[12]=0xFF;
			u3_sendbuf[13]=0xFF;
			send_len = 14;
		}
	}
		
	else if(ch==1)
	{
		if(data==0)
			u3_sendbuf[8]=0x41;
		else if(data==1)
			u3_sendbuf[8]=0x4D;
		u3_sendbuf[9]=0x22;
		u3_sendbuf[10]=0xFF;
		u3_sendbuf[11]=0xFF;
		u3_sendbuf[12]=0xFF;
		send_len = 13;
	}
	usart3_send(u3_sendbuf, send_len);
	Memory_Clr(u3_sendbuf, send_len);
}

/*
	* @name   HMI_Send_Float
	* @brief  向串口屏浮点控件发送函数
	* @param  ch		-> 浮点控件对应的标号 x0 x1 x2 x3
			  data      -> 浮点控件对应的字符串 
			  len		-> 字符串长度
	* @retval None      
*/
void HMI_Send_Float(uint8_t ch,uint8_t *data,uint8_t len)
{
	uint8_t i;
	uint8_t send_len;
	
	u3_sendbuf[0]=0x78;
	if(ch==0)
		u3_sendbuf[1]=0x30;
	else if(ch==1)
		u3_sendbuf[1]=0x31;
	else if(ch==2)
		u3_sendbuf[1]=0x32;
	else if(ch==3)
		u3_sendbuf[1]=0x33;
	u3_sendbuf[2]=0x2E;
	u3_sendbuf[3]=0x76;
	u3_sendbuf[4]=0x61;
	u3_sendbuf[5]=0x6C;
	u3_sendbuf[6]=0x3D;
	for(i=0;i<len;i++)
	{
		u3_sendbuf[7+i]=data[i];
	}
	u3_sendbuf[7+len]=0xFF;
	u3_sendbuf[8+len]=0xFF;
	u3_sendbuf[9+len]=0xFF;
	send_len = (uint8_t)(10 + len);
	
	usart3_send(u3_sendbuf, send_len);
	Memory_Clr(u3_sendbuf, send_len);
}

/*
	* @name   HMI_Cle
	* @brief  清除曲线控件数据函数
	* @param  ch		-> 0：炉温曲线 1：PWM曲线 只在初始化时调用，复位清除曲线
	* @retval None      
*/
void HMI_Cle(uint8_t ch)
{
    uint8_t send_len;

    u3_sendbuf[0] = 0x63;
    u3_sendbuf[1] = 0x6C;
    u3_sendbuf[2] = 0x65;
    u3_sendbuf[3] = 0x20;
    u3_sendbuf[4] = 0x73;
    u3_sendbuf[5] = 0x30;
    u3_sendbuf[6] = 0x2E;
    u3_sendbuf[7] = 0x69;
    u3_sendbuf[8] = 0x64;
    u3_sendbuf[9] = 0x2C;
    if(ch==0)
    {
        u3_sendbuf[10] = 0x30;
    }
    else if(ch == 1)
    {
        u3_sendbuf[10] = 0x31;
    }
    u3_sendbuf[11] = 0xFF;
    u3_sendbuf[12] = 0xFF;
    u3_sendbuf[13] = 0xFF;
    send_len = 14;

    usart3_send(u3_sendbuf, send_len);
    Memory_Clr(u3_sendbuf, send_len);
}

extern int my_goal;
extern int my_pwm;
extern uint8_t Uint_pwm[5];
extern uint8_t Uint_Goal[5];
extern int   temp_ctr_val;						//存储温度的当前给定控制量 ±100，正加热，负散热

/*
	* @name   HMI_init
	* @brief  串口屏初始化函数
	* @param  None
	* @retval None      
*/
void HMI_init(void)
{
    HMI_Send_txt(0,1);        //文本控件，步进值，默认是1
    HMI_Send_txt(1,1);        //文本控件，模式，默认是手动M
    HMI_Cle(0);               //清除曲线通道0，炉温曲线
    HMI_Cle(1);               //清除曲线通道1，PWM曲线
    my_pwm = temp_ctr_val;    //读取当前PWM值
    sprintf((char*)Uint_pwm, "%d", my_pwm);
    HMI_Send_Float(2, Uint_pwm, strlen((const char*)Uint_pwm));
    my_goal = (int)(g_config.target_temp * 10.0f + (g_config.target_temp >= 0.0f ? 0.5f : -0.5f));
    sprintf((char*)Uint_Goal, "%d", my_goal);
    HMI_Send_Float(3, Uint_Goal, strlen((const char*)Uint_Goal));
}



