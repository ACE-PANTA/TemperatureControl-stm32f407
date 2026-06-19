#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "ds18b20.h"
#include "key.h"
#include "beep.h"
#include "timer.h"
#include "adc.h"
#include "hmi.h"
#include "pwm.h"
#include "DataScope_DP.h"
#include "flash_params.h"
#include "app_config.h"
#include "app_comm.h"
#include "app_input.h"
#include "pid_control.h"
#include "eth.h"
#include <stdio.h>
#include <string.h>

static void Beep_StartupTone(void);

uint16_t temperature = 0;
float temp_feedback = 0.0f;
int   temp_ctr_val;
float mytemp_goal = 30.0f;
float MainBoard_temp;

u8 Manual_Flag = 1;
u8 Step_Value = 1;

uint8_t temp_L[5];
uint8_t Uint_Goal[5];
uint16_t L_temp;
int my_goal;
int my_pwm;
uint8_t Uint_pwm[5];

extern uint16_t timer_send;

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	delay_init(168);
	uart_init(115200);
	Uart_HMI_Init(115200);
	LED_Init();
	BEEP_Init();
	Beep_StartupTone();
	KEY_Init();
	Heat_io_Init();
	TIM9_PWM_Init();
	Fan_io_Init();
	DS18B20_Init();
	TIM3_Init(1000 - 1, 84 - 1);

	delay_ms(500);
	AppConfig_Init();
	HMI_init();
	HMI_Refresh_AllConfig();

	if (Eth_Init())
		Net_InitCallbacks();

	Adc_Init();

	BEEP = 1;
	delay_ms(1000);
	BEEP = 0;

	while (1)
	{
		delay_ms(2);
		Uart_Command_Process();
		Eth_Process();
		Key_Process(KEY_Scan(0));
		AppConfig_Process();

		if (timer_send > 1000)
		{
			timer_send = 0;
			temp_feedback = Get_Temperature(Get_Resistance());

			if (g_config.manual_flag == 0)
				My_PID_Ctr();

			L_temp = temp_feedback * 10;
			temperature = DS18B20_Get_Temp();
			MainBoard_temp = temperature * 0.1f;
			my_pwm = temp_ctr_val;

			sprintf((char *)Uint_pwm, "%d", my_pwm);
			HMI_Send_Float(2, Uint_pwm, strlen((const char *)Uint_pwm));

			sprintf((char *)Tempbuf_data, "%d", temperature);
			sprintf((char *)temp_L, "%d", L_temp);
			HMI_Send_Float(1, temp_L, strlen((const char *)temp_L));
			HMI_Send_Float(0, Tempbuf_data, strlen((const char *)Tempbuf_data));

			SendData_Uart1();
			Uart_Send_WorkPhase();
			Net_BroadcastState();
			Net_BroadcastPhase();
		}

		if (Manual_Flag != 0)
			My_Ctr(temp_ctr_val);
	}
}

static void Beep_StartupTone(void)
{
	BEEP = 1;
	delay_ms(200);
	BEEP = 0;
	delay_ms(200);
	BEEP = 1;
	delay_ms(200);
	BEEP = 0;
	delay_ms(200);
	BEEP = 1;
	delay_ms(200);
	BEEP = 0;
	delay_ms(200);
}
