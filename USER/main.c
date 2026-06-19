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
#include "eth.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void Key_Process(u8 KeyState);
void My_PID_Ctr(void);
void My_Ctr(int heat);
void Uart_Command_Process(void);
void Net_Command_Dispatch(const char *body, const char *value);
void Net_BroadcastState(void);
void Net_BroadcastPhase(void);
void App_Reset_ControlState(u8 clear_output);
static void PID_SyncIntegralToOutput(float target_output, float error, float derivative);
static void PID_Fine_SyncPosBase(float current_output, float error);
static void Net_RxCallback(const u8 *data, u16 len);
static void Flash_Param_Refresh_Hmi(void);
static u8 Uart_Get_Checksum(const char *buf, u16 len);
static u8 Uart_Parse_Hex_Byte(const char *buf, u8 *value);
static u8 Uart_Check_And_Strip_Frame(char *buf);
const char *App_Get_WorkPhase(void);
void Uart_Send_Frame(const char *payload);
void Uart_Send_Ack(u8 ok);
static void Uart_Send_WorkPhase(void);


/* PID state now managed entirely in My_PID_Ctr() with g_config params */

/* 网络: TCP 接收缓冲�? */
static char g_net_rx_buf[512];
static u16  g_net_rx_len = 0;

/* ============================================================
 * Dual-Mode PID Controller (Independent Parameters)
	 *
	 *   Mode 1 - Transient (Positional PID):
	 *     u = tran_kp*e + tran_ki*integral(e) + tran_kd*de/dt
	 *     Interval: tran_interval (default 3s)
	 *     Integral separation threshold: tran_sep_threshold
	 *     Adaptive integral: min_scale/full_error/limit/overshoot_leak.
	 *
	 *   Stability Detection (stable_window sliding window):
	 *     If max-min <= stable_delta -> "nearly stable".
	 *
	 *   Mode 2 - Fine-tuning (Incremental PID):
	 *     Activated: |error| <= fine_entry_max.
	 *     delta_u = fine_kp*de + fine_ki*e*Ts + fine_kd*dde/Ts
	 *     Interval: fine_interval (default 8s)
	 *     Constrained: +/- fine_range (default 5%%)
	 *     Exits when |error| > PID_FINE_EXIT_GAIN * fine_entry_max.
	 * ============================================================ */

static float g_inc_err[3];             /* e(k), e(k-1), e(k-2) */
static float g_pid_output  = 0.0f;     /* previous final output */
static float g_fine_inc_output = 0.0f; /* fine-mode incremental PID output */
static float g_fine_pos_output = 0.0f; /* fine-mode positional PID output */
static float g_fine_pos_base = 0.0f;   /* fine-mode positional output baseline */
static float g_fine_pos_integral = 0.0f; /* fine-mode positional correction integral */
static float g_integral    = 0.0f;     /* integral accumulator */
static float g_error_filt  = 0.0f;     /* low-pass filtered error for near-target control */
static u8    g_error_filt_valid = 0;
static u8    g_pid_output_valid = 0;
static u16  g_tran_tick    = 0;        /* transient interval counter */
static u16  g_fine_tick    = 0;        /* fine-tuning interval counter */
static u8   g_fine_mode    = 0;        /* 1=incremental fine-tuning */

#define PID_OUTPUT_MAX      100.0f
#define PID_OUTPUT_MIN        0.0f
#define PID_FINE_OUTPUT_MIN   0.0f
#define PID_ERROR_FILTER_ALPHA 0.25f
#define PID_FINE_EXIT_GAIN  2.5f
#define PID_FINE_INC_WEIGHT_MIN 0.25f
#define PID_FINE_INC_WEIGHT_MAX 0.75f
#define PID_FINE_POS_WINDOW_GAIN 2.0f
#define PID_FINE_POS_INTEGRAL_LIMIT 30.0f
#define PID_FINE_POS_INTEGRAL_LEAK 0.90f
#define FAN_PWM_MIN         15
#define FAN_PWM_MAX         95

#define STABLE_WINDOW_MAX  120
static float g_temp_hist[STABLE_WINDOW_MAX];
static u8    g_hist_idx  = 0;
static u16   g_hist_count = 0;

uint16_t temperature = 0; 				//�����¶�

float temp_feedback=0.0f;				//�洢��ǰ�¶ȷ�����
int   temp_ctr_val;						//�洢�¶ȵĵ�ǰ���������� ��100�������ȣ���ɢ��
float mytemp_goal=30.0f;				//�Զ�ģʽ��Ŀ���¶�   Ĭ��30��
float uart_set_pid_kp = 1.1546f;
float uart_set_pid_ki = 0.0054f;
float uart_set_pid_kd = 0.0f;
int Heat_PWM;							//����PWMֵ
float MainBoard_temp;

u8 Manual_Flag=1;						//���ģʽ��?0=Auto�Զ�; 1=Man�ֶ�
u8 Step_Value=1;						//���������Ĳ���ֵ

uint8_t temp_L[5];						//�洢��·���¶��ַ�������
uint8_t Uint_Goal[5];					//�洢�Զ�ģʽ��Ŀ���¶��ַ�������

uint16_t L_temp;						//���͸����������¶�ֵ���������ɼ����¶�ֵ����10��ȡ��

int my_goal;							//���͸����������Զ�ģʽ�趨�¶�ֵ
int my_pwm;								//���͸��������ĵ�ǰPWMֵ
uint8_t Uint_pwm[5];					//���ڷ��͸�������PWMֵ����
extern uint16_t timer_send;				//��ʱ������					

int main(void)
{ 
	    
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����ϵͳ�ж����ȼ�����2
	delay_init(168);  								//��ʼ����ʱ����
	uart_init(115200);								//��ʼ�����ڲ�����Ϊ115200��PA9/PA10
	Uart_HMI_Init(115200);							//��ʼ����������PD8/PD9
	LED_Init();										//��ʼ��LED��PB3/PB4
	BEEP_Init();									//��������ʼ��,PB6
	KEY_Init();										//������ʼ��	PD1��2��3��4
	Heat_io_Init();									//���ȿ���IO��ʼ����PE5
	TIM9_PWM_Init();								//����PWM��������TIM9��ͨ��2
	Fan_io_Init();									//��ѭ�����ȿ���IO	PE4
 	DS18B20_Init();									//DS18B20��ʼ��	PB5
	TIM3_Init(1000-1,84-1);							//��ʱ1ms
										
	delay_ms(500);
	AppConfig_Init();
	HMI_init();										//��ʼ������������ʾ����ǰ�趨���¶ȣ�PWM������ֵ�Լ���ģʽ
	Flash_Param_Refresh_Hmi();

	/* ---- 以太网初始化 ---- */
	{
		u8 eth_ok = Eth_Init();
		if (eth_ok)
		{
			/* 注册 TCP 接收回调 */
			Tcp_SetRxCallback(Net_RxCallback);
		}
	}

	Adc_Init();										//ADC1��ʼ��	PA0
	
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
	
	while(1)
	{	
		delay_ms(2);
		Uart_Command_Process();
		Eth_Process();
		Key_Process(KEY_Scan(0));	//�������� ��������
		AppConfig_Process();
		
		if(timer_send>1000)			//��ʱ��1����һ���¶Ȳ�������������
		{
			timer_send = 0;
			temp_feedback = Get_Temperature(Get_Resistance()); //��ȡ����ֵ���㵱ǰ�¶�ֵ
			if(g_config.manual_flag==0)
			{
				My_PID_Ctr();//�Զ�ģʽ�����¶�������������һ��PID���?
			}
			L_temp = temp_feedback*10;							//���͸��������¶�ֵ��
			
			temperature = DS18B20_Get_Temp();					//��ȡ����DS18B20�����¶�
			MainBoard_temp = temperature*0.1;					//���͸�����ʾ�������¶�ֵ
			
			my_pwm = temp_ctr_val;								//��ȡ��ǰPWMֵ
			
			sprintf((char*)Uint_pwm,"%d",my_pwm);				//��PWMֵ���ַ�������ʽд������
			HMI_Send_Float(2,Uint_pwm,strlen((const char*)Uint_pwm));//���͸���������ʾ
			
			sprintf((char*)Tempbuf_data,"%d",temperature);
			sprintf((char*)temp_L,"%d",L_temp);
				
			HMI_Send_Float(1,temp_L,strlen((const char*)temp_L));
			HMI_Send_Float(0,Tempbuf_data,strlen((const char*)Tempbuf_data));
			
			SendData_Uart1();									//����������ʾ����������
			Uart_Send_WorkPhase();
			Net_BroadcastState();
			Net_BroadcastPhase();
		}
		if(Manual_Flag!=0)/*�ֶ�״̬ʱ�Ŀ��Ƴ���*/
		{
			My_Ctr(temp_ctr_val);
		}	
	}
}

static void Flash_Param_Refresh_Hmi(void)
{
	my_pwm = temp_ctr_val;
	my_goal = (int)(g_config.target_temp * 10.0f);
	sprintf((char*)Uint_pwm,"%d",my_pwm);
	sprintf((char*)Uint_Goal,"%d",my_goal);
	HMI_Send_txt(1,g_config.manual_flag);
	HMI_Send_Float(2,Uint_pwm,strlen((const char*)Uint_pwm));
	HMI_Send_Float(3,Uint_Goal,strlen((const char*)Uint_Goal));
}

static u8 Uart_Get_Checksum(const char *buf, u16 len)
{
	u8 checksum = 0;

	while(len > 0)
	{
		checksum ^= (u8)(*buf);
		buf++;
		len--;
	}

	return checksum;
}

static u8 Uart_Parse_Hex_Byte(const char *buf, u8 *value)
{
	u8 high;
	u8 low;

	if((buf == 0) || (value == 0) || (buf[0] == 0) || (buf[1] == 0) || (buf[2] != 0))
	{
		return 0;
	}

	if((buf[0] >= '0') && (buf[0] <= '9'))
		high = (u8)(buf[0] - '0');
	else if((buf[0] >= 'A') && (buf[0] <= 'F'))
		high = (u8)(buf[0] - 'A' + 10);
	else if((buf[0] >= 'a') && (buf[0] <= 'f'))
		high = (u8)(buf[0] - 'a' + 10);
	else
		return 0;

	if((buf[1] >= '0') && (buf[1] <= '9'))
		low = (u8)(buf[1] - '0');
	else if((buf[1] >= 'A') && (buf[1] <= 'F'))
		low = (u8)(buf[1] - 'A' + 10);
	else if((buf[1] >= 'a') && (buf[1] <= 'f'))
		low = (u8)(buf[1] - 'a' + 10);
	else
		return 0;

	*value = (u8)((high << 4) | low);
	return 1;
}

static u8 Uart_Check_And_Strip_Frame(char *buf)
{
	char *star;
	u8 recv_sum;
	u8 calc_sum;

	if((buf == 0) || (buf[0] != '!'))
	{
		return 0;
	}

	star = strchr(buf, '*');
	if(star == 0)
	{
		return 1;
	}

	if(Uart_Parse_Hex_Byte(star + 1, &recv_sum) == 0)
	{
		return 0;
	}

	calc_sum = Uart_Get_Checksum(buf, (u16)(star - buf));
	if(calc_sum != recv_sum)
	{
		return 0;
	}

	*star = 0;
	return 1;
}

void Uart_Send_Frame(const char *payload)
{
	char tx_buf[USART1_CMD_MAX_LEN + 16];
	u8 checksum;

	if(payload == 0)
	{
		return;
	}

	sprintf(tx_buf, "!%s", payload);
	checksum = Uart_Get_Checksum(tx_buf, (u16)strlen(tx_buf));
	sprintf(tx_buf + strlen(tx_buf), "*%02X\r\n", checksum);
	usart1_send_string(tx_buf);
}

void Uart_Send_Ack(u8 ok)
{
	if(ok != 0)
		Uart_Send_Frame("ACK=OK");
	else
		Uart_Send_Frame("ACK=ERR");
}

const char *App_Get_WorkPhase(void)
{
	if(g_config.manual_flag != 0)
	{
		return "MAN";
	}

	if(g_fine_mode != 0)
	{
		return "FINE";
	}

	return "TRAN";
}

static void Uart_Send_WorkPhase(void)
{
	char payload[32];

	sprintf(payload, "PHASE=%s", App_Get_WorkPhase());
	Uart_Send_Frame(payload);
}


void Uart_Command_Process(void)
{
	char cmd_buf[USART1_CMD_MAX_LEN];
	char *split;
	char *body;
	char *value;
	int handled;

	while(usart1_read_line(cmd_buf, sizeof(cmd_buf)) != 0)
	{
		handled = 0;
		if(Uart_Check_And_Strip_Frame(cmd_buf) == 0)
		{
			Uart_Send_Ack(0);
			continue;
		}

		body = &cmd_buf[1];
		split = strchr(body, '=');
		if(split == 0)
		{
			Uart_Send_Ack(0);
			continue;
		}

		*split = 0;
		value = split + 1;

			/* 统一指令分发: 调用 AppCmd_Dispatch (串口来源) */
			g_cmd_from_net = 0;
			handled = AppCmd_Dispatch(body, value);

			if(handled == 1)
			{
				Uart_Send_Ack(1);
			}
			else if(handled == 0)
			{
				Uart_Send_Ack(0);
			}
	}
}
 
//�����¼���������
/* ============================================================
 * 网络: TCP 回复函数
 * ============================================================ */
static void Net_SendFrame(const char *payload)
{
	char tx_buf[256];
	u8   checksum;
	u16  len;

	if (!Tcp_IsConnected() || payload == 0) return;

	sprintf(tx_buf, "!%s", payload);
	len      = (u16)strlen(tx_buf);
	checksum = Uart_Get_Checksum(tx_buf, len);
	sprintf(tx_buf + len, "*%02X\r\n", checksum);
	Tcp_Send((u8 *)tx_buf, (u16)strlen(tx_buf));
}

static void Net_SendAck(u8 ok)
{
	if (ok) Net_SendFrame("ACK=OK");
	else    Net_SendFrame("ACK=ERR");
}

static void Net_SendState(void)
{
	char payload[256];
	int  goal_x10;
	int  feedback_x10;

	goal_x10     = (int)(g_config.target_temp * 10.0f);
	feedback_x10 = (int)(temp_feedback * 10.0f);
	sprintf(payload, "STATE=MODE:%d,PHASE:%s,PWM:%d,GOAL:%d,FB:%d",
	        g_config.manual_flag, App_Get_WorkPhase(), temp_ctr_val, goal_x10, feedback_x10);
	Net_SendFrame(payload);
}

static void Net_SendPhase(void)
{
	char payload[32];

	sprintf(payload, "PHASE=%s", App_Get_WorkPhase());
	Net_SendFrame(payload);
}

static void Net_SendPid(void)
{
	char payload[256];
	sprintf(payload, "PID=KP:%.3f,KI:%.3f,KD:%.3f",
	        uart_set_pid_kp, uart_set_pid_ki, uart_set_pid_kd);
	Net_SendFrame(payload);
}

/* ============================================================
 * 网络: 命令分发
 * ============================================================ */
void Net_Command_Dispatch(const char *body, const char *value)
{
	int handled;
	if (body == 0 || value == 0) return;

	/* 统一指令分发: 调用 AppCmd_Dispatch (网口来源) */
	g_cmd_from_net = 1;
	handled = AppCmd_Dispatch(body, value);

	if (handled == 1)
		Net_SendAck(1);
	else if (handled == 0)
		Net_SendAck(0);
}
static void Net_RxCallback(const u8 *data, u16 len)
{
	u16 copy_len;

	if (g_net_rx_len + len > sizeof(g_net_rx_buf))
	{
		g_net_rx_len = 0;
	}
	copy_len = len;
	if (g_net_rx_len + copy_len >= sizeof(g_net_rx_buf))
		copy_len = sizeof(g_net_rx_buf) - g_net_rx_len - 1;

	memcpy(g_net_rx_buf + g_net_rx_len, data, copy_len);
	g_net_rx_len += copy_len;
	g_net_rx_buf[g_net_rx_len] = 0;

	while (g_net_rx_len > 0)
	{
		char *body;
		char *split;
		char *end;
		u16   frame_end;
		char  frame[512];
		u16   flen;

		body = strchr(g_net_rx_buf, '!');
		if (body == 0)
		{
			g_net_rx_len = 0;
			break;
		}

		if (body != g_net_rx_buf)
		{
			u16 offset = (u16)(body - g_net_rx_buf);
			memmove(g_net_rx_buf, body, g_net_rx_len - offset);
			g_net_rx_len -= offset;
			g_net_rx_buf[g_net_rx_len] = 0;
		}

		end = strstr(g_net_rx_buf, "\r\n");
		if (end == 0) break;

		frame_end = (u16)(end - g_net_rx_buf);
		flen = frame_end;
		if (flen >= sizeof(frame)) flen = sizeof(frame) - 1;
		memcpy(frame, g_net_rx_buf, flen);
		frame[flen] = 0;

		frame_end += 2;
		if (frame_end >= g_net_rx_len)
			g_net_rx_len = 0;
		else
		{
			memmove(g_net_rx_buf, g_net_rx_buf + frame_end,
			        g_net_rx_len - frame_end);
			g_net_rx_len -= frame_end;
			g_net_rx_buf[g_net_rx_len] = 0;
		}

		{
			u8  recv_sum, calc_sum;
			char *star_ptr = strchr(frame, '*');
			if (star_ptr != 0
			    && Uart_Parse_Hex_Byte(star_ptr + 1, &recv_sum))
			{
				calc_sum = Uart_Get_Checksum(frame, (u16)(star_ptr - frame));
				if (calc_sum != recv_sum)
				{
					Net_SendAck(0);
					continue;
				}
				*star_ptr = 0;
			}

			body  = &frame[1];
			split = strchr(body, '=');
			if (split != 0)
			{
				*split = 0;
				Net_Command_Dispatch(body, split + 1);
			}
			else
			{
				Net_SendAck(0);
			}
		}
	}
}

/* ============================================================
 * 网络: 广播 STATE �? TCP 客户�?
 * ============================================================ */
void Net_BroadcastState(void)
{
	if (Tcp_IsConnected())
	{
		Net_SendState();
	}
}

void Net_BroadcastPhase(void)
{
	if (Tcp_IsConnected())
	{
		Net_SendPhase();
	}
}

void App_Reset_ControlState(u8 clear_output)
{
	float start_output;

	g_inc_err[0] = 0.0f;
	g_inc_err[1] = 0.0f;
	g_inc_err[2] = 0.0f;
	g_integral = 0.0f;
	g_error_filt = 0.0f;
	g_error_filt_valid = 0;
	g_pid_output_valid = 0;
	g_tran_tick = 0;
	g_fine_tick = 0;
	g_fine_mode = 0;
	g_fine_pos_base = 0.0f;
	g_fine_pos_integral = 0.0f;
	g_hist_idx = 0;
	g_hist_count = 0;

	if(clear_output != 0)
	{
		temp_ctr_val = 0;
	}

	start_output = (float)temp_ctr_val;
	if (start_output > PID_OUTPUT_MAX) start_output = PID_OUTPUT_MAX;
	if (start_output < PID_OUTPUT_MIN) start_output = PID_OUTPUT_MIN;
	g_pid_output = start_output;
	g_fine_inc_output = start_output;
	g_fine_pos_output = start_output;
	g_fine_pos_base = start_output;
}

static void PID_SyncIntegralToOutput(float target_output, float error, float derivative)
{
	if (g_config.tran_ki > 0.0f) {
		g_integral = (target_output
		              - g_config.tran_kp * error
		              - g_config.tran_kd * derivative) / g_config.tran_ki;
		if (g_integral >  g_config.tran_i_limit) g_integral =  g_config.tran_i_limit;
		if (g_integral < -g_config.tran_i_limit) g_integral = -g_config.tran_i_limit;
	}
}

static void PID_Fine_SyncPosBase(float current_output, float error)
{
	g_fine_pos_base = current_output;
	g_fine_pos_output = current_output;
	g_fine_pos_integral = 0.0f;
	PID_SyncIntegralToOutput(current_output, error, 0.0f);
}

void Key_Process(u8 KeyState)
{
		switch(KeyState)
		{
			/*��*/
			case KEY_UP_PRES:
				if(g_config.manual_flag==0)//�Զ�ʱ����Ŀ���¶�
				{
					if(g_config.target_temp < (100.0f - g_config.step_value))
						g_config.target_temp += g_config.step_value;
					else
						g_config.target_temp=100.0f;
				my_goal = (int)(g_config.target_temp * 10.0f);
					sprintf((char*)Uint_Goal,"%d",my_goal);
					HMI_Send_Float(3,Uint_Goal,strlen((const char*)Uint_Goal));
					AppConfig_MarkDirty();

				}
				else if(g_config.manual_flag==1)//�ֶ�����ʱ���Ӹ���ֵ
				{
					if(temp_ctr_val < (100 - g_config.step_value))
						temp_ctr_val += g_config.step_value;
					else
						temp_ctr_val=100;
					my_pwm = temp_ctr_val;
					sprintf((char*)Uint_pwm,"%d",my_pwm);
					HMI_Send_Float(2,Uint_pwm,strlen((const char*)Uint_pwm));
					AppConfig_MarkDirty();
				}
				break;
			
				/*��*/
			case KEY_DOWN_PRES:
				if(g_config.manual_flag==0)//�Զ�ʱ��СԤ���¶�
				{
					if(g_config.target_temp > (-10.0f + g_config.step_value))
						g_config.target_temp -= g_config.step_value;
					else
						g_config.target_temp=-10.0f;
				my_goal = (int)(g_config.target_temp * 10.0f);
					sprintf((char*)Uint_Goal,"%d",my_goal);
					HMI_Send_Float(3,Uint_Goal,strlen((const char*)Uint_Goal));
					AppConfig_MarkDirty();
				}
				else if(g_config.manual_flag==1)//�ֶ�����ʱ��С����PWM
				{
					if(temp_ctr_val > g_config.step_value - 100)
						temp_ctr_val -= g_config.step_value;
					else
						temp_ctr_val=-100;
					my_pwm = temp_ctr_val;
					sprintf((char*)Uint_pwm,"%d",my_pwm);
					HMI_Send_Float(2,Uint_pwm,strlen((const char*)Uint_pwm));
					AppConfig_MarkDirty();
				}
				break;
			
				/*��������*/
			case KEY_STEP_PRES:
				if(g_config.step_value==1)
					g_config.step_value=5;
				else if(g_config.step_value==5)
					g_config.step_value=10;
				else if(g_config.step_value==10)
					g_config.step_value=1;
				HMI_Send_txt(0, g_config.step_value);
				break;
			
			/*�Զ�or�ֶ� �л�*/
			case KEY_AUTO_PRES:
				if(g_config.manual_flag==0)
				{
					g_config.manual_flag=1;
					g_config.manual_pwm = 0;
					App_Reset_ControlState(1);
				}
				else if(g_config.manual_flag==1)
				{
					g_config.manual_flag=0;
					App_Reset_ControlState(0);
				}
				HMI_Send_txt(1,g_config.manual_flag);
				AppConfig_MarkDirty();
				break;
				
			default:
				;
		}
}

void My_PID_Ctr(void)
{
	float error, raw_error, abs_error, output, delta;
	float near_limit, output_step, derivative;
	float inc_output, pos_output, inc_weight, blend_span;
	float Ts;
	float i_scale, i_span;
	u8   i;
	float t_min, t_max;
	u8   is_stable = 0;

	raw_error = g_config.target_temp - temp_feedback;
	if (g_error_filt_valid == 0) {
		g_error_filt = raw_error;
		g_error_filt_valid = 1;
	} else {
		g_error_filt += PID_ERROR_FILTER_ALPHA * (raw_error - g_error_filt);
	}
	error     = g_error_filt;
	abs_error = (error > 0.0f) ? error : -error;

	if (g_pid_output_valid == 0) {
		g_pid_output = (float)temp_ctr_val;
		if (g_pid_output > PID_OUTPUT_MAX) g_pid_output = PID_OUTPUT_MAX;
		if (g_pid_output < PID_OUTPUT_MIN) g_pid_output = PID_OUTPUT_MIN;
		g_fine_inc_output = g_pid_output;
		g_fine_pos_output = g_pid_output;
		PID_Fine_SyncPosBase(g_pid_output, error);
		g_pid_output_valid = 1;
	}

	/* ---- Stability Detection (40s sliding window) ---- */
	g_temp_hist[g_hist_idx] = temp_feedback;
	g_hist_idx = (g_hist_idx + 1) % STABLE_WINDOW_MAX;
	if (g_hist_count < STABLE_WINDOW_MAX) g_hist_count++;

	if (g_hist_count >= g_config.stable_window) {
		u16 win = g_config.stable_window;
		u16 start = (g_hist_idx + STABLE_WINDOW_MAX - win) % STABLE_WINDOW_MAX;
		t_min = t_max = g_temp_hist[start];
		for (i = 1; i < win; i++) {
			u16 idx = (start + i) % STABLE_WINDOW_MAX;
			if (g_temp_hist[idx] < t_min) t_min = g_temp_hist[idx];
			if (g_temp_hist[idx] > t_max) t_max = g_temp_hist[idx];
		}
		if ((t_max - t_min) <= g_config.stable_delta) is_stable = 1;
	}
	(void)is_stable;  /* FINE is now selected by fine_entry_max, not stability. */

	/* ---- Mode Switching (bumpless transfer) ----
	 * Enter/keep FINE whenever |error| <= fine_entry_max.
	 * Deadband remains inside FINE and only freezes PWM.
	 * (Do NOT exit on !is_stable — fine-tuning naturally causes
	 *  small temp changes that would break the stability flag.) */
	if (g_config.fine_enable == 0 && g_fine_mode) {
		g_pid_output = (float)temp_ctr_val;
		PID_SyncIntegralToOutput(g_pid_output, error, 0.0f);
		g_fine_mode = 0;
		g_tran_tick = 0;
	} else if (g_config.fine_enable != 0 && abs_error <= g_config.fine_entry_max && !g_fine_mode) {
		g_fine_mode = 1;
		g_fine_tick = 0;
		g_inc_err[0] = error; g_inc_err[1] = error; g_inc_err[2] = error;
		g_pid_output = (float)temp_ctr_val;
		if (g_pid_output > PID_OUTPUT_MAX) g_pid_output = PID_OUTPUT_MAX;
		if (g_pid_output < PID_FINE_OUTPUT_MIN) g_pid_output = PID_FINE_OUTPUT_MIN;
		g_fine_inc_output = g_pid_output;
		PID_Fine_SyncPosBase(g_pid_output, error);
	}
	if (g_fine_mode && abs_error > (PID_FINE_EXIT_GAIN * g_config.fine_entry_max)) {
		derivative = error - g_inc_err[0];
		if (abs_error <= g_config.tran_sep_threshold)
			PID_SyncIntegralToOutput(g_pid_output, error, derivative);
		g_fine_inc_output = g_pid_output;
		g_fine_pos_output = g_pid_output;
		g_fine_pos_base = g_pid_output;
		g_fine_pos_integral = 0.0f;
		g_fine_mode = 0;
		g_tran_tick = 0;
	}

	/* ---- Update Error History ---- */
	g_inc_err[2] = g_inc_err[1];
	g_inc_err[1] = g_inc_err[0];
	g_inc_err[0] = error;

	if (!g_fine_mode) {
			/* ==============================================
			 * Mode 1: Transient - Positional PID
			 * Integral accumulates every second (like original pid_control.c).
			 * PID output updates every tran_interval seconds.
			 * ============================================== */

			/* --- Integral: accumulate every second --- */
			if (abs_error > g_config.tran_sep_threshold) {
				g_integral = 0.0f;
			} else if (abs_error <= g_config.pid_deadband) {
				g_integral *= g_config.tran_i_overshoot_leak;
			} else {
				i_scale = 0.0f;
				if (error < 0.0f) {
					g_integral *= g_config.tran_i_overshoot_leak;
				} else {
					if (abs_error <= g_config.tran_i_full_error) {
						i_scale = 1.0f;
					} else {
						i_span = g_config.tran_sep_threshold - g_config.tran_i_full_error;
						if (i_span < 0.1f) i_span = 0.1f;
						i_scale = (g_config.tran_sep_threshold - abs_error) / i_span;
						if (i_scale < g_config.tran_i_min_scale)
							i_scale = g_config.tran_i_min_scale;
						if (i_scale > 1.0f)
							i_scale = 1.0f;
					}
					g_integral += error * i_scale * 1.0f;  /* dt=1s */
				}

				if (g_integral >  g_config.tran_i_limit) g_integral =  g_config.tran_i_limit;
				if (g_integral < -g_config.tran_i_limit) g_integral = -g_config.tran_i_limit;

				if ((g_pid_output >= PID_OUTPUT_MAX && error > 0) ||
				     (g_pid_output <= PID_OUTPUT_MIN && error < 0)) {
					g_integral -= error * i_scale * 1.0f;
				}
			}

			/* --- PID output: every tran_interval seconds --- */
			g_tran_tick++;
			if (g_tran_tick < g_config.tran_interval) {
				output = g_pid_output;
				goto apply_output;
			}
			g_tran_tick = 0;

			if (abs_error <= g_config.pid_deadband) {
				/* Deadband holds the previous PWM value. */
				output = g_pid_output;
				goto apply_output;
			}

			output = g_config.tran_kp * error
			        + g_config.tran_ki * g_integral
			        + g_config.tran_kd * (g_inc_err[0] - g_inc_err[1]);
			near_limit = g_config.fine_entry_max + g_config.fine_entry_min;
			if (abs_error <= near_limit) {
				output_step = output - g_pid_output;
				if (output_step >  g_config.fine_range) output = g_pid_output + g_config.fine_range;
				if (output_step < -g_config.fine_range) output = g_pid_output - g_config.fine_range;
			}
} else {
		/* ==============================================
		 * Mode 2: Fine-tuning - Incremental PID
		 * Interval: fine_interval, Range: fine_range
		 * ============================================== */
		g_fine_tick++;
		if (g_fine_tick < g_config.fine_interval) {
			output = g_pid_output;
			goto apply_output;
		}
		g_fine_tick = 0;

		if (abs_error <= g_config.pid_deadband) {
			/* Deadband holds the previous PWM value. */
			output = g_pid_output;
			g_fine_pos_integral *= PID_FINE_POS_INTEGRAL_LEAK;
			goto apply_output;
		}

		Ts = (float)g_config.fine_interval;
		delta = g_config.fine_kp * (g_inc_err[0] - g_inc_err[1])
		       + g_config.fine_ki * g_inc_err[0] * Ts
		       + g_config.fine_kd * (g_inc_err[0] - 2.0f*g_inc_err[1] + g_inc_err[2]);

		if (delta >  g_config.fine_range) delta =  g_config.fine_range;
		if (delta < -g_config.fine_range) delta = -g_config.fine_range;

		inc_output = g_fine_inc_output + delta;
		if (inc_output > PID_OUTPUT_MAX) inc_output = PID_OUTPUT_MAX;
		if (inc_output < PID_FINE_OUTPUT_MIN) inc_output = PID_FINE_OUTPUT_MIN;

		if (error > g_config.pid_deadband) {
			g_fine_pos_integral += error * Ts;
		} else if (error < -g_config.pid_deadband) {
			g_fine_pos_integral *= PID_FINE_POS_INTEGRAL_LEAK;
		}
		if (g_fine_pos_integral > PID_FINE_POS_INTEGRAL_LIMIT)
			g_fine_pos_integral = PID_FINE_POS_INTEGRAL_LIMIT;
		if (g_fine_pos_integral < 0.0f)
			g_fine_pos_integral = 0.0f;

		pos_output = g_fine_pos_base
		           + g_config.tran_kp * error
		           + g_config.tran_ki * g_fine_pos_integral
		           + g_config.tran_kd * (g_inc_err[0] - g_inc_err[1]);
		if (pos_output > g_fine_pos_base + PID_FINE_POS_WINDOW_GAIN * g_config.fine_range)
			pos_output = g_fine_pos_base + PID_FINE_POS_WINDOW_GAIN * g_config.fine_range;
		if (pos_output < g_fine_pos_base - PID_FINE_POS_WINDOW_GAIN * g_config.fine_range)
			pos_output = g_fine_pos_base - PID_FINE_POS_WINDOW_GAIN * g_config.fine_range;
		output_step = pos_output - g_fine_pos_output;
		if (output_step >  g_config.fine_range) pos_output = g_fine_pos_output + g_config.fine_range;
		if (output_step < -g_config.fine_range) pos_output = g_fine_pos_output - g_config.fine_range;

		blend_span = g_config.fine_entry_max - g_config.pid_deadband;
		if (blend_span < 0.1f) blend_span = 0.1f;
		inc_weight = (abs_error - g_config.pid_deadband) / blend_span;
		if (inc_weight > 1.0f) inc_weight = 1.0f;
		if (inc_weight < 0.0f) inc_weight = 0.0f;
		inc_weight = PID_FINE_INC_WEIGHT_MIN
		           + inc_weight * (PID_FINE_INC_WEIGHT_MAX - PID_FINE_INC_WEIGHT_MIN);

		output = (inc_weight * inc_output) + ((1.0f - inc_weight) * pos_output);
		output_step = output - g_pid_output;
		if (output_step >  g_config.fine_range) output = g_pid_output + g_config.fine_range;
		if (output_step < -g_config.fine_range) output = g_pid_output - g_config.fine_range;
		g_fine_inc_output = inc_output;
		g_fine_pos_output = pos_output;
	}

apply_output:
	if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
	if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
	if (g_fine_mode && output < PID_FINE_OUTPUT_MIN) output = PID_FINE_OUTPUT_MIN;
	g_pid_output = output;
	g_pid_output_valid = 1;
	temp_ctr_val = (int)output;
	My_Ctr(temp_ctr_val);
}


//����heat���ж��Ǽ��Ȼ��Ƿ���ɢ��
void My_Ctr(int heat)
{
	if(heat>100 || heat<-100) return;//������-100,100����Χ��ֱ�ӷ���
	
	if(heat>0)//����0,�ͼ���,ͬʱ�ط���
	{
		Heat_PWM=heat;
		TIM_SetCompare2(TIM9,0); //TIM9PWMռ�ձ�0���ط���
		
	}else if (heat<0)//С��0��ɢ�� ��ͬʱֹͣ����
	{
		Heat_PWM=heat;	//��Heat_PWM��ֵ�������൱��ֹͣ���ȣ���Ϊtimer.c�жϺ��������жϣ�heatΪ��ֵ��ֹͣ����
		{
				int fan_pwm = FAN_PWM_MIN + ((-heat) * (FAN_PWM_MAX - FAN_PWM_MIN)) / 100;
				if (fan_pwm > FAN_PWM_MAX) fan_pwm = FAN_PWM_MAX;
				TIM_SetCompare2(TIM9, fan_pwm);
			}
	}else if(heat == 0)//������Ҳ��ɢ��
	{
		Heat_PWM=0;
		TIM_SetCompare2(TIM9,0);
	}
}
