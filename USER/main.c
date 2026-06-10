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
#include "pid_control.h"
#include "cascade_control.h"
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
static void Net_RxCallback(const u8 *data, u16 len);
static void Flash_Param_Refresh_Hmi(void);
static u8 Uart_Get_Checksum(const char *buf, u16 len);
static u8 Uart_Parse_Hex_Byte(const char *buf, u8 *value);
static u8 Uart_Check_And_Strip_Frame(char *buf);
void Uart_Send_Frame(const char *payload);
void Uart_Send_Ack(u8 ok);


PID_Controller g_temp_pid;
Cascade_Controller g_cascade;

/* 网络: TCP 接收缓冲区 */
static char g_net_rx_buf[512];
static u16  g_net_rx_len = 0;

/* ============================================================
 * 混合控制器: 远区串级 + 近区分层增量式 PID
 *
 *   |error| > 5°C          → 串级控制器 (全速逼近)
 *   2°C < |error| ≤ 5°C    → 增量PID, 每秒更新
 *   0.5°C < |error| ≤ 2°C  → 增量PID, 每8秒更新 (慢慢调)
 *   |error| ≤ 0.5°C        → 死区, PWM完全不动
 *
 * 死区的作用: 温度已经够近了, 再调只会来回振荡, 不如不动。
 * 慢速区的作用: 接近目标时缓慢微调, 避免 PWM 每秒都在变。
 * ============================================================ */
#define HYBRID_FAR_THRESHOLD    5.0f   /* 远区→近区切换 */
#define HYBRID_SLOW_THRESHOLD   2.0f   /* 正常→慢速切换 */
#define HYBRID_DEADZONE         0.5f   /* 死区: PWM不动 */

static float g_inc_err[3];             /* e(k), e(k-1), e(k-2) */
static float g_inc_output  = 0.0f;     /* u(k-1): 上一周期输出 */
static u8    g_hybrid_near_mode = 0;   /* 1=进入近区 */
static u8    g_hybrid_dz_active  = 0;  /* 1=当前在死区 */
static u16   g_hybrid_slow_cnt  = 0;   /* 慢速区跳帧计数 */
float g_hybrid_kp = 3.0f;              /* 增量Kp (extern to app_config) */
float g_hybrid_ki = 0.3f;              /* 增量Ki (extern to app_config) */
float g_hybrid_kd = 1.0f;              /* 增量Kd (extern to app_config) */

uint16_t temperature = 0; 				//�����¶�

float temp_feedback=0.0f;				//�洢��ǰ�¶ȷ�����
int   temp_ctr_val;						//�洢�¶ȵĵ�ǰ���������� ��100�������ȣ���ɢ��
float mytemp_goal=30.0f;				//�Զ�ģʽ��Ŀ���¶�   Ĭ��30��
float uart_set_pid_kp = 1.1546f;
float uart_set_pid_ki = 0.0054f;
float uart_set_pid_kd = 0.0f;
int Heat_PWM;							//����PWMֵ
float MainBoard_temp;

u8 Manual_Flag=1;						//���ģʽ��0=Auto�Զ�; 1=Man�ֶ�
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
	PID_Config_Thermal_Default(&g_temp_pid, uart_set_pid_kp, uart_set_pid_ki, uart_set_pid_kd);
	Cascade_Init(&g_cascade);
	/* 串级控制器参数说明 (无需根据目标温度调整):
	 *   k_outer=0.5:   温度误差→目标速率系数 (10°C误差→5°C/s, 被max截断)
	 *   max_heat_rate:  最大升温速率, 按实际加热器能力设 (默认3°C/s)
	 *   kp_inner=40:    速率误差→PWM (1°C/s速率差→40%PWM)
	 *   ki_inner=12:    速率积分→PWM (快速消除稳态)
	 * 如果升温太慢: 减小 max_heat_rate 或增大 kp_inner
	 * 如果超调振荡: 减小 k_outer 或减小 kp_inner */
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
				My_PID_Ctr();//�Զ�ģʽ�����¶�������������һ��PID���
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
			Net_BroadcastState();
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

	goal_x10     = (int)(mytemp_goal * 10.0f);
	feedback_x10 = (int)(temp_feedback * 10.0f);
	sprintf(payload, "STATE=MODE:%d,PWM:%d,GOAL:%d,FB:%d",
	        Manual_Flag, temp_ctr_val, goal_x10, feedback_x10);
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
 * 网络: 广播 STATE 到 TCP 客户端
 * ============================================================ */
void Net_BroadcastState(void)
{
	if (Tcp_IsConnected())
	{
		Net_SendState();
	}
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
					PID_Controller_Set_Setpoint(&g_temp_pid, g_config.target_temp);
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
					PID_Controller_Set_Setpoint(&g_temp_pid, g_config.target_temp);
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
				}
				else if(g_config.manual_flag==1)
				{
					g_config.manual_flag=0;
				}
				temp_ctr_val=0;
				PID_Controller_Reset(&g_temp_pid);
		Cascade_Reset(&g_cascade);
		PID_Controller_Set_Setpoint(&g_temp_pid, g_config.target_temp);
				HMI_Send_txt(1,g_config.manual_flag);
				AppConfig_MarkDirty();
				break;
				
			default:
				;
		}
}

void My_PID_Ctr(void)
{
	float error, abs_error, output;
	float cascade_out;

	error     = g_config.target_temp - temp_feedback;
	abs_error = (error > 0.0f) ? error : -error;

	/* 始终运行串级控制器, 保持其内部速率跟踪状态 */
	cascade_out = Cascade_Step(&g_cascade, g_config.target_temp, temp_feedback, 1.0f);

	if (abs_error > HYBRID_FAR_THRESHOLD)
	{
		/* ================================================================
		 * 远区 (>5°C): 串级控制器 — 全速逼近目标
		 * ================================================================ */
		g_hybrid_near_mode = 0;
		g_hybrid_dz_active  = 0;
		g_hybrid_slow_cnt  = 0;
		g_inc_err[0] = error;
		g_inc_err[1] = error;
		g_inc_err[2] = error;
		g_inc_output = cascade_out;
		if (g_inc_output < 0.0f)  g_inc_output = 0.0f;
		if (g_inc_output > 100.0f) g_inc_output = 100.0f;

		output = cascade_out;
	}
	else if (abs_error <= HYBRID_DEADZONE)
	{
		/* ================================================================
		 * 死区 (|error| ≤ 0.5°C): PWM 完全不动
		 *
		 * 温度已经在目标附近 ±0.5°C 以内, 再调只会来回振荡。
		 * 冻结一切: 误差历史、输出值全部不变。
		 * 出死区时误差历史被重置, 防止突变。
		 * ================================================================ */
		if (!g_hybrid_dz_active)
		{
			g_hybrid_dz_active = 1;
			/* 冻结误差历史, 出死区时重置 */
			g_inc_err[0] = error;
			g_inc_err[1] = error;
			g_inc_err[2] = error;
		}
		g_hybrid_near_mode = 1;
		g_hybrid_slow_cnt  = 0;
		output = g_inc_output;  /* PWM 不变 */
	}
	else if (abs_error <= HYBRID_SLOW_THRESHOLD)
	{
		/* ================================================================
		 * 慢速区 (0.5°C < |error| ≤ 2°C): 每 N 秒更新一次 PID
		 *
		 * 已经很接近目标了, 不需要每秒改 PWM。
		 * 间隔由 g_config.hyb_slow_interval 控制 (默认 15s)。
		 * 中间跳过的周期保持 PWM 不变, 但误差历史照常更新。
		 * ================================================================ */
		float delta_u;
		float dt = 1.0f;

		g_hybrid_near_mode = 1;
		g_hybrid_dz_active  = 0;

		/* 更新误差历史 (每周期都更新, 保证 P/D 项用最新的变化率) */
		g_inc_err[2] = g_inc_err[1];
		g_inc_err[1] = g_inc_err[0];
		g_inc_err[0] = error;

		g_hybrid_slow_cnt++;

		if (g_hybrid_slow_cnt >= g_config.hyb_slow_interval)
		{
			g_hybrid_slow_cnt = 0;

			delta_u = g_hybrid_kp * (g_inc_err[0] - g_inc_err[1])
			        + g_hybrid_ki * g_inc_err[0] * dt
			        + g_hybrid_kd * (g_inc_err[0] - 2.0f * g_inc_err[1] + g_inc_err[2]) / dt;

			/* 慢速区: 非对称限幅, 降幅比涨幅更严 */
			{
				float d_up   = 3.0f;  /* 慢速区: 每秒最多 +3% */
				float d_down = 2.0f;  /* 慢速区: 每秒最多 -2% (降幅更严,防振荡) */
				if (delta_u > d_up)    delta_u = d_up;
				if (delta_u < -d_down) delta_u = -d_down;
			}

			output = g_inc_output + delta_u;
			if (output > 100.0f) output = 100.0f;
			if (output < 0.0f)   output = 0.0f;
			g_inc_output = output;
		}
		else
		{
			/* 跳帧: PWM 不变 */
			output = g_inc_output;
		}
	}
	else
	{
		/* ================================================================
		 * 正常近区 (2°C < |error| ≤ 5°C): 增量PID 每秒更新
		 * ================================================================ */
		float delta_u;
		float dt = 1.0f;

		g_hybrid_near_mode = 1;
		g_hybrid_dz_active  = 0;
		g_hybrid_slow_cnt  = 0;

		/* 更新误差历史 */
		g_inc_err[2] = g_inc_err[1];
		g_inc_err[1] = g_inc_err[0];
		g_inc_err[0] = error;

		delta_u = g_hybrid_kp * (g_inc_err[0] - g_inc_err[1])
		        + g_hybrid_ki * g_inc_err[0] * dt
		        + g_hybrid_kd * (g_inc_err[0] - 2.0f * g_inc_err[1] + g_inc_err[2]) / dt;

		{
			/* 正常近区: 非对称限幅, 降幅比涨幅更严。
			 * 防止传感器抖动或温差突变导致 PWM 砸到 0 又弹回 100 的振荡 */
			float d_up   = 8.0f;   /* 每秒最多 +8% */
			float d_down = 4.0f;   /* 每秒最多 -4% (更严) */
			if (delta_u > d_up)    delta_u = d_up;
			if (delta_u < -d_down) delta_u = -d_down;
		}

		output = g_inc_output + delta_u;
		if (output > 100.0f) output = 100.0f;
		if (output < 0.0f)   output = 0.0f;

		g_inc_output = output;
	}

	/* 输出: 正=加热, 负=制冷 */
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
				int fan_pwm = 25 + ((-heat) * 75) / 100;
				if (fan_pwm > 100) fan_pwm = 100;
				TIM_SetCompare2(TIM9, fan_pwm);
			}
	}else if(heat == 0)//������Ҳ��ɢ��
	{
		Heat_PWM=0;
		TIM_SetCompare2(TIM9,0);
	}
}

