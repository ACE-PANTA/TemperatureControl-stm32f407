#include "app_comm.h"
#include "app_config.h"
#include "pid_control.h"
#include "usart.h"
#include "eth.h"
#include <stdio.h>
#include <string.h>

extern float temp_feedback;
extern int   temp_ctr_val;

static char g_net_rx_buf[512];
static u16  g_net_rx_len = 0;

static u8 Uart_Get_Checksum(const char *buf, u16 len)
{
	u8 checksum = 0;

	while (len > 0)
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

	if ((buf == 0) || (value == 0) || (buf[0] == 0) || (buf[1] == 0) || (buf[2] != 0))
		return 0;

	if ((buf[0] >= '0') && (buf[0] <= '9'))
		high = (u8)(buf[0] - '0');
	else if ((buf[0] >= 'A') && (buf[0] <= 'F'))
		high = (u8)(buf[0] - 'A' + 10);
	else if ((buf[0] >= 'a') && (buf[0] <= 'f'))
		high = (u8)(buf[0] - 'a' + 10);
	else
		return 0;

	if ((buf[1] >= '0') && (buf[1] <= '9'))
		low = (u8)(buf[1] - '0');
	else if ((buf[1] >= 'A') && (buf[1] <= 'F'))
		low = (u8)(buf[1] - 'A' + 10);
	else if ((buf[1] >= 'a') && (buf[1] <= 'f'))
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

	if ((buf == 0) || (buf[0] != '!'))
		return 0;

	star = strchr(buf, '*');
	if (star == 0)
		return 1;

	if (Uart_Parse_Hex_Byte(star + 1, &recv_sum) == 0)
		return 0;

	calc_sum = Uart_Get_Checksum(buf, (u16)(star - buf));
	if (calc_sum != recv_sum)
		return 0;

	*star = 0;
	return 1;
}

void Uart_Send_Frame(const char *payload)
{
	char tx_buf[USART1_CMD_MAX_LEN + 16];
	u8 checksum;

	if (payload == 0)
		return;

	sprintf(tx_buf, "!%s", payload);
	checksum = Uart_Get_Checksum(tx_buf, (u16)strlen(tx_buf));
	sprintf(tx_buf + strlen(tx_buf), "*%02X\r\n", checksum);
	usart1_send_string(tx_buf);
}

void Uart_Send_Ack(u8 ok)
{
	if (ok != 0)
		Uart_Send_Frame("ACK=OK");
	else
		Uart_Send_Frame("ACK=ERR");
}

void Uart_Send_WorkPhase(void)
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

	while (usart1_read_line(cmd_buf, sizeof(cmd_buf)) != 0)
	{
		handled = 0;
		if (Uart_Check_And_Strip_Frame(cmd_buf) == 0)
		{
			Uart_Send_Ack(0);
			continue;
		}

		body = &cmd_buf[1];
		split = strchr(body, '=');
		if (split == 0)
		{
			Uart_Send_Ack(0);
			continue;
		}

		*split = 0;
		value = split + 1;
		g_cmd_from_net = 0;
		handled = AppCmd_Dispatch(body, value);

		if (handled == 1)
			Uart_Send_Ack(1);
		else if (handled == 0)
			Uart_Send_Ack(0);
	}
}

static void Net_SendFrame(const char *payload)
{
	char tx_buf[256];
	u8 checksum;
	u16 len;

	if (!Tcp_IsConnected() || payload == 0) return;

	sprintf(tx_buf, "!%s", payload);
	len = (u16)strlen(tx_buf);
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
	int goal_x10;
	int feedback_x10;

	goal_x10 = (int)(g_config.target_temp * 10.0f);
	feedback_x10 = (int)(temp_feedback * 10.0f);
	sprintf(payload, "STATE=MODE:%d,PHASE:%s,PWM:%d,GOAL:%d,FB:%d,PFB:%d",
	        g_config.manual_flag, App_Get_WorkPhase(), temp_ctr_val, goal_x10,
	        feedback_x10, (int)(temp_control_feedback * 10.0f));
	Net_SendFrame(payload);
}

static void Net_SendPhase(void)
{
	char payload[32];

	sprintf(payload, "PHASE=%s", App_Get_WorkPhase());
	Net_SendFrame(payload);
}

static void Net_Command_Dispatch(const char *body, const char *value)
{
	int handled;

	if (body == 0 || value == 0) return;

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

	if (len == 0)
	{
		g_net_rx_len = 0;
		g_net_rx_buf[0] = 0;
		return;
	}

	if (g_net_rx_len + len > sizeof(g_net_rx_buf))
		g_net_rx_len = 0;

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
		u16 frame_end;
		char frame[512];
		u16 flen;

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
		{
			g_net_rx_len = 0;
		}
		else
		{
			memmove(g_net_rx_buf, g_net_rx_buf + frame_end, g_net_rx_len - frame_end);
			g_net_rx_len -= frame_end;
			g_net_rx_buf[g_net_rx_len] = 0;
		}

		{
			u8 recv_sum;
			u8 calc_sum;
			char *star_ptr = strchr(frame, '*');
			if (star_ptr != 0 && Uart_Parse_Hex_Byte(star_ptr + 1, &recv_sum))
			{
				calc_sum = Uart_Get_Checksum(frame, (u16)(star_ptr - frame));
				if (calc_sum != recv_sum)
				{
					Net_SendAck(0);
					continue;
				}
				*star_ptr = 0;
			}

			body = &frame[1];
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

void Net_InitCallbacks(void)
{
	Tcp_SetRxCallback(Net_RxCallback);
}

void Net_BroadcastState(void)
{
	if (Tcp_IsConnected())
		Net_SendState();
}

void Net_BroadcastPhase(void)
{
	if (Tcp_IsConnected())
		Net_SendPhase();
}
