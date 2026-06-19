#include "app_config.h"
#include "flash_params.h"

#include "eth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================
 * 出厂默认值 (C90 位置初始化)
 * 字段顺序: manual_flag, target_temp, manual_pwm, step_value,
 *           csc_k_outer, csc_max_heat_rate, csc_kp_inner, csc_ki_inner, csc_output_rate_limit,
 *           hyb_threshold, hyb_deadzone, hyb_kp, hyb_ki, hyb_kd, hyb_min_output, hyb_slow_interval,
 *           pid_kp, pid_ki, pid_kd,
 *           eth_ip, eth_gateway, eth_netmask, tcp_port, eth_mac, eth_dhcp
 * ============================================================ */
const AppConfig g_config_defaults = {
	1, 30.0f, 0, 1,
	3.0f, 0.3f, 1.0f, 3, 10.0f,
	0.20f, 2.0f, 80.0f, 0.85f,
	1, 1.5f, 0.1f, 2.0f, 8, 5.0f, 1.0f, 3.0f, 20, 1.0f,
	0.3f,
	0, 40.0f, 120, 30, 0.7f, 8.0f,
	{192,168,1,100}, {192,168,1,1}, {255,255,255,0}, 8000,
	{0x02,0x00,0x00,0x00,0x00,0x01}, 0
};


/* ============================================================
 * 运行时全局实例
 * ============================================================ */
AppConfig g_config;

/* 命令来源标记: 0=串口, 1=网口 (回复路由用) */
u8 g_cmd_from_net = 0;

/* ---- 前向引用 ---- */
extern u8  Manual_Flag;
extern float mytemp_goal;
extern int   temp_ctr_val;
extern u8   Step_Value;
extern const char *App_Get_WorkPhase(void);
extern void App_Reset_ControlState(u8 clear_output);
extern u8   g_eth_mac[6];
extern u8   g_eth_ip[4];
extern u8   g_eth_gateway[4];
extern u8   g_eth_netmask[4];
extern u16  g_tcp_port;

/* ---- 串口/网口回复函数 (定义在 main.c) ---- */
extern void Uart_Send_Frame(const char *payload);
extern void Uart_Send_Ack(u8 ok);
extern u16  Tcp_Send(const u8 *data, u16 len);
extern u8   Tcp_IsConnected(void);

/* ============================================================
 * 初始化: 默认值 → Flash加载覆盖 → 应用到驱动
 * ============================================================ */
void AppConfig_Init(void)
{
	/* 1. 加载出厂默认值 */
	g_config = g_config_defaults;

	/* 2. Flash 数据有效则覆盖 */
	Flash_Param_Load_Runtime();

	/* 3. 同步到运行变量 */
	AppConfig_Apply();
}

void AppConfig_LoadDefaults(void)
{
	u8 saved_mac[6];
	/* 保留 MAC 地址 (每台设备应唯一) */
	memcpy(saved_mac, g_config.eth_mac, 6);
	g_config = g_config_defaults;
	memcpy(g_config.eth_mac, saved_mac, 6);
	AppConfig_Apply();
	AppConfig_MarkDirty();
}

u8 AppConfig_Save(void)
{
	return Flash_Param_Save(&g_config);
}

void AppConfig_MarkDirty(void)
{
	Flash_Param_MarkDirty();
}

void AppConfig_Process(void)
{
	Flash_Param_Process();
}

/* ============================================================
 * 将配置同步到各个驱动模块和全局变量
 * ============================================================ */
void AppConfig_Apply(void)
{
	/* A. 系统控制 */
	Manual_Flag  = g_config.manual_flag;
	mytemp_goal  = g_config.target_temp;
	temp_ctr_val = g_config.manual_pwm;
	Step_Value   = g_config.step_value;

	/* B. 串级控制器 (仅设置参数, 不重置状态) */

	/* C. PID (仅设置参数, 不重置状态) */

	/* C2. 近区增量PID (同步到运行时变量) */
	/* hyb_slow_interval 由 main.c 直接读 g_config, 无需同步 */

	/* D. 网络 (仅 IP/端口, MAC 在 Eth_Init 前已设置) */
	memcpy(g_eth_ip,      g_config.eth_ip,      4);
	memcpy(g_eth_gateway, g_config.eth_gateway, 4);
	memcpy(g_eth_netmask, g_config.eth_netmask, 4);
	memcpy(g_eth_mac,     g_config.eth_mac,     6);
	g_tcp_port = g_config.tcp_port;
}

/* ============================================================
 * 指令回复路由
 * ============================================================ */
void AppCmd_SendAck(u8 ok)
{
	if (g_cmd_from_net)
	{
		char buf[64];
		int len;
		if (ok)
			len = sprintf(buf, "!ACK=OK\r\n");
		else
			len = sprintf(buf, "!ACK=ERR\r\n");
		Tcp_Send((u8 *)buf, (u16)len);
	}
	else
	{
		Uart_Send_Ack(ok);
	}
}

void AppCmd_SendFrame(const char *payload)
{
	if (g_cmd_from_net)
	{
		char buf[384];
		int  len;
		len = sprintf(buf, "!%s\r\n", payload);
		Tcp_Send((u8 *)buf, (u16)len);
	}
	else
	{
		Uart_Send_Frame(payload);
	}
}

/* ============================================================
 * 辅助解析函数
 * ============================================================ */
static u8 ParseFloat(const char *s, float *out)
{
	char *end;
	double v;
	if (s == 0 || out == 0 || s[0] == 0) return 0;
	v = strtod(s, &end);
	if (end == s) return 0;
	while (*end == ' ' || *end == '\t') end++;
	if (*end != 0) return 0;
	*out = (float)v;
	return 1;
}

static u8 ParseInt(const char *s, int *out)
{
	char *end;
	long v;
	if (s == 0 || out == 0 || s[0] == 0) return 0;
	v = strtol(s, &end, 10);
	if (end == s) return 0;
	while (*end == ' ' || *end == '\t') end++;
	if (*end != 0) return 0;
	*out = (int)v;
	return 1;
}

static u8 ParseWholeFloatAsInt(const char *s, int *out)
{
	float v;
	int iv;
	if (!ParseFloat(s, &v)) return 0;
	iv = (int)v;
	if (v != (float)iv) return 0;
	*out = iv;
	return 1;
}

/* 解析 IP 地址字符串 "192.168.1.100" -> u8[4] */
static u8 ParseIp4(const char *s, u8 ip[4])
{
	u32 a, b, c, d;
	if (s == 0 || ip == 0) return 0;
	if (sscanf(s, "%lu.%lu.%lu.%lu", &a, &b, &c, &d) != 4) return 0;
	if (a > 255 || b > 255 || c > 255 || d > 255) return 0;
	ip[0] = (u8)a; ip[1] = (u8)b; ip[2] = (u8)c; ip[3] = (u8)d;
	return 1;
}

/* 解析 MAC 地址 "02:00:00:00:00:01" -> u8[6] */
static u8 ParseMac(const char *s, u8 mac[6])
{
	u32 m[6];
	int i;
	if (s == 0 || mac == 0) return 0;
	if (sscanf(s, "%lx:%lx:%lx:%lx:%lx:%lx",
	           &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
	{
		/* try dash separator */
		if (sscanf(s, "%lx-%lx-%lx-%lx-%lx-%lx",
		           &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) != 6)
			return 0;
	}
	for (i = 0; i < 6; i++) {
		if (m[i] > 255) return 0;
		mac[i] = (u8)m[i];
	}
	return 1;
}

/* ============================================================
 * 指令分发总入口
 * ============================================================ */
int AppCmd_Dispatch(const char *body, const char *value)
{
	if (body == 0 || value == 0) return 0;

	/* ---- MODE ---- */
	if (strcmp(body, "MODE") == 0)
	{
		if (strcmp(value, "AUTO") == 0 || strcmp(value, "auto") == 0
		    || strcmp(value, "0") == 0)
		{
			g_config.manual_flag = 0;
			App_Reset_ControlState(0);
			Manual_Flag = g_config.manual_flag;
			mytemp_goal = g_config.target_temp;
			AppConfig_MarkDirty();
			return 1;
		}
		if (strcmp(value, "MAN") == 0 || strcmp(value, "man") == 0
		    || strcmp(value, "MANUAL") == 0 || strcmp(value, "manual") == 0
		    || strcmp(value, "1") == 0)
		{
			g_config.manual_flag = 1;
			g_config.manual_pwm  = 0;
			AppConfig_Apply();
			App_Reset_ControlState(1);
			AppConfig_MarkDirty();
			return 1;
		}
		return 0;
	}

	/* ---- TEMP ---- */
	if (strcmp(body, "TEMP") == 0)
	{
		float v;
		if (!ParseFloat(value, &v)) return 0;
		if (v < -10.0f || v > 100.0f) return 0;
		g_config.target_temp = v;
		g_config.target_temp = v;
		mytemp_goal = v;
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- PWM ---- */
	if (strcmp(body, "PWM") == 0)
	{
		int v;
		if (!ParseInt(value, &v)) return 0;
		if (v < -100 || v > 100) return 0;
		if (g_config.manual_flag != 1) return 0;
		g_config.manual_pwm = v;
		temp_ctr_val = v;
		AppConfig_MarkDirty();
		return 1;
	}

		/* ---- TRAN (Transient Mode) ---- */
		if (strcmp(body, "TRAN") == 0)
		{
			float kp, ki, kd, st;
			int   interval = 0;
			char  buf[64];
			char *s1, *s2, *s3, *s4, *s5;
			strncpy(buf, value, sizeof(buf) - 1);
			buf[sizeof(buf) - 1] = 0;
			s1 = strtok(buf, ","); s2 = strtok(0, ",");
			s3 = strtok(0, ",");  s4 = strtok(0, ",");
			s5 = strtok(0, ",");
			if (!s1 || !s2 || !s3) return 0;
			if (!ParseFloat(s1, &kp)) return 0;
			if (!ParseFloat(s2, &ki)) return 0;
			if (!ParseFloat(s3, &kd)) return 0;
			if (s4) { if (!ParseInt(s4, &interval)) return 0; }
			if (s5) { if (!ParseFloat(s5, &st) || st < 1.0f || st > 30.0f) return 0; }
			if (kp < 0.1f || kp > 50.0f)  return 0;
			if (ki < 0.0f || ki > 5.0f)   return 0;
			if (kd < 0.0f || kd > 10.0f)  return 0;
			if (interval != 0 && (interval < 1 || interval > 60)) return 0;
			g_config.tran_kp = kp;
			g_config.tran_ki = ki;
			g_config.tran_kd = kd;
			if (interval > 0) g_config.tran_interval = (u16)interval;
			if (s5) g_config.tran_sep_threshold = st;
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- IADAPT (Adaptive integral for transient mode) ---- */
		if (strcmp(body, "IADAPT") == 0)
		{
			float min_scale, full_error, limit, leak;
			char  buf[80];
			char *s1, *s2, *s3, *s4;
			strncpy(buf, value, sizeof(buf) - 1);
			buf[sizeof(buf) - 1] = 0;
			s1 = strtok(buf, ","); s2 = strtok(0, ",");
			s3 = strtok(0, ",");  s4 = strtok(0, ",");
			if (!s1 || !s2 || !s3 || !s4) return 0;
			if (!ParseFloat(s1, &min_scale)) return 0;
			if (!ParseFloat(s2, &full_error)) return 0;
			if (!ParseFloat(s3, &limit)) return 0;
			if (!ParseFloat(s4, &leak)) return 0;
			if (min_scale < 0.0f || min_scale > 1.0f) return 0;
			if (full_error < 0.1f || full_error > 10.0f) return 0;
			if (limit < 1.0f || limit > 300.0f) return 0;
			if (leak < 0.0f || leak > 1.0f) return 0;
			g_config.tran_i_min_scale = min_scale;
			g_config.tran_i_full_error = full_error;
			g_config.tran_i_limit = limit;
			g_config.tran_i_overshoot_leak = leak;
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- FINE (Fine-tuning Mode) ---- */
		if (strcmp(body, "FINE") == 0)
		{
			float kp, ki, kd, fr, emin, emax, sd;
			int   interval = 0, sw = 0;
			char  buf[64];
			char *s1, *s2, *s3, *s4, *s5, *s6, *s7, *s8, *s9;
			strncpy(buf, value, sizeof(buf) - 1);
			buf[sizeof(buf) - 1] = 0;
			s1 = strtok(buf, ","); s2 = strtok(0, ",");
			s3 = strtok(0, ",");  s4 = strtok(0, ",");
			s5 = strtok(0, ",");
			s6 = strtok(0, ",");  /* optional: entry_min */
			s7 = strtok(0, ",");  /* optional: entry_max */
			s8 = strtok(0, ",");  /* optional: stable_window */
			s9 = strtok(0, ",");  /* optional: stable_delta */
			if (!s1 || !s2 || !s3) return 0;
			if (!ParseFloat(s1, &kp)) return 0;
			if (!ParseFloat(s2, &ki)) return 0;
			if (!ParseFloat(s3, &kd)) return 0;
			if (s4) { if (!ParseInt(s4, &interval)) return 0; }
			if (s5) { if (!ParseFloat(s5, &fr) || fr < 1.0f || fr > 20.0f) return 0; }
			if (s6) { if (!ParseFloat(s6, &emin) || emin < 0.1f || emin > 5.0f) return 0; }
			if (s7) { if (!ParseFloat(s7, &emax) || emax < 1.0f || emax > 10.0f) return 0; }
			if (s8) { if (!ParseInt(s8, &sw) || sw < 10 || sw > 120) return 0; }
			if (s9) { if (!ParseFloat(s9, &sd) || sd < 0.2f || sd > 5.0f) return 0; }
			if (kp < 0.1f || kp > 20.0f)  return 0;
			if (ki < 0.0f || ki > 3.0f)   return 0;
			if (kd < 0.0f || kd > 10.0f)  return 0;
			if (interval != 0 && (interval < 1 || interval > 60)) return 0;
			if (s6 && s7 && emin > emax) return 0;
			g_config.fine_kp = kp;
			g_config.fine_ki = ki;
			g_config.fine_kd = kd;
			if (interval > 0) g_config.fine_interval = (u16)interval;
			if (s5) g_config.fine_range = fr;
			if (s6) g_config.fine_entry_min = emin;
			if (s7) g_config.fine_entry_max = emax;
			if (s8) g_config.stable_window = (u16)sw;
			if (s9) g_config.stable_delta = sd;
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- FINEEN ---- */
		if (strcmp(body, "FINEEN") == 0)
		{
			int v;
			if (!ParseInt(value, &v)) return 0;
			if (v != 0 && v != 1) return 0;
			g_config.fine_enable = (u8)v;
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- DEADBAND ---- */
		if (strcmp(body, "DEADBAND") == 0)
		{
			float v;
			if (!ParseFloat(value, &v)) return 0;
			if (v < 0.1f || v > 2.0f) return 0;
			g_config.pid_deadband = v;
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- SMITH: enable,gain,tau,delay,blend,maxlead ---- */
		if (strcmp(body, "SMITH") == 0)
		{
			int en, tau, delay_s;
			float gain, blend, maxlead;
			char  buf[96];
			char *s1, *s2, *s3, *s4, *s5, *s6;
			strncpy(buf, value, sizeof(buf) - 1);
			buf[sizeof(buf) - 1] = 0;
			s1 = strtok(buf, ","); s2 = strtok(0, ",");
			s3 = strtok(0, ",");  s4 = strtok(0, ",");
			s5 = strtok(0, ",");  s6 = strtok(0, ",");
			if (!s1 || !s2 || !s3 || !s4 || !s5 || !s6) return 0;
			if (!ParseInt(s1, &en)) return 0;
			if (!ParseFloat(s2, &gain)) return 0;
			if (!ParseWholeFloatAsInt(s3, &tau)) return 0;
			if (!ParseWholeFloatAsInt(s4, &delay_s)) return 0;
			if (!ParseFloat(s5, &blend)) return 0;
			if (!ParseFloat(s6, &maxlead)) return 0;
			if (en != 0 && en != 1) return 0;
			if (gain < 1.0f || gain > 200.0f) return 0;
			if (tau < 5 || tau > 3600) return 0;
			if (delay_s < 0 || delay_s > 180) return 0;
			if (blend < 0.0f || blend > 1.0f) return 0;
			if (maxlead < 0.5f || maxlead > 30.0f) return 0;
			g_config.smith_enable = (u8)en;
			g_config.smith_gain = gain;
			g_config.smith_tau = (u16)tau;
			g_config.smith_delay = (u16)delay_s;
			g_config.smith_blend = blend;
			g_config.smith_max_lead = maxlead;
			App_Reset_ControlState(0);
			AppConfig_MarkDirty();
			return 1;
		}

		/* ---- SMITHEN ---- */
		if (strcmp(body, "SMITHEN") == 0)
		{
			int v;
			if (!ParseInt(value, &v)) return 0;
			if (v != 0 && v != 1) return 0;
			g_config.smith_enable = (u8)v;
			App_Reset_ControlState(0);
			AppConfig_MarkDirty();
			return 1;
		}
/* ---- STEP ---- */
	if (strcmp(body, "STEP") == 0)
	{
		int v;
		if (!ParseInt(value, &v)) return 0;
		if (v != 1 && v != 5 && v != 10) return 0;
		g_config.step_value = (u8)v;
		Step_Value = (u8)v;
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- GET ---- */
	if (strcmp(body, "GET") == 0)
	{
		char out[384];
		extern float temp_feedback;
		extern float temp_control_feedback;
		extern int   temp_ctr_val;
		extern u8    Manual_Flag;

		if (strcmp(value, "STATE") == 0)
		{
			int goal_x10    = (int)(g_config.target_temp * 10.0f);
			int feedback_x10 = (int)(temp_feedback * 10.0f);
			sprintf(out, "STATE=MODE:%d,PHASE:%s,PWM:%d,GOAL:%d,FB:%d,PFB:%d",
			        g_config.manual_flag, App_Get_WorkPhase(), temp_ctr_val, goal_x10,
			        feedback_x10, (int)(temp_control_feedback * 10.0f));
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "PID") == 0)
		{
			sprintf(out, "TRAN=KP:%.2f,KI:%.3f,KD:%.2f,INT:%u,SEP:%.1f,IMS:%.2f,IFE:%.1f,ILIM:%.1f,ILEAK:%.2f",
			        g_config.tran_kp, g_config.tran_ki, g_config.tran_kd,
			        g_config.tran_interval, g_config.tran_sep_threshold,
			        g_config.tran_i_min_scale, g_config.tran_i_full_error,
			        g_config.tran_i_limit, g_config.tran_i_overshoot_leak);
			AppCmd_SendFrame(out);
			sprintf(out, "FINE=EN:%d,KP:%.2f,KI:%.3f,KD:%.2f,INT:%u,FR:%.1f,EMIN:%.1f,EMAX:%.1f,SW:%u,SD:%.1f",
			        g_config.fine_enable,
			        g_config.fine_kp, g_config.fine_ki, g_config.fine_kd,
			        g_config.fine_interval, g_config.fine_range, g_config.fine_entry_min, g_config.fine_entry_max, g_config.stable_window, g_config.stable_delta);
			AppCmd_SendFrame(out);
			sprintf(out, "DEADBAND=%.2f", g_config.pid_deadband);
			AppCmd_SendFrame(out);
			sprintf(out, "SMITH=EN:%d,GAIN:%.1f,TAU:%u,DELAY:%u,BLEND:%.2f,MAXLEAD:%.1f",
			        g_config.smith_enable, g_config.smith_gain,
			        g_config.smith_tau, g_config.smith_delay,
			        g_config.smith_blend, g_config.smith_max_lead);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "TRAN") == 0)
		{
			sprintf(out, "TRAN=KP:%.2f,KI:%.3f,KD:%.2f,INT:%u,SEP:%.1f,IMS:%.2f,IFE:%.1f,ILIM:%.1f,ILEAK:%.2f",
			        g_config.tran_kp, g_config.tran_ki, g_config.tran_kd,
			        g_config.tran_interval, g_config.tran_sep_threshold,
			        g_config.tran_i_min_scale, g_config.tran_i_full_error,
			        g_config.tran_i_limit, g_config.tran_i_overshoot_leak);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "IADAPT") == 0)
		{
			sprintf(out, "IADAPT=%.2f,%.1f,%.1f,%.2f",
			        g_config.tran_i_min_scale, g_config.tran_i_full_error,
			        g_config.tran_i_limit, g_config.tran_i_overshoot_leak);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "FINE") == 0)
		{
			sprintf(out, "FINE=EN:%d,KP:%.2f,KI:%.3f,KD:%.2f,INT:%u,FR:%.1f,EMIN:%.1f,EMAX:%.1f,SW:%u,SD:%.1f",
			        g_config.fine_enable,
			        g_config.fine_kp, g_config.fine_ki, g_config.fine_kd,
			        g_config.fine_interval, g_config.fine_range, g_config.fine_entry_min, g_config.fine_entry_max, g_config.stable_window, g_config.stable_delta);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "DEADBAND") == 0)
		{
			sprintf(out, "DEADBAND=%.2f", g_config.pid_deadband);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "FINEEN") == 0)
		{
			sprintf(out, "FINEEN=%d", g_config.fine_enable);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "SMITH") == 0)
		{
			sprintf(out, "SMITH=EN:%d,GAIN:%.1f,TAU:%u,DELAY:%u,BLEND:%.2f,MAXLEAD:%.1f",
			        g_config.smith_enable, g_config.smith_gain,
			        g_config.smith_tau, g_config.smith_delay,
			        g_config.smith_blend, g_config.smith_max_lead);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "SMITHEN") == 0)
		{
			sprintf(out, "SMITHEN=%d", g_config.smith_enable);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "NET") == 0)
		{
			sprintf(out, "NET=IP:%d.%d.%d.%d,GW:%d.%d.%d.%d,NM:%d.%d.%d.%d,PORT:%d",
			        g_config.eth_ip[0], g_config.eth_ip[1],
			        g_config.eth_ip[2], g_config.eth_ip[3],
			        g_config.eth_gateway[0], g_config.eth_gateway[1],
			        g_config.eth_gateway[2], g_config.eth_gateway[3],
			        g_config.eth_netmask[0], g_config.eth_netmask[1],
			        g_config.eth_netmask[2], g_config.eth_netmask[3],
			        g_config.tcp_port);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "CONFIG") == 0)
		{
			AppCmd_SendFrame("CONFIG=SEE_README");
			sprintf(out, "TRAN=%.2f,%.3f,%.2f,%u,%.1f",
			        g_config.tran_kp, g_config.tran_ki, g_config.tran_kd,
			        g_config.tran_interval, g_config.tran_sep_threshold);
			AppCmd_SendFrame(out);
			sprintf(out, "IADAPT=%.2f,%.1f,%.1f,%.2f",
			        g_config.tran_i_min_scale, g_config.tran_i_full_error,
			        g_config.tran_i_limit, g_config.tran_i_overshoot_leak);
			AppCmd_SendFrame(out);
			sprintf(out, "FINE=%d,%.2f,%.3f,%.2f,%u,%.1f,%.1f,%.1f,%u,%.1f",
			        g_config.fine_enable,
			        g_config.fine_kp, g_config.fine_ki, g_config.fine_kd,
			        g_config.fine_interval, g_config.fine_range, g_config.fine_entry_min, g_config.fine_entry_max, g_config.stable_window, g_config.stable_delta);
			AppCmd_SendFrame(out);
			sprintf(out, "DEADBAND=%.2f", g_config.pid_deadband);
			AppCmd_SendFrame(out);
			sprintf(out, "SMITH=%d,%.1f,%u,%u,%.2f,%.1f",
			        g_config.smith_enable, g_config.smith_gain,
			        g_config.smith_tau, g_config.smith_delay,
			        g_config.smith_blend, g_config.smith_max_lead);
			AppCmd_SendFrame(out);
			sprintf(out, "TEMP_GOAL=%.1f", g_config.target_temp);
			AppCmd_SendFrame(out);
			sprintf(out, "MODE=%d", g_config.manual_flag);
			AppCmd_SendFrame(out);
			return 2;
		}
		/* ---- GET=ETH: Ethernet diagnostics (serial-friendly) ---- */
		if (strcmp(value, "ETH") == 0)
		{
			char diag[64];
			Eth_GetDiag(diag, sizeof(diag));
			sprintf(out, "ETH=%s,TCP:%d", diag,
			        Tcp_IsConnected() ? 1 : 0);
			AppCmd_SendFrame(out);
			return 2;
		}
		return 0;
	}

	/* ---- SAVE ---- */
	if (strcmp(body, "SAVE") == 0)
	{
		if (AppConfig_Save())
			AppCmd_SendFrame("SAVE=OK");
		else
			AppCmd_SendFrame("SAVE=ERR");
		return 2;
	}

	/* ---- RESET ---- */
	if (strcmp(body, "RESET") == 0)
	{
		AppConfig_LoadDefaults();
		AppCmd_SendFrame("RESET=OK");
		return 2;
	}

	return 0;
}
