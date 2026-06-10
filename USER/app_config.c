#include "app_config.h"
#include "flash_params.h"
#include "pid_control.h"
#include "cascade_control.h"
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
	0.5f, 3.0f, 40.0f, 12.0f, 20.0f,
	5.0f, 0.5f, 3.0f, 0.3f, 1.0f, 5.0f, 15,
	1.1546f, 0.0054f, 0.0f,
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
extern float uart_set_pid_kp;
extern float uart_set_pid_ki;
extern float uart_set_pid_kd;
extern u8   g_eth_mac[6];
extern u8   g_eth_ip[4];
extern u8   g_eth_gateway[4];
extern u8   g_eth_netmask[4];
extern float g_hybrid_kp;
extern float g_hybrid_ki;
extern float g_hybrid_kd;
extern u16  g_tcp_port;

/* ---- 串口/网口回复函数 (定义在 main.c) ---- */
extern void Uart_Send_Frame(const char *payload);
extern void Uart_Send_Ack(u8 ok);
extern u16  Tcp_Send(const u8 *data, u16 len);
extern u8   Tcp_IsConnected(void);

/* ---- 外部控制器引用 ---- */
extern PID_Controller g_temp_pid;
extern Cascade_Controller g_cascade;

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
	g_cascade.k_outer       = g_config.csc_k_outer;
	g_cascade.max_heat_rate = g_config.csc_max_heat_rate;
	g_cascade.kp_inner      = g_config.csc_kp_inner;
	g_cascade.ki_inner      = g_config.csc_ki_inner;
		g_cascade.output_rate_limit = g_config.csc_output_rate_limit;

	/* C. PID (仅设置参数, 不重置状态) */
	uart_set_pid_kp = g_config.pid_kp;
	uart_set_pid_ki = g_config.pid_ki;
	uart_set_pid_kd = g_config.pid_kd;
	PID_Controller_Set_Gains(&g_temp_pid, g_config.pid_kp, g_config.pid_ki, g_config.pid_kd);

	/* C2. 近区增量PID (同步到运行时变量) */
	g_hybrid_kp = g_config.hyb_kp;
	g_hybrid_ki = g_config.hyb_ki;
	g_hybrid_kd = g_config.hyb_kd;
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
	if (s == 0 || out == 0 || s[0] == 0) return 0;
	*out = (float)atof(s);
	return 1;
}

static u8 ParseInt(const char *s, int *out)
{
	if (s == 0 || out == 0 || s[0] == 0) return 0;
	*out = atoi(s);
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
			g_config.manual_pwm  = 0;
			AppConfig_Apply();
			Cascade_Reset(&g_cascade);
			PID_Controller_Reset(&g_temp_pid);
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
			Cascade_Reset(&g_cascade);
			PID_Controller_Reset(&g_temp_pid);
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
		PID_Controller_Set_Setpoint(&g_temp_pid, v);
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

	/* ---- PID ---- */
	if (strcmp(body, "PID") == 0)
	{
		float kp, ki, kd;
		char  buf[64];
		char *s_kp, *s_ki, *s_kd;
		strncpy(buf, value, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = 0;
		s_kp = strtok(buf, ",");
		s_ki = strtok(0, ",");
		s_kd = strtok(0, ",");
		if (!s_kp || !s_ki || !s_kd) return 0;
		if (!ParseFloat(s_kp, &kp)) return 0;
		if (!ParseFloat(s_ki, &ki)) return 0;
		if (!ParseFloat(s_kd, &kd)) return 0;
		g_config.pid_kp = kp;
		g_config.pid_ki = ki;
		g_config.pid_kd = kd;
		AppConfig_Apply();
		PID_Controller_Reset(&g_temp_pid);
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- CASCADE ---- */
	if (strcmp(body, "CASCADE") == 0)
	{
		float ko, mr, kpi, kii, orl;
		char  buf[64];
		char *s1, *s2, *s3, *s4, *s5;
		strncpy(buf, value, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = 0;
		s1 = strtok(buf, ","); s2 = strtok(0, ",");
		s3 = strtok(0, ",");  s4 = strtok(0, ",");
		s5 = strtok(0, ",");  /* optional 5th: output_rate_limit */
		if (!s1 || !s2 || !s3 || !s4) return 0;
		if (!ParseFloat(s1, &ko))  return 0;
		if (!ParseFloat(s2, &mr))  return 0;
		if (!ParseFloat(s3, &kpi)) return 0;
		if (!ParseFloat(s4, &kii)) return 0;
		if (s5) { if (!ParseFloat(s5, &orl) || orl < 0.0f || orl > 100.0f) return 0; }
		g_config.csc_k_outer       = ko;
		g_config.csc_max_heat_rate = mr;
		g_config.csc_kp_inner      = kpi;
		g_config.csc_ki_inner      = kii;
		if (s5) g_config.csc_output_rate_limit = orl;
		AppConfig_Apply();
		Cascade_Reset(&g_cascade);
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- HYBRID ---- */
	if (strcmp(body, "HYBRID") == 0)
	{
		float th, kp, ki, kd, dz, mn;
		int   interval = 0;
		char  buf[64];
		char *s1, *s2, *s3, *s4, *s5, *s6, *s7;
		strncpy(buf, value, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = 0;
		s1 = strtok(buf, ","); s2 = strtok(0, ",");
		s3 = strtok(0, ",");  s4 = strtok(0, ",");
		s5 = strtok(0, ",");  /* optional 5th: slow_interval */
		s6 = strtok(0, ",");  /* optional 6th: deadzone */
		s7 = strtok(0, ",");  /* optional 7th: min_output */
		if (!s1 || !s2 || !s3 || !s4) return 0;
		if (!ParseFloat(s1, &th)) return 0;
		if (!ParseFloat(s2, &kp)) return 0;
		if (!ParseFloat(s3, &ki)) return 0;
		if (!ParseFloat(s4, &kd)) return 0;
		if (s5) { if (!ParseInt(s5, &interval)) return 0; }
		if (s6) { if (!ParseFloat(s6, &dz) || dz < 0.1f || dz > 2.0f) return 0; }
		if (s7) { if (!ParseFloat(s7, &mn) || mn < 0.0f || mn > 30.0f) return 0; }
		if (th < 0.5f || th > 20.0f)  return 0;
		if (kp < 0.1f || kp > 50.0f)  return 0;
		if (ki < 0.0f || ki > 5.0f)   return 0;
		if (kd < 0.0f || kd > 10.0f)  return 0;  /* kd = Kd */
		if (interval != 0 && (interval < 3 || interval > 60)) return 0;
		g_config.hyb_threshold      = th;
		g_config.hyb_kp             = kp;
		g_config.hyb_ki             = ki;
		g_config.hyb_kd = kd;
		if (interval > 0) g_config.hyb_slow_interval = (u16)interval;
		/* 即时同步到运行时变量, 无需重启 */
		g_hybrid_kp = kp;
		g_hybrid_ki = ki;
		g_hybrid_kd = kd;
		if (s6) g_config.hyb_deadzone = dz;
		if (s7) g_config.hyb_min_output = mn;
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- NET ---- */
	if (strcmp(body, "NET") == 0)
	{
		u8   ip[4], gw[4], nm[4];
		int  port;
		char buf[64];
		char *s_ip, *s_gw, *s_nm, *s_port;
		strncpy(buf, value, sizeof(buf) - 1);
		buf[sizeof(buf) - 1] = 0;
		s_ip   = strtok(buf, ",");
		s_gw   = strtok(0, ",");
		s_nm   = strtok(0, ",");
		s_port = strtok(0, ",");
		if (!s_ip || !s_gw || !s_nm || !s_port) return 0;
		if (!ParseIp4(s_ip, ip))     return 0;
		if (!ParseIp4(s_gw, gw))     return 0;
		if (!ParseIp4(s_nm, nm))     return 0;
		if (!ParseInt(s_port, &port)) return 0;
		if (port < 1 || port > 65535) return 0;
		memcpy(g_config.eth_ip,      ip,   4);
		memcpy(g_config.eth_gateway, gw,   4);
		memcpy(g_config.eth_netmask, nm,   4);
		g_config.tcp_port = (u16)port;
		AppConfig_MarkDirty();
		return 1;
	}

	/* ---- MAC ---- */
	if (strcmp(body, "MAC") == 0)
	{
		u8 mac[6];
		if (!ParseMac(value, mac)) return 0;
		memcpy(g_config.eth_mac, mac, 6);
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
		extern int   temp_ctr_val;
		extern u8    Manual_Flag;

		if (strcmp(value, "STATE") == 0)
		{
			int goal_x10    = (int)(g_config.target_temp * 10.0f);
			int feedback_x10 = (int)(temp_feedback * 10.0f);
			sprintf(out, "STATE=MODE:%d,PWM:%d,GOAL:%d,FB:%d",
			        Manual_Flag, temp_ctr_val, goal_x10, feedback_x10);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "PID") == 0)
		{
			sprintf(out, "PID=KP:%.4f,KI:%.4f,KD:%.4f",
			        g_config.pid_kp, g_config.pid_ki, g_config.pid_kd);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "CASCADE") == 0)
		{
			sprintf(out, "CASCADE=KO:%.2f,MR:%.1f,KPI:%.1f,KII:%.1f,ORL:%.1f",
			        g_config.csc_k_outer, g_config.csc_max_heat_rate,
			        g_config.csc_kp_inner, g_config.csc_ki_inner, g_config.csc_output_rate_limit);
			AppCmd_SendFrame(out);
			return 2;
		}
		if (strcmp(value, "HYBRID") == 0)
		{
			sprintf(out, "HYBRID=TH:%.1f,KP:%.2f,KI:%.3f,KD:%.2f,DZ:%.2f,MN:%.1f,INT:%u",
			        g_config.hyb_threshold, g_config.hyb_kp,
			        g_config.hyb_ki, g_config.hyb_kd,
			        g_config.hyb_deadzone, g_config.hyb_min_output,
				        g_config.hyb_slow_interval);
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
			sprintf(out, "PID=%.4f,%.4f,%.4f",
			        g_config.pid_kp, g_config.pid_ki, g_config.pid_kd);
			AppCmd_SendFrame(out);
			sprintf(out, "CASCADE=%.2f,%.1f,%.1f,%.1f,%.1f",
			        g_config.csc_k_outer, g_config.csc_max_heat_rate,
			        g_config.csc_kp_inner, g_config.csc_ki_inner, g_config.csc_output_rate_limit);
			AppCmd_SendFrame(out);
			sprintf(out, "HYBRID=%.1f,%.2f,%.4f,%.4f,%.2f,%.1f,%u",
			        g_config.hyb_threshold, g_config.hyb_kp,
			        g_config.hyb_ki, g_config.hyb_kd,
			        g_config.hyb_deadzone, g_config.hyb_min_output,
				        g_config.hyb_slow_interval);
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
