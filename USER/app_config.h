#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "sys.h"

/* ============================================================
 * Unified Application Configuration
 * ============================================================ */

/* ============================================================
 * 1. Config Struct
 *
 * Field order (for Flash serialization):
 *   manual_flag, target_temp, manual_pwm, step_value,
 *   tran_kp, tran_ki, tran_kd, tran_interval, tran_sep_threshold,
 *   tran_i_min_scale, tran_i_full_error, tran_i_limit, tran_i_overshoot_leak,
 *   fine_enable, fine_kp, fine_ki, fine_kd, fine_interval, fine_range,
 *   pid_deadband,
 *   smith_enable, smith_gain, smith_tau, smith_delay, smith_blend, smith_max_lead,
 *   eth_ip, eth_gateway, eth_netmask, tcp_port, eth_mac, eth_dhcp
 * ============================================================ */
typedef struct
{
	/* ---- A. System Control ---- */
	u8   manual_flag;        /* 0=Auto, 1=Manual PWM */
	float target_temp;       /* Target temperature (degC) */
	int   manual_pwm;        /* Manual PWM [-100, 100] */
	u8   step_value;         /* Key step: 1/5/10 */

	/* ---- B. Transient Mode - Positional PID ---- */
	float tran_kp;           /* Kp, default 3.0, range 0.1~50 */
	float tran_ki;           /* Ki, default 0.3, range 0~5 */
	float tran_kd;           /* Kd, default 1.0, range 0~10 */
	u16   tran_interval;       /* Update interval (sec), default 3, range 1~60 */
	float tran_sep_threshold;  /* Integral separation threshold (degC), default 10, range 1~30 */
	float tran_i_min_scale;    /* Adaptive integral min speed, default 0.2, range 0~1 */
	float tran_i_full_error;   /* Full-speed integral error band (degC), default 2, range 0.1~10 */
	float tran_i_limit;        /* Integral clamp, default 80, range 1~300 */
	float tran_i_overshoot_leak; /* Integral leak when overshoot/deadband, default 0.85, range 0~1 */

	/* ---- C. Fine-tuning Mode - Incremental PID ---- */
	u8    fine_enable;       /* 0=disable fine mode, 1=enable fine mode */
	float fine_kp;           /* Kp, default 1.5, range 0.1~20 */
	float fine_ki;           /* Ki, default 0.1, range 0~3 */
	float fine_kd;           /* Kd, default 2.0, range 0~10 */
	u16   fine_interval;     /* Update interval (sec), default 8, range 1~60 */
	float fine_range;        /* Max output change per step (%), default 5, range 1~20 */
	float fine_entry_min;    /* Min |error| to enter fine mode (degC), default 1.0, range 0.1~5 */
	float fine_entry_max;    /* Max |error| to enter fine mode (degC), default 3.0, range 1~10 */
	u16   stable_window;     /* Stability detection window (sec), default 20, range 10~120 */
	float stable_delta;      /* Max temp change in window = stable (degC), default 1.0, range 0.2~5 */

	/* ---- D. Shared ---- */
	float pid_deadband;      /* Deadband (degC), default 0.3, range 0.1~2.0 */

	/* ---- D2. Smith Predictor ---- */
	u8    smith_enable;      /* 0=disable, 1=enable Smith predictor */
	float smith_gain;        /* Model steady temp change at 100% output (degC), range 1~200 */
	u16   smith_tau;         /* Model time constant (sec), range 5~3600 */
	u16   smith_delay;       /* Pure delay (sec), range 0~180 */
	float smith_blend;       /* Predictor blend, range 0~1 */
	float smith_max_lead;    /* Max predictor correction (degC), range 0.5~30 */

	/* ---- E. Network ---- */
	u8   eth_ip[4];          /* Static IP, default 192.168.1.100 */
	u8   eth_gateway[4];     /* Gateway,  default 192.168.1.1 */
	u8   eth_netmask[4];     /* Netmask,  default 255.255.255.0 */
	u16  tcp_port;           /* TCP port,  default 8000 */
	u8   eth_mac[6];         /* MAC address, default 02:00:00:00:00:01 */
	u8   eth_dhcp;           /* 0=Static IP, 1=DHCP (reserved) */

} AppConfig;

/* ============================================================
 * 2. Factory Defaults
 * ============================================================ */
extern const AppConfig g_config_defaults;

/* ============================================================
 * 3. Runtime Instance
 * ============================================================ */
extern AppConfig g_config;

/* ============================================================
 * 4. API
 * ============================================================ */
void AppConfig_Init(void);
void AppConfig_LoadDefaults(void);
u8   AppConfig_Save(void);
void AppConfig_MarkDirty(void);
void AppConfig_Process(void);
void AppConfig_Apply(void);

/* ============================================================
 * 5. Command Dispatch
 *   Frame: !BODY=VALUE*XX\r\n
 *   g_cmd_from_net: 0=UART, 1=TCP
 *   Return: 1=handled+ACK, 2=handled no ACK, 0=unrecognized
 * ============================================================ */
extern u8 g_cmd_from_net;

int  AppCmd_Dispatch(const char *body, const char *value);
void AppCmd_SendAck(u8 ok);
void AppCmd_SendFrame(const char *payload);

#endif /* __APP_CONFIG_H */
