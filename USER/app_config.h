#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "sys.h"

/* ============================================================
 * 统一应用配置系统 (Unified Application Configuration)
 *
 * 所有可调参数集中管理, 支持:
 *   - 串口/网口统一指令读写
 *   - Flash 持久化存储 (自动延迟保存)
 *   - 出厂默认值一键恢复
 *   - 模块化参数分组
 * ============================================================ */

/* ============================================================
 * 1. 统一参数结构体
 * ============================================================ */
typedef struct
{
	/* ---- A. 系统控制 ---- */
	u8   manual_flag;        /* 0=自动温控, 1=手动PWM */
	float target_temp;       /* 目标温度 (°C), 自动模式使用 */
	int   manual_pwm;        /* 手动 PWM 值 [-100, 100] */
	u8   step_value;         /* 按键步进值: 1/5/10 */

	/* ---- B. 串级控制器 (Cascade) ---- */
	float csc_k_outer;       /* 外环增益: 温度误差→目标速率 */
	float csc_max_heat_rate; /* 最大升温速率 (°C/s) */
	float csc_kp_inner;      /* 内环 P 增益: 速率误差→PWM */
	float csc_ki_inner;      /* 内环 I 增益 */
	float csc_output_rate_limit;  /* ����仯������(%/s), Ĭ�� 20, 0=������ */

	/* ---- C. 近区增量式 PID (Hybrid) ---- */
	float hyb_threshold;     /* 远区->近区切换阈值(°C), 默认 5.0 */
	float hyb_deadzone;      /* ��������(��), Ĭ�� 0.5, ��Χ 0.1~2.0 */
	float hyb_kp;            /* 增量 Kp: 误差变化趋势增益 (推荐 1~8) */
	float hyb_ki;            /* 增量 Ki: 持续误差驱动增益 (推荐 0.1~1.0) */
	float hyb_kd;            /* 增量 Kd: 误差加速度阻尼 (推荐 0~5, 防过冲振荡) */
	float hyb_min_output;    /* ������СPWM(%), Ĭ�� 5, ��Χ 0~30 (�������ҵ�) */
	u16  hyb_slow_interval;  /* 慢速区更新间隔(秒), 默认 15, 范围 3~60 */

	/* ---- D. PID 参数 (保留, 兼容旧接口) ---- */
	float pid_kp;
	float pid_ki;
	float pid_kd;

	/* ---- E. 网络配置 ---- */
	u8   eth_ip[4];          /* 本机 IP, 默认 192.168.1.100 */
	u8   eth_gateway[4];     /* 网关,   默认 192.168.1.1 */
	u8   eth_netmask[4];     /* 子网掩码, 默认 255.255.255.0 */
	u16  tcp_port;           /* TCP 监听端口, 默认 8000 */
	u8   eth_mac[6];         /* MAC 地址, 默认 02:00:00:00:00:01 */
	u8   eth_dhcp;           /* 0=静态IP, 1=DHCP (预留) */

} AppConfig;

/* ============================================================
 * 2. 出厂默认值
 * ============================================================ */
extern const AppConfig g_config_defaults;

/* ============================================================
 * 3. 运行时全局配置实例
 * ============================================================ */
extern AppConfig g_config;

/* ============================================================
 * 4. API
 * ============================================================ */

/* 初始化: 先用默认值, 再尝试从 Flash 加载 */
void AppConfig_Init(void);

/* 载入出厂默认值 (保留 MAC 地址不变) */
void AppConfig_LoadDefaults(void);

/* 立即保存到 Flash, 返回 1=成功 */
u8   AppConfig_Save(void);

/* 标记为脏, 延迟 500ms 后自动保存 (多次调用会重置计时器) */
void AppConfig_MarkDirty(void);

/* 在主循环中调用, 处理延迟保存 */
void AppConfig_Process(void);

/* 将当前配置同步到各驱动模块 (PID/串级/网络等) */
void AppConfig_Apply(void);

/* ============================================================
 * 5. 统一指令分发
 *
 *    帧格式: !BODY=VALUE*XX\r\n
 *    回复格式: !ACK=OK*XX\r\n 或 !ACK=ERR*XX\r\n
 *              !响应内容*XX\r\n
 *
 *    g_cmd_from_net: 0=从串口来(回复走串口), 1=从网口来(回复走TCP)
 *
 *    返回值: 1=已处理+发ACK, 2=已处理不发ACK, 0=未识别
 * ============================================================ */
extern u8 g_cmd_from_net;   /* 调用前设置, 用于回复路由 */

int  AppCmd_Dispatch(const char *body, const char *value);

/* 发送 ACK (根据 g_cmd_from_net 路由到串口或TCP) */
void AppCmd_SendAck(u8 ok);

/* 发送结构化响应 (根据 g_cmd_from_net 路由) */
void AppCmd_SendFrame(const char *payload);

#endif /* __APP_CONFIG_H */
