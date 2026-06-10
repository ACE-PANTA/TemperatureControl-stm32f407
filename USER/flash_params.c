#include "flash_params.h"
#include "stm32f4xx_flash.h"
#include <string.h>

/* ============================================================
 * Flash 布局
 * ============================================================ */
#define FLASH_MAGIC             0x54454D50u   /* "TEMP" */
#define FLASH_VERSION           0x00020002u   /* v2.0.2 + hyb deadzone/min_output + csc rate_limit */
#define FLASH_ADDRESS           ((u32)0x08060000)
#define FLASH_SECTOR            FLASH_Sector_7
#define FLASH_SAVE_DELAY_TICK   500           /* 500ms 延迟去抖 */

/* ============================================================
 * Flash 存储结构 (直接从 AppConfig 序列化)
 * ============================================================ */
typedef struct {
	u32 magic;
	u32 version;

	/* A. 系统控制 */
	u32   manual_flag;        /* u8 → u32 对齐 */
	float target_temp;
	int   manual_pwm;
	u32   step_value;

	/* B. 串级控制器 */
	float csc_k_outer;
	float csc_max_heat_rate;
	float csc_kp_inner;
	float csc_ki_inner;
	float csc_output_rate_limit;

	/* C. 混合控制器 */
	float hyb_threshold;
	float hyb_deadzone;
	float hyb_kp;
	float hyb_ki;
	float hyb_kd;
	float hyb_min_output;
	u16   hyb_slow_interval;
	u8    _pad1[2];            /* 对齐 */

	/* D. PID */
	float pid_kp;
	float pid_ki;
	float pid_kd;

	/* E. 网络 */
	u8   eth_ip[4];
	u8   eth_gateway[4];
	u8   eth_netmask[4];
	u16  tcp_port;
	u8   eth_mac[6];
	u8   eth_dhcp;
	u8   _pad[3];             /* 4字节对齐 */

	u32  checksum;
} FlashStore;

/* ============================================================
 * 脏标记 + 延迟写入计时器
 * ============================================================ */
static u8  s_dirty = 0;
static u16 s_tick  = 0;

/* ============================================================
 * 校验和 (异或 + 种子)
 * ============================================================ */
static u32 Flash_Checksum(const FlashStore *s)
{
	const u32 *p = (const u32 *)s;
	u32 cs = 0x5A5AA5A5u;
	u16 i;
	/* 校验覆盖除 checksum 字段外的全部 */
	for (i = 0; i < (sizeof(FlashStore) / 4) - 1; i++)
		cs ^= p[i];
	return cs;
}

/* ============================================================
 * AppConfig → FlashStore
 * ============================================================ */
static void Flash_Build(FlashStore *fs, const AppConfig *cfg)
{
	memset(fs, 0, sizeof(FlashStore));
	fs->magic   = FLASH_MAGIC;
	fs->version = FLASH_VERSION;

	/* A */
	fs->manual_flag = cfg->manual_flag;
	fs->target_temp = cfg->target_temp;
	fs->manual_pwm  = cfg->manual_pwm;
	fs->step_value  = cfg->step_value;

	/* B */
	fs->csc_k_outer       = cfg->csc_k_outer;
	fs->csc_max_heat_rate = cfg->csc_max_heat_rate;
	fs->csc_kp_inner      = cfg->csc_kp_inner;
	fs->csc_ki_inner      = cfg->csc_ki_inner;
	fs->csc_output_rate_limit = cfg->csc_output_rate_limit;

	/* C */
	fs->hyb_threshold      = cfg->hyb_threshold;
	fs->hyb_deadzone       = cfg->hyb_deadzone;
	fs->hyb_kp             = cfg->hyb_kp;
	fs->hyb_ki             = cfg->hyb_ki;
	fs->hyb_kd = cfg->hyb_kd;
	fs->hyb_min_output      = cfg->hyb_min_output;
	fs->hyb_slow_interval  = cfg->hyb_slow_interval;

	/* D */
	fs->pid_kp = cfg->pid_kp;
	fs->pid_ki = cfg->pid_ki;
	fs->pid_kd = cfg->pid_kd;

	/* E */
	memcpy(fs->eth_ip,      cfg->eth_ip,      4);
	memcpy(fs->eth_gateway, cfg->eth_gateway, 4);
	memcpy(fs->eth_netmask, cfg->eth_netmask, 4);
	fs->tcp_port = cfg->tcp_port;
	memcpy(fs->eth_mac, cfg->eth_mac, 6);
	fs->eth_dhcp  = cfg->eth_dhcp;

	fs->checksum = Flash_Checksum(fs);
}

/* ============================================================
 * 校验 Flash 数据是否有效
 * ============================================================ */
static u8 Flash_IsValid(const FlashStore *fs)
{
	/* Magic & version */
	if (fs->magic != FLASH_MAGIC)   return 0;
	if (fs->version != FLASH_VERSION) return 0;

	/* Checksum */
	if (fs->checksum != Flash_Checksum(fs)) return 0;

	/* 范围校验: 手动标志 */
	if (fs->manual_flag > 1) return 0;

	/* 范围校验: 温度 */
	if (fs->target_temp < -10.0f || fs->target_temp > 100.0f) return 0;

	/* 范围校验: PWM */
	if (fs->manual_pwm < -100 || fs->manual_pwm > 100) return 0;

	/* 范围校验: 步进 */
	if (fs->step_value != 1 && fs->step_value != 5 && fs->step_value != 10) return 0;

	/* 范围校验: 串级参数 */
	if (fs->csc_k_outer < 0.01f       || fs->csc_k_outer > 10.0f)  return 0;
	if (fs->csc_max_heat_rate < 0.1f  || fs->csc_max_heat_rate > 20.0f) return 0;
	if (fs->csc_kp_inner < 1.0f       || fs->csc_kp_inner > 200.0f) return 0;
	if (fs->csc_ki_inner < 0.0f       || fs->csc_ki_inner > 100.0f) return 0;
	if (fs->csc_output_rate_limit < 0.0f || fs->csc_output_rate_limit > 100.0f) return 0;

	/* 范围校验: 混合控制器 */
	if (fs->hyb_threshold < 0.5f      || fs->hyb_threshold > 20.0f) return 0;
	if (fs->hyb_deadzone < 0.1f       || fs->hyb_deadzone > 2.0f)  return 0;
	if (fs->hyb_kp < 0.1f             || fs->hyb_kp > 50.0f)       return 0;
	if (fs->hyb_ki < 0.0f             || fs->hyb_ki > 5.0f)        return 0;
	if (fs->hyb_kd < 0.0f || fs->hyb_kd > 10.0f) return 0;
	if (fs->hyb_min_output < 0.0f  || fs->hyb_min_output > 30.0f)  return 0;
	if (fs->hyb_slow_interval < 3 || fs->hyb_slow_interval > 60)         return 0;

	/* 范围校验: PID */
	if (fs->pid_kp < -1000.0f || fs->pid_kp > 1000.0f) return 0;
	if (fs->pid_ki < -1000.0f || fs->pid_ki > 1000.0f) return 0;
	if (fs->pid_kd < -1000.0f || fs->pid_kd > 1000.0f) return 0;

	/* 范围校验: 网络 */
	if (fs->tcp_port == 0 || fs->tcp_port > 65535) return 0;

	return 1;
}

/* ============================================================
 * FlashStore → AppConfig (覆盖非默认字段)
 * ============================================================ */
static void Flash_ToConfig(AppConfig *cfg, const FlashStore *fs)
{
	/* A */
	cfg->manual_flag = (u8)fs->manual_flag;
	cfg->target_temp = fs->target_temp;
	cfg->manual_pwm  = fs->manual_pwm;
	cfg->step_value  = (u8)fs->step_value;

	/* B */
	cfg->csc_k_outer       = fs->csc_k_outer;
	cfg->csc_max_heat_rate = fs->csc_max_heat_rate;
	cfg->csc_kp_inner      = fs->csc_kp_inner;
	cfg->csc_ki_inner      = fs->csc_ki_inner;
	cfg->csc_output_rate_limit = fs->csc_output_rate_limit;

	/* C */
	cfg->hyb_threshold      = fs->hyb_threshold;
	cfg->hyb_deadzone       = fs->hyb_deadzone;
	cfg->hyb_kp             = fs->hyb_kp;
	cfg->hyb_ki             = fs->hyb_ki;
	cfg->hyb_kd = fs->hyb_kd;
	cfg->hyb_min_output      = fs->hyb_min_output;
	cfg->hyb_slow_interval  = fs->hyb_slow_interval;

	/* D */
	cfg->pid_kp = fs->pid_kp;
	cfg->pid_ki = fs->pid_ki;
	cfg->pid_kd = fs->pid_kd;

	/* E */
	memcpy(cfg->eth_ip,      fs->eth_ip,      4);
	memcpy(cfg->eth_gateway, fs->eth_gateway, 4);
	memcpy(cfg->eth_netmask, fs->eth_netmask, 4);
	cfg->tcp_port = fs->tcp_port;
	memcpy(cfg->eth_mac, fs->eth_mac, 6);
	cfg->eth_dhcp  = fs->eth_dhcp;
}

/* ============================================================
 * 公共 API
 * ============================================================ */

/* 从 Flash 加载到 g_config */
void Flash_Param_Load_Runtime(void)
{
	const FlashStore *fs = (const FlashStore *)FLASH_ADDRESS;

	if (Flash_IsValid(fs))
	{
		Flash_ToConfig(&g_config, fs);
	}
	/* 无效则保持 AppConfig_Init 中设置的默认值 */
}

/* 保存 g_config 到 Flash */
u8 Flash_Param_Save(const AppConfig *cfg)
{
	FlashStore fs;
	const FlashStore *cur;
	const u32 *pw;
	FLASH_Status status;
	u16 i;

	if (cfg == 0) return 0;

	Flash_Build(&fs, cfg);

	/* 与当前 Flash 内容比较, 相同则跳过写入 (延长 Flash 寿命) */
	cur = (const FlashStore *)FLASH_ADDRESS;
	if (memcmp(cur, &fs, sizeof(FlashStore)) == 0)
		return 1;

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR
	                | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	status = FLASH_EraseSector(FLASH_SECTOR, VoltageRange_3);
	if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }

	pw = (const u32 *)&fs;
	for (i = 0; i < (sizeof(FlashStore) / 4); i++)
	{
		status = FLASH_ProgramWord(FLASH_ADDRESS + (i * 4), pw[i]);
		if (status != FLASH_COMPLETE) { FLASH_Lock(); return 0; }
	}

	FLASH_Lock();
	return 1;
}

/* 标记脏 (多次调用自动重置计时器) */
void Flash_Param_MarkDirty(void)
{
	s_dirty = 1;
	s_tick  = FLASH_SAVE_DELAY_TICK;
}

/* 主循环调用 */
void Flash_Param_Process(void)
{
	if (s_dirty == 0) return;

	if (s_tick > 0)
	{
		s_tick--;
		return;
	}

	if (Flash_Param_Save(&g_config))
	{
		s_dirty = 0;
	}
	else
	{
		/* 写入失败, 延迟重试 */
		s_tick = FLASH_SAVE_DELAY_TICK;
	}
}
