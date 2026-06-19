#include "flash_params.h"
#include "stm32f4xx_flash.h"
#include <string.h>

#define FLASH_MAGIC             0x54454D50u   /* "TEMP" */
#define FLASH_VERSION           0x00070000u   /* v7.0: Smith predictor params */
#define FLASH_ADDRESS           ((u32)0x08060000)
#define FLASH_SECTOR            FLASH_Sector_7
#define FLASH_SAVE_DELAY_TICK   500

typedef struct {
	u32 magic;
	u32 version;

	/* A. System Control */
	u32   manual_flag;
	float target_temp;
	int   manual_pwm;
	u32   step_value;

	/* B. Transient Mode - Positional PID */
	float tran_kp;
	float tran_ki;
	float tran_kd;
	u16   tran_interval;
	float tran_sep_threshold;
	float tran_i_min_scale;
	float tran_i_full_error;
	float tran_i_limit;
	float tran_i_overshoot_leak;

	/* C. Fine-tuning Mode - Incremental PID */
	u32   fine_enable;
	float fine_kp;
	float fine_ki;
	float fine_kd;
	u16   fine_interval;
	u8    _pad2[2];
	float fine_range;
	float fine_entry_min;
	float fine_entry_max;
	u16   stable_window;
	u8    _pad3[2];
	float stable_delta;

	/* D. Shared */
	float pid_deadband;

	/* D2. Smith Predictor */
	u32   smith_enable;
	float smith_gain;
	u16   smith_tau;
	u16   smith_delay;
	float smith_blend;
	float smith_max_lead;

	/* E. Network */
	u8   eth_ip[4];
	u8   eth_gateway[4];
	u8   eth_netmask[4];
	u16  tcp_port;
	u8   eth_mac[6];
	u8   eth_dhcp;
	u8   _pad[3];

	u32  checksum;
} FlashStore;

static u8  s_dirty = 0;
static u16 s_tick  = 0;

static u32 Flash_Checksum(const FlashStore *s)
{
	const u32 *p = (const u32 *)s;
	u32 cs = 0x5A5AA5A5u;
	u16 i;
	for (i = 0; i < (sizeof(FlashStore) / 4) - 1; i++)
		cs ^= p[i];
	return cs;
}

static void Flash_Build(FlashStore *fs, const AppConfig *cfg)
{
	memset(fs, 0, sizeof(FlashStore));
	fs->magic   = FLASH_MAGIC;
	fs->version = FLASH_VERSION;

	fs->manual_flag = cfg->manual_flag;
	fs->target_temp = cfg->target_temp;
	fs->manual_pwm  = cfg->manual_pwm;
	fs->step_value  = cfg->step_value;

	fs->tran_kp        = cfg->tran_kp;
	fs->tran_ki        = cfg->tran_ki;
	fs->tran_kd        = cfg->tran_kd;
	fs->tran_interval       = cfg->tran_interval;
	fs->tran_sep_threshold  = cfg->tran_sep_threshold;
	fs->tran_i_min_scale = cfg->tran_i_min_scale;
	fs->tran_i_full_error = cfg->tran_i_full_error;
	fs->tran_i_limit = cfg->tran_i_limit;
	fs->tran_i_overshoot_leak = cfg->tran_i_overshoot_leak;

	fs->fine_kp        = cfg->fine_kp;
	fs->fine_enable    = cfg->fine_enable;
	fs->fine_ki        = cfg->fine_ki;
	fs->fine_kd        = cfg->fine_kd;
	fs->fine_interval  = cfg->fine_interval;
	fs->fine_range      = cfg->fine_range;
	fs->fine_entry_min  = cfg->fine_entry_min;
	fs->fine_entry_max  = cfg->fine_entry_max;
	fs->stable_window   = cfg->stable_window;
	fs->stable_delta    = cfg->stable_delta;

	fs->pid_deadband = cfg->pid_deadband;

	fs->smith_enable = cfg->smith_enable;
	fs->smith_gain = cfg->smith_gain;
	fs->smith_tau = cfg->smith_tau;
	fs->smith_delay = cfg->smith_delay;
	fs->smith_blend = cfg->smith_blend;
	fs->smith_max_lead = cfg->smith_max_lead;

	memcpy(fs->eth_ip,      cfg->eth_ip,      4);
	memcpy(fs->eth_gateway, cfg->eth_gateway, 4);
	memcpy(fs->eth_netmask, cfg->eth_netmask, 4);
	fs->tcp_port = cfg->tcp_port;
	memcpy(fs->eth_mac, cfg->eth_mac, 6);
	fs->eth_dhcp  = cfg->eth_dhcp;

	fs->checksum = Flash_Checksum(fs);
}

static u8 Flash_IsValid(const FlashStore *fs)
{
	if (fs->magic != FLASH_MAGIC)   return 0;
	if (fs->version != FLASH_VERSION) return 0;
	if (fs->checksum != Flash_Checksum(fs)) return 0;

	if (fs->manual_flag > 1) return 0;
	if (fs->target_temp < -10.0f || fs->target_temp > 100.0f) return 0;
	if (fs->manual_pwm < -100 || fs->manual_pwm > 100) return 0;
	if (fs->step_value != 1 && fs->step_value != 5 && fs->step_value != 10) return 0;

	if (fs->tran_kp < 0.1f   || fs->tran_kp > 50.0f)  return 0;
	if (fs->tran_ki < 0.0f   || fs->tran_ki > 5.0f)   return 0;
	if (fs->tran_kd < 0.0f   || fs->tran_kd > 10.0f)  return 0;
	if (fs->tran_interval < 1 || fs->tran_interval > 60) return 0;
	if (fs->tran_sep_threshold < 1.0f || fs->tran_sep_threshold > 30.0f) return 0;
	if (fs->tran_i_min_scale < 0.0f || fs->tran_i_min_scale > 1.0f) return 0;
	if (fs->tran_i_full_error < 0.1f || fs->tran_i_full_error > 10.0f) return 0;
	if (fs->tran_i_limit < 1.0f || fs->tran_i_limit > 300.0f) return 0;
	if (fs->tran_i_overshoot_leak < 0.0f || fs->tran_i_overshoot_leak > 1.0f) return 0;

	if (fs->fine_kp < 0.1f   || fs->fine_kp > 20.0f)  return 0;
	if (fs->fine_enable > 1) return 0;
	if (fs->fine_ki < 0.0f   || fs->fine_ki > 3.0f)   return 0;
	if (fs->fine_kd < 0.0f   || fs->fine_kd > 10.0f)  return 0;
	if (fs->fine_interval < 1 || fs->fine_interval > 60) return 0;
	if (fs->fine_range < 1.0f || fs->fine_range > 20.0f) return 0;
	if (fs->fine_entry_min < 0.1f || fs->fine_entry_min > 5.0f) return 0;
	if (fs->fine_entry_max < 1.0f  || fs->fine_entry_max > 10.0f) return 0;
	if (fs->fine_entry_min > fs->fine_entry_max) return 0;
	if (fs->stable_window < 10     || fs->stable_window > 120)   return 0;
	if (fs->stable_delta < 0.2f    || fs->stable_delta > 5.0f)   return 0;

	if (fs->pid_deadband < 0.1f || fs->pid_deadband > 2.0f) return 0;

	if (fs->smith_enable > 1) return 0;
	if (fs->smith_gain < 1.0f || fs->smith_gain > 200.0f) return 0;
	if (fs->smith_tau < 5 || fs->smith_tau > 3600) return 0;
	if (fs->smith_delay > 180) return 0;
	if (fs->smith_blend < 0.0f || fs->smith_blend > 1.0f) return 0;
	if (fs->smith_max_lead < 0.5f || fs->smith_max_lead > 30.0f) return 0;

	if (fs->tcp_port == 0 || fs->tcp_port > 65535) return 0;

	return 1;
}

static void Flash_ToConfig(AppConfig *cfg, const FlashStore *fs)
{
	cfg->manual_flag = (u8)fs->manual_flag;
	cfg->target_temp = fs->target_temp;
	cfg->manual_pwm  = fs->manual_pwm;
	cfg->step_value  = (u8)fs->step_value;

	cfg->tran_kp        = fs->tran_kp;
	cfg->tran_ki        = fs->tran_ki;
	cfg->tran_kd        = fs->tran_kd;
	cfg->tran_interval       = fs->tran_interval;
	cfg->tran_sep_threshold  = fs->tran_sep_threshold;
	cfg->tran_i_min_scale = fs->tran_i_min_scale;
	cfg->tran_i_full_error = fs->tran_i_full_error;
	cfg->tran_i_limit = fs->tran_i_limit;
	cfg->tran_i_overshoot_leak = fs->tran_i_overshoot_leak;

	cfg->fine_kp        = fs->fine_kp;
	cfg->fine_enable    = (u8)fs->fine_enable;
	cfg->fine_ki        = fs->fine_ki;
	cfg->fine_kd        = fs->fine_kd;
	cfg->fine_interval  = fs->fine_interval;
	cfg->fine_range      = fs->fine_range;
	cfg->fine_entry_min  = fs->fine_entry_min;
	cfg->fine_entry_max  = fs->fine_entry_max;
	cfg->stable_window   = fs->stable_window;
	cfg->stable_delta    = fs->stable_delta;

	cfg->pid_deadband = fs->pid_deadband;

	cfg->smith_enable = (u8)fs->smith_enable;
	cfg->smith_gain = fs->smith_gain;
	cfg->smith_tau = fs->smith_tau;
	cfg->smith_delay = fs->smith_delay;
	cfg->smith_blend = fs->smith_blend;
	cfg->smith_max_lead = fs->smith_max_lead;

	memcpy(cfg->eth_ip,      fs->eth_ip,      4);
	memcpy(cfg->eth_gateway, fs->eth_gateway, 4);
	memcpy(cfg->eth_netmask, fs->eth_netmask, 4);
	cfg->tcp_port = fs->tcp_port;
	memcpy(cfg->eth_mac, fs->eth_mac, 6);
	cfg->eth_dhcp  = fs->eth_dhcp;
}

void Flash_Param_Load_Runtime(void)
{
	const FlashStore *fs = (const FlashStore *)FLASH_ADDRESS;
	if (Flash_IsValid(fs)) {
		Flash_ToConfig(&g_config, fs);
	}
}

u8 Flash_Param_Save(const AppConfig *cfg)
{
	FlashStore fs;
	const FlashStore *cur;
	const u32 *pw;
	FLASH_Status status;
	u16 i;

	if (cfg == 0) return 0;

	Flash_Build(&fs, cfg);

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

void Flash_Param_MarkDirty(void)
{
	s_dirty = 1;
	s_tick  = FLASH_SAVE_DELAY_TICK;
}

void Flash_Param_Process(void)
{
	if (s_dirty == 0) return;
	if (s_tick > 0) { s_tick--; return; }
	if (Flash_Param_Save(&g_config))
		s_dirty = 0;
	else
		s_tick = FLASH_SAVE_DELAY_TICK;
}
