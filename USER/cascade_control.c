#include "cascade_control.h"

/* ---- 辅助: C90 兼容 ---- */
static float Clamp(float v, float limit) {
	if (v > limit)  return limit;
	if (v < -limit) return -limit;
	return v;
}

static float FabsF(float x) {
	if (x < 0.0f) return -x;
	return x;
}

/* ============================================================
 * 初始化
 * ============================================================ */
void Cascade_Init(Cascade_Controller *cc)
{
	if (cc == 0) return;

	/* 外环: 温度误差 → 目标升温速率 */
	cc->k_outer         = 0.5f;    /* error=10°C → target_rate=5°C/s (会被max截断) */
	cc->max_heat_rate   = 3.0f;    /* 最大升温速率 °C/s */
	cc->min_output_temp = -999.0f; /* 低于此温度强制满功率, 默认禁用 */

	/* 内环: 速率误差 → PWM (高增益) */
	cc->kp_inner = 40.0f;  /* 速率误差 1°C/s → 40% PWM */
	cc->ki_inner = 12.0f;  /* 快速消除速率稳态误差 */
	cc->kd_inner = 0.0f;

	/* 内环状态 */
	cc->inner_integral       = 0.0f;
	cc->inner_integral_limit = 100.0f;
	cc->last_feedback        = 0.0f;
	cc->last_output          = 0.0f;
	cc->has_last_sample      = 0;

	/* 通用 */
	cc->output_limit      = 100.0f;
	cc->output_rate_limit = 20.0f;  /* 每秒最多变 20%, 防大惯性突变 */
	cc->deadband          = 0.1f;   /* ±0.1°C 温度死区 */
}

void Cascade_Reset(Cascade_Controller *cc)
{
	if (cc == 0) return;

	cc->inner_integral  = 0.0f;
	cc->last_feedback   = 0.0f;
	cc->last_output     = 0.0f;
	cc->has_last_sample = 0;
}

/* ============================================================
 * 核心: 双回路串级
 *
 * 外环: target_rate = clamp(k_outer * temp_error, 0, max_heat_rate)
 * 内环: rate_error = target_rate - actual_rate
 *       PWM = kp_inner * rate_error + ki_inner * ∫rate_error
 *
 * 死区: |temp_error| < deadband → 输出冻结 (防 PWM 抖动)
 *       此死区极小 (0.1°C)，只在精确定温时生效
 * ============================================================ */
float Cascade_Step(Cascade_Controller *cc, float setpoint, float feedback, float dt_s)
{
	float temp_error;
	float target_rate;
	float actual_rate;
	float rate_error;
	float output;
	float pre_sat_output;

	if ((cc == 0) || (dt_s <= 0.0f))
		return 0.0f;

	/* ======== 外环: 温度误差 → 目标速率 ======== */
	temp_error = setpoint - feedback;

	/* 死区: 误差极小 → 输出冻结 */
	if (cc->deadband > 0.0f && FabsF(temp_error) < cc->deadband)
	{
		cc->last_feedback = feedback;
		return cc->last_output;
	}

	/* 强制全功率: 极低温直接拉满 */
	if (feedback < cc->min_output_temp)
	{
		cc->last_feedback = feedback;
		cc->last_output   = cc->output_limit;
		/* 清零积分防饱和 */
		cc->inner_integral = 0.0f;
		return cc->output_limit;
	}

	/* 外环计算目标速率 (仅加热, 下限=0) */
	target_rate = cc->k_outer * temp_error;
	if (target_rate > cc->max_heat_rate)
		target_rate = cc->max_heat_rate;
	if (target_rate < 0.0f)
		target_rate = 0.0f;   /* 被动散热, 不主动制冷 */

	/* ======== 内环: 速率误差 → PWM ======== */

	/* 计算实际升温速率 */
	if (cc->has_last_sample == 0)
	{
		actual_rate = 0.0f;
		cc->has_last_sample = 1;
	}
	else
	{
		actual_rate = (feedback - cc->last_feedback) / dt_s;
	}

	rate_error = target_rate - actual_rate;

	/* 内环抗饱和: 输出饱和时不往饱和方向积分 */
	pre_sat_output = cc->kp_inner * rate_error
	                 + cc->ki_inner * cc->inner_integral;

	if ((pre_sat_output > -cc->output_limit
	     && pre_sat_output < cc->output_limit)
	    || (pre_sat_output >= cc->output_limit && rate_error < 0.0f)
	    || (pre_sat_output <= 0.0f && rate_error > 0.0f))
	{
		cc->inner_integral += rate_error * dt_s;
		cc->inner_integral = Clamp(cc->inner_integral,
		                           cc->inner_integral_limit);
	}

	/* 内环 PI 输出 */
	output = cc->kp_inner * rate_error
	         + cc->ki_inner * cc->inner_integral;

	/* 输出限幅 [0, output_limit] */
	if (output < 0.0f) output = 0.0f;
	if (output > cc->output_limit) output = cc->output_limit;

	/* 输出变化率限制 (可选) */
	if (cc->output_rate_limit > 0.0f && cc->has_last_sample != 0)
	{
		float max_delta = cc->output_rate_limit * dt_s;
		if (output - cc->last_output > max_delta)
			output = cc->last_output + max_delta;
		else if (cc->last_output - output > max_delta)
			output = cc->last_output - max_delta;
	}

	/* 保存状态 */
	cc->last_feedback = feedback;
	cc->last_output   = output;

	return output;
}
