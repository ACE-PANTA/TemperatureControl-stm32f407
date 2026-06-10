#include "pid_control.h"
#include <math.h>

/* ---- 内部辅助函数 ---- */

static float PID_Clamp(float value, float limit)
{
	if (value > limit)  return limit;
	if (value < -limit) return -limit;
	return value;
}

/* 变速积分系数：|error| 越小，系数越接近 1.0 */
static float PID_Compute_Variable_Ki_Scale(float abs_error,
                                           float small_err,
                                           float big_err)
{
	if (abs_error <= small_err)
	{
		return 1.0f;    /* 小误差: 全积分 */
	}
	if (abs_error >= big_err)
	{
		return 0.0f;    /* 大误差: 零积分 */
	}
	/* 中间区域线性插值: (big - |error|) / (big - small) */
	return (big_err - abs_error) / (big_err - small_err);
}

/* 条件积分抗饱和判断：当输出饱和且误差方向与饱和方向一致时暂停积分 */
static u8 PID_Should_Integrate(float output, float output_limit, float error)
{
	/* 输出未饱和 → 允许积分 */
	if ((output > -output_limit) && (output < output_limit))
	{
		return 1;
	}
	/* 饱和在上限，但误差为负（需要减小输出）→ 允许积分 */
	if ((output >= output_limit) && (error < 0.0f))
	{
		return 1;
	}
	/* 饱和在下限，但误差为正（需要增大输出）→ 允许积分 */
	if ((output <= -output_limit) && (error > 0.0f))
	{
		return 1;
	}
	/* 饱和且误差方向与饱和一致 → 停止积分 */
	return 0;
}

/* ============================================================
 * 设置值斜坡更新
 * 每步向最终目标靠近 ramp_rate * dt_s 度
 * 返回当前步的有效设定值
 * ============================================================ */
static float PID_Ramp_Update(PID_Controller *pid, float dt_s)
{
	float step;
	float diff;

	if (pid->ramp_rate <= 0.0f)
	{
		pid->ramp_active = 0;
		return pid->ramp_final;
	}

	step = pid->ramp_rate * dt_s;
	diff = pid->ramp_final - pid->ramp_current;

	if (fabsf(diff) <= step)
	{
		/* 已到达最终目标 */
		pid->ramp_current = pid->ramp_final;
		pid->ramp_active = 0;
	}
	else
	{
		if (diff > 0.0f)
			pid->ramp_current += step;
		else
			pid->ramp_current -= step;
	}

	return pid->ramp_current;
}

/* ============================================================
 * 公共 API
 * ============================================================ */

void PID_Controller_Init(PID_Controller *pid, float kp, float ki, float kd)
{
	if (pid == 0) return;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;

	pid->integral     = 0.0f;
	pid->last_error    = 0.0f;
	pid->last_feedback = 0.0f;
	pid->last_output   = 0.0f;
	pid->has_last_error = 0;

	/* 限幅默认值 */
	pid->integral_limit    = 100.0f;
	pid->output_limit      = 100.0f;
	pid->output_rate_limit = 30.0f;   /* 每秒最多变化 30% 占空比 */

	/* 积分分离: |error| > 10°C 时关闭积分 */
	pid->separation_threshold = 10.0f;
	pid->separation_enabled   = 1;

	/* 变速积分: 误差 < 1°C 全积分, > 10°C 零积分, 区间内线性 */
	pid->variable_ki_enabled = 0;     /* 默认关闭，用积分分离即可 */
	pid->vi_small_error       = 1.0f;
	pid->vi_big_error         = 10.0f;

	/* 死区: 误差 < 0.3°C 时锁定输出，避免微小振荡 */
	pid->deadband = 0.3f;

	/* 前馈: 默认关闭，需根据系统标定 */
	pid->ff_gain    = 0.0f;
	pid->ff_enabled = 0;

	/* 设定值斜坡: 默认关闭 */
	pid->ramp_rate    = 0.0f;
	pid->ramp_current = 0.0f;
	pid->ramp_final   = 0.0f;
	pid->ramp_active  = 0;

	/* 微分先行: 默认开启（对测量值微分，避免设定值突变时的微分冲击） */
	pid->deriv_on_meas = 1;
}

void PID_Controller_Reset(PID_Controller *pid)
{
	if (pid == 0) return;

	pid->integral       = 0.0f;
	pid->last_error      = 0.0f;
	pid->last_feedback   = 0.0f;
	pid->last_output     = 0.0f;
	pid->has_last_error  = 0;
	pid->ramp_active     = 0;
	pid->ramp_current    = 0.0f;
	pid->ramp_final      = 0.0f;
}

void PID_Controller_Set_Gains(PID_Controller *pid, float kp, float ki, float kd)
{
	if (pid == 0) return;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/* 设定新的目标温度（自动触发斜坡） */
void PID_Controller_Set_Setpoint(PID_Controller *pid, float new_setpoint)
{
	if (pid == 0) return;

	if (pid->ramp_rate > 0.0f && pid->has_last_error != 0)
	{
		/* 启用斜坡：从当前反馈温度或当前斜坡位置开始爬升 */
		pid->ramp_final   = new_setpoint;
		/* 如果斜坡未激活，从当前测量值开始 */
		if (!pid->ramp_active)
		{
			pid->ramp_current = pid->last_feedback;
		}
		pid->ramp_active = 1;
	}
	else
	{
		/* 无斜坡：直接设置目标 */
		pid->ramp_final   = new_setpoint;
		pid->ramp_current = new_setpoint;
		pid->ramp_active  = 0;
	}
}

/* 推荐温控默认配置 */
void PID_Config_Thermal_Default(PID_Controller *pid, float kp, float ki, float kd)
{
	if (pid == 0) return;

	PID_Controller_Init(pid, kp, ki, kd);

	/* 温控推荐开启的特性 */
	pid->separation_enabled  = 1;      /* 积分分离 */
	pid->separation_threshold = 5.0f;  /* >5°C 关闭积分 */
	pid->deadband             = 0.2f;  /* ±0.2°C 死区 */
	pid->deriv_on_meas        = 1;     /* 微分先行 */
	pid->output_rate_limit    = 15.0f; /* 每秒最多变 15% */

	/* 前馈和斜坡默认关闭，需要根据实际系统标定后开启 */
	pid->ff_enabled = 0;
	pid->ramp_rate  = 0.0f;
}

/* ============================================================
 * 核心: PID 一步计算
 *
 * 处理流程:
 *   1. 设定值斜坡更新 → effective_sp
 *   2. 计算误差
 *   3. 死区判断（锁定输出）
 *   4. 积分分离 / 变速积分 → effective_ki
 *   5. 微分计算（微分先行 or 对误差微分）
 *   6. 条件积分抗饱和 → 积分更新
 *   7. PID 输出 = P + I(变速) + D + 前馈
 *   8. 输出限幅
 *   9. 输出变化率限制
 * ============================================================ */
float PID_Controller_Step(PID_Controller *pid, float setpoint, float feedback, float dt_s)
{
	float effective_sp;
	float error;
	float abs_error;
	float effective_ki;
	float derivative;
	float output;
	float output_before_clamp;
	float abs_output_diff;

	if ((pid == 0) || (dt_s <= 0.0f))
	{
		return 0.0f;
	}

	/* ---- 1. 设定值斜坡 ---- */
	if (pid->ramp_active)
	{
		/* 检查最终目标是否改变 */
		if (pid->ramp_final != setpoint)
		{
			pid->ramp_final = setpoint;
		}
		effective_sp = PID_Ramp_Update(pid, dt_s);
	}
	else
	{
		effective_sp = setpoint;
	}

	/* ---- 2. 误差 ---- */
	error     = effective_sp - feedback;
	abs_error = fabsf(error);

	/* ---- 3. 死区判断 ---- */
	if (abs_error < pid->deadband)
	{
		/* 误差在死区内，锁定输出不变，但继续累积积分以消除稳态误差 */
		/* 注: 积分仍然正常累积，下次出死区时 P 项会正确响应 */
		/* 但输出冻结，避免继电器/PWM 频繁切换 */
		/* 这里我们做一个折中：允许积分缓慢累积但输出不变 */
		/* 如果积分分离开启，死区内积分继续工作（误差小） */
		if (pid->separation_enabled == 0 || abs_error <= pid->separation_threshold)
		{
			pid->integral += error * dt_s;
			pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
		}
		pid->last_error    = error;
		pid->last_feedback = feedback;
		return pid->last_output;
	}

	/* ---- 4. 积分分离 & 变速积分 → effective_ki ---- */
	effective_ki = pid->ki;

	if (pid->separation_enabled && abs_error > pid->separation_threshold)
	{
		/* 积分分离: 大误差时完全关闭积分，避免积分饱和和超调 */
		effective_ki = 0.0f;
		/* 同时清零历史积分，确保切回时从零开始 */
		pid->integral = 0.0f;
	}
	else if (pid->variable_ki_enabled)
	{
		/* 变速积分: 误差越大积分系数越小（平滑版的积分分离） */
		float scale = PID_Compute_Variable_Ki_Scale(abs_error,
		                                             pid->vi_small_error,
		                                             pid->vi_big_error);
		effective_ki = pid->ki * scale;
	}

	/* ---- 5. 微分计算 ---- */
	if (pid->has_last_error == 0)
	{
		derivative          = 0.0f;
		pid->has_last_error = 1;
	}
	else
	{
		if (pid->deriv_on_meas)
		{
			/* 微分先行: 对测量值微分，避免设定值突变时产生微分冲击
			 *   derivative = -d(feedback)/dt = -(fb_now - fb_last) / dt
			 *   等价于: 只对过程变量变化做阻尼 */
			derivative = -(feedback - pid->last_feedback) / dt_s;
		}
		else
		{
			/* 标准误差微分 */
			derivative = (error - pid->last_error) / dt_s;
		}
	}

	/* ---- 6. 条件积分抗饱和 ---- */
	/* 先计算 P + D（不含 I），用于判断是否会饱和 */
	output_before_clamp = (pid->kp * error)
	                      + (effective_ki * pid->integral)
	                      + (pid->kd * derivative);

	if (PID_Should_Integrate(output_before_clamp, pid->output_limit, error))
	{
		pid->integral += error * dt_s;
		pid->integral = PID_Clamp(pid->integral, pid->integral_limit);
	}

	/* ---- 7. PID 输出 + 前馈 ---- */
	output = (pid->kp * error)
	         + (effective_ki * pid->integral)
	         + (pid->kd * derivative);

	/* 前馈: 根据目标温度预估算输出功率
	 * 例如: ff_gain=2.0, setpoint=30°C → 额外加 60 的 PWM
	 * 合理的前馈值能大幅缩短上升时间且不引起超调
	 * 标定方法: 稳定在某温度 T 时，记录稳态 PWM 值 P，
	 *          则 ff_gain ≈ P / T (假设 0°C 时 PWM≈0) */
	if (pid->ff_enabled)
	{
		output += pid->ff_gain * effective_sp;
	}

	/* ---- 8. 输出限幅 ---- */
	output = PID_Clamp(output, pid->output_limit);

	/* ---- 9. 输出变化率限制 ---- */
	if (pid->output_rate_limit > 0.0f && pid->has_last_error != 0)
	{
		abs_output_diff = fabsf(output - pid->last_output);
		if (abs_output_diff > pid->output_rate_limit * dt_s)
		{
			/* 限制变化速率 */
			if (output > pid->last_output)
				output = pid->last_output + pid->output_rate_limit * dt_s;
			else
				output = pid->last_output - pid->output_rate_limit * dt_s;
		}
	}

	/* ---- 10. 保存状态 ---- */
	pid->last_error    = error;
	pid->last_feedback = feedback;
	pid->last_output   = output;

	return output;
}
