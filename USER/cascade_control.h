#ifndef __CASCADE_CONTROL_H
#define __CASCADE_CONTROL_H

#include "sys.h"

/* ============================================================
 * 双回路串级温控器 (Cascade Temperature Controller)
 *
 * 架构:
 *   外环 (温度 → 目标升温速率):
 *     target_rate = K_outer × (setpoint - feedback)
 *     限幅于 [0, max_heat_rate]（被动散热，不强制制冷）
 *
 *   内环 (升温速率 → PWM 占空比):
 *     rate_error = target_rate - actual_rate
 *     用高增益 PI 控制速率误差 → PWM
 *
 * 为什么比单回路 PID 强:
 *   - 内环控制「升温速率」，对象响应快、线性好 → 可以用高增益不怕振荡
 *   - 外环只管「目标速率」，远离目标时自动满功率，接近时自动减速
 *   - 不受温度区间限制：30°C 和 58°C 用同一套参数都能到
 * ============================================================ */

typedef struct
{
	/* ---- 外环参数 (温度 → 目标速率) ---- */
	float k_outer;           // 外环比率增益: target_rate = k_outer * error
	float max_heat_rate;     // 最大升温速率 (°C/s), 默认 3.0
	float min_output_temp;   // 低于此温度强制满功率, 默认禁用(-999)

	/* ---- 内环参数 (速率误差 → PWM) ---- */
	float kp_inner;          // 内环比率增益 (高增益, ~30-60)
	float ki_inner;          // 内环积分增益 (~5-15)
	float kd_inner;          // 内环微分增益 (通常 0)

	/* ---- 内环状态 ---- */
	float inner_integral;       // 内环积分累加
	float inner_integral_limit; // 内环积分限幅
	float last_feedback;        // 上一次温度反馈 (算实际速率用)
	float last_output;          // 上一次输出 (变化率限制/死区用)
	u8    has_last_sample;      // 是否有历史采样数据

	/* ---- 通用 ---- */
	float output_limit;      // 输出限幅 (0~100)
	float output_rate_limit; // 输出变化率限制 (每步最大), 0=不限制
	float deadband;          // 死区: 温度误差在此范围内冻结输出
} Cascade_Controller;

/* ---- API ---- */
void  Cascade_Init(Cascade_Controller *cc);
void  Cascade_Reset(Cascade_Controller *cc);

/* 一步计算: 返回 PWM 占空比 [-100, 100]
 * setpoint: 目标温度 °C
 * feedback: 当前温度 °C
 * dt_s:     距上次调用的时间间隔 (秒) */
float Cascade_Step(Cascade_Controller *cc, float setpoint, float feedback, float dt_s);

#endif
