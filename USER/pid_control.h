#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "sys.h"

/* ============================================================
 * 增强型 PID 控制器 (Enhanced PID Controller)
 *
 * 内置温控优化策略:
 *   1. 积分分离 (Integral Separation)   — 大误差时关闭积分，消除超调
 *   2. 变速积分 (Variable Integral)     — 误差大时减小积分系数，平滑过渡
 *   3. 前馈控制 (Feed-forward)          — 根据目标温度预估计输出，加速上升
 *   4. 设定值斜坡 (Setpoint Ramping)    — 目标温度逐步爬升，抑制超调
 *   5. 死区 (Dead Band)                 — 小误差时锁定输出，避免振荡
 *   6. 微分先行 (Derivative on Measurement) — 对测量值微分，避免微分冲击
 *   7. 条件积分抗饱和 (Conditional Anti-windup) — 输出饱和时不积分
 *   8. 输出变化率限制 (Output Rate Limit)     — 限制每次输出变化幅度
 * ============================================================ */
typedef struct
{
	/* ---- PID 参数 ---- */
	float kp;                          // 比例系数
	float ki;                          // 积分系数
	float kd;                          // 微分系数

	/* ---- 内部状态 ---- */
	float integral;                    // 积分累加值
	float last_error;                  // 上一次误差
	float last_feedback;               // 上一次测量值（微分先行用）
	float last_output;                 // 上一次输出值（变化率限制/死区锁定用）
	u8    has_last_error;              // 是否有历史误差

	/* ---- 限幅参数 ---- */
	float integral_limit;              // 积分限幅 (±)
	float output_limit;                // 输出限幅 (±)
	float output_rate_limit;           // 输出变化率限制 (每步最大变化量), 0=不限制

	/* ---- 积分分离 (Integral Separation) ---- */
	float separation_threshold;        // 积分分离阈值 — |error| 超过此值时 Ki 置 0
	u8    separation_enabled;          // 是否启用积分分离 (1=启用)

	/* ---- 变速积分 (Variable Integral Gain) ---- */
	u8    variable_ki_enabled;         // 是否启用变速积分
	float vi_small_error;              // 变速积分: "小误差"阈值
	float vi_big_error;                // 变速积分: "大误差"阈值

	/* ---- 死区 (Dead Band) ---- */
	float deadband;                    // 死区宽度 — |error| < deadband 时输出锁定

	/* ---- 前馈控制 (Feed-forward) ---- */
	float ff_gain;                     // 前馈增益 — 输出 = PID_out + ff_gain * setpoint
	u8    ff_enabled;                  // 是否启用前馈

	/* ---- 设定值斜坡 (Setpoint Ramping) ---- */
	float ramp_rate;                   // 斜坡速率 (°C/s), 0=不启用
	float ramp_current;               // 当前斜坡目标（内部）
	float ramp_final;                  // 最终目标（内部）
	u8    ramp_active;                 // 斜坡是否激活中

	/* ---- 微分先行 (Derivative on Measurement) ---- */
	u8    deriv_on_meas;               // 1=对测量值微分（推荐）, 0=对误差微分
} PID_Controller;

/* ---- API ---- */
void  PID_Controller_Init(PID_Controller *pid, float kp, float ki, float kd);
void  PID_Controller_Reset(PID_Controller *pid);
void  PID_Controller_Set_Gains(PID_Controller *pid, float kp, float ki, float kd);
void  PID_Controller_Set_Setpoint(PID_Controller *pid, float new_setpoint);
float PID_Controller_Step(PID_Controller *pid, float setpoint, float feedback, float dt_s);

/* ---- 便捷宏：推荐温控配置 ---- */
void PID_Config_Thermal_Default(PID_Controller *pid, float kp, float ki, float kd);

#endif
