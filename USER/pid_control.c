#include "pid_control.h"
#include "app_config.h"
#include "pwm.h"
#include "stm32f4xx_tim.h"

#define PID_OUTPUT_MAX      100.0f
#define PID_OUTPUT_MIN        0.0f
#define PID_FINE_OUTPUT_MIN   0.0f
#define PID_ERROR_FILTER_ALPHA 0.25f
#define PID_FINE_EXIT_GAIN  2.5f
#define PID_FINE_INC_WEIGHT_MIN 0.25f
#define PID_FINE_INC_WEIGHT_MAX 0.75f
#define PID_FINE_POS_WINDOW_GAIN 2.0f
#define PID_FINE_POS_INTEGRAL_LIMIT 30.0f
#define PID_FINE_POS_INTEGRAL_LEAK 0.90f
#define FAN_PWM_MIN         15
#define FAN_PWM_MAX         95
#define STABLE_WINDOW_MAX  120
#define SMITH_DELAY_MAX    180

extern float temp_feedback;
extern int   temp_ctr_val;

float temp_control_feedback = 0.0f;
int Heat_PWM;

static float g_inc_err[3];
static float g_pid_output = 0.0f;
static float g_fine_inc_output = 0.0f;
static float g_fine_pos_output = 0.0f;
static float g_fine_pos_base = 0.0f;
static float g_fine_pos_integral = 0.0f;
static float g_integral = 0.0f;
static float g_error_filt = 0.0f;
static u8    g_error_filt_valid = 0;
static u8    g_pid_output_valid = 0;
static u16   g_tran_tick = 0;
static u16   g_fine_tick = 0;
static u8    g_fine_mode = 0;
static float g_temp_hist[STABLE_WINDOW_MAX];
static u8    g_hist_idx = 0;
static u16   g_hist_count = 0;
static float g_smith_delta = 0.0f;
static float g_smith_delay_buf[SMITH_DELAY_MAX + 1];
static u16   g_smith_idx = 0;
static u8    g_smith_valid = 0;

static void PID_SyncIntegralToOutput(float target_output, float error, float derivative);
static void PID_Fine_SyncPosBase(float current_output, float error);
static float Smith_UpdateFeedback(float measured, int output);

const char *App_Get_WorkPhase(void)
{
	if (g_config.manual_flag != 0)
		return "MAN";
	if (g_fine_mode != 0)
		return "FINE";
	return "TRAN";
}

void App_Reset_ControlState(u8 clear_output)
{
	float start_output;

	g_inc_err[0] = 0.0f;
	g_inc_err[1] = 0.0f;
	g_inc_err[2] = 0.0f;
	g_integral = 0.0f;
	g_error_filt = 0.0f;
	g_error_filt_valid = 0;
	g_pid_output_valid = 0;
	g_tran_tick = 0;
	g_fine_tick = 0;
	g_fine_mode = 0;
	g_fine_pos_base = 0.0f;
	g_fine_pos_integral = 0.0f;
	g_hist_idx = 0;
	g_hist_count = 0;
	g_smith_delta = 0.0f;
	g_smith_idx = 0;
	g_smith_valid = 0;
	temp_control_feedback = temp_feedback;

	if (clear_output != 0)
		temp_ctr_val = 0;

	start_output = (float)temp_ctr_val;
	if (start_output > PID_OUTPUT_MAX) start_output = PID_OUTPUT_MAX;
	if (start_output < PID_OUTPUT_MIN) start_output = PID_OUTPUT_MIN;
	g_pid_output = start_output;
	g_fine_inc_output = start_output;
	g_fine_pos_output = start_output;
	g_fine_pos_base = start_output;
}

static float Smith_UpdateFeedback(float measured, int output)
{
	u16 i;
	u16 delay_s;
	u16 delayed_idx;
	float tau;
	float target_delta;
	float delayed_delta;
	float lead;

	if (g_config.smith_enable == 0)
	{
		g_smith_valid = 0;
		return measured;
	}

	delay_s = g_config.smith_delay;
	if (delay_s > SMITH_DELAY_MAX)
		delay_s = SMITH_DELAY_MAX;

	if (g_smith_valid == 0)
	{
		g_smith_delta = 0.0f;
		for (i = 0; i <= SMITH_DELAY_MAX; i++)
			g_smith_delay_buf[i] = 0.0f;
		g_smith_idx = 0;
		g_smith_valid = 1;
	}

	tau = (float)g_config.smith_tau;
	if (tau < 1.0f)
		tau = 1.0f;

	target_delta = g_config.smith_gain * ((float)output / 100.0f);
	g_smith_delta += (target_delta - g_smith_delta) / tau;

	g_smith_delay_buf[g_smith_idx] = g_smith_delta;
	if (g_smith_idx >= delay_s)
		delayed_idx = g_smith_idx - delay_s;
	else
		delayed_idx = (u16)(SMITH_DELAY_MAX + 1 + g_smith_idx - delay_s);
	delayed_delta = g_smith_delay_buf[delayed_idx];

	g_smith_idx++;
	if (g_smith_idx > SMITH_DELAY_MAX)
		g_smith_idx = 0;

	lead = (g_smith_delta - delayed_delta) * g_config.smith_blend;
	if (lead > g_config.smith_max_lead)
		lead = g_config.smith_max_lead;
	if (lead < -g_config.smith_max_lead)
		lead = -g_config.smith_max_lead;

	return measured + lead;
}

static void PID_SyncIntegralToOutput(float target_output, float error, float derivative)
{
	if (g_config.tran_ki > 0.0f)
	{
		g_integral = (target_output
		              - g_config.tran_kp * error
		              - g_config.tran_kd * derivative) / g_config.tran_ki;
		if (g_integral >  g_config.tran_i_limit) g_integral =  g_config.tran_i_limit;
		if (g_integral < -g_config.tran_i_limit) g_integral = -g_config.tran_i_limit;
	}
}

static void PID_Fine_SyncPosBase(float current_output, float error)
{
	g_fine_pos_base = current_output;
	g_fine_pos_output = current_output;
	g_fine_pos_integral = 0.0f;
	PID_SyncIntegralToOutput(current_output, error, 0.0f);
}

void My_PID_Ctr(void)
{
	float error, raw_error, abs_error, output, delta;
	float control_feedback;
	float near_limit, output_step, derivative;
	float inc_output, pos_output, inc_weight, blend_span;
	float Ts;
	float i_scale, i_span;
	u8   i;
	float t_min, t_max;
	u8   is_stable = 0;

	control_feedback = Smith_UpdateFeedback(temp_feedback, temp_ctr_val);
	temp_control_feedback = control_feedback;
	raw_error = g_config.target_temp - control_feedback;
	if (g_error_filt_valid == 0)
	{
		g_error_filt = raw_error;
		g_error_filt_valid = 1;
	}
	else
	{
		g_error_filt += PID_ERROR_FILTER_ALPHA * (raw_error - g_error_filt);
	}
	error = g_error_filt;
	abs_error = (error > 0.0f) ? error : -error;

	if (g_pid_output_valid == 0)
	{
		g_pid_output = (float)temp_ctr_val;
		if (g_pid_output > PID_OUTPUT_MAX) g_pid_output = PID_OUTPUT_MAX;
		if (g_pid_output < PID_OUTPUT_MIN) g_pid_output = PID_OUTPUT_MIN;
		g_fine_inc_output = g_pid_output;
		g_fine_pos_output = g_pid_output;
		PID_Fine_SyncPosBase(g_pid_output, error);
		g_pid_output_valid = 1;
	}

	g_temp_hist[g_hist_idx] = temp_feedback;
	g_hist_idx = (g_hist_idx + 1) % STABLE_WINDOW_MAX;
	if (g_hist_count < STABLE_WINDOW_MAX) g_hist_count++;

	if (g_hist_count >= g_config.stable_window)
	{
		u16 win = g_config.stable_window;
		u16 start = (g_hist_idx + STABLE_WINDOW_MAX - win) % STABLE_WINDOW_MAX;
		t_min = t_max = g_temp_hist[start];
		for (i = 1; i < win; i++)
		{
			u16 idx = (start + i) % STABLE_WINDOW_MAX;
			if (g_temp_hist[idx] < t_min) t_min = g_temp_hist[idx];
			if (g_temp_hist[idx] > t_max) t_max = g_temp_hist[idx];
		}
		if ((t_max - t_min) <= g_config.stable_delta) is_stable = 1;
	}
	(void)is_stable;

	if (g_config.fine_enable == 0 && g_fine_mode)
	{
		g_pid_output = (float)temp_ctr_val;
		PID_SyncIntegralToOutput(g_pid_output, error, 0.0f);
		g_fine_mode = 0;
		g_tran_tick = 0;
	}
	else if (g_config.fine_enable != 0 && abs_error <= g_config.fine_entry_max && !g_fine_mode)
	{
		g_fine_mode = 1;
		g_fine_tick = 0;
		g_inc_err[0] = error;
		g_inc_err[1] = error;
		g_inc_err[2] = error;
		g_pid_output = (float)temp_ctr_val;
		if (g_pid_output > PID_OUTPUT_MAX) g_pid_output = PID_OUTPUT_MAX;
		if (g_pid_output < PID_FINE_OUTPUT_MIN) g_pid_output = PID_FINE_OUTPUT_MIN;
		g_fine_inc_output = g_pid_output;
		PID_Fine_SyncPosBase(g_pid_output, error);
	}
	if (g_fine_mode && abs_error > (PID_FINE_EXIT_GAIN * g_config.fine_entry_max))
	{
		derivative = error - g_inc_err[0];
		if (abs_error <= g_config.tran_sep_threshold)
			PID_SyncIntegralToOutput(g_pid_output, error, derivative);
		g_fine_inc_output = g_pid_output;
		g_fine_pos_output = g_pid_output;
		g_fine_pos_base = g_pid_output;
		g_fine_pos_integral = 0.0f;
		g_fine_mode = 0;
		g_tran_tick = 0;
	}

	g_inc_err[2] = g_inc_err[1];
	g_inc_err[1] = g_inc_err[0];
	g_inc_err[0] = error;

	if (!g_fine_mode)
	{
		if (abs_error > g_config.tran_sep_threshold)
		{
			g_integral = 0.0f;
		}
		else if (abs_error <= g_config.pid_deadband)
		{
			g_integral *= g_config.tran_i_overshoot_leak;
		}
		else
		{
			i_scale = 0.0f;
			if (error < 0.0f)
			{
				g_integral *= g_config.tran_i_overshoot_leak;
			}
			else
			{
				if (abs_error <= g_config.tran_i_full_error)
				{
					i_scale = 1.0f;
				}
				else
				{
					i_span = g_config.tran_sep_threshold - g_config.tran_i_full_error;
					if (i_span < 0.1f) i_span = 0.1f;
					i_scale = (g_config.tran_sep_threshold - abs_error) / i_span;
					if (i_scale < g_config.tran_i_min_scale) i_scale = g_config.tran_i_min_scale;
					if (i_scale > 1.0f) i_scale = 1.0f;
				}
				g_integral += error * i_scale * 1.0f;
			}

			if (g_integral >  g_config.tran_i_limit) g_integral =  g_config.tran_i_limit;
			if (g_integral < -g_config.tran_i_limit) g_integral = -g_config.tran_i_limit;
			if ((g_pid_output >= PID_OUTPUT_MAX && error > 0) ||
			    (g_pid_output <= PID_OUTPUT_MIN && error < 0))
			{
				g_integral -= error * i_scale * 1.0f;
			}
		}

		g_tran_tick++;
		if (g_tran_tick < g_config.tran_interval)
		{
			output = g_pid_output;
			goto apply_output;
		}
		g_tran_tick = 0;

		if (abs_error <= g_config.pid_deadband)
		{
			output = g_pid_output;
			goto apply_output;
		}

		output = g_config.tran_kp * error
		         + g_config.tran_ki * g_integral
		         + g_config.tran_kd * (g_inc_err[0] - g_inc_err[1]);
		near_limit = g_config.fine_entry_max + g_config.fine_entry_min;
		if (abs_error <= near_limit)
		{
			output_step = output - g_pid_output;
			if (output_step >  g_config.fine_range) output = g_pid_output + g_config.fine_range;
			if (output_step < -g_config.fine_range) output = g_pid_output - g_config.fine_range;
		}
	}
	else
	{
		g_fine_tick++;
		if (g_fine_tick < g_config.fine_interval)
		{
			output = g_pid_output;
			goto apply_output;
		}
		g_fine_tick = 0;

		if (abs_error <= g_config.pid_deadband)
		{
			output = g_pid_output;
			g_fine_pos_integral *= PID_FINE_POS_INTEGRAL_LEAK;
			goto apply_output;
		}

		Ts = (float)g_config.fine_interval;
		delta = g_config.fine_kp * (g_inc_err[0] - g_inc_err[1])
		        + g_config.fine_ki * g_inc_err[0] * Ts
		        + g_config.fine_kd * (g_inc_err[0] - 2.0f * g_inc_err[1] + g_inc_err[2]);

		if (delta >  g_config.fine_range) delta =  g_config.fine_range;
		if (delta < -g_config.fine_range) delta = -g_config.fine_range;

		inc_output = g_fine_inc_output + delta;
		if (inc_output > PID_OUTPUT_MAX) inc_output = PID_OUTPUT_MAX;
		if (inc_output < PID_FINE_OUTPUT_MIN) inc_output = PID_FINE_OUTPUT_MIN;

		if (error > g_config.pid_deadband)
			g_fine_pos_integral += error * Ts;
		else if (error < -g_config.pid_deadband)
			g_fine_pos_integral *= PID_FINE_POS_INTEGRAL_LEAK;
		if (g_fine_pos_integral > PID_FINE_POS_INTEGRAL_LIMIT)
			g_fine_pos_integral = PID_FINE_POS_INTEGRAL_LIMIT;
		if (g_fine_pos_integral < 0.0f)
			g_fine_pos_integral = 0.0f;

		pos_output = g_fine_pos_base
		             + g_config.tran_kp * error
		             + g_config.tran_ki * g_fine_pos_integral
		             + g_config.tran_kd * (g_inc_err[0] - g_inc_err[1]);
		if (pos_output > g_fine_pos_base + PID_FINE_POS_WINDOW_GAIN * g_config.fine_range)
			pos_output = g_fine_pos_base + PID_FINE_POS_WINDOW_GAIN * g_config.fine_range;
		if (pos_output < g_fine_pos_base - PID_FINE_POS_WINDOW_GAIN * g_config.fine_range)
			pos_output = g_fine_pos_base - PID_FINE_POS_WINDOW_GAIN * g_config.fine_range;
		output_step = pos_output - g_fine_pos_output;
		if (output_step >  g_config.fine_range) pos_output = g_fine_pos_output + g_config.fine_range;
		if (output_step < -g_config.fine_range) pos_output = g_fine_pos_output - g_config.fine_range;

		blend_span = g_config.fine_entry_max - g_config.pid_deadband;
		if (blend_span < 0.1f) blend_span = 0.1f;
		inc_weight = (abs_error - g_config.pid_deadband) / blend_span;
		if (inc_weight > 1.0f) inc_weight = 1.0f;
		if (inc_weight < 0.0f) inc_weight = 0.0f;
		inc_weight = PID_FINE_INC_WEIGHT_MIN
		             + inc_weight * (PID_FINE_INC_WEIGHT_MAX - PID_FINE_INC_WEIGHT_MIN);

		output = (inc_weight * inc_output) + ((1.0f - inc_weight) * pos_output);
		output_step = output - g_pid_output;
		if (output_step >  g_config.fine_range) output = g_pid_output + g_config.fine_range;
		if (output_step < -g_config.fine_range) output = g_pid_output - g_config.fine_range;
		g_fine_inc_output = inc_output;
		g_fine_pos_output = pos_output;
	}

apply_output:
	if (output > PID_OUTPUT_MAX) output = PID_OUTPUT_MAX;
	if (output < PID_OUTPUT_MIN) output = PID_OUTPUT_MIN;
	if (g_fine_mode && output < PID_FINE_OUTPUT_MIN) output = PID_FINE_OUTPUT_MIN;
	g_pid_output = output;
	g_pid_output_valid = 1;
	temp_ctr_val = (int)output;
	My_Ctr(temp_ctr_val);
}

void My_Ctr(int heat)
{
	if (heat > 100 || heat < -100) return;

	if (heat > 0)
	{
		Heat_PWM = heat;
		TIM_SetCompare2(TIM9, 0);
	}
	else if (heat < 0)
	{
		int fan_pwm;
		Heat_PWM = heat;
		fan_pwm = FAN_PWM_MIN + ((-heat) * (FAN_PWM_MAX - FAN_PWM_MIN)) / 100;
		if (fan_pwm > FAN_PWM_MAX) fan_pwm = FAN_PWM_MAX;
		TIM_SetCompare2(TIM9, fan_pwm);
	}
	else
	{
		Heat_PWM = 0;
		TIM_SetCompare2(TIM9, 0);
	}
}
