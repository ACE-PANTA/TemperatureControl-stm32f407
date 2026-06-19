#include "app_input.h"
#include "app_config.h"
#include "pid_control.h"
#include "key.h"
#include "hmi.h"
#include <stdio.h>
#include <string.h>

extern int temp_ctr_val;
extern int my_goal;
extern int my_pwm;
extern uint8_t Uint_Goal[5];
extern uint8_t Uint_pwm[5];

void Key_Process(u8 KeyState)
{
	switch (KeyState)
	{
	case KEY_UP_PRES:
		if (g_config.manual_flag == 0)
		{
			if (g_config.target_temp < (100.0f - g_config.step_value))
				g_config.target_temp += g_config.step_value;
			else
				g_config.target_temp = 100.0f;
			my_goal = (int)(g_config.target_temp * 10.0f);
			sprintf((char *)Uint_Goal, "%d", my_goal);
			HMI_Send_Float(3, Uint_Goal, strlen((const char *)Uint_Goal));
			AppConfig_MarkDirty();
		}
		else if (g_config.manual_flag == 1)
		{
			if (temp_ctr_val < (100 - g_config.step_value))
				temp_ctr_val += g_config.step_value;
			else
				temp_ctr_val = 100;
			my_pwm = temp_ctr_val;
			sprintf((char *)Uint_pwm, "%d", my_pwm);
			HMI_Send_Float(2, Uint_pwm, strlen((const char *)Uint_pwm));
			AppConfig_MarkDirty();
		}
		break;

	case KEY_DOWN_PRES:
		if (g_config.manual_flag == 0)
		{
			if (g_config.target_temp > (-10.0f + g_config.step_value))
				g_config.target_temp -= g_config.step_value;
			else
				g_config.target_temp = -10.0f;
			my_goal = (int)(g_config.target_temp * 10.0f);
			sprintf((char *)Uint_Goal, "%d", my_goal);
			HMI_Send_Float(3, Uint_Goal, strlen((const char *)Uint_Goal));
			AppConfig_MarkDirty();
		}
		else if (g_config.manual_flag == 1)
		{
			if (temp_ctr_val > g_config.step_value - 100)
				temp_ctr_val -= g_config.step_value;
			else
				temp_ctr_val = -100;
			my_pwm = temp_ctr_val;
			sprintf((char *)Uint_pwm, "%d", my_pwm);
			HMI_Send_Float(2, Uint_pwm, strlen((const char *)Uint_pwm));
			AppConfig_MarkDirty();
		}
		break;

	case KEY_STEP_PRES:
		if (g_config.step_value == 1)
			g_config.step_value = 5;
		else if (g_config.step_value == 5)
			g_config.step_value = 10;
		else if (g_config.step_value == 10)
			g_config.step_value = 1;
		HMI_Send_txt(0, g_config.step_value);
		break;

	case KEY_AUTO_PRES:
		if (g_config.manual_flag == 0)
		{
			g_config.manual_flag = 1;
			g_config.manual_pwm = 0;
			App_Reset_ControlState(1);
		}
		else if (g_config.manual_flag == 1)
		{
			g_config.manual_flag = 0;
			App_Reset_ControlState(0);
		}
		HMI_Send_txt(1, g_config.manual_flag);
		AppConfig_MarkDirty();
		break;

	default:
		break;
	}
}
