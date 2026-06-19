#ifndef __PID_CONTROL_H
#define __PID_CONTROL_H

#include "sys.h"

extern float temp_control_feedback;
extern int Heat_PWM;

void My_PID_Ctr(void);
void My_Ctr(int heat);
void App_Reset_ControlState(u8 clear_output);
const char *App_Get_WorkPhase(void);

#endif
