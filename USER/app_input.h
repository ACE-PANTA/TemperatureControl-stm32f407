#ifndef __APP_INPUT_H
#define __APP_INPUT_H

#include "sys.h"

void Key_Process(u8 KeyState);
void HMI_Refresh_Mode(void);
void HMI_Refresh_Goal(void);
void HMI_Refresh_Pwm(void);
void HMI_Refresh_Step(void);
void HMI_Refresh_AllConfig(void);

#endif
