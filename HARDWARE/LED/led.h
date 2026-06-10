#ifndef __LED_H
#define __LED_H
#include "sys.h"

//LED端口定义
#define RUN_LED		 PBout(4)	// DS0
#define HEAT_LED	 PBout(3)	// DS1	 
#define BEEP		 PBout(6)


#define FAN1		 PEout(4)
#define HEAT		 PEout(5)
#define FAN2		 PEout(6)


void LED_Init(void);//初始化
void Fan_io_Init(void);
void Heat_io_Init(void);
#endif
