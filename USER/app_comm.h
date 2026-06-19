#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "sys.h"

void Uart_Command_Process(void);
void Uart_Send_Frame(const char *payload);
void Uart_Send_Ack(u8 ok);
void Uart_Send_WorkPhase(void);

void Net_InitCallbacks(void);
void Net_BroadcastState(void);
void Net_BroadcastPhase(void);

#endif
