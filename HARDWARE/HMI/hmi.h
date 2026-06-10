#ifndef __HMI_H
#define __HMI_H 
#include "sys.h"

extern uint8_t u3_sendbuf[15];
extern uint8_t Tempbuf_data[5];

void U3_SendBuf(uint8_t *buf,uint8_t len);
void Uart_HMI_Init(uint32_t bound);
void Clc_str(char* buf,uint8_t len);
void HMI_init(void);
void HMI_Send_Float(uint8_t ch,uint8_t *data,uint8_t len);
void usart3_send(unsigned char *data,unsigned char len);
void HMI_Send_txt(uint8_t ch,uint8_t data);

#endif



