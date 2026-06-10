#include "sys.h"
#include "usart.h"	
#include <string.h>

//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮������������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
//V1.5�޸�˵��
//1,�����˶�UCOSII��֧��
////////////////////////////////////////////////////////////////////////////////// 	  


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	

#define USART1_CMD_QUEUE_SIZE 4
#define USART1_TX_QUEUE_SIZE 8
#define USART1_TX_QUEUE_PRIORITY_SIZE 8

typedef struct
{
	u8 len;
	u8 data[USART1_TX_PACKET_MAX_LEN];
} USART1_TxPacket;

static volatile u8 usart1_cmd_queue[USART1_CMD_QUEUE_SIZE][USART1_CMD_MAX_LEN];
static volatile u8 usart1_cmd_lengths[USART1_CMD_QUEUE_SIZE];
static volatile u8 usart1_cmd_head = 0;
static volatile u8 usart1_cmd_tail = 0;
static volatile u8 usart1_cmd_count = 0;
static volatile u8 usart1_rx_line[USART1_CMD_MAX_LEN];
static volatile u8 usart1_rx_index = 0;
static volatile USART1_TxPacket usart1_tx_queue[USART1_TX_QUEUE_SIZE];
static volatile USART1_TxPacket usart1_tx_priority_queue[USART1_TX_QUEUE_PRIORITY_SIZE];
static volatile u8 usart1_tx_head = 0;
static volatile u8 usart1_tx_tail = 0;
static volatile u8 usart1_tx_count = 0;
static volatile u8 usart1_tx_priority_head = 0;
static volatile u8 usart1_tx_priority_tail = 0;
static volatile u8 usart1_tx_priority_count = 0;
static volatile u8 usart1_tx_busy = 0;
static volatile u8 usart1_tx_index = 0;
static volatile u8 usart1_tx_len = 0;
static volatile u8 usart1_tx_data[USART1_TX_PACKET_MAX_LEN];

static void usart1_queue_line(const u8 *data, u8 len)
{
	u8 index;

	if((len == 0) || (usart1_cmd_count >= USART1_CMD_QUEUE_SIZE))
	{
		return;
	}

	index = usart1_cmd_head;
	memcpy((void *)usart1_cmd_queue[index], data, len);
	usart1_cmd_queue[index][len] = 0;
	usart1_cmd_lengths[index] = len;
	usart1_cmd_head++;
	if(usart1_cmd_head >= USART1_CMD_QUEUE_SIZE)
	{
		usart1_cmd_head = 0;
	}
	usart1_cmd_count++;
}

static void usart1_start_tx_locked(void)
{
	if(usart1_tx_busy != 0)
	{
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		return;
	}

	if(usart1_tx_priority_count > 0)
	{
		memcpy((void *)usart1_tx_data, (const void *)usart1_tx_priority_queue[usart1_tx_priority_tail].data,
			usart1_tx_priority_queue[usart1_tx_priority_tail].len);
		usart1_tx_len = usart1_tx_priority_queue[usart1_tx_priority_tail].len;
		usart1_tx_priority_tail++;
		if(usart1_tx_priority_tail >= USART1_TX_QUEUE_PRIORITY_SIZE)
		{
			usart1_tx_priority_tail = 0;
		}
		usart1_tx_priority_count--;
	}
	else if(usart1_tx_count > 0)
	{
		memcpy((void *)usart1_tx_data, (const void *)usart1_tx_queue[usart1_tx_tail].data,
			usart1_tx_queue[usart1_tx_tail].len);
		usart1_tx_len = usart1_tx_queue[usart1_tx_tail].len;
		usart1_tx_tail++;
		if(usart1_tx_tail >= USART1_TX_QUEUE_SIZE)
		{
			usart1_tx_tail = 0;
		}
		usart1_tx_count--;
	}
	else
	{
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		return;
	}

	usart1_tx_busy = 1;
	usart1_tx_index = 0;
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

static void usart1_enqueue_tx_packet(const u8 *buf, u8 len, u8 priority)
{
	volatile USART1_TxPacket *queue;
	volatile u8 *head;
	volatile u8 *count;
	u8 queue_size;
	u8 index;

	if((buf == 0) || (len == 0))
	{
		return;
	}
	if(len > USART1_TX_PACKET_MAX_LEN)
	{
		len = USART1_TX_PACKET_MAX_LEN;
	}

	if(priority != 0)
	{
		queue = usart1_tx_priority_queue;
		head = &usart1_tx_priority_head;
		count = &usart1_tx_priority_count;
		queue_size = USART1_TX_QUEUE_PRIORITY_SIZE;
	}
	else
	{
		queue = usart1_tx_queue;
		head = &usart1_tx_head;
		count = &usart1_tx_count;
		queue_size = USART1_TX_QUEUE_SIZE;
	}

	__disable_irq();
	if((*count) >= queue_size)
	{
		__enable_irq();
		return;
	}

	index = *head;
	memcpy((void *)queue[index].data, buf, len);
	queue[index].len = len;
	(*head)++;
	if((*head) >= queue_size)
	{
		*head = 0;
	}
	(*count)++;
	usart1_start_tx_locked();
	__enable_irq();
}

//��ʼ��IO ����1 
//bound:������
void uart_init(u32 bound)
{
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��

	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1

	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1

	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 

	//USART_ClearFlag(USART1, USART_FLAG_TC);


	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//Ĭ�Ϲرշ����ж�

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}


void USART1_IRQHandler(void)
{
	u8 res;
	u16 sta;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		res = (u8)USART_ReceiveData(USART1);
		sta = USART_RX_STA;

		if((sta & 0x8000) == 0)
		{
			if(res == '\r')
			{
				USART_RX_STA = sta | 0x4000;
			}
			else if(res == '\n')
			{
				if((sta & 0x4000) != 0)
				{
					USART_RX_STA = sta | 0x8000;
				}
				if(usart1_rx_index > 0)
				{
					usart1_queue_line((const u8 *)usart1_rx_line, usart1_rx_index);
					usart1_rx_index = 0;
				}
				USART_RX_STA = 0;
			}
			else
			{
				if((sta & 0x4000) != 0)
				{
					USART_RX_STA = 0;
					usart1_rx_index = 0;
				}
				if(usart1_rx_index < (USART1_CMD_MAX_LEN - 1))
				{
					usart1_rx_line[usart1_rx_index++] = res;
				}
				else
				{
					usart1_rx_index = 0;
					USART_RX_STA = 0;
				}

				if((sta & 0x3FFF) < (USART_REC_LEN - 1))
				{
					USART_RX_BUF[sta & 0x3FFF] = res;
					USART_RX_STA = (sta & 0xC000) | ((sta & 0x3FFF) + 1);
				}
				else
				{
					USART_RX_STA = 0;
				}
			}
		}
	}

	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if((usart1_tx_busy != 0) && (usart1_tx_index < usart1_tx_len))
		{
			USART1->DR = usart1_tx_data[usart1_tx_index++];
		}
		else
		{
			usart1_tx_busy = 0;
			usart1_tx_len = 0;
			usart1_tx_index = 0;
			usart1_start_tx_locked();
		}
	}
}

void usart1_send_buf(u8 *buf , u8 len)
{
	usart1_enqueue_tx_packet(buf, len, 0);
}

void usart1_send_buf_priority(u8 *buf , u8 len)
{
	usart1_enqueue_tx_packet(buf, len, 1);
}

void usart1_send_string(const char *str)
{
	u8 len;

	if(str == 0)
	{
		return;
	}
	len = (u8)strlen(str);
	usart1_enqueue_tx_packet((const u8 *)str, len, 1);
}

u8 usart1_read_line(char *buf, u16 buf_len)
{
	u8 index;
	u8 len;

	if((buf == 0) || (buf_len == 0) || (usart1_cmd_count == 0))
	{
		return 0;
	}

	index = usart1_cmd_tail;
	len = usart1_cmd_lengths[index];
	if(len >= buf_len)
	{
		len = (u8)(buf_len - 1);
	}

	memcpy(buf, (const void *)usart1_cmd_queue[index], len);
	buf[len] = 0;

	usart1_cmd_tail++;
	if(usart1_cmd_tail >= USART1_CMD_QUEUE_SIZE)
	{
		usart1_cmd_tail = 0;
	}
	if(usart1_cmd_count > 0)
	{
		usart1_cmd_count--;
	}

	return 1;
}








