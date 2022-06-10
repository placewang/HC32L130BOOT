#ifndef  __UART__
#define  __UART__

#include "ddl.h"

#define UART0_TX_BUF_LEN	256		// ���ͻ���������
#define UART0_RX_BUF_LEN	256		// ���ջ���������
#define UART0_TX_BUF_LOOP	0xFF
#define UART0_RX_BUF_LOOP	0xFF

void Uart0_init(uint32_t br);
void Uart0_send_data(unsigned char data);
void Uart0_send_btye(unsigned char data);
void Uart0_loop(void);
#endif


