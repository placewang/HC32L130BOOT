#ifndef  __UART__
#define  __UART__

#include "ddl.h"

#define UART0_TX_BUF_LEN	256		// 发送缓冲区长度
#define UART0_RX_BUF_LEN	256		// 接收缓冲区长度
#define UART0_TX_BUF_LOOP	0xFF
#define UART0_RX_BUF_LOOP	0xFF

// 指示灯定义
#define  LED0_PORT		  (GpioPortC) 
#define  LED0_PIN		    (GpioPin13) 
#define  LED0_TOGGLE()	(Gpio_GetInputIO(LED0_PORT, LED0_PIN))?(Gpio_ClrIO(LED0_PORT, LED0_PIN)):(Gpio_SetIO(LED0_PORT, LED0_PIN))
#define  LED0_ON()  	  (Gpio_ClrIO(LED0_PORT, LED0_PIN))
//触屏背光开关
#define Backlight_PORT    (GpioPortA)
#define Backlight_PIN		  (GpioPin8)
#define Backlight_ON()	  Gpio_SetIO(Backlight_PORT,Backlight_PIN)
#define Backlight_OFF()		Gpio_ClrIO(Backlight_PORT,Backlight_PIN)


void Uart0_init(uint32_t br);
void Uart0_send_data(unsigned char data);
void Uart0_send_btye(unsigned char data);
void Uart0_loop(void);
#endif


