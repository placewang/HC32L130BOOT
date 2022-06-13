#include "uart.h"
#include "gpio.h"
#include "bsp_uart.h"
#include "upgrade.h"
struct UART0_SUB
{
	unsigned char tx_buf_wr;
	unsigned char tx_buf_rd;
	unsigned char rx_buf_wr;
	unsigned char rx_buf_rd;
	unsigned char tx_buf[UART0_TX_BUF_LEN];
	unsigned char rx_buf[UART0_RX_BUF_LEN];
}__attribute__((packed));

struct UART0_SUB G_uart0_sub;

/*****************************************************************
Uart0_buf_init:
	串口0的缓冲区初始化
******************************************************************/
void Uart0_buf_init(void)
{
	G_uart0_sub.tx_buf_wr = 0;
	G_uart0_sub.tx_buf_rd = 0;
	G_uart0_sub.rx_buf_wr = 0;
	G_uart0_sub.rx_buf_rd = 0;
}

/*****************************************************************
Uart0_IRQHandler:
	UART0中断函数，用于数据接收
******************************************************************/
void Uart0_IRQHandler(void)
{
	unsigned char rx_data;
	if(Uart_GetStatus(M0P_UART0, UartFE))	//错误请求
	{
		Uart_ClrStatus(M0P_UART0, UartFE);	//清除帧错误标记
		Uart_ClrStatus(M0P_UART0, UartRC);  
	}
	if(Uart_GetStatus(M0P_UART0, UartPE))
	{
		Uart_ClrStatus(M0P_UART0, UartPE);	//清除奇偶校验错误标记
		Uart_ClrStatus(M0P_UART0, UartRC);  
	}

	if(Uart_GetStatus(M0P_UART0, UartRC))	//UART1数据接收
	{
		Uart_ClrStatus(M0P_UART0, UartRC);	//清中断状态位
		rx_data = Uart_ReceiveData(M0P_UART0);	//接收数据字节
		G_uart0_sub.rx_buf[G_uart0_sub.rx_buf_wr] = rx_data;
		G_uart0_sub.rx_buf_wr++;
	}
}

/*****************************************************************
Uart0_send_msg:
	UART0发送一串数据
******************************************************************/
void Uart0_send_msg(unsigned char *data_p, unsigned char len)
{
	unsigned short i;
	if ((data_p == NULL) || (len == 0))
	{
		return;
	}
	
	for (i = 0; i < len; i++)
	{
		G_uart0_sub.tx_buf[G_uart0_sub.tx_buf_wr] = *data_p;
		G_uart0_sub.tx_buf_wr++;
		data_p++;
	}
}

/*****************************************************************
Uart0_send_btye:
	UART0发送一个字节
******************************************************************/
void Uart0_send_btye(unsigned char data)
{
	G_uart0_sub.tx_buf[G_uart0_sub.tx_buf_wr] = data;
	G_uart0_sub.tx_buf_wr++;
}

/*****************************************************************
Uart0_init:
	UART0初始化函数，包含初始化IO口，波特率，模式，还有串口的发送和接收缓冲区
	br:波特率
******************************************************************/
void Uart0_init(uint32_t br)
{
	stc_gpio_cfg_t stcGpioCfg;
	stc_uart_cfg_t    stcCfg;
	DDL_ZERO_STRUCT(stcGpioCfg);
	DDL_ZERO_STRUCT(stcCfg);
	//GPIO set
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  //使能GPIO模块时钟
	///<TX
	stcGpioCfg.enDir = GpioDirOut;
	stcGpioCfg.enPu = GpioPuEnable;                         //< 端口上下拉配置->上拉
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enCtrlMode = GpioAHB;
	stcGpioCfg.enDrv = GpioDrvH;
	stcGpioCfg.enOD = GpioOdDisable;
	Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
	Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf1);           //配置PA9 端口为URART0_TX
	///<RX
	stcGpioCfg.enDir = GpioDirIn;
	Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg);
	Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf1);          //配置PA10 端口为URART0_RX

	///< 开启外设时钟
	Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE); ///<使能uart0模块时钟

	///<UART Init
	stcCfg.enRunMode        = UartMskMode1;                 //模式
	stcCfg.enStopBit        = UartMsk1bit;                  //1bit停止位
	//    stcCfg.enMmdorCk        = UartMskDataOrAddr;      //无检验
	stcCfg.stcBaud.u32Baud  = br;                           //波特率
	stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;              //通道采样分频配置
	stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq();        //获得外设时钟（PCLK）频率值
	Uart_Init(M0P_UART0, &stcCfg);                          //串口初始化
	
	///<UART中断使能
	Uart_ClrStatus(M0P_UART0,UartRC);                       //清接收请求
	//    Uart_ClrStatus(M0P_UART0,UartTC);                 //清发送收请求
	Uart_EnableIrq(M0P_UART0,UartRxIrq);                    //使能串口接收中断
	//    Uart_EnableIrq(M0P_UART0,UartTxIrq);              //使能串口发送收中断
	EnableNvic(UART0_IRQn, IrqLevel0, TRUE);                //系统中断使能
	
	Uart0_buf_init();
}

/*****************************************************************
Uart0_loop:
	串口0主循环函数，包含发送缓冲区的数据发送，接收数据的处理
	在main主循环中调用
******************************************************************/
void Uart0_loop(void)
{
	if (G_uart0_sub.tx_buf_wr != G_uart0_sub.tx_buf_rd)
	{
		Uart_SendDataPoll(M0P_UART0, G_uart0_sub.tx_buf[G_uart0_sub.tx_buf_rd]);
		G_uart0_sub.tx_buf_rd++;
	}
	
	if (G_uart0_sub.rx_buf_wr != G_uart0_sub.rx_buf_rd)
	{
		Upgrade_data_deal(G_uart0_sub.rx_buf[G_uart0_sub.rx_buf_rd]);
		G_uart0_sub.rx_buf_rd++;
	}
}
