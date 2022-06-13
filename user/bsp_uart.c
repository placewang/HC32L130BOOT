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
	����0�Ļ�������ʼ��
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
	UART0�жϺ������������ݽ���
******************************************************************/
void Uart0_IRQHandler(void)
{
	unsigned char rx_data;
	if(Uart_GetStatus(M0P_UART0, UartFE))	//��������
	{
		Uart_ClrStatus(M0P_UART0, UartFE);	//���֡������
		Uart_ClrStatus(M0P_UART0, UartRC);  
	}
	if(Uart_GetStatus(M0P_UART0, UartPE))
	{
		Uart_ClrStatus(M0P_UART0, UartPE);	//�����żУ�������
		Uart_ClrStatus(M0P_UART0, UartRC);  
	}

	if(Uart_GetStatus(M0P_UART0, UartRC))	//UART1���ݽ���
	{
		Uart_ClrStatus(M0P_UART0, UartRC);	//���ж�״̬λ
		rx_data = Uart_ReceiveData(M0P_UART0);	//���������ֽ�
		G_uart0_sub.rx_buf[G_uart0_sub.rx_buf_wr] = rx_data;
		G_uart0_sub.rx_buf_wr++;
	}
}

/*****************************************************************
Uart0_send_msg:
	UART0����һ������
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
	UART0����һ���ֽ�
******************************************************************/
void Uart0_send_btye(unsigned char data)
{
	G_uart0_sub.tx_buf[G_uart0_sub.tx_buf_wr] = data;
	G_uart0_sub.tx_buf_wr++;
}

/*****************************************************************
Uart0_init:
	UART0��ʼ��������������ʼ��IO�ڣ������ʣ�ģʽ�����д��ڵķ��ͺͽ��ջ�����
	br:������
******************************************************************/
void Uart0_init(uint32_t br)
{
	stc_gpio_cfg_t stcGpioCfg;
	stc_uart_cfg_t    stcCfg;
	DDL_ZERO_STRUCT(stcGpioCfg);
	DDL_ZERO_STRUCT(stcCfg);
	//GPIO set
	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio,TRUE);  //ʹ��GPIOģ��ʱ��
	///<TX
	stcGpioCfg.enDir = GpioDirOut;
	stcGpioCfg.enPu = GpioPuEnable;                         //< �˿�����������->����
	stcGpioCfg.enPd = GpioPdDisable;
	stcGpioCfg.enCtrlMode = GpioAHB;
	stcGpioCfg.enDrv = GpioDrvH;
	stcGpioCfg.enOD = GpioOdDisable;
	Gpio_Init(GpioPortA, GpioPin9, &stcGpioCfg);
	Gpio_SetAfMode(GpioPortA, GpioPin9, GpioAf1);           //����PA9 �˿�ΪURART0_TX
	///<RX
	stcGpioCfg.enDir = GpioDirIn;
	Gpio_Init(GpioPortA, GpioPin10, &stcGpioCfg);
	Gpio_SetAfMode(GpioPortA, GpioPin10, GpioAf1);          //����PA10 �˿�ΪURART0_RX

	///< ��������ʱ��
	Sysctrl_SetPeripheralGate(SysctrlPeripheralUart0,TRUE); ///<ʹ��uart0ģ��ʱ��

	///<UART Init
	stcCfg.enRunMode        = UartMskMode1;                 //ģʽ
	stcCfg.enStopBit        = UartMsk1bit;                  //1bitֹͣλ
	//    stcCfg.enMmdorCk        = UartMskDataOrAddr;      //�޼���
	stcCfg.stcBaud.u32Baud  = br;                           //������
	stcCfg.stcBaud.enClkDiv = UartMsk8Or16Div;              //ͨ��������Ƶ����
	stcCfg.stcBaud.u32Pclk  = Sysctrl_GetPClkFreq();        //�������ʱ�ӣ�PCLK��Ƶ��ֵ
	Uart_Init(M0P_UART0, &stcCfg);                          //���ڳ�ʼ��
	
	///<UART�ж�ʹ��
	Uart_ClrStatus(M0P_UART0,UartRC);                       //���������
	//    Uart_ClrStatus(M0P_UART0,UartTC);                 //�巢��������
	Uart_EnableIrq(M0P_UART0,UartRxIrq);                    //ʹ�ܴ��ڽ����ж�
	//    Uart_EnableIrq(M0P_UART0,UartTxIrq);              //ʹ�ܴ��ڷ������ж�
	EnableNvic(UART0_IRQn, IrqLevel0, TRUE);                //ϵͳ�ж�ʹ��
	
	Uart0_buf_init();
}

/*****************************************************************
Uart0_loop:
	����0��ѭ���������������ͻ����������ݷ��ͣ��������ݵĴ���
	��main��ѭ���е���
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
