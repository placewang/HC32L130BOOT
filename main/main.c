#include "bsp_time.h"
#include "bsp_uart.h"
#include "flash.h"
#include "sysctrl.h"
#include "gpio.h"
#include "upgrade.h"
#include "wdt.h"

// ָʾ�ƶ���
#define  LED0_PORT		  (GpioPortC) 
#define  LED0_PIN		    (GpioPin13) 
#define  LED0_TOGGLE()	(Gpio_GetInputIO(LED0_PORT, LED0_PIN))?(Gpio_ClrIO(LED0_PORT, LED0_PIN)):(Gpio_SetIO(LED0_PORT, LED0_PIN))
#define  LED0_ON()  	  (Gpio_ClrIO(LED0_PORT, LED0_PIN))

extern unsigned char G_timer_2ms_wr;
unsigned char        G_timer_2ms_rd;

/*****************************************************************
Systimeclk:
	ϵͳʱ�ӳ�ʼ��
******************************************************************/
void Systimeclk(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    if(Sysctrl_GetHClkFreq()==48000000u&&Sysctrl_GetPClkFreq()==48000000u)
		{
			
		}
		else{
    ///< ����FLASH����ʱ��
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE); 
    Flash_WaitCycle(FlashWaitCycle2);                           // ��Ҫ��Ƶ��PLL��Ϊϵͳʱ��HCLK��ﵽ48MHz�����Դ˴�Ԥ������FLASH ���ȴ�����Ϊ1 cycle(Ĭ��ֵΪ0 cycle)
          
    Sysctrl_SetXTHFreq(SysctrlXthFreq6_12MHz);                  //�л�ʱ��ǰ�������ⲿ���پ�������XTHƵ�ʷ�Χ,���þ��������ʹ��Ŀ��ʱ�ӣ��˴�Ϊ8MHz
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
		
	  stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;         //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;       //PLL ���48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;               //����ʱ��Դѡ��XTH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;                  //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);       
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    stcCfg.enClkSrc  = SysctrlClkPLL;                        //< ѡ��PLL��ΪHCLKʱ��Դ
    stcCfg.enHClkDiv = SysctrlHclkDiv1;                      //< HCLK SYSCLK/1
    stcCfg.enPClkDiv = SysctrlPclkDiv1;                      //< PCLK ΪHCLK/1
    Sysctrl_ClkInit(&stcCfg);                                //< ϵͳʱ�ӳ�ʼ��
		
    Sysctrl_SysClkSwitch(SysctrlClkPLL);							       //< ʱ���л�
		}
		delay1ms(20);		
}

/*****************************************************************
Led_init:
	ָʾ�Ƴ�ʼ������
******************************************************************/
void Led_init(void) 
{ 
	stc_gpio_cfg_t stcGpioCfg; 

	Sysctrl_SetPeripheralGate(SysctrlPeripheralGpio, TRUE);
	stcGpioCfg.enDir = GpioDirOut; 
	stcGpioCfg.enOD = GpioOdDisable;
	stcGpioCfg.enPu = GpioPuEnable; 
	stcGpioCfg.enPd = GpioPdDisable;

	Gpio_Init(LED0_PORT, LED0_PIN, &stcGpioCfg); 
	Gpio_SetIO(LED0_PORT, LED0_PIN); 
}
 
/*****************************************************************
Bsp_Init:
	�豸�����ʼ������
******************************************************************/
void Bsp_Init(void)
{
	G_timer_2ms_wr = 0;
	G_timer_2ms_rd = 0;
	Time_init(400);	// 2MS
	Led_init();

	while(Ok != Flash_Init(12, TRUE)){;}                         //ȷ����ʼ����ȷִ�к��ܽ���FLASH��̲�����FLASH��ʼ�������ʱ��,����ģʽ���ã�
}


int main(void)
{
	unsigned short led_count = 0;
	Systimeclk();
	Bsp_Init();
	Upgrade_init();
	Wdt_Init(WdtResetEn, WdtT3s28);															//WDT ��ʼ��	
	Wdt_Start();
	Wdt_Feed(); 
	while(1)
	{
		Uart0_loop();
		if (G_timer_2ms_wr != G_timer_2ms_rd)
		{
			G_timer_2ms_rd++;
			led_count++;
			if (led_count >= 50)
			{
				led_count = 0;
				if (Upgrade_is_rebooting())
				{
					LED0_ON();
				}
				else
				{
					LED0_TOGGLE();
				}
			}	
			Upgrade_loop_2ms();
		}
		Wdt_Feed(); 
	}
}

