#include "bsp_time.h"
#include "bsp_uart.h"
#include "flash.h"
#include "sysctrl.h"
#include "gpio.h"
#include "upgrade.h"
#include "wdt.h"

// 指示灯定义
#define  LED0_PORT		  (GpioPortC) 
#define  LED0_PIN		    (GpioPin13) 
#define  LED0_TOGGLE()	(Gpio_GetInputIO(LED0_PORT, LED0_PIN))?(Gpio_ClrIO(LED0_PORT, LED0_PIN)):(Gpio_SetIO(LED0_PORT, LED0_PIN))
#define  LED0_ON()  	  (Gpio_ClrIO(LED0_PORT, LED0_PIN))

extern unsigned char G_timer_2ms_wr;
unsigned char        G_timer_2ms_rd;

/*****************************************************************
Systimeclk:
	系统时钟初始化
******************************************************************/
void Systimeclk(void)
{
    stc_sysctrl_clk_cfg_t stcCfg;
    stc_sysctrl_pll_cfg_t stcPLLCfg;    
    if(Sysctrl_GetHClkFreq()==48000000u&&Sysctrl_GetPClkFreq()==48000000u)
		{
			
		}
		else{
    ///< 开启FLASH外设时钟
    Sysctrl_SetPeripheralGate(SysctrlPeripheralFlash, TRUE); 
    Flash_WaitCycle(FlashWaitCycle2);                           // 因将要倍频的PLL作为系统时钟HCLK会达到48MHz：所以此处预先设置FLASH 读等待周期为1 cycle(默认值为0 cycle)
          
    Sysctrl_SetXTHFreq(SysctrlXthFreq6_12MHz);                  //切换时钟前（根据外部高速晶振）设置XTH频率范围,配置晶振参数，使能目标时钟，此处为8MHz
    Sysctrl_XTHDriverCfg(SysctrlXtalDriver3);
    Sysctrl_SetXTHStableTime(SysctrlXthStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkXTH, TRUE);
		
	  stcPLLCfg.enInFreq    = SysctrlPllInFreq6_12MHz;         //XTH 8MHz
    stcPLLCfg.enOutFreq   = SysctrlPllOutFreq36_48MHz;       //PLL 输出48MHz
    stcPLLCfg.enPllClkSrc = SysctrlPllXthXtal;               //输入时钟源选择XTH
    stcPLLCfg.enPllMul    = SysctrlPllMul6;                  //8MHz x 6 = 48MHz
    Sysctrl_SetPLLFreq(&stcPLLCfg);       
    Sysctrl_SetPLLStableTime(SysctrlPllStableCycle16384);
    Sysctrl_ClkSourceEnable(SysctrlClkPLL, TRUE);
    
    stcCfg.enClkSrc  = SysctrlClkPLL;                        //< 选择PLL作为HCLK时钟源
    stcCfg.enHClkDiv = SysctrlHclkDiv1;                      //< HCLK SYSCLK/1
    stcCfg.enPClkDiv = SysctrlPclkDiv1;                      //< PCLK 为HCLK/1
    Sysctrl_ClkInit(&stcCfg);                                //< 系统时钟初始化
		
    Sysctrl_SysClkSwitch(SysctrlClkPLL);							       //< 时钟切换
		}
		delay1ms(20);		
}

/*****************************************************************
Led_init:
	指示灯初始化函数
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
	设备外设初始化函数
******************************************************************/
void Bsp_Init(void)
{
	G_timer_2ms_wr = 0;
	G_timer_2ms_rd = 0;
	Time_init(400);	// 2MS
	Led_init();

	while(Ok != Flash_Init(12, TRUE)){;}                         //确保初始化正确执行后方能进行FLASH编程操作，FLASH初始化（编程时间,休眠模式配置）
}


int main(void)
{
	unsigned short led_count = 0;
	Systimeclk();
	Bsp_Init();
	Upgrade_init();
	Wdt_Init(WdtResetEn, WdtT3s28);															//WDT 初始化	
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

