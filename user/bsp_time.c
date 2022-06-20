#include "bsp_time.h"
#include "bt.h"
#include "flash.h"

/*******TIM0中断服务函数*****************/
unsigned char G_timer_2ms_wr;
void Tim0_IRQHandler(void)
{
		G_timer_2ms_wr++;
    //Timer0 模式0 溢出中断
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
			Bt_ClearIntFlag(TIM0,BtUevIrq); //中断标志清零
    }
}

int  Flash_writeBy(uint32_t u32Addr, uint8_t u8TestData)	
{	
	///< FLASH 字节写、校验
	if (Ok == Flash_WriteByte(u32Addr, u8TestData))
	{
		while(*((volatile uint8_t*)u32Addr) != u8TestData)     //如果写入的数据不对，在此处死循环
		{
			;
		}
	}		
	return 1;
}

uint8_t  Flash_readBy(uint32_t u32Addr)	
{	
	return(*((volatile uint8_t*)u32Addr));                   //从Flash读数据
}


void Time_init(uint16_t u16Period)
{
	uint16_t u16ArrValue;
	uint16_t u16CntValue;
	stc_bt_mode0_cfg_t stcBtBaseCfg;

	DDL_ZERO_STRUCT(stcBtBaseCfg);

	Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer外设时钟使能

	stcBtBaseCfg.enWorkMode = BtWorkMode0;                     //定时器模式
	stcBtBaseCfg.enCT       = BtTimer;                         //定时器功能，计数时钟为内部PCLK
	stcBtBaseCfg.enPRS      = BtPCLKDiv256;                    //PCLK/256
	stcBtBaseCfg.enCntMode  = Bt16bitArrMode;                  //自动重载16位计数器/定时器
	stcBtBaseCfg.bEnTog     = FALSE;
	stcBtBaseCfg.bEnGate    = FALSE;
	//stcBtBaseCfg.pfnTim0Cb  = &Tim0_IRQHandler;
	stcBtBaseCfg.enGateP    = BtGatePositive;
	Bt_Mode0_Init(TIM0, &stcBtBaseCfg);                        //TIM0 的模式0功能初始化

	u16ArrValue = 0x10000 - u16Period;
	Bt_M0_ARRSet(TIM0, u16ArrValue);                           //设置重载值(ARR = 0x10000 - 周期)

	u16CntValue = 0x10000 - u16Period;
	Bt_M0_Cnt16Set(TIM0, u16CntValue);                         //设置计数初值

	Bt_ClearIntFlag(TIM0,BtUevIrq);                            //清中断标志   
	Bt_Mode0_EnableIrq(TIM0);                                  //使能TIM0中断(模式0时只有一个中断)
	EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                    //TIM0中断使能
	Bt_M0_Run(TIM0);
}


