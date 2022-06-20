#include "bsp_time.h"
#include "bt.h"
#include "flash.h"

/*******TIM0�жϷ�����*****************/
unsigned char G_timer_2ms_wr;
void Tim0_IRQHandler(void)
{
		G_timer_2ms_wr++;
    //Timer0 ģʽ0 ����ж�
    if(TRUE == Bt_GetIntFlag(TIM0, BtUevIrq))
    {
			Bt_ClearIntFlag(TIM0,BtUevIrq); //�жϱ�־����
    }
}

int  Flash_writeBy(uint32_t u32Addr, uint8_t u8TestData)	
{	
	///< FLASH �ֽ�д��У��
	if (Ok == Flash_WriteByte(u32Addr, u8TestData))
	{
		while(*((volatile uint8_t*)u32Addr) != u8TestData)     //���д������ݲ��ԣ��ڴ˴���ѭ��
		{
			;
		}
	}		
	return 1;
}

uint8_t  Flash_readBy(uint32_t u32Addr)	
{	
	return(*((volatile uint8_t*)u32Addr));                   //��Flash������
}


void Time_init(uint16_t u16Period)
{
	uint16_t u16ArrValue;
	uint16_t u16CntValue;
	stc_bt_mode0_cfg_t stcBtBaseCfg;

	DDL_ZERO_STRUCT(stcBtBaseCfg);

	Sysctrl_SetPeripheralGate(SysctrlPeripheralBaseTim, TRUE); //Base Timer����ʱ��ʹ��

	stcBtBaseCfg.enWorkMode = BtWorkMode0;                     //��ʱ��ģʽ
	stcBtBaseCfg.enCT       = BtTimer;                         //��ʱ�����ܣ�����ʱ��Ϊ�ڲ�PCLK
	stcBtBaseCfg.enPRS      = BtPCLKDiv256;                    //PCLK/256
	stcBtBaseCfg.enCntMode  = Bt16bitArrMode;                  //�Զ�����16λ������/��ʱ��
	stcBtBaseCfg.bEnTog     = FALSE;
	stcBtBaseCfg.bEnGate    = FALSE;
	//stcBtBaseCfg.pfnTim0Cb  = &Tim0_IRQHandler;
	stcBtBaseCfg.enGateP    = BtGatePositive;
	Bt_Mode0_Init(TIM0, &stcBtBaseCfg);                        //TIM0 ��ģʽ0���ܳ�ʼ��

	u16ArrValue = 0x10000 - u16Period;
	Bt_M0_ARRSet(TIM0, u16ArrValue);                           //��������ֵ(ARR = 0x10000 - ����)

	u16CntValue = 0x10000 - u16Period;
	Bt_M0_Cnt16Set(TIM0, u16CntValue);                         //���ü�����ֵ

	Bt_ClearIntFlag(TIM0,BtUevIrq);                            //���жϱ�־   
	Bt_Mode0_EnableIrq(TIM0);                                  //ʹ��TIM0�ж�(ģʽ0ʱֻ��һ���ж�)
	EnableNvic(TIM0_IRQn, IrqLevel3, TRUE);                    //TIM0�ж�ʹ��
	Bt_M0_Run(TIM0);
}


