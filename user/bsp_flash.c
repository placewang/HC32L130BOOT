#include "bsp_flash.h"	
#include "flash.h"
int  Flash_writeBy(uint32_t u32Addr,uint8_t u8TestData)	
{	
        ///< FLASH �ֽ�д��У��
        if (Ok == Flash_WriteByte(u32Addr,u8TestData))
        {
            while(*((volatile uint8_t*)u32Addr) != u8TestData)  //���д������ݲ��ԣ��ڴ˴���ѭ��
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












