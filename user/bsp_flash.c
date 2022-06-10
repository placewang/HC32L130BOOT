#include "bsp_flash.h"	
#include "flash.h"
int  Flash_writeBy(uint32_t u32Addr,uint8_t u8TestData)	
{	
        ///< FLASH 字节写、校验
        if (Ok == Flash_WriteByte(u32Addr,u8TestData))
        {
            while(*((volatile uint8_t*)u32Addr) != u8TestData)  //如果写入的数据不对，在此处死循环
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












