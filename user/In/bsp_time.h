#ifndef __TRIME__
#define __TRIME__

#include "ddl.h"
void Time_init(uint16_t u16Period);


int  Flash_writeBy(uint32_t u32Addr,uint8_t u8TestData);	

uint8_t  Flash_readBy(uint32_t u32Addr);
#endif

