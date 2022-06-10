#ifndef __BSPFLASH_H__
#define __BSPFLASH_H__

#include "ddl.h"

int  Flash_writeBy(uint32_t u32Addr,uint8_t u8TestData);	

uint8_t  Flash_readBy(uint32_t u32Addr);

#endif







