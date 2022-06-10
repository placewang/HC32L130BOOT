#ifndef  APP__CMD__H
#define  APP__CMD__H
#include "ddl.h"
#include "queue.h"

extern uint32_t flashaddr;                                           //读写flash开始地址
extern uint8_t  Rflashdtata[100];                                     //读flash发至主控数据暂存
extern uint8_t W_Flash_s;                                            //读写Flash 成功标志位 																						
extern uint8_t R_Flash_s;
extern uint8_t Key_Inquire;                                          //键盘查询标志位
extern uint8_t Version_Inquire;                                      //版本查询标志位
extern uint8_t Toggle;                                               //主控与GRS新老协议切换标志位
extern uint8_t ToggleAN;                                            //版本转换应答 
extern uint16_t  FlashLen;                                          //存入flash数据长度


uint8_t Cmd_Task(void);
void Cmd_char(uint8_t * );
unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, unsigned short CRCinit);


#endif



