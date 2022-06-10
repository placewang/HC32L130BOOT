#ifndef  APP__CMD__H
#define  APP__CMD__H
#include "ddl.h"
#include "queue.h"

extern uint32_t flashaddr;                                           //��дflash��ʼ��ַ
extern uint8_t  Rflashdtata[100];                                     //��flash�������������ݴ�
extern uint8_t W_Flash_s;                                            //��дFlash �ɹ���־λ 																						
extern uint8_t R_Flash_s;
extern uint8_t Key_Inquire;                                          //���̲�ѯ��־λ
extern uint8_t Version_Inquire;                                      //�汾��ѯ��־λ
extern uint8_t Toggle;                                               //������GRS����Э���л���־λ
extern uint8_t ToggleAN;                                            //�汾ת��Ӧ�� 
extern uint16_t  FlashLen;                                          //����flash���ݳ���


uint8_t Cmd_Task(void);
void Cmd_char(uint8_t * );
unsigned short CRC16(unsigned char *buf,unsigned long dlen, int poly, unsigned short CRCinit);


#endif



