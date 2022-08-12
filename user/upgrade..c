/********************************************************************************
XmodemЭ��
һ��֡��ʽ
    | SOH | ��Ϣ����� | ��Ϣ����ŵķ��� | �������� | ����У��� |
    |_____|____________|__________________|__________|____________|
����˵����
����SOH ֡�Ŀ�ͷ�ֽڣ�������Ϣ���еĵ�һ���ֽ�
������Ϣ����ţ� �� 256 ȡģ���õ���ǰ���ţ���һ����Ϣ�������Ϊ 1
��������Ϣ����ŷ�Χ0~255
������Ϣ����ŵķ��룺 ��ǰ��Ϣ���ŵķ���
�����������Σ� �������εĳ��ȹ̶�Ϊ 128 �ֽڣ�������û���κ����ƣ��������ı����ݻ����������
��������У��ͣ� 1�ֽڵ�����У��ͣ�ֻ���������μ����� 256 ȡģ����

���������߼�
����1> �շ�˫��������ͨ�󣬷��ͷ��ȴ����շ����� NAK �źš�����һ�� NAK ������ͷ�����Ϊ ��ʼ���͵�һ����
����2> ���ͷ�һ���յ���һ�� NAK �������˴��䣬���ͷ��ͽ�������ÿ�� 128 �ֽڴ����֡��ʽ���ͣ��ٵȴ����շ���ȷ���ź�
����3> ���ͷ��յ����շ������� ACK �źţ�����Ϊ��Ϣ������ȷ���գ����з�����һ�����ĺ���
����4> ���ͷ��յ����շ������� NAK �źţ�����Ϊ�����ط�ͬһ���ݰ�
����5> ���ͷ��յ����շ������� CAN �źţ�����Ϊ����������ֹͣ�������
����6> ���ͷ�����������ȫ�����ݣ���Ҫ�������������� EOT �ź�֪ͨ���շ������շ��� ACK ����ȷ��
����7> ���շ����� CAN ������ֹͣ������̣����ͷ��յ� CAN �󣬲����� EOT ȷ��
����8> ��Ȼ��Ϣ������ SOH ����־һ����Ϣ������ʼ�ģ����� SOH λ���ϳ��ֵ� EOT���ʾ���ݴ����������Ҳû�����ݴ�����
����9> ���շ�����Ӧȷ����Ϣ����ŵ������ԣ�ͨ������Ϣ�����ȡ����Ȼ�����Ϣ����ŵĲ�����򣬽��Ϊ 0 ��ʾ��ȷ�������Ϊ 0 ���� NAK �����ش�
����10> ���շ�ȷ����Ϣ�������ȷ��Ȼ�����Ƿ���������š�������������õ�����Ϣ����ţ�˵���������ش���Ӧ�÷���һ�� CAN ����ֹ����
����11> ����10>�����Ψһ���⣬���յ��İ�����Ϣ�������ǰһ����Ϣ�������ͬ��������������շ��򵥺�������ظ��İ������ͷ����� ACK ��׼��������һ����
����12> ���շ�ȷ������Ϣ����ŵ������Ժ�����ȷ�����ĺ�ֻ�� 128�ֽڵ��������ν���������У�飬�����֡�����һ���ֽڣ�����У��ͣ��Ƚϣ���ͬ ���� ACK����ͬ���� NAK

������ʱ����
����1> ���շ��ȴ�һ����Ϣ���ĵ��������еĳ�ʱʱ��Ϊ 10 �룬ÿ����ʱ���� NAK
����2> ���յ���ʱ�����չ�����ÿ���ַ��ĳ�ʱ���Ϊ 1 ��
����3> Ϊ���֡����շ������������ͷ��ڵȴ�һ�������ֽ�ʱ��Ӧ�ò��ó�ʱ����
����4> һ�����俪ʼ�����ͷ����õ����� 1 ���ӳ�ʱʱ�ޣ������շ������ʱ��������ACK ,NAK ,CAN ֮ǰ�ı��봦��
����5> ���еĳ�ʱ�������¼��������� 10 ��

�ġ������ַ�
���������ַ����� ASICII ��׼���壬���Ⱦ�Ϊ 1 �ֽ�
����SOH 0x01
����EOT 0x04
����ACK 0x06
����NAK 0x15
����CAN 0x18
��������������������������������
*********************************************************************************/


#include "bsp_uart.h"
#include "upgrade.h"
#include "flash.h"
#include "gpio.h"
// ����״̬����
#define UPGRADE_STATUS_IDLE				      0x00	// ��״̬
#define UPGRADE_STATUS_POWON			      0x01	// �ϵ����״̬�������ϵ���ɣ��ȴ��ظ�������ǿ����������ʱ�����APP
#define UPGRADE_STATUS_UPGRADE_START	  0x02	// ����������Ϣ���ȴ���Ӧ
#define UPGRADE_STATUS_XMODEM			      0x03	// ����XMODEMģʽ
#define UPGRADE_STATUS_CRC_CEHCK		    0x04	// CRCУ��
#define UPGRADE_STATUS_COPY_DATA		    0x05	// ��������״̬
#define UPGRADE_STATUS_REBOOT			      0x06	// ��ʱ����ʱ��ϵͳ����
#define UPGRADE_STATUS_JUMP_APP			    0x07	// ��ʱ�� ��ʱ����ת��APP

//Xmodem�����״̬����Ŀǰֻ֧�ֱ�׼��Э�飬����֧����չЭ��
//�������ݣ����浽FLASH_APP_BACKUP_START_ADDRESS�ĵ�ַ��
#define XMODEM_STATUS_IDLE				      0x00	// ��״̬
#define XMODEM_STATUS_NAK_WAIT_DATA		  0x01	// ����NAK�ȴ�����״̬
#define XMODEM_STATUS_ACK_WAIT_DATA		  0x02	// ����ACK�ȴ�����״̬
#define XMODEM_STATUS_RECV_DATA			    0x03	// �������ݰ�״̬��1s��ʱΪ�հ�����
#define XMODEM_STATUS_CANCEL			      0x04	// ȡ������
#define XMODEM_STATUS_SUCCESS			      0x05	// �������


unsigned int G_flash_data[128];	              //����ĵ�ǰ��Ҫд����FLASH�е�����

struct UPGRADE_SUB
{
	unsigned char status;				       // ��ǰ״̬
	unsigned char retry_count;			   // ʣ������Դ�����ÿ�μ�
	unsigned short retry_timer;			   // ���Լ�ʱ����ÿ�μ�
	unsigned short recv_timer;			   // ���ݽ��ռ�ʱ������ʱ����Ϊһ��������ÿ�μ�
	
	unsigned char rx_buf[256];			   // ���յ��İ�
	unsigned char rx_buf_len;			     // ��ǰ�Ľ��ճ���
	
	// xmodemЭ�����
	unsigned char  xmodem_status;		   // xmodem�����״̬��
	unsigned int   flash_address;			 // ��ǰ������flash��ַ
	unsigned int   flash_max_address;	 // ��ǰ���������flash��ַ
	unsigned int   *flash_data_p;			 // ��ǰ�յ���flash���ݣ�ÿһ������512�ֽ�
	unsigned short flash_data_len;	   // ��ǰ�յ���flash���ݳ���
	unsigned int   recv_all_data_len;	 // �յ��������ܳ���
	unsigned char  next_sq;				     // ��һ���������
}__attribute__((packed));

struct UPGRADE_SUB G_upgrade_sub;

struct XMODEM_MSG_SOH
{
	unsigned char cmd;					      // ��ǰ״̬
	unsigned char sequence;				    // ���
	unsigned char sequence_inverse;		// ��ŵķ���
	unsigned char data[128];			    // ���յ�������
	unsigned char check;				      // У���
}__attribute__((packed));

typedef void (*pFunction)(void);

void GoApp(void) 
{ 
   volatile uint32_t JumpAddress; 
   pFunction Jump_To; 

   JumpAddress = *(volatile uint32_t*) (FLASH_APP_START_ADDRESS + 4); 
   Jump_To = (pFunction) JumpAddress; 
   /* Initialize user application's Stack Pointer */ 
   __set_MSP(*(volatile uint32_t*) FLASH_APP_START_ADDRESS); 
   Jump_To(); 
}

unsigned char FLASH_read(unsigned char *u32Addr)
{
	 return *((volatile uint8_t*)u32Addr);
}

unsigned int FLASH_u32_read(unsigned int *u32Addr)
{
	 return *((volatile uint8_t*)u32Addr);
}


void Upgrade_sector_erase_carefull(unsigned int address)
{
	int i;
	unsigned int read_data;

	for (i = 0; i < 5; i++)
	{
		Flash_SectorErase(address);
		read_data = FLASH_u32_read((unsigned int *)address);
		if (read_data == 0xFFFFFFFF)
		{
			break;
		}
	}
}

void Upgrade_wirte_word_carefull(unsigned int address, unsigned int data)
{
	int i;
	unsigned int read_data;
	
	for(i = 0; i < 5; i++)
	{
		Flash_WriteWord(address, data);
		read_data = FLASH_u32_read((unsigned int *)address);
		if(read_data == data)
		{
			break;
		}
	}
}

unsigned short CRC16Check(unsigned char* buf, unsigned int len)
{
	unsigned int i, j;
	unsigned short usCRCReg;
	unsigned char temp;

	usCRCReg = FLASH_read(buf) << 8;
	usCRCReg |= FLASH_read(buf + 1);

	for (i = 2; i < len; i++)
	{
		temp = FLASH_read(buf + i);
		for (j = 8; j > 0; j--)
		{
			if ((usCRCReg & 0x8000) != 0)
			{
				usCRCReg <<= 1;
				if (((temp >> (j - 1)) & 0x01) != 0)
					usCRCReg |= 0x0001;
					usCRCReg ^= 0x1021;
			}
			else
			{
				usCRCReg <<= 1;
				if (((temp >> (j - 1)) & 0x01) != 0)
					usCRCReg |= 0x0001;
			}
		}
	}
	
	for (j = 16; j > 0; j--)
	{
		if ((usCRCReg & 0x8000) != 0)
		{
			usCRCReg <<= 1;
			usCRCReg ^= 0x1021;
		}
		else
			usCRCReg <<= 1;
	}
	return usCRCReg;
}

/*****************************************************************
Upgrade_get_baud_rate:
	��ȡ�����ʵ�ֵ
	����ֵ����������ֵ
******************************************************************/
unsigned int Upgrade_get_baud_rate(void)
{
//	struct CONFIG_BAUD *config_baud_p;
	uint32_t bd=0;
//	config_baud_p = (struct CONFIG_BAUD *)CONFIG_BAUD_RATE_ADDRESS;
	bd |=(*(volatile uint8_t*)(CONFIG_BAUD_RATE_ADDRESS+2))<<24;
	bd |=(*(volatile uint8_t*)(CONFIG_BAUD_RATE_ADDRESS+3))<<16;
	bd |=(*(volatile uint8_t*)(CONFIG_BAUD_RATE_ADDRESS+4))<<8;
	bd |=(*(volatile uint8_t*)(CONFIG_BAUD_RATE_ADDRESS+5));
	return bd;
}

/*****************************************************************
Upgrade_get_upgrade_flag:
	��ȡ��ǰ��������־λ
******************************************************************/
unsigned int Upgrade_get_upgrade_flag(void)
{
	unsigned int *upgrade_flag_p;
	upgrade_flag_p = (unsigned int *)UPGRADE_FLAG_ADDRESS;
	return *upgrade_flag_p;
}

/*****************************************************************
Upgrade_get_baud_rate:
	��ȡ�����ʵ�ֵ
	����ֵ����������ֵ
******************************************************************/
void Upgrade_update_upgrade_flag(unsigned int flag)
{
	// ������ʼ���Ȳ���
	Upgrade_sector_erase_carefull(UPGRADE_FLAG_ADDRESS);
	// д��־λ
	Upgrade_wirte_word_carefull(UPGRADE_FLAG_ADDRESS, flag);
}

/*****************************************************************
Upgrade_send_pow_on:
	�����ϵ���Ϣ
******************************************************************/
void Upgrade_send_pow_on(void)
{
	Uart0_send_btye(0x5A);
	Uart0_send_btye(0x04);
	Uart0_send_btye(0x53);
	Uart0_send_btye(0xA5);
}

/*****************************************************************
Upgrade_send_upgrade_start:
	����������ʼ��Ϣ
******************************************************************/
void Upgrade_send_upgrade_start(void)
{
	Uart0_send_btye(0x5A);
	Uart0_send_btye(0x04);
	Uart0_send_btye(0x54);
	Uart0_send_btye(0xA5);
}

/*****************************************************************
Upgrade_send_upgrade_start:
	����������ʼ��Ϣ
	result��0�������ɹ���1������ʧ��
******************************************************************/
void Upgrade_send_upgrade_result(unsigned char result)
{
	Uart0_send_btye(0x5A);
	Uart0_send_btye(0x04);
	Uart0_send_btye(0x79);
	Uart0_send_btye(result);
	Uart0_send_btye(0xA5);
}

/*****************************************************************
Upgrade_set_timer:
	���������У��������Լ�����
	count��ÿ��2ms
******************************************************************/
void Upgrade_set_retry_timer(unsigned short count)
{
	G_upgrade_sub.retry_timer = count;
}

/*****************************************************************
Upgrade_set_timeout_count:
	���������У��������Լ�����
	count����ʱ����
******************************************************************/
void Upgrade_set_retry_count(unsigned short count)
{
	G_upgrade_sub.retry_count = count;
}

/*****************************************************************
Upgrade_set_recv_timer:
	���������У��������ݽ��ռ�ʱ��
	count����ʱ����
******************************************************************/
void Upgrade_set_recv_timer(unsigned short count)
{
	G_upgrade_sub.recv_timer = count;
}

/*****************************************************************
Upgrade_xmodem_send_cmd:
	���������У�����xmodemЭ��Ŀ��Ʒ�
	cmd�����Ʒ�
	#define XMODEM_CMD_EOT			0x04		// ���ݴ�����գ��ظ�ACK
	#define XMODEM_CMD_ACK			0x06		// �ظ�
	#define XMODEM_CMD_NAK			0x15		// �ظ����ش���ǰ��
	#define XMODEM_CMD_CAN			0x18		// ���շ���Ҫ��ֹͣ����
******************************************************************/
void Upgrade_xmodem_send_cmd(unsigned char cmd)
{
	Uart0_send_btye(cmd);
}

/*****************************************************************
Upgrade_sector_write:
	���������У��ڲ�FLASH������д����
	address��д�ĵ�ַ
	data_p��д�����ݣ�512���ֽ�
******************************************************************/
void Upgrade_sector_write(unsigned int address, unsigned int *data_p)
{
	unsigned int i;
	
	// ������ʼ���Ȳ���
	Upgrade_sector_erase_carefull(address);
	
	for (i = 0; i < 128; i++)
	{
		Upgrade_wirte_word_carefull(address, *data_p);
		address += 4;
		data_p++;
	}
}

/*****************************************************************
Upgrade_sector_write:
	���������У��ڲ�FLASH������д����
	address��д�ĵ�ַ
	data_p��д�����ݣ�512���ֽ�
******************************************************************/
unsigned short Upgrade_file_crc_check(unsigned short len)
{
	unsigned char *backup_address_p;
	unsigned char  *Packend_CRCaddress;
	Packend_CRCaddress=(unsigned char *)(FLASH_APP_BACKUP_START_ADDRESS+len-6);
	if(Packend_CRCaddress[0]=='C'&&Packend_CRCaddress[1]=='R'&&Packend_CRCaddress[2]=='C'&&Packend_CRCaddress[3]==0x10)
	{
		len=len-6;
	}
	backup_address_p = (unsigned char *)FLASH_APP_BACKUP_START_ADDRESS;
	return CRC16Check(backup_address_p, len);
}


/*****************************************************************
Upgrade_sector_write:
	���������У��ڲ�FLASH������д����
	address��д�ĵ�ַ
	data_p��д�����ݣ�512���ֽ�
******************************************************************/
void Upgrade_cope_flash_to_app(void)
{
	unsigned int app_address;
	unsigned int backup_address;
	
	backup_address = FLASH_APP_BACKUP_START_ADDRESS;
	for (app_address = FLASH_APP_START_ADDRESS; app_address < FLASH_APP_STOP_ADDRESS; app_address+=512)
	{
		Upgrade_sector_write(app_address, (unsigned int *)backup_address);
		backup_address += 512;
	}
}

unsigned char Upgrade_test_appflash(void)
{
	unsigned int address;
	unsigned char data[512];
	unsigned int *data_p;
	unsigned char *addr_p;
	
	for (address = 0; address < 512; address++)
	{
		data[address] = address;
	}
	
	data_p = (unsigned int *)data;
	for (address = FLASH_APP_START_ADDRESS; address <= FLASH_APP_STOP_ADDRESS; address+=512)
	{
		// д����
		Upgrade_sector_write(address, data_p);
	}
	
	for (address = FLASH_APP_START_ADDRESS; address <= FLASH_APP_STOP_ADDRESS; address++)
	{
		addr_p = (unsigned char *)address;
		if (*addr_p != (address & 0xFF))
		{
			return 1;
		}
	}
	return 0;
}

void Upgrade_xmodem_ask_resend(void)
{
	Upgrade_xmodem_send_cmd(XMODEM_CMD_NAK);
	Upgrade_set_retry_timer(3 * TIMEOUT_1S);
	Upgrade_set_retry_count(10);
	Upgrade_set_recv_timer(0);
	G_upgrade_sub.rx_buf_len = 0;
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_NAK_WAIT_DATA;
}

void Upgrade_xmodem_ask_next(void)
{
	Upgrade_xmodem_send_cmd(XMODEM_CMD_ACK);
	Upgrade_set_retry_timer(3 * TIMEOUT_1S);
	Upgrade_set_retry_count(10);
	Upgrade_set_recv_timer(0);
	G_upgrade_sub.rx_buf_len = 0;
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_ACK_WAIT_DATA;
}

void Upgrade_xmodem_cannel(void)
{
	Upgrade_xmodem_send_cmd(XMODEM_CMD_CAN);
	Upgrade_set_retry_timer(TIMEOUT_1S);	// 1����˳�
	Upgrade_set_retry_count(1);
	Upgrade_set_recv_timer(0);
	G_upgrade_sub.rx_buf_len = 0;
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
}

void Upgrade_xmodem_recv_data(void)
{
	Upgrade_set_retry_timer(0);
	Upgrade_set_retry_count(0);
	Upgrade_set_recv_timer(TIMEOUT_100ms);
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_RECV_DATA;
}

/*****************************************************************
Upgrade_start_xmodem:
	������������xmodem����
******************************************************************/
void Upgrade_start_xmodem(void)
{
	//Upgrade_xmodem_send_cmd(XMODEM_CMD_NAK);
	Upgrade_set_retry_timer(3 * TIMEOUT_1S);
	Upgrade_set_retry_count(11);
	Upgrade_set_recv_timer(0);
	G_upgrade_sub.rx_buf_len = 0;
	G_upgrade_sub.recv_all_data_len = 0;
	G_upgrade_sub.next_sq = 1;
	
	G_upgrade_sub.flash_address = FLASH_APP_BACKUP_START_ADDRESS;
	G_upgrade_sub.flash_max_address = FLASH_APP_BACKUP_STOP_ADDRESS;
	G_upgrade_sub.flash_data_len = 0;
	G_upgrade_sub.flash_data_p = G_flash_data;
	memset((unsigned char *)G_upgrade_sub.flash_data_p, 0xFF, 512);
	
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_NAK_WAIT_DATA;
}

/*****************************************************************
Upgrade_soh_sumcheck:
	���������� xmodem��soh��Ϣ����У���
	msg_soh_p����ҪУ�����Ϣ
	����ֵ��0��У��ɹ�����0��У��ʧ��
******************************************************************/
unsigned char Upgrade_soh_sumcheck(struct XMODEM_MSG_SOH *msg_soh_p)
{
	unsigned short checksum;
	int i;
	
	if (msg_soh_p == NULL)
	{
		return 1;
	}
	
	checksum = 0;
	for (i = 0; i < 128; i++)
	{
		checksum += msg_soh_p->data[i];
	}
	
	if ((checksum & 0xFF) != msg_soh_p->check)
	{
		return 1;
	}
	
	return 0;
}

/*****************************************************************
Upgrade_flash_data_deal:
	���������Ľ��յ����ݵ�flash����
	data_p:�����յ���128�����ݵ�ָ��
******************************************************************/
void Upgrade_flash_data_deal(unsigned char *data_p)
{
	if (data_p == NULL)
	{
		return;
	}
	
	if (G_upgrade_sub.flash_address < G_upgrade_sub.flash_max_address)
	{// Ŀǰ�Ĳ�����ַ���ڵ�ַ��Χ��
		memcpy((unsigned char *)G_upgrade_sub.flash_data_p + G_upgrade_sub.flash_data_len, data_p, 128);
		G_upgrade_sub.flash_data_len += 128;
		
		if (G_upgrade_sub.flash_data_len >= 512)
		{// д��flash��
			Upgrade_sector_write(G_upgrade_sub.flash_address, G_upgrade_sub.flash_data_p);
			memset((unsigned char *)G_upgrade_sub.flash_data_p, 0xFF, 512);
			G_upgrade_sub.flash_address += 512;
			G_upgrade_sub.flash_data_len = 0;
		}
	}
}

/*****************************************************************
Upgrade_flash_end_deal:
	���������Ľ��յ����ݵ�flash����
******************************************************************/
void Upgrade_flash_end_deal(void)
{
	if (G_upgrade_sub.flash_address < G_upgrade_sub.flash_max_address)
	{// Ŀǰ�Ĳ�����ַ���ڵ�ַ��Χ��
		if (G_upgrade_sub.flash_data_len > 0)
		{
			G_upgrade_sub.flash_data_len = 512;
		}
		
		if (G_upgrade_sub.flash_data_len >= 512)
		{// д��flash��
			Upgrade_sector_write(G_upgrade_sub.flash_address, G_upgrade_sub.flash_data_p);
			memset((unsigned char *)G_upgrade_sub.flash_data_p, 0xFF, 512);
			G_upgrade_sub.flash_address += 512;
			G_upgrade_sub.flash_data_len = 0;
		}
	}
}

/*****************************************************************
Upgrade_xmodem_message_deal:
	����������xmodemģʽ���յ���Ϣ����
******************************************************************/
void Upgrade_xmodem_message_deal(void)
{
	struct XMODEM_MSG_SOH *msg_soh_p;
	
	// ���ȼ��
	if (G_upgrade_sub.rx_buf_len != sizeof(struct XMODEM_MSG_SOH))
	{
		Upgrade_xmodem_ask_resend();
		return;
	}
	
	msg_soh_p = (struct XMODEM_MSG_SOH *)G_upgrade_sub.rx_buf;
	
	// ���������У��
	if ((msg_soh_p->sequence ^ msg_soh_p->sequence_inverse) != 0xFF)
	{// 
		Upgrade_xmodem_ask_resend();
		return;
	}
	
	// �����ȷ��У��
	if (msg_soh_p->sequence != G_upgrade_sub.next_sq)
	{// �����ڴ������
		if ((msg_soh_p->sequence + 1) == G_upgrade_sub.next_sq)
		{// ��һ��
			Upgrade_xmodem_ask_next();
			return;
		}
		else
		{// ��Ŵ���ȡ������
			Upgrade_xmodem_cannel();
			return;
		}
	}

	// ����У���
	if (Upgrade_soh_sumcheck(msg_soh_p) != 0)
	{
		Upgrade_xmodem_ask_resend();
		return;
	}
	
	Upgrade_flash_data_deal(msg_soh_p->data);
	
	G_upgrade_sub.recv_all_data_len += 128;
	G_upgrade_sub.next_sq++;
	Upgrade_xmodem_ask_next();
	return;
}

/*****************************************************************
Upgrade_message_deal:
	�����������յ�������Ϣ����
******************************************************************/
void Upgrade_message_deal(void)
{
	unsigned short file_bytes;
	unsigned short recv_check;
	unsigned short my_check;
	
	switch (G_upgrade_sub.status)
	{
		case UPGRADE_STATUS_IDLE:	// ��״̬
			break;
		
		case UPGRADE_STATUS_POWON:	// �ϵ����״̬�������ϵ���ɣ��ȴ��ظ�������ǿ����������ʱ�����APP
			if (G_upgrade_sub.rx_buf_len >= 4)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x52) &&
					(G_upgrade_sub.rx_buf[3] == 0xA5))
				{// �յ���Ӧ������������ʼ
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
					Upgrade_set_retry_count(10);
					Upgrade_send_upgrade_start();
					G_upgrade_sub.status = UPGRADE_STATUS_UPGRADE_START;
				}
			}
			break;
		
		case UPGRADE_STATUS_UPGRADE_START:	// ����������Ϣ���ȴ���Ӧ
			Backlight_ON();
			if (G_upgrade_sub.rx_buf_len >= 4)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x54) &&
					(G_upgrade_sub.rx_buf[3] == 0xA5))
				{// �յ���Ӧ����ʼ����
					Upgrade_update_upgrade_flag(CMD_UPGRADE_RCVDATA);
					Upgrade_start_xmodem();
					G_upgrade_sub.status = UPGRADE_STATUS_XMODEM;
				}
			}
			break;
		
		case UPGRADE_STATUS_XMODEM:			// ����XMODEMģʽ
			Backlight_ON();
			break;
		
		case UPGRADE_STATUS_CRC_CEHCK:		// CRCУ��
			if (G_upgrade_sub.rx_buf_len >= 8)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x78) &&
					(G_upgrade_sub.rx_buf[7] == 0xA5))
				{// �յ���Ӧ����ʼ����
//					if ((G_upgrade_sub.rx_buf[3] == 0x11) &&
//						(G_upgrade_sub.rx_buf[4] == 0x22) &&
//						(G_upgrade_sub.rx_buf[5] == 0x33) &&
//						(G_upgrade_sub.rx_buf[6] == 0x44))
//					{
//						// ���ñ�־λ����ʼ����
//						Upgrade_update_upgrade_flag(CMD_UPGRADE_BURN);
//						// ��������
//						Upgrade_cope_flash_to_app();
//						
//						// ���������ݣ�׼������
//						Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
//						Upgrade_send_upgrade_result(0);
//						Upgrade_set_retry_timer(TIMEOUT_1S);
//						Upgrade_set_retry_count(1);
//						G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
//						break;
//					}
					
						file_bytes = G_upgrade_sub.rx_buf[3];
						file_bytes = file_bytes << 8;
						file_bytes += G_upgrade_sub.rx_buf[4];
						
						recv_check = G_upgrade_sub.rx_buf[5];
						recv_check = recv_check << 8;
						recv_check += G_upgrade_sub.rx_buf[6];
						my_check = Upgrade_file_crc_check(file_bytes);
					if (my_check != recv_check)
					{ // У��ʧ��
						Upgrade_send_upgrade_result(1);
						Upgrade_set_retry_timer(TIMEOUT_1S);
						Upgrade_set_retry_count(1);
						G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
					}
					else
					{// У��ɹ�
						// ���ñ�־λ����ʼ����
						Upgrade_update_upgrade_flag(CMD_UPGRADE_BURN);
						// ��������
						Upgrade_cope_flash_to_app();
						
						// ���������ݣ�׼������
						Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
						Upgrade_send_upgrade_result(0);
						Upgrade_set_retry_timer(TIMEOUT_1S);
						Upgrade_set_retry_count(1);
						G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
						Backlight_ON();
					}
				}
			}
			break;
		
		case UPGRADE_STATUS_REBOOT:			// ��ʱ����ʱ��ϵͳ����
			break;
		
		case UPGRADE_STATUS_JUMP_APP:		// ��ʱ�� ��ʱ����ת��APP
			break;
		
		default:
			break;
	}
	
	// ������󣬳�������
	G_upgrade_sub.rx_buf_len = 0;
	// ��ʱ������
	Upgrade_set_recv_timer(0);
	return;
}

/*****************************************************************
Upgrade_data_deal:
	���������Ľ��յ����ݴ���
******************************************************************/
void Upgrade_xmodem_data_deal(unsigned char data)
{
	switch (G_upgrade_sub.xmodem_status)
	{
		case XMODEM_STATUS_IDLE:
			break;
		
		case XMODEM_STATUS_NAK_WAIT_DATA:
			if ((data == XMODEM_CMD_EOT) || (data == XMODEM_CMD_CAN))
			{// ����ֱ������
				Upgrade_set_retry_timer(TIMEOUT_1S);
				Upgrade_set_retry_count(1);
				Upgrade_set_recv_timer(0);
				G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
			}
			else if (data == XMODEM_CMD_SOH)
			{// �յ����ݰ�����
				G_upgrade_sub.rx_buf[0] = data;
				G_upgrade_sub.rx_buf_len = 1;
				Upgrade_xmodem_recv_data();
			}
			break;
			
		case XMODEM_STATUS_ACK_WAIT_DATA:
			if (data == XMODEM_CMD_EOT)
			{// ���ݴ������
				Upgrade_flash_end_deal();
				Upgrade_xmodem_send_cmd(XMODEM_CMD_ACK);
				
				Upgrade_set_retry_timer(TIMEOUT_1S);
				Upgrade_set_retry_count(1);
				Upgrade_set_recv_timer(0);
				G_upgrade_sub.xmodem_status = XMODEM_STATUS_SUCCESS;
			}
			else if (data == XMODEM_CMD_CAN)
			{// �Է�ȡ����ֱ������
				Upgrade_set_retry_timer(TIMEOUT_1S);
				Upgrade_set_retry_count(1);
				Upgrade_set_recv_timer(0);
				G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
			}
			else if (data == XMODEM_CMD_SOH)
			{
				G_upgrade_sub.rx_buf[0] = data;
				G_upgrade_sub.rx_buf_len = 1;
				Upgrade_xmodem_recv_data();
			}
			break;
		
		case XMODEM_STATUS_RECV_DATA:
			G_upgrade_sub.rx_buf[G_upgrade_sub.rx_buf_len] = data;
			G_upgrade_sub.rx_buf_len++;
			
			if (G_upgrade_sub.rx_buf_len < sizeof(struct XMODEM_MSG_SOH))
			{
				Upgrade_set_recv_timer(TIMEOUT_100ms);
				break;
			}
			
			// �������
			Upgrade_xmodem_message_deal();
			break;
		
		default:
			break;
	}
}

/*****************************************************************
Upgrade_data_deal:
	���������Ľ��յ����ݴ���
******************************************************************/
void Upgrade_data_deal(unsigned char data)
{
	if (G_upgrade_sub.status == UPGRADE_STATUS_XMODEM)
	{
		Upgrade_xmodem_data_deal(data);
	}
	else
	{
		if (G_upgrade_sub.rx_buf_len <= 255)
		{
			G_upgrade_sub.rx_buf[G_upgrade_sub.rx_buf_len] = data;
			G_upgrade_sub.rx_buf_len++;
		}
		Upgrade_set_recv_timer(TIMEOUT_100ms);
	}
}

void UPgrade_xmodem_retry_timer_deal(void)
{
	switch (G_upgrade_sub.xmodem_status)
	{
		case XMODEM_STATUS_IDLE:				// ��״̬
			break;
		
		case XMODEM_STATUS_NAK_WAIT_DATA:		// ����NAK�ȴ�����״̬
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// �л�����ת״̬���ȴ���ת
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					Upgrade_set_recv_timer(0);
					G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
				}
				else
				{// �ط�
					Upgrade_xmodem_send_cmd(XMODEM_CMD_NAK);
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case XMODEM_STATUS_ACK_WAIT_DATA:		// ����NAK�ȴ�����״̬
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// �л�����ת״̬���ȴ���ת
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					Upgrade_set_recv_timer(0);
					G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
				}
				else
				{// �ط�
					Upgrade_xmodem_send_cmd(XMODEM_CMD_ACK);
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case XMODEM_STATUS_RECV_DATA:			// �������ݰ�״̬��1s��ʱΪ�հ�����
			break;
			
		case XMODEM_STATUS_CANCEL:				// ȡ������
			// ����ʧ�ܣ��˻�ȥ.
			Upgrade_set_retry_timer(TIMEOUT_1S);
			Upgrade_set_retry_count(1);
			G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
			break;
		
		case XMODEM_STATUS_SUCCESS:
			// ����ɹ�������ȴ�CRCУ��״̬
			Upgrade_set_retry_timer(3 * TIMEOUT_1S);
			Upgrade_set_retry_count(10);
			Upgrade_set_recv_timer(0);
			G_upgrade_sub.status = UPGRADE_STATUS_CRC_CEHCK;
			break;
		
		default:
			break;
	}
}

void UPgrade_retry_timer_deal(void)
{
	switch (G_upgrade_sub.status)
	{
		case UPGRADE_STATUS_IDLE:	// ��״̬
			break;
		
		case UPGRADE_STATUS_POWON:	// �ϵ����״̬�������ϵ���ɣ��ȴ��ظ�������ǿ����������ʱ�����APP
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{ // �л�����ת״̬���ȴ���ת
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{ // �ط�
					Upgrade_send_pow_on();
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_UPGRADE_START:	// ����������Ϣ���ȴ���Ӧ
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// �л�����ת״̬���ȴ���ת
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{// �ط�
					Upgrade_send_upgrade_start();
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_XMODEM:			// ����XMODEMģʽ
			UPgrade_xmodem_retry_timer_deal();
			break;
		
		case UPGRADE_STATUS_CRC_CEHCK:		// CRCУ��
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// �л�����ת״̬���ȴ���ת
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{// ���µȴ�
					Upgrade_set_retry_timer(10 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_REBOOT:			// ��ʱ����ʱ��ϵͳ����
			delay1ms(5*1000);
			NVIC_SystemReset();
			break;
		
		case UPGRADE_STATUS_JUMP_APP:		// ��ʱ�� ��ʱ����ת��APP
			Backlight_OFF();
			GoApp();
			break;
		
		default:
			break;
	}
}

void UPgrade_xmodem_recv_timer_deal(void)
{
	if (G_upgrade_sub.xmodem_status == XMODEM_STATUS_RECV_DATA)
	{// ���ݽ��ճ�ʱ��˵�����ݴ���Ҫ���ط�
		Upgrade_xmodem_ask_resend();
	}
}

void UPgrade_recv_timer_deal(void)
{
	if (G_upgrade_sub.status == UPGRADE_STATUS_XMODEM)
	{
		UPgrade_xmodem_recv_timer_deal();
	}
	else
	{// ��ʱ�൱��һ����ɣ�ֱ�Ӵ���
		Upgrade_message_deal();
	}
}

/*****************************************************************
Upgrade_loop_2ms:
	������������ѭ����2msÿ��
******************************************************************/
void Upgrade_loop_2ms(void)
{
	if (G_upgrade_sub.retry_timer)
	{
		G_upgrade_sub.retry_timer--;
		if (G_upgrade_sub.retry_timer == 0)
		{
			// ��ʱ
			UPgrade_retry_timer_deal();
		}
	}
	
	if (G_upgrade_sub.recv_timer)
	{
		G_upgrade_sub.recv_timer--;
		if (G_upgrade_sub.recv_timer == 0)
		{// ��ʱ
			UPgrade_recv_timer_deal();
		}
	}
}


unsigned char Upgrade_is_rebooting(void)
{
	if ((G_upgrade_sub.status == UPGRADE_STATUS_REBOOT) || 
		(G_upgrade_sub.status == UPGRADE_STATUS_JUMP_APP))
	{
		return 1;
	}
	return 0;
}


/*****************************************************************
Upgrade_init:
	���������ĳ�ʼ����������ȡ�����еı�־λ�����жϵ�ǰ��״̬
******************************************************************/
unsigned char Upgrade_init(void)
{
//	Upgrade_test_appflash();
	unsigned int baud_rate = 0;
	unsigned int upgrade_flag;
	
	upgrade_flag = Upgrade_get_upgrade_flag();
	baud_rate = Upgrade_get_baud_rate();
	
	// ��ʼ������
	G_upgrade_sub.status  = UPGRADE_STATUS_IDLE;
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_IDLE;
	
	// ��ȡ��־�����ò������������ò������ò�����
	if ((baud_rate == 0) || (baud_rate == 0xFFFFFFFF))
	{
		Uart0_init(DEFAULT_BAUD_RATE);
	}
	else
	{
		Uart0_init(baud_rate);
	}
	
	if (upgrade_flag == CMD_UPGRADE_START)
	{// �����������̣���������Ӧ��
		Backlight_ON();
		Upgrade_set_retry_timer(3 * TIMEOUT_1S);
		Upgrade_set_retry_count(10);
		Upgrade_set_recv_timer(0);
		Upgrade_send_upgrade_start();
		Upgrade_send_upgrade_start();
		G_upgrade_sub.status = UPGRADE_STATUS_UPGRADE_START;

	}
	else if (upgrade_flag == CMD_UPGRADE_RCVDATA)
	{// ���ݽ��ս׶������жϣ�����ֱ����תAPP
		Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
		Upgrade_set_retry_timer(TIMEOUT_1S);
		Upgrade_set_retry_count(1);
		Upgrade_set_recv_timer(0);
		G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
	}
	else if (upgrade_flag == CMD_UPGRADE_BURN)
	{ 		
		//���ݰ��ƽ׶����жϣ����¿������ݣ�����ֱ����תAPP
		//��������
		Upgrade_cope_flash_to_app();
		
		// ���������ݣ�׼������
		Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
		Upgrade_send_upgrade_result(0);
		Upgrade_set_retry_timer(TIMEOUT_1S);
		Upgrade_set_retry_count(1);
		G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
	}
	else if (upgrade_flag == CMD_UPGRADE_SUCCESS)
	{
		// ����ǿ������
		Upgrade_set_retry_timer(TIMEOUT_1S);
		Upgrade_set_retry_count(1);
		Upgrade_set_recv_timer(0);
		Upgrade_send_pow_on();
		G_upgrade_sub.status = UPGRADE_STATUS_POWON;
	}
	else
	{
		Upgrade_set_retry_timer(3 * TIMEOUT_1S);
		Upgrade_set_retry_count(10);
		Upgrade_set_recv_timer(0);
		Upgrade_send_upgrade_start();
		G_upgrade_sub.status = UPGRADE_STATUS_UPGRADE_START;
	}	
	return 0;
}

