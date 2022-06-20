#ifndef _UPGRADE_H_
#define _UPGRADE_H_

#define DEFAULT_BAUD_RATE				        115200//12500

#define MAX_FLASH_ADDRESS				        0x0000FFFF	// �ڲ�FLASH�ܴ�С��64K
#define FLASH_BYTES_PER_SECTOR			    512			    // ÿ��������FLASH��С
// FLASH ��ַ����
#define FLASH_BOOT_START_ADDRESS		    0x00000000	// BOOT����ʼ��ַ��22������
#define FLASH_BOOT_STOP_ADDRESS			    0x00002BFF	// BOOT�Ľ�����ַ
#define FLASH_APP_START_ADDRESS			    0x00002C00	// APP����ʼ��ַ��52������
#define FLASH_APP_STOP_ADDRESS			    0x000093FF	// APP�Ľ�����ַ
#define FLASH_APP_BACKUP_START_ADDRESS	0x00009400	// APP���ݵ���ʼ��ַ��52������
#define FLASH_APP_BACKUP_STOP_ADDRESS	  0x0000FBFF	// APP���ݵĽ�����ַ
#define FLASH_DATA_START_ADDRESS		    0x0000FC00	// �������ݵ���ʼ��ַ��2������
#define FLASH_DATA_STOP_ADDRESS			    0x0000FFFF	// �������ݵĽ�����ַ

// ����APP��������Ϣ
#define CONFIG_BAUD_RATE_ADDRESS		    0x0000FC64	// ��������������
#define UPGRADE_FLAG_ADDRESS			      0x0000FE00	// ������־��boot��Ҫ�޸�

struct CONFIG_BAUD
{
	unsigned short data_len;			// ���ݳ���
	unsigned int baud_rate;				// ���
	unsigned short crc_check;			// CRC16У��
}__attribute__((packed));



// FLASH ������־
#define CMD_UPGRADE_START				  0xA050AA00	// ���������ı�־
#define CMD_UPGRADE_RCVDATA				0xA050AA55	// �����������ݱ�־
#define CMD_UPGRADE_BURN				  0xA0505A5A	// �������ݱ�־
#define CMD_UPGRADE_SUCCESS				0xA05055AA	// ������ɱ�־

// Xmodem�Ŀ����ַ�����
#define XMODEM_CMD_SOH			0x01		// ���ݣ��ظ�ACK��ʾ������һ�������ظ�NAK��ʾ�ط����ظ�CAN��ʾֹͣ����
#define XMODEM_CMD_EOT			0x04		// ���ݴ�����գ��ظ�ACK
#define XMODEM_CMD_ACK			0x06		// �ظ�
#define XMODEM_CMD_NAK			0x15		// �ظ����ش���ǰ��
#define XMODEM_CMD_CAN			0x18		// ���շ���Ҫ��ֹͣ����

// ��ʱ��
#define TIMEOUT_1S		500
#define TIMEOUT_100ms	50
#define RETRY_COUNT		10

unsigned char Upgrade_init(void);
void Upgrade_data_deal(unsigned char data);
void Upgrade_loop_2ms(void);
unsigned char Upgrade_is_rebooting(void);

#endif

