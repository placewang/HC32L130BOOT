#ifndef _UPGRADE_H_
#define _UPGRADE_H_

#define DEFAULT_BAUD_RATE				        115200//12500

#define MAX_FLASH_ADDRESS				        0x0000FFFF	// 内部FLASH总大小，64K
#define FLASH_BYTES_PER_SECTOR			    512			    // 每个扇区的FLASH大小
// FLASH 地址定义
#define FLASH_BOOT_START_ADDRESS		    0x00000000	// BOOT的起始地址，22个扇区
#define FLASH_BOOT_STOP_ADDRESS			    0x00002BFF	// BOOT的结束地址
#define FLASH_APP_START_ADDRESS			    0x00002C00	// APP的起始地址，52个扇区
#define FLASH_APP_STOP_ADDRESS			    0x000093FF	// APP的结束地址
#define FLASH_APP_BACKUP_START_ADDRESS	0x00009400	// APP备份的起始地址，52个扇区
#define FLASH_APP_BACKUP_STOP_ADDRESS	  0x0000FBFF	// APP备份的结束地址
#define FLASH_DATA_START_ADDRESS		    0x0000FC00	// 保存数据的起始地址，2个扇区
#define FLASH_DATA_STOP_ADDRESS			    0x0000FFFF	// 保存数据的结束地址

// 来自APP的配置信息
#define CONFIG_BAUD_RATE_ADDRESS		    0x0000FC64	// 波特率配置数据
#define UPGRADE_FLAG_ADDRESS			      0x0000FE00	// 升级标志，boot需要修改

struct CONFIG_BAUD
{
	unsigned short data_len;			// 数据长度
	unsigned int baud_rate;				// 序号
	unsigned short crc_check;			// CRC16校验
}__attribute__((packed));



// FLASH 升级标志
#define CMD_UPGRADE_START				  0xA050AA00	// 启动升级的标志
#define CMD_UPGRADE_RCVDATA				0xA050AA55	// 接收升级数据标志
#define CMD_UPGRADE_BURN				  0xA0505A5A	// 搬运数据标志
#define CMD_UPGRADE_SUCCESS				0xA05055AA	// 升级完成标志

// Xmodem的控制字符定义
#define XMODEM_CMD_SOH			0x01		// 数据，回复ACK表示发送下一个包，回复NAK表示重发，回复CAN表示停止传输
#define XMODEM_CMD_EOT			0x04		// 数据传输接收，回复ACK
#define XMODEM_CMD_ACK			0x06		// 回复
#define XMODEM_CMD_NAK			0x15		// 回复，重传当前包
#define XMODEM_CMD_CAN			0x18		// 接收方，要求停止传输

// 定时器
#define TIMEOUT_1S		500
#define TIMEOUT_100ms	50
#define RETRY_COUNT		10

unsigned char Upgrade_init(void);
void Upgrade_data_deal(unsigned char data);
void Upgrade_loop_2ms(void);
unsigned char Upgrade_is_rebooting(void);

#endif

