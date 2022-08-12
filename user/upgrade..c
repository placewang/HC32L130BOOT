/********************************************************************************
Xmodem协议
一、帧格式
    | SOH | 信息包序号 | 信息包序号的反码 | 数据区段 | 算术校验和 |
    |_____|____________|__________________|__________|____________|
　　说明：
　　SOH 帧的开头字节，代表信息包中的第一个字节
　　信息包序号： 对 256 取模所得到当前包号，第一个信息包的序号为 1
　　而信息包序号范围0~255
　　信息包序号的反码： 当前信息包号的反码
　　数据区段： 数据区段的长度固定为 128 字节，其内容没有任何限制，可以是文本数据或二进制数据
　　算术校验和： 1字节的算术校验和，只对数据区段计算后对 256 取模而得

二、传输逻辑
　　1> 收发双方拨号连通后，发送方等待接收方传来 NAK 信号。当第一个 NAK 到达，发送方解释为 开始发送第一个包
　　2> 发送方一旦收到第一个 NAK ，启动了传输，发送方就将数据以每次 128 字节打包成帧格式传送，再等待接收方的确认信号
　　3> 发送方收到接收方传来的 ACK 信号，解释为信息包被正确接收，并有发送下一个包的含义
　　4> 发送方收到接收方传来的 NAK 信号，解释为请求重发同一数据包
　　5> 发送方收到接收方传来的 CAN 信号，解释为请求无条件停止传输过程
　　6> 发送方正常传输完全部数据，需要正常结束，发送 EOT 信号通知接收方。接收方用 ACK 进行确认
　　7> 接收方发送 CAN 无条件停止传输过程，发送方收到 CAN 后，不发送 EOT 确认
　　8> 虽然信息包是以 SOH 来标志一个信息包的起始的，但在 SOH 位置上出现的 EOT则表示数据传输结束，再也没有数据传过来
　　9> 接收方首先应确认信息包序号的完整性，通过对信息包序号取补，然后和信息包序号的补码异或，结果为 0 表示正确，结果不为 0 则发送 NAK 请求重传
　　10> 接收方确认信息包序号正确后，然后检查是否期望的序号。如果不是期望得到的信息包序号，说明发生严重错误，应该发送一个 CAN 来中止传输
　　11> 对于10>情况的唯一例外，是收到的包的信息包序号与前一个信息包序号相同，此中情况，接收方简单忽略这个重复的包，向发送方发出 ACK ，准备接收下一个包
　　12> 接收方确认了信息包序号的完整性和是正确期望的后，只对 128字节的数据区段进行算术和校验，结果与帧中最后一个字节（算术校验和）比较，相同 发送 ACK！不同发送 NAK

三、超时处理
　　1> 接收方等待一个信息包的到来所具有的超时时限为 10 秒，每个超时后发送 NAK
　　2> 当收到包时，接收过程中每个字符的超时间隔为 1 秒
　　3> 为保持“接收方驱动”，发送方在等待一个启动字节时不应该采用超时处理
　　4> 一旦传输开始，发送方采用单独的 1 分钟超时时限，给接收方充足的时间做发送ACK ,NAK ,CAN 之前的必须处理
　　5> 所有的超时及错误事件至少重试 10 次

四、控制字符
　　控制字符符合 ASICII 标准定义，长度均为 1 字节
　　SOH 0x01
　　EOT 0x04
　　ACK 0x06
　　NAK 0x15
　　CAN 0x18
————————————————
*********************************************************************************/


#include "bsp_uart.h"
#include "upgrade.h"
#include "flash.h"
#include "gpio.h"
// 升级状态定义
#define UPGRADE_STATUS_IDLE				      0x00	// 空状态
#define UPGRADE_STATUS_POWON			      0x01	// 上电完成状态，发送上电完成，等待回复，进入强制升级；超时则进入APP
#define UPGRADE_STATUS_UPGRADE_START	  0x02	// 发送升级消息，等待回应
#define UPGRADE_STATUS_XMODEM			      0x03	// 进入XMODEM模式
#define UPGRADE_STATUS_CRC_CEHCK		    0x04	// CRC校验
#define UPGRADE_STATUS_COPY_DATA		    0x05	// 拷贝数据状态
#define UPGRADE_STATUS_REBOOT			      0x06	// 定时器超时后，系统重启
#define UPGRADE_STATUS_JUMP_APP			    0x07	// 定时器 超时后，跳转到APP

//Xmodem传输的状态机，目前只支持标准的协议，还不支持扩展协议
//接收数据，保存到FLASH_APP_BACKUP_START_ADDRESS的地址中
#define XMODEM_STATUS_IDLE				      0x00	// 空状态
#define XMODEM_STATUS_NAK_WAIT_DATA		  0x01	// 发送NAK等待数据状态
#define XMODEM_STATUS_ACK_WAIT_DATA		  0x02	// 发送ACK等待数据状态
#define XMODEM_STATUS_RECV_DATA			    0x03	// 接收数据包状态，1s超时为收包结束
#define XMODEM_STATUS_CANCEL			      0x04	// 取消传输
#define XMODEM_STATUS_SUCCESS			      0x05	// 传输完成


unsigned int G_flash_data[128];	              //保存的当前需要写到的FLASH中的数据

struct UPGRADE_SUB
{
	unsigned char status;				       // 当前状态
	unsigned char retry_count;			   // 剩余的重试次数，每次减
	unsigned short retry_timer;			   // 重试计时器，每次减
	unsigned short recv_timer;			   // 数据接收计时器，超时后认为一包结束，每次减
	
	unsigned char rx_buf[256];			   // 接收到的包
	unsigned char rx_buf_len;			     // 当前的接收长度
	
	// xmodem协议相关
	unsigned char  xmodem_status;		   // xmodem传输的状态机
	unsigned int   flash_address;			 // 当前操作的flash地址
	unsigned int   flash_max_address;	 // 当前操作的最大flash地址
	unsigned int   *flash_data_p;			 // 当前收到的flash数据，每一个扇区512字节
	unsigned short flash_data_len;	   // 当前收到的flash数据长度
	unsigned int   recv_all_data_len;	 // 收到的数据总长度
	unsigned char  next_sq;				     // 下一个包的序号
}__attribute__((packed));

struct UPGRADE_SUB G_upgrade_sub;

struct XMODEM_MSG_SOH
{
	unsigned char cmd;					      // 当前状态
	unsigned char sequence;				    // 序号
	unsigned char sequence_inverse;		// 序号的反码
	unsigned char data[128];			    // 接收到的数据
	unsigned char check;				      // 校验和
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
	读取波特率的值
	返回值：波特率数值
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
	获取当前的升级标志位
******************************************************************/
unsigned int Upgrade_get_upgrade_flag(void)
{
	unsigned int *upgrade_flag_p;
	upgrade_flag_p = (unsigned int *)UPGRADE_FLAG_ADDRESS;
	return *upgrade_flag_p;
}

/*****************************************************************
Upgrade_get_baud_rate:
	读取波特率的值
	返回值：波特率数值
******************************************************************/
void Upgrade_update_upgrade_flag(unsigned int flag)
{
	// 扇区开始，先擦除
	Upgrade_sector_erase_carefull(UPGRADE_FLAG_ADDRESS);
	// 写标志位
	Upgrade_wirte_word_carefull(UPGRADE_FLAG_ADDRESS, flag);
}

/*****************************************************************
Upgrade_send_pow_on:
	发送上电消息
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
	发送升级开始消息
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
	发送升级开始消息
	result：0：升级成功；1：升级失败
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
	在线升级中，设置重试计数器
	count：每次2ms
******************************************************************/
void Upgrade_set_retry_timer(unsigned short count)
{
	G_upgrade_sub.retry_timer = count;
}

/*****************************************************************
Upgrade_set_timeout_count:
	在线升级中，设置重试计数器
	count：超时次数
******************************************************************/
void Upgrade_set_retry_count(unsigned short count)
{
	G_upgrade_sub.retry_count = count;
}

/*****************************************************************
Upgrade_set_recv_timer:
	在线升级中，设置数据接收计时器
	count：超时次数
******************************************************************/
void Upgrade_set_recv_timer(unsigned short count)
{
	G_upgrade_sub.recv_timer = count;
}

/*****************************************************************
Upgrade_xmodem_send_cmd:
	在线升级中，发送xmodem协议的控制符
	cmd：控制符
	#define XMODEM_CMD_EOT			0x04		// 数据传输接收，回复ACK
	#define XMODEM_CMD_ACK			0x06		// 回复
	#define XMODEM_CMD_NAK			0x15		// 回复，重传当前包
	#define XMODEM_CMD_CAN			0x18		// 接收方，要求停止传输
******************************************************************/
void Upgrade_xmodem_send_cmd(unsigned char cmd)
{
	Uart0_send_btye(cmd);
}

/*****************************************************************
Upgrade_sector_write:
	在线升级中，内部FLASH的扇区写操作
	address：写的地址
	data_p：写的数据，512个字节
******************************************************************/
void Upgrade_sector_write(unsigned int address, unsigned int *data_p)
{
	unsigned int i;
	
	// 扇区开始，先擦除
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
	在线升级中，内部FLASH的扇区写操作
	address：写的地址
	data_p：写的数据，512个字节
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
	在线升级中，内部FLASH的扇区写操作
	address：写的地址
	data_p：写的数据，512个字节
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
		// 写数据
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
	Upgrade_set_retry_timer(TIMEOUT_1S);	// 1秒后退出
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
	在线升级启动xmodem传输
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
	在线升级中 xmodem的soh消息计算校验和
	msg_soh_p：需要校验的消息
	返回值：0：校验成功；非0：校验失败
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
	在线升级的接收的数据的flash操作
	data_p:本次收到的128个数据的指针
******************************************************************/
void Upgrade_flash_data_deal(unsigned char *data_p)
{
	if (data_p == NULL)
	{
		return;
	}
	
	if (G_upgrade_sub.flash_address < G_upgrade_sub.flash_max_address)
	{// 目前的操作地址还在地址范围内
		memcpy((unsigned char *)G_upgrade_sub.flash_data_p + G_upgrade_sub.flash_data_len, data_p, 128);
		G_upgrade_sub.flash_data_len += 128;
		
		if (G_upgrade_sub.flash_data_len >= 512)
		{// 写到flash中
			Upgrade_sector_write(G_upgrade_sub.flash_address, G_upgrade_sub.flash_data_p);
			memset((unsigned char *)G_upgrade_sub.flash_data_p, 0xFF, 512);
			G_upgrade_sub.flash_address += 512;
			G_upgrade_sub.flash_data_len = 0;
		}
	}
}

/*****************************************************************
Upgrade_flash_end_deal:
	在线升级的接收的数据的flash操作
******************************************************************/
void Upgrade_flash_end_deal(void)
{
	if (G_upgrade_sub.flash_address < G_upgrade_sub.flash_max_address)
	{// 目前的操作地址还在地址范围内
		if (G_upgrade_sub.flash_data_len > 0)
		{
			G_upgrade_sub.flash_data_len = 512;
		}
		
		if (G_upgrade_sub.flash_data_len >= 512)
		{// 写到flash中
			Upgrade_sector_write(G_upgrade_sub.flash_address, G_upgrade_sub.flash_data_p);
			memset((unsigned char *)G_upgrade_sub.flash_data_p, 0xFF, 512);
			G_upgrade_sub.flash_address += 512;
			G_upgrade_sub.flash_data_len = 0;
		}
	}
}

/*****************************************************************
Upgrade_xmodem_message_deal:
	在线升级的xmodem模式接收的消息处理
******************************************************************/
void Upgrade_xmodem_message_deal(void)
{
	struct XMODEM_MSG_SOH *msg_soh_p;
	
	// 长度检查
	if (G_upgrade_sub.rx_buf_len != sizeof(struct XMODEM_MSG_SOH))
	{
		Upgrade_xmodem_ask_resend();
		return;
	}
	
	msg_soh_p = (struct XMODEM_MSG_SOH *)G_upgrade_sub.rx_buf;
	
	// 序号完整性校验
	if ((msg_soh_p->sequence ^ msg_soh_p->sequence_inverse) != 0xFF)
	{// 
		Upgrade_xmodem_ask_resend();
		return;
	}
	
	// 序号正确性校验
	if (msg_soh_p->sequence != G_upgrade_sub.next_sq)
	{// 不是期待的序号
		if ((msg_soh_p->sequence + 1) == G_upgrade_sub.next_sq)
		{// 上一包
			Upgrade_xmodem_ask_next();
			return;
		}
		else
		{// 序号错误，取消发送
			Upgrade_xmodem_cannel();
			return;
		}
	}

	// 数据校验和
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
	在线升级接收的数据消息处理
******************************************************************/
void Upgrade_message_deal(void)
{
	unsigned short file_bytes;
	unsigned short recv_check;
	unsigned short my_check;
	
	switch (G_upgrade_sub.status)
	{
		case UPGRADE_STATUS_IDLE:	// 空状态
			break;
		
		case UPGRADE_STATUS_POWON:	// 上电完成状态，发送上电完成，等待回复，进入强制升级；超时则进入APP
			if (G_upgrade_sub.rx_buf_len >= 4)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x52) &&
					(G_upgrade_sub.rx_buf[3] == 0xA5))
				{// 收到回应，发送升级开始
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
					Upgrade_set_retry_count(10);
					Upgrade_send_upgrade_start();
					G_upgrade_sub.status = UPGRADE_STATUS_UPGRADE_START;
				}
			}
			break;
		
		case UPGRADE_STATUS_UPGRADE_START:	// 发送升级消息，等待回应
			Backlight_ON();
			if (G_upgrade_sub.rx_buf_len >= 4)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x54) &&
					(G_upgrade_sub.rx_buf[3] == 0xA5))
				{// 收到回应，开始升级
					Upgrade_update_upgrade_flag(CMD_UPGRADE_RCVDATA);
					Upgrade_start_xmodem();
					G_upgrade_sub.status = UPGRADE_STATUS_XMODEM;
				}
			}
			break;
		
		case UPGRADE_STATUS_XMODEM:			// 进入XMODEM模式
			Backlight_ON();
			break;
		
		case UPGRADE_STATUS_CRC_CEHCK:		// CRC校验
			if (G_upgrade_sub.rx_buf_len >= 8)
			{
				if ((G_upgrade_sub.rx_buf[0] == 0x5A) &&
					(G_upgrade_sub.rx_buf[1] == 0x04) &&
					(G_upgrade_sub.rx_buf[2] == 0x78) &&
					(G_upgrade_sub.rx_buf[7] == 0xA5))
				{// 收到回应，开始升级
//					if ((G_upgrade_sub.rx_buf[3] == 0x11) &&
//						(G_upgrade_sub.rx_buf[4] == 0x22) &&
//						(G_upgrade_sub.rx_buf[5] == 0x33) &&
//						(G_upgrade_sub.rx_buf[6] == 0x44))
//					{
//						// 设置标志位，开始拷贝
//						Upgrade_update_upgrade_flag(CMD_UPGRADE_BURN);
//						// 拷贝数据
//						Upgrade_cope_flash_to_app();
//						
//						// 拷贝完数据，准备重启
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
					{ // 校验失败
						Upgrade_send_upgrade_result(1);
						Upgrade_set_retry_timer(TIMEOUT_1S);
						Upgrade_set_retry_count(1);
						G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
					}
					else
					{// 校验成功
						// 设置标志位，开始拷贝
						Upgrade_update_upgrade_flag(CMD_UPGRADE_BURN);
						// 拷贝数据
						Upgrade_cope_flash_to_app();
						
						// 拷贝完数据，准备重启
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
		
		case UPGRADE_STATUS_REBOOT:			// 定时器超时后，系统重启
			break;
		
		case UPGRADE_STATUS_JUMP_APP:		// 定时器 超时后，跳转到APP
			break;
		
		default:
			break;
	}
	
	// 处理完后，长度清零
	G_upgrade_sub.rx_buf_len = 0;
	// 定时器清零
	Upgrade_set_recv_timer(0);
	return;
}

/*****************************************************************
Upgrade_data_deal:
	在线升级的接收的数据处理
******************************************************************/
void Upgrade_xmodem_data_deal(unsigned char data)
{
	switch (G_upgrade_sub.xmodem_status)
	{
		case XMODEM_STATUS_IDLE:
			break;
		
		case XMODEM_STATUS_NAK_WAIT_DATA:
			if ((data == XMODEM_CMD_EOT) || (data == XMODEM_CMD_CAN))
			{// 错误，直接重启
				Upgrade_set_retry_timer(TIMEOUT_1S);
				Upgrade_set_retry_count(1);
				Upgrade_set_recv_timer(0);
				G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
			}
			else if (data == XMODEM_CMD_SOH)
			{// 收到数据包命令
				G_upgrade_sub.rx_buf[0] = data;
				G_upgrade_sub.rx_buf_len = 1;
				Upgrade_xmodem_recv_data();
			}
			break;
			
		case XMODEM_STATUS_ACK_WAIT_DATA:
			if (data == XMODEM_CMD_EOT)
			{// 数据传输结束
				Upgrade_flash_end_deal();
				Upgrade_xmodem_send_cmd(XMODEM_CMD_ACK);
				
				Upgrade_set_retry_timer(TIMEOUT_1S);
				Upgrade_set_retry_count(1);
				Upgrade_set_recv_timer(0);
				G_upgrade_sub.xmodem_status = XMODEM_STATUS_SUCCESS;
			}
			else if (data == XMODEM_CMD_CAN)
			{// 对方取消，直接重启
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
			
			// 接收完成
			Upgrade_xmodem_message_deal();
			break;
		
		default:
			break;
	}
}

/*****************************************************************
Upgrade_data_deal:
	在线升级的接收的数据处理
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
		case XMODEM_STATUS_IDLE:				// 空状态
			break;
		
		case XMODEM_STATUS_NAK_WAIT_DATA:		// 发送NAK等待数据状态
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// 切换到跳转状态，等待跳转
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					Upgrade_set_recv_timer(0);
					G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
				}
				else
				{// 重发
					Upgrade_xmodem_send_cmd(XMODEM_CMD_NAK);
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case XMODEM_STATUS_ACK_WAIT_DATA:		// 发送NAK等待数据状态
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// 切换到跳转状态，等待跳转
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					Upgrade_set_recv_timer(0);
					G_upgrade_sub.xmodem_status = XMODEM_STATUS_CANCEL;
				}
				else
				{// 重发
					Upgrade_xmodem_send_cmd(XMODEM_CMD_ACK);
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case XMODEM_STATUS_RECV_DATA:			// 接收数据包状态，1s超时为收包结束
			break;
			
		case XMODEM_STATUS_CANCEL:				// 取消传输
			// 传输失败，退回去.
			Upgrade_set_retry_timer(TIMEOUT_1S);
			Upgrade_set_retry_count(1);
			G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
			break;
		
		case XMODEM_STATUS_SUCCESS:
			// 传输成功，进入等待CRC校验状态
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
		case UPGRADE_STATUS_IDLE:	// 空状态
			break;
		
		case UPGRADE_STATUS_POWON:	// 上电完成状态，发送上电完成，等待回复，进入强制升级；超时则进入APP
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{ // 切换到跳转状态，等待跳转
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{ // 重发
					Upgrade_send_pow_on();
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_UPGRADE_START:	// 发送升级消息，等待回应
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// 切换到跳转状态，等待跳转
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{// 重发
					Upgrade_send_upgrade_start();
					Upgrade_set_retry_timer(3 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_XMODEM:			// 进入XMODEM模式
			UPgrade_xmodem_retry_timer_deal();
			break;
		
		case UPGRADE_STATUS_CRC_CEHCK:		// CRC校验
			if (G_upgrade_sub.retry_count)
			{
				G_upgrade_sub.retry_count--;
				if (G_upgrade_sub.retry_count == 0)
				{// 切换到跳转状态，等待跳转
					Upgrade_set_retry_timer(TIMEOUT_1S);
					Upgrade_set_retry_count(1);
					G_upgrade_sub.status = UPGRADE_STATUS_JUMP_APP;
				}
				else
				{// 重新等待
					Upgrade_set_retry_timer(10 * TIMEOUT_1S);
				}
			}
			break;
		
		case UPGRADE_STATUS_REBOOT:			// 定时器超时后，系统重启
			delay1ms(5*1000);
			NVIC_SystemReset();
			break;
		
		case UPGRADE_STATUS_JUMP_APP:		// 定时器 超时后，跳转到APP
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
	{// 数据接收超时，说明数据错误，要求重发
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
	{// 超时相当于一包完成，直接处理
		Upgrade_message_deal();
	}
}

/*****************************************************************
Upgrade_loop_2ms:
	在线升级的主循环，2ms每次
******************************************************************/
void Upgrade_loop_2ms(void)
{
	if (G_upgrade_sub.retry_timer)
	{
		G_upgrade_sub.retry_timer--;
		if (G_upgrade_sub.retry_timer == 0)
		{
			// 超时
			UPgrade_retry_timer_deal();
		}
	}
	
	if (G_upgrade_sub.recv_timer)
	{
		G_upgrade_sub.recv_timer--;
		if (G_upgrade_sub.recv_timer == 0)
		{// 超时
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
	在线升级的初始化函数，读取数据中的标志位，来判断当前的状态
******************************************************************/
unsigned char Upgrade_init(void)
{
//	Upgrade_test_appflash();
	unsigned int baud_rate = 0;
	unsigned int upgrade_flag;
	
	upgrade_flag = Upgrade_get_upgrade_flag();
	baud_rate = Upgrade_get_baud_rate();
	
	// 初始化数据
	G_upgrade_sub.status  = UPGRADE_STATUS_IDLE;
	G_upgrade_sub.xmodem_status = XMODEM_STATUS_IDLE;
	
	// 读取标志和配置参数，根据配置参数设置波特率
	if ((baud_rate == 0) || (baud_rate == 0xFFFFFFFF))
	{
		Uart0_init(DEFAULT_BAUD_RATE);
	}
	else
	{
		Uart0_init(baud_rate);
	}
	
	if (upgrade_flag == CMD_UPGRADE_START)
	{// 启动升级流程，发送升级应答
		Backlight_ON();
		Upgrade_set_retry_timer(3 * TIMEOUT_1S);
		Upgrade_set_retry_count(10);
		Upgrade_set_recv_timer(0);
		Upgrade_send_upgrade_start();
		Upgrade_send_upgrade_start();
		G_upgrade_sub.status = UPGRADE_STATUS_UPGRADE_START;

	}
	else if (upgrade_flag == CMD_UPGRADE_RCVDATA)
	{// 数据接收阶段意外中断，可以直接跳转APP
		Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
		Upgrade_set_retry_timer(TIMEOUT_1S);
		Upgrade_set_retry_count(1);
		Upgrade_set_recv_timer(0);
		G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
	}
	else if (upgrade_flag == CMD_UPGRADE_BURN)
	{ 		
		//数据搬移阶段外中断，重新拷贝数据，可以直接跳转APP
		//拷贝数据
		Upgrade_cope_flash_to_app();
		
		// 拷贝完数据，准备重启
		Upgrade_update_upgrade_flag(CMD_UPGRADE_SUCCESS);
		Upgrade_send_upgrade_result(0);
		Upgrade_set_retry_timer(TIMEOUT_1S);
		Upgrade_set_retry_count(1);
		G_upgrade_sub.status = UPGRADE_STATUS_REBOOT;
	}
	else if (upgrade_flag == CMD_UPGRADE_SUCCESS)
	{
		// 进入强制升级
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

