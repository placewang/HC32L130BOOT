**HC32L130BOOT**  
**与HC32L130工程配合使用**  
**22.6/13--修改了读写Flash方法**  
**22.6/29--修改了升级标志FLASH读取地址**  
**22.7/1增加了看门狗；添加了升级数据包CRC过滤如下代码**  
**22.7/21升级开始后开背光**  
```c
	unsigned char  *Packend_CRCaddress;
	Packend_CRCaddress=(unsigned char *)(FLASH_APP_BACKUP_START_ADDRESS+len-6);
	if(Packend_CRCaddress[0]=='C'&&Packend_CRCaddress[1]=='R'&&Packend_CRCaddress[2]=='C'&&Packend_CRCaddress[3]==0x10)
	{
		len=len-6;
	}
```


