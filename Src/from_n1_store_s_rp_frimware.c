#include "stm32f4xx_hal.h"
#include "string.h"
#include "ap_param.h"
#include "to_n1.h"
#include "rf_hal.h"
#include "eeprom.h"
#include "flash.h"
#include "from_n1.h"
#include "typedef_struct.h"

//标定当前下发的是那个固件
#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02


#define FIRM_HEAD_ERROR 0XFFFE
#define FIRM_LINES_ERROR 0XFFFD


extern uint8_t ap_param_write_flash_flag ;

extern struct_systerm_info systerm_info;  
char firm_debug_buff[12];
extern void copy_string_to_double_buff(char *pstr);

#pragma pack(1)

typedef struct _FW_HEADER_t_5xx
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[64u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
} FW_HEADER_t_5xx;

typedef struct _FW_HEADER_t_5xxRP
{
 uint32_t  ulId;    // 0x00 - ??'Header'??.
  #define FW_HEADER_ID   0xDADA10AFul
 uint32_t  ulCheckSum;   // 0x04 - ???, ??'update.c'??'UfCalcCheckSum'.
 uint16_t  uiSize;    // 0x08 - sizeof( FW_HEADER_t ).
 uint16_t  uiFwSize;   // 0x0A - ???????.
 uint16_t  uiBuildNr;   // 0x0C - ??? ????'Build'??.
 uint16_t  auiVector[16u];  // 0x0E - ???????????.
 //u16_t  uiFwAddr;   // 0x2E - ???????.
 uint32_t  uiFwAddr;   // 0x2E - ???????.
 uint16_t  uiHwVer;   // 0x30 - ???????????.??????
 uint16_t  ucCfgComb;   // 0x32 - ??? configuration combination.
         //   ??????????:
         //   0x03: ???.
         //   0x07: ???.
         //   0x08: ????'Stop Bar'??.
 uint16_t  uiFwVer;   // 0x34 - ????.
} FW_HEADER_t_5xxRP;
#pragma pack()


//解析出来的固件头 存在flash里 便于检索使用
struct _my_firmware_head{
	unsigned char firmware_type;
	unsigned short fw_ver;   //固件版本
	unsigned short hw_ver;   //硬件版本
	unsigned short packet_num;  //固件长度，单位：行
}my_firmware_head;




typedef struct _n1_send_firmware_head
{
	uint16_t packet_count;
	uint16_t packet_syn;
	uint8_t data[1];
} struct_n1_send_firmware_head;






unsigned char firmware_head[38*5];       //保存固件头 用来查看固件版本 rp sensor
unsigned char firmware_struct_head[32*5];  //用来解析前4包数据，是一个结构体，存储了固件的一些信息




uint16_t ee_write_packet_syn = 0;
uint32_t ee_write_addr_add = 0;

void read_firmware_rp_head();
void read_firmware_sensor_head();

FW_HEADER_t_5xxRP pheader_5xxRP;
FW_HEADER_t_5xx pheader_5xx;



//将十六进制字符串转化为二进制数。
void strhex_to_hex(unsigned char *output,char *input,int len)
{
	char data[3];
		int i;
	data[2] = 0;

	
	for(i=0;i<len;i++)
	{
		data[0] = *input++;
		data[1] = *input++;
		sscanf(data,"%X",output++);
	}
}



/**
 * Func:  校验固件。按照每行的和为0来进行校验
 * 
 * Param: 固件存放的地址。
 *
 * Return：0：校准成功  -1：校准失败
 */
int verify_firmware(int addr)
{
	struct _my_firmware_head * phead = (struct _my_firmware_head * )addr;
	int i,j;
	unsigned char *pdata;
	unsigned char check;
	
	for(i=0;i<phead->packet_num;i++)
	{
		
		check = 0;
		pdata = (unsigned char *)(addr+sizeof(struct _my_firmware_head)+i*38);
		for(j=0;j<37;j++)
		{
			check +=*pdata++;
		}
		if(check!=0)
			return -1;
	}
	return 0;

}


void check_firmware_header()
{
	unsigned short *ptr = (unsigned short *)firmware_struct_head;
	int i = 0;
	volatile int sum = 0;
	FW_HEADER_t_5xx *sptr = (FW_HEADER_t_5xx *)firmware_struct_head;
	
	
	for(i=0;i<sptr->uiSize/2;i++)
	{
		sum += *ptr++;
	}

	sum -= sptr->ulCheckSum & 0xffff;
	sum -= sptr->ulCheckSum >> 16;

}



/*

	func : 从N1接收检测器和rp的固件程序，分别存储在AP1不同的flash地址处。







*/
void receive_rp_sensor_firmware(struct_ap_n1_protocol *ptr,int who)
{

	unsigned char ack[7];                //给N1发送ack
	int i;
	unsigned char check = 0;             //校验数据包
	static int firmware_type = 0;        //固件类型决定存储位置（rp or sensor）
//	static int packet_count = 0;         //包计数 用来指示存储到第几包了。
	static int check_error = 0;
	unsigned char cmd = 0;
	FW_HEADER_t_5xx *pheader_5xx;
	FW_HEADER_t_5xxRP *pheader_5xxRP;
	
	struct _my_firmware_head  *flash_head; 
	uint16_t updata_FwVer;
	uint16_t updata_HwVer;
	static uint32_t rtc_time1 = 0;
	struct_n1_send_firmware_head *p_n1_send_firmware_head;
		
	p_n1_send_firmware_head = (struct_n1_send_firmware_head *)&ptr->data[0];

	uint32_t begin_address = 0;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;	
	
	
//#ifdef _DEBUG
//	//if(cmd_firmware_data->sequence==0xffff)return;
//	sprintf(debug_,"receive packet_num=%d   error=%d \r",cmd_firmware_data->sequence,check_error);
//	debug(debug_);
//#endif

	if(who == N1_SEND_RP_FIRMWARE)   //rp固件
	{

		begin_address = FLASH_RP_FIRMWARE_BEGIN;
		EraseInit.Sector = FLASH_SECTOR_17;
		cmd = AP_SEND_RP_FIRMWARE;
	}
	else if(who == N1_SEND_S_FIRMWARE)   //sensor固件
	{

		begin_address = FLASH_SENSOR_FIRMWARE_BEGIN;	
		EraseInit.Sector = FLASH_SECTOR_18;		
		cmd = AP_SEND_S_FIRMWARE;
	}



	if(p_n1_send_firmware_head->data[0] != 'l' && p_n1_send_firmware_head->packet_syn != 0xffff)   //每一行都是l开头的 除了最后的结尾行。。。
	{	
		i = i;
		return;
	}
	
	
	if(ee_write_packet_syn != p_n1_send_firmware_head->packet_syn)  //假如N1发来的不是我需要的包序号 向N1索要需要的包序号
	{
		  if(systerm_info.slot - rtc_time1 > 50) //相对于上一次发送 大于100ms 才会继续发送
			{
				rtc_time1 = systerm_info.slot;
				if(p_n1_send_firmware_head->packet_syn == 0) //假如N1还是发送第0包  认为N1主动重新开始发送
				{
					ee_write_packet_syn = 0;
					ee_write_addr_add = 0;
					return;
				}
				p_n1_send_firmware_head->packet_syn =ee_write_packet_syn;
				insert_to_n1_buff(&ptr->data[0],4,cmd);					
			}
			return;
	}
	
	
	memcpy(firm_debug_buff,"firm:",5);
	memcpy(&firm_debug_buff[5],&p_n1_send_firmware_head->data[1],4);
	firm_debug_buff[9] = '\r';
	firm_debug_buff[10] = '\n';
	firm_debug_buff[11] = 0;
	copy_string_to_double_buff(firm_debug_buff);
	
	if(p_n1_send_firmware_head->packet_syn<5)   //前4包是固件头 用来判别是rp或者是sensor（149 or 5系列）
	{
		strhex_to_hex(firmware_head + p_n1_send_firmware_head->packet_syn*37,&(p_n1_send_firmware_head->data[1]),37);
		for(i=0;i<32+5;i++)
		{
			check += firmware_head[i+p_n1_send_firmware_head->packet_syn*37];
		}
		if(check !=0)               //error  行校验没有通过
		{	
			check_error++;		
			insert_to_n1_buff(&ptr->data[0],4,cmd);
			return;                          //校验失败，发送上一包包序号，这样N1就会重发这一包。
		}
		
		if(p_n1_send_firmware_head->packet_syn == 0)  //第0包 擦除对应的flash
		{
			ee_write_addr_add = 0;
			//__disable_irq() ;  //关总中断
			HAL_FLASH_Unlock();
			EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
			EraseInit.Banks = FLASH_BANK_2;
			EraseInit.NbSectors = 1;		
			EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
			HAL_FLASHEx_Erase(&EraseInit,&SectorError);
			HAL_FLASH_Lock();
			if(who == N1_SEND_RP_FIRMWARE)   //rp固件
			{
				sys_flash_param.ap_param.rp_version = FIRM_HEAD_ERROR;
			}
				else if(who == N1_SEND_S_FIRMWARE)   //sensor固件
			{
				sys_flash_param.ap_param.sensor_version = FIRM_HEAD_ERROR;	
			}
			//__enable_irq() ; //开总中断
		}

		if(0 == write_bin_flash(begin_address + ee_write_addr_add,firmware_head + p_n1_send_firmware_head->packet_syn*37,37)) //写flash成功
		{
				ee_write_addr_add += 37;
				ee_write_packet_syn++;
				p_n1_send_firmware_head->packet_syn +=1;
				insert_to_n1_buff(&ptr->data[0],4,cmd);	

		}
		else
		{
				p_n1_send_firmware_head->packet_syn = 0;
				insert_to_n1_buff(&ptr->data[0],4,cmd);			
				return;
		}		
		
		if(p_n1_send_firmware_head->packet_syn==5)
		{
			memcpy(firmware_struct_head,firmware_head+5,32);
			memcpy(firmware_struct_head+32,firmware_head+5+37,32);
			memcpy(firmware_struct_head+32*2,firmware_head+5+37*2,32);
			memcpy(firmware_struct_head+32*3,firmware_head+5+37*3,32);
			memcpy(firmware_struct_head+32*4,firmware_head+5+37*4,32);
			pheader_5xx = (FW_HEADER_t_5xx *)firmware_struct_head; 
			pheader_5xxRP = (FW_HEADER_t_5xxRP *)firmware_struct_head; 
			
			check_firmware_header();
			
			if(who == N1_SEND_RP_FIRMWARE)   //rp固件
			{
				if(pheader_5xxRP->ucCfgComb==0x03 || pheader_5xxRP->ucCfgComb==0x08)
					my_firmware_head.firmware_type = FIRM_SENSOR;
				else if(pheader_5xxRP->ucCfgComb==0x07)
					my_firmware_head.firmware_type = FIRM_RP;
				my_firmware_head.fw_ver = pheader_5xxRP->uiBuildNr;
				my_firmware_head.hw_ver = pheader_5xxRP->uiHwVer;
			}
			else if(who == N1_SEND_S_FIRMWARE) 
			{				
				if(pheader_5xx->ucCfgComb==0x03 || pheader_5xx->ucCfgComb==0x08)
					my_firmware_head.firmware_type = FIRM_SENSOR;
				else if(pheader_5xx->ucCfgComb==0x07)
					my_firmware_head.firmware_type = FIRM_RP;
				my_firmware_head.fw_ver = pheader_5xx->uiBuildNr;
				my_firmware_head.hw_ver = pheader_5xx->uiHwVer;				
			}
			

		}
		

		
	}

	else if(ee_write_packet_syn == p_n1_send_firmware_head->packet_syn)   //包序号是我需要的包。
	{
		strhex_to_hex(firmware_head,&(p_n1_send_firmware_head->data[1]),37);
		for(i=0;i<32+5;i++)
		{
			check += firmware_head[i];
		}
		if(check !=0)               //error  行校验没有通过
		{	
			check_error++;
			insert_to_n1_buff(&ptr->data[0],4,cmd);
//			debug("校验出错");
			return;
		}
		
		if(0 == write_bin_flash(begin_address + ee_write_addr_add,firmware_head,37)) //写flash成功
		{
				ee_write_addr_add += 37;
				ee_write_packet_syn++;
				p_n1_send_firmware_head->packet_syn +=1;
				insert_to_n1_buff(&ptr->data[0],4,cmd);	

		}
		else
		{
				p_n1_send_firmware_head->packet_syn = 0;
				insert_to_n1_buff(&ptr->data[0],4,cmd);		
				return;
		}	


		if(p_n1_send_firmware_head->packet_syn == p_n1_send_firmware_head->packet_count) //传输完成
		{
			ee_write_packet_syn = 0;
			ee_write_addr_add = 0;
			if(who == N1_SEND_RP_FIRMWARE)
			{
				read_firmware_rp_head();
			}
			else if(who == N1_SEND_S_FIRMWARE)
			{
				read_firmware_sensor_head();
			}
			
		}


		
#ifdef _DEBUG
	//if(cmd_firmware_data->sequence==0xffff)return;
	sprintf(debug_,"send ack=%d %d %d %d   \r",ack[0],ack[1],ack[2],ack[3]);
	debug(debug_);
#endif		
	}
	
	
	

}







/*
固件存储方式：[4bytes(固件地址)1bytes(行校验)32bytes(固件内容) ] X (n行)

*/
void read_firmware_rp_head()
{
	unsigned char * begin_addr = (unsigned char * )FLASH_RP_FIRMWARE_BEGIN;
	uint16_t i,j;
	uint32_t check_bytes_add = 0;
	uint16_t lines_size = 0;
	uint8_t *p_data;
	static uint32_t last_addr = 0;
	uint32_t addr = 0;
	
	for(i=0;i<8;i++)  //读取固件头
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xxRP+i*32),begin_addr+i*37+5,32);
	}
	
	if(pheader_5xxRP.ulId != 0xdada10af || pheader_5xxRP.ucCfgComb != 7)
	{
		sys_flash_param.ap_param.rp_version = FIRM_HEAD_ERROR;	
		return;
	}
	
	begin_addr += 37*8;
	lines_size = pheader_5xxRP.uiFwSize/32;
	if(lines_size>2000)
	{
		sys_flash_param.ap_param.rp_version = FIRM_LINES_ERROR;	
		return;
	}	
	for(i=0;i<lines_size;i++)
	{
		p_data = begin_addr+i*37;  //指向每行的起始地址
		
		addr = *((uint32_t *)p_data);

		for(j=5;j<37;j++)
		{
			check_bytes_add +=p_data[j];
		}
		if(addr == last_addr)
		{
			check_bytes_add = 0;
			break;
		}
		last_addr = addr;		
	}
	if(check_bytes_add == pheader_5xxRP.ulCheckSum)
	{
		sys_flash_param.ap_param.rp_version = pheader_5xxRP.uiFwVer;
		ap_param_write_flash_flag = 1;
	}
	else
	{
		sys_flash_param.ap_param.rp_version = 0xfffe;
		ap_param_write_flash_flag = 1;		
	}	
}



/*
固件存储方式：[4bytes(固件地址)1bytes(行校验)32bytes(固件内容) ] X (n行)

*/
void read_firmware_sensor_head()
{
	unsigned char * begin_addr = (unsigned char * )FLASH_SENSOR_FIRMWARE_BEGIN;
	uint16_t i,j;
	uint32_t check_bytes_add = 0;
	uint16_t lines_size = 0;
	uint8_t *p_data;
	static uint32_t last_addr = 0;
	uint32_t addr = 0;
	
	for(i=0;i<8;i++)  //读取固件头
	{
		memcpy((uint8_t *)((uint32_t)&pheader_5xx+i*32),begin_addr+i*37+5,32);
	}
	
	if(pheader_5xx.ulId != 0xdada10af || pheader_5xx.ucCfgComb != 3)
	{
		sys_flash_param.ap_param.sensor_version = FIRM_HEAD_ERROR;	
		return;
	}
	
	begin_addr += 37*8;
	lines_size = pheader_5xx.uiFwSize/32;
	
	for(i=0;i<lines_size;i++)
	{
		p_data = begin_addr+i*37;  //指向每行的起始地址
		for(j=5;j<37;j++)
		{
			check_bytes_add +=p_data[j];
		}
		addr = *((uint32_t *)p_data);
		if(addr == last_addr)
		{
			check_bytes_add = 0;
			break;
		}
		last_addr = addr;				
	}
	if(check_bytes_add == pheader_5xx.ulCheckSum)
	{
		sys_flash_param.ap_param.sensor_version = pheader_5xx.uiFwVer;
		ap_param_write_flash_flag = 1;
	}
	else
	{
		sys_flash_param.ap_param.sensor_version = 0xfffe;
		ap_param_write_flash_flag = 1;		
	}
}





















void n1_send_firmware(struct_ap_n1_protocol *ptr,int who)
{
	uint32_t begin_address = 0;
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	uint8_t data[2];
	
	if(who == N1_SEND_RP_FIRMWARE)
	{

		begin_address = FLASH_RP_FIRMWARE_BEGIN;
		EraseInit.Sector = FLASH_SECTOR_17;
	}
	else if(who == N1_SEND_S_FIRMWARE)
	{

		begin_address = FLASH_SENSOR_FIRMWARE_BEGIN;	
		EraseInit.Sector = FLASH_SECTOR_18;		
	}
	if(ptr->data[1] == 0)
	{
		ee_write_addr_add = 0;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);		
	}
	
	if(0 == write_bin_flash(begin_address + ee_write_addr_add,&ptr->data[2],ptr->lengh-3))
	{
			ee_write_addr_add += ptr->lengh-4;
			ee_write_packet_syn++;
			data[0] = ptr->data[0];
			data[1] = ee_write_packet_syn;
			insert_to_n1_buff(data,2,1);		
			if(ptr->data[0] == ptr->data[1]+1)       //传输完毕
			{
				
			}
	}
	else
	{
			data[0] = ptr->data[0];
			data[1] = 0;
			insert_to_n1_buff(data,2,1);			
	}

	
}