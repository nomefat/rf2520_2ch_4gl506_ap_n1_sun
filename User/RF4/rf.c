#include "stm32f4xx_hal.h"
#include "snp.h"
#include "typedef_struct.h"
#include "ap_param.h"
#include "rf_hal.h"
#include "string.h"
#include "to_n1.h"
#include "eeprom.h"
#include "debug.h"
#include "flash.h"
#include "update_s_rp.h"
#include "hal_cc2520.h"
#include "rf_data_handle.h"



#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02

SNP_SYNC_PACKET_t syn_packet;
SNP_AP_ACK_PACKET_t ack_packet;
SNP_UF_DATA_PACKET_t upadate_packet;


struct_systerm_info systerm_info;
SNP_AP_ACK_PACKET_t     s_sApAckPacket[4]; 
uint8_t		        uiIndex[4]={0};      //收到事件包的索引
extern int enable_print_updata_flag;
extern uint8_t wait_cfg_rp_num;
extern uint8_t wait_cfg_s_num;
extern uint32_t timer_reboot_system;




struct _sensor_rp_data_recode{

	uint16_t id;
	uint8_t syn;
}sensor_rp_data_recode[256];

SNNDATA_MESSAGE   g_sSnnDataMessage=
{
  {{SNNMAGIC,
   SNN_DATA_MSG_NO,//2013 lhj  SNN_DATA_MSG_NO原  0x05
   0x2Bu,
  },
  0u,SOURCEID,0u,VERSION,0u,},
  0u,0u,0u,0u,0u
  
};


extern uint8_t gprs_sec_flag ;
extern uint8_t rf_sec_flag ;


extern uint8_t rf_rx_buff[4][256];
extern struct_rf_stat rf_stat[4];
extern char gprs_debug_buff[256];
extern struct_sensor_rp_param sensor_rp_param ;


extern void rf_write_buff(SPI_HandleTypeDef* hspi,void *ptr,int len);
extern void get_s_rp_input_update_stat(uint16_t dev_id, uint16_t flash_times);
extern void rf_send_update_packet(void);
extern void rf_fast_init();



extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

extern struct_update_s_rp_manage update_s_rp_manage ;

void ApPacketsetting(u16_t uiCurSlotNr,u8_t ucCurPacketerNr);
void rf_rev_1000p_test(int index);
void auto_jiaozhun_sensor_add_sensorid(uint16_t sensor_id);
uint16_t auto_jiaozhun_sensor_send_cmd();



int rev_syn_db = -45;
int syn_disable;
int test;
int enable_print_sensor_event = 0;
int enable_print_crc_error = 0;
void clear_recode();
uint8_t get_slot_num();

extern unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);

uint8_t add(uint8_t *pdata,uint32_t size)
{
	int i=0;
	uint8_t ret = 0;
	
	for(i=0;i<size;i++)
	{
		ret += pdata[i];
	}
	return ret;
}

extern int rf1_flush_rx ;
extern int rf2_flush_rx ;
extern int rf3_flush_rx ;
extern int rf4_flush_rx ;

void rf_send_syn_packet(void)
{
  memcpy( (uint8_t *)&syn_packet.sensor_param[0], (uint8_t *)sys_flash_param.ap_param.ap_syn_param, 6 );
	syn_packet.sPhr.ucSize = sizeof( SNP_SYNC_PACKET_t)+1;
	syn_packet.sPhr.uiFcf = 0x0080;
	syn_packet.sPhr.ucSerNr++;
	syn_packet.sPhr.ucType = SNP_PACKET_TYPE_WORK;
	syn_packet.sPhr.uiDevId = sys_flash_param.ap_param.band_id;
	syn_packet.ucCtrlBm = 0x07;
	syn_packet.uiRemainSlot = 512-(systerm_info.slot%512);

	if(syn_disable)
		return;
				
	if(syn_packet.sPhr.ucSerNr%8 == 0)  //秒点
	{
		syn_packet.ucCurSecNr++;
		if(syn_packet.ucCurSecNr>29)
			syn_packet.ucCurSecNr = 0;
		
		syn_packet.ucCtrlBm = 0x06;   //秒点标志位
#if (AP_VERSION   == 0X8888)		
		syn_packet.sPhr.uiDevId = 0xFFF2;
		auto_jiaozhun_sensor_send_cmd();
#endif
#if (AP_VERSION   == 0X8889)		
		syn_packet.sPhr.uiDevId = 0xFFF2;
#endif		
		if(sensor_rp_param.ParaFram.uiCmd !=0)  //有新的snesor 或者 rp参数
		{
			memcpy(&syn_packet.uiCmd,&sensor_rp_param.ParaFram.uiCmd,14);
			syn_packet.sPhr.ucSensorMode = sensor_rp_param.ucSensorMode;
			sensor_rp_param.ParaFram.uiCmd = 0;
			syn_packet.sPhr.ucType |= 1<<4;
			
		}
		else 
		{
			memset(&syn_packet.uiCmd,0,14);
			syn_packet.sPhr.ucSensorMode = 0;
			syn_packet.sPhr.ucType &= ~(1<<4);
		}
	}
	
	syn_packet.uiBindId = sys_flash_param.ap_param.band_id;	
	
	

	
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)  //点灯
	{
		HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
	}


	


	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));
		rf_write_buff(RF1, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF2, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<6);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF3, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		syn_packet.sPhr.ucType = (syn_packet.sPhr.ucType & 0x9f) | (1<<5) |(1<<6);
//		syn_packet.uiCrc = 0;
//		syn_packet.uiCrc = add((uint8_t *)&syn_packet,sizeof(syn_packet));		
		rf_write_buff(RF4, (uint8_t *)&syn_packet ,syn_packet.sPhr.ucSize);	
	}
	
			rf1_flush_rx = 0;
			rf2_flush_rx = 0;
			rf3_flush_rx = 0;
			rf4_flush_rx = 0;		
	
	rf_io_tx_4();
	
	clear_recode();
	
	if(syn_packet.sPhr.ucSerNr%8 == 0) 
	{
		g_sSnnDataMessage.basePhr.sPhr.ucLength=0x11+sizeof(SNP_SYNC_PACKET_t);
		g_sSnnDataMessage.lTimestamp=(u32_t)0u;
		g_sSnnDataMessage.sTime=0u;
		g_sSnnDataMessage.ucLqi=0u;
		g_sSnnDataMessage.ucRssi=0u;
		g_sSnnDataMessage.ucOffset=0x11u;
		memcpy(&g_sSnnDataMessage.ucLoadData[0],&syn_packet,syn_packet.sPhr.ucSize);	
		insert_to_n1_buff((unsigned char *)&g_sSnnDataMessage,syn_packet.sPhr.ucSize+0X11,AP_RF_DATA);		
	}
	
	
		if(syn_packet.uiCmd !=0 && syn_packet.sPhr.ucSerNr%8 == 0)  //有新的snesor 或者 rp参数
		{			
			sprintf(gprs_debug_buff,"syn:set s_rp pram %04X rp:%d->%d s:%d->%d\r\n",
			sensor_rp_param.ParaFram.uiPoll,sys_flash_param.rp_num,wait_cfg_rp_num,sys_flash_param.sensor_num,wait_cfg_s_num);
			copy_string_to_double_buff(gprs_debug_buff);		
		}
}


void rf_send_updata_packet(uint8_t s_or_rp,uint16_t flash_seq)
{
	
	uint32_t flash_base_addr = 0;
	
	uint8_t *p_crc = 0;
	uint8_t i =0;
	
	static int end_flag_send_times;
	
	//更新包 初始化
	upadate_packet.sPhr.ucType = SNP_PACKET_TYPE_UF;
	upadate_packet.sPhr.ucSize = sizeof( SNP_UF_DATA_PACKET_t)+1;
	upadate_packet.sPhr.uiDevId = sys_flash_param.ap_param.band_id;
	upadate_packet.sPhr.ucSerNr++;
	upadate_packet.sPhr.uiFcf = 0x0080;
	
	//选择固件基地址
	if(FIRM_RP == s_or_rp)
	{
		flash_base_addr = FLASH_RP_FIRMWARE_BEGIN;

	}
	else if(FIRM_SENSOR == s_or_rp)
	{
		flash_base_addr = FLASH_SENSOR_FIRMWARE_BEGIN;

	}
	else
		return;

//按照FLASH序号到指定的地址取得数据	
	upadate_packet.uiAddress = *((uint32_t *)(flash_base_addr + flash_seq*37));
	memcpy(upadate_packet.auiBuffer,(uint8_t *)(flash_base_addr + flash_seq*37 + 5),32);

	if(upadate_packet.uiAddress == 0xffffffff)
	{
		upadate_packet.uiAddress = 0xffffffff;
		if(update_s_rp_manage.now_send_times>20)
		{
			update_s_rp_manage.upadate_s_rp_enable = 0;
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq = 0;
			update_s_rp_manage.dev_num = 0;
		}
	}
	//计算CRC
	p_crc = (uint8_t *)(flash_base_addr + flash_seq*37);
	upadate_packet.CrcSum = 0;
/*	for(i=0;i<37;i++)
	{
		if(i == 4)
			continue;
		upadate_packet.CrcSum += p_crc[i];
	}
*/
	
	upadate_packet.CrcSum = crc16(0,(uint8_t *)upadate_packet.auiBuffer,32);
	
	
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
	}
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
	}


	


	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF1, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);
	}
		
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF2, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF3, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
	{
		rf_write_buff(RF4, (uint8_t *)&upadate_packet ,upadate_packet.sPhr.ucSize);	
	}
	

	
	rf_io_tx_4();		
}

int ack_num = 0;
int ack_disable = 0;

void rf_send_ack_packet(int slot)
{
	int32_t i = 72*40;
	
	while(i--);
	
	if(ack_disable == 1)
		return;
	
	if(s_sApAckPacket[slot].ulSlotBm >0) //有slot事件处理
	{
		
		ack_num++;
	//需要确定包序列号	     
		s_sApAckPacket[slot].sPhr.ucSerNr++;

		if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_RESET);
		}
		if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_RESET);	
		}
		if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_RESET);
		}
		if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
		{
			HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_RESET);
		}


		


		if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF1, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);
		}
		if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF2, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF3, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK)
		{
			rf_write_buff(RF4, (uint8_t *)&s_sApAckPacket[slot] ,s_sApAckPacket[slot].sPhr.ucSize);	
		}
		

		
		rf_io_tx_4();		

		if(enable_print_sensor_event > 0)	
		{			
			sprintf(gprs_debug_buff,"send_ack:s=%d slot=%d %X ack_num=%d\r\n",systerm_info.slot/get_slot_num(),slot,s_sApAckPacket[slot].ulSlotBm,ack_num);
			copy_string_to_double_buff(gprs_debug_buff);
	  }
		s_sApAckPacket[slot].ulSlotBm=0;
		uiIndex[slot]=0;		
		memset(s_sApAckPacket[slot].aucSerNr,0,sizeof(s_sApAckPacket[slot].aucSerNr));  //2013 7/25 lhj
			 

	}
}

int flag = 0;

void HAL_SYSTICK_Callback(void)
{

	systerm_info.slot++;
	if(timer_reboot_system == systerm_info.slot)
	{
		HAL_NVIC_SystemReset();
	}
	if(ee_task.read_write > 0)   //有读写ee的任务
	{
		ee_task.timeout++;
	}

	if((systerm_info.slot%512) == 0) //gprs使用秒标志
	{
		gprs_sec_flag += 1;
		rf_sec_flag +=1;
	}
	while(flag);
	if(systerm_info.enable_rf == 0)
		return;
	
	if((systerm_info.slot%64) == 0)  //发送同步包
		rf_send_syn_packet();
	
	if ((systerm_info.slot%64) == 1)  //发送同步包
		rf_fast_init();

	if((systerm_info.slot%get_slot_num()) == 33)  //发送升级包
		rf_send_update_packet();	
		
	if((systerm_info.slot%get_slot_num()) == 2)  //发送ack包
		rf_send_ack_packet(0);
	if((systerm_info.slot%get_slot_num()) == 32)  //发送ack包
		rf_send_ack_packet(1);	
	if((systerm_info.slot%get_slot_num()) == 66)  //发送ack包
		rf_send_ack_packet(2);	
	if((systerm_info.slot%get_slot_num()) == 96)  //发送ack包
		rf_send_ack_packet(3);	
	
}







void clear_recode()
{
	int i = 0;
	for(i=0;i<256;i++)
	{
		sensor_rp_data_recode[i].id = 0;
		sensor_rp_data_recode[i].syn = 0;
	}
	
}


int check_recode_data_if_repeat(uint16_t id,uint8_t syn)
{
	int i = 0;
	for(i=0;i<256;i++)
	{
		if(sensor_rp_data_recode[i].id == id)
		{
			if(sensor_rp_data_recode[i].syn == syn)
				return -1;
		}
		if(sensor_rp_data_recode[i].id == 0)
		{
			sensor_rp_data_recode[i].id = id;
			sensor_rp_data_recode[i].syn = syn;
			return 0;
		}
	}
}

signed rssi1 = 0;

void make_data_to_n1(int index)
{
	
	g_sSnnDataMessage.basePhr.sPhr.ucLength=0x11+(rf_rx_buff[index][0]);
	
	g_sSnnDataMessage.lTimestamp=(u32_t)0u;
	
	g_sSnnDataMessage.sTime = systerm_info.slot%get_slot_num();			

	if(rssi1 != 0)
		g_sSnnDataMessage.ucLqi= rssi1-76;//链路质量
	else
		g_sSnnDataMessage.ucLqi= (rf_rx_buff[index][rf_rx_buff[index][0]] & 0x7F);//链路质量
			 
	g_sSnnDataMessage.ucRssi= rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76;      //信号强度 
			 
	g_sSnnDataMessage.ucOffset=0x11u;       
	
	memcpy(&g_sSnnDataMessage.ucLoadData[0],(char *)&rf_rx_buff[index][0],((rf_rx_buff[index][0]) & 0x3F));	

	insert_to_n1_buff((unsigned char *)&g_sSnnDataMessage,g_sSnnDataMessage.basePhr.sPhr.ucLength-1,AP_RF_DATA);  //发给SM 2013 0824 mod lhj 应答机制	
	

}

void make_ack(uint8_t packet_syn)
{
	ApPacketsetting(systerm_info.slot%get_slot_num(),packet_syn);
}


void rf_rx_data_handle(int index)
{
	SNP_PHR_t *ptr = (SNP_PHR_t *)&rf_rx_buff[index];

	SNP_SYNC_PACKET_tt	*ptr_syn = (SNP_SYNC_PACKET_tt *)&rf_rx_buff[index];
	SNP_STATE_PACKET_RP_t	*ptr_rp_stat = (SNP_STATE_PACKET_RP_t *)&rf_rx_buff[index];
	SNP_STATE_PACKET_SENSOR_t *ptr_s_stat = (SNP_STATE_PACKET_SENSOR_t *)&rf_rx_buff[index];
	SNP_SEN_MODE_B_PACKET_t *ptr_s_event = (SNP_SEN_MODE_B_PACKET_t *)&rf_rx_buff[index];
	
	uint8_t slot = systerm_info.slot%get_slot_num();
	SPI_HandleTypeDef* rf_index[] = {RF1,RF2,RF3,RF4};
	
	
	
//	sprintf(gprs_debug_buff,"rf%d_rev_data slot=%d t=%d:%X %X %X %X %X %X %X %X %X %X %X %X\r\n",index,systerm_info.slot,328125-SysTick->VAL,rf_rx_buff[index][0],rf_rx_buff[index][1],rf_rx_buff[index][2],rf_rx_buff[index][3],
//	rf_rx_buff[index][4],rf_rx_buff[index][5],rf_rx_buff[index][6],rf_rx_buff[index][7],rf_rx_buff[index][8],rf_rx_buff[index][9],rf_rx_buff[index][10],rf_rx_buff[index][11]);
//	copy_string_to_double_buff(gprs_debug_buff);	
//	return;
	
	
	
	if((rf_rx_buff[index][rf_rx_buff[index][0]] & 0x80 )!= 0x80)
	{
		CC2520_send_cmd(rf_index[index],CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(rf_index[index],CC2520_INS_SFLUSHRX );
		if(enable_print_crc_error)
		{
			sprintf(gprs_debug_buff,"rf%dreceive_crc_error:id=%04X type=%d seq=%d len=%d slot=%d\r\n",index+1,ptr->uiDevId,ptr->ucType,ptr->ucSerNr,rf_rx_buff[index][0],systerm_info.slot%get_slot_num());
			copy_string_to_double_buff(gprs_debug_buff);		
		}
		return;	
	}
	
	rf_rev_1000p_test(index);
	
	if(ptr->uiDevId == 0x0111 && ptr->ucType == SNP_PACKET_TYPE_EVENT)
		slot =slot;
	
	rf_stat[index].rf_no_data_sec = 0;

//	if(/*ptr->uiFcf == 0x0080 && */(ptr->ucType & 0x0f) == SNP_PACKET_TYPE_WORK)
//	{
//		if((signed char)(rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76) > rev_syn_db)
//		{
//			sprintf(gprs_debug_buff,"rf%d_rev_syn:id=%04X seq=%d rssi=%d\r\n",index+1,ptr->uiDevId,ptr->ucSerNr,(signed char)(rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76));
//			copy_string_to_double_buff(gprs_debug_buff);		
//		}			
//	}
	
	if(ptr->uiFcf != 0x4180)
		return;
	                                                                       
	//检测器状态包
	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE)
	{
		if((ptr_s_stat->uiSlot & 0x00ff)!= slot && ptr_s_stat->ucFilterFlag !=0)
			return;
		if(ptr_s_stat->sPhr.ucSize>48)
			return;		
	}
	//事件包
	if(ptr->ucType == SNP_PACKET_TYPE_EVENT)
	{
		
		if(ptr_s_event->sPhr.uiDevId == 0xab47)
		{
			ptr_s_event->slot &= 0xff00;
			ptr_s_event->slot |= slot;
		}
		else if((ptr_s_event->slot & 0x00ff) != slot)
			return;
		
		if(ptr_s_event->sPhr.ucSize>48)
			return;
	}	
	rssi1 = 0;
	//rp状态包
	if(ptr->ucType == SNP_PACKET_TYPE_RP_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE)
	{
		if((ptr_rp_stat->uiSlot & 0x00ff) != slot && (ptr_rp_stat->reserve & 0x3f)  != 0)
			return;
		if(ptr_rp_stat->sPhr.ucSize>48)
			return;	
		rssi1 = ptr_rp_stat->uiRssi;
	}	
	
	if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_STATE ||ptr->ucType == SNP_PACKET_TYPE_EVENT
		|| ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE|| ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE||  ptr->ucType == SNP_PACKET_TYPE_SENSOR_D) //sensor状态包 事件包 rp 状态包
	{

		if(check_recode_data_if_repeat(ptr->uiDevId,ptr->ucSerNr)==0)
		{
			if(ptr_s_stat->sPhr.ucType == SNP_PACKET_TYPE_SEN_UF_STATE && enable_print_updata_flag)
				{
					sprintf(gprs_debug_buff,"rev_updata_status:id=%04X type=%d syn=%d flash=%d slot=%d\r\n",ptr->uiDevId,ptr->ucType,ptr->ucSerNr,ptr_s_stat->uiSubData,systerm_info.slot%get_slot_num());
					copy_string_to_double_buff(gprs_debug_buff);						
				}					
			if( ptr_rp_stat->sPhr.ucType == SNP_PACKET_TYPE_RP_UF_STATE && enable_print_updata_flag) 
				{
					sprintf(gprs_debug_buff,"rev_updata_status:id=%04X type=%d syn=%d flash=%d slot=%d\r\n",ptr->uiDevId,ptr->ucType,ptr->ucSerNr,ptr_rp_stat->uiSubData,systerm_info.slot%get_slot_num());
					copy_string_to_double_buff(gprs_debug_buff);						
				}			
			if(enable_print_sensor_event > 0)
			{
				{
				sprintf(gprs_debug_buff,"receive_rf_data:id=%04X type=%d seq=%d s=%d slot=%d\r\n",ptr->uiDevId,ptr->ucType,ptr->ucSerNr,systerm_info.slot/get_slot_num(),systerm_info.slot%get_slot_num());
				copy_string_to_double_buff(gprs_debug_buff);
				}					
			}				
			make_data_to_n1(index);
			
			make_ack(ptr->ucSerNr);
			
			if(ptr->ucType == SNP_PACKET_TYPE_EVENT) //事件包
				insert_sensor_event(ptr_s_event,(int8_t)rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76,(rf_rx_buff[index][rf_rx_buff[index][0]] & 0x7F),slot);
			
			if(ptr->ucType == SNP_PACKET_TYPE_SEN_STATE || ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE)
				insert_sensor_stat_packet(ptr_s_stat,(int8_t)rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76,(rf_rx_buff[index][rf_rx_buff[index][0]] & 0x7F),slot);

			if(ptr->ucType == SNP_PACKET_TYPE_RP_STATE || ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE)
				insert_rp_stat_packet(ptr_rp_stat,(int8_t)rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76,(rf_rx_buff[index][rf_rx_buff[index][0]] & 0x7F),slot);			
			
			if(ptr->ucType == SNP_PACKET_TYPE_EVENT) //事件包
			{
				debug_insert_sensor_event(ptr->uiDevId,ptr->ucSerNr,(int8_t)rf_rx_buff[index][rf_rx_buff[index][0]-1] - 76,slot,(ptr_s_event->slot)>>8);
#if (AP_VERSION   == 0X8888)					
				if(ptr_s_event->asEvent->uiAll == 0xFF7F)
					auto_jiaozhun_sensor_add_sensorid(ptr->uiDevId);
#endif				
			}
			if(ptr->ucType == SNP_PACKET_TYPE_SEN_UF_STATE) 
				get_s_rp_input_update_stat(ptr->uiDevId,ptr_s_stat->uiSubData);
			if(ptr->ucType == SNP_PACKET_TYPE_RP_UF_STATE )
				get_s_rp_input_update_stat(ptr->uiDevId,ptr_rp_stat->uiSubData);

		}
	}
}



uint8_t get_slot_num()
{
	uint8_t lowLatency;
	uint8_t transmitInterval;
	
	lowLatency = (sys_flash_param.ap_param.ap_syn_param[5] & 0x10);

  transmitInterval = ((sys_flash_param.ap_param.ap_syn_param[0] >> 4) & 0x07);
       
	if(lowLatency==0)  //     
			 transmitInterval += 2;
	else
			transmitInterval = (2 - transmitInterval) & 0x01;
	
	if((transmitInterval+1)>=1u && transmitInterval<=6)
		 transmitInterval=1<<transmitInterval;

	return transmitInterval*16;
}




void ApPacketsetting(u16_t uiCurSlotNr,u8_t ucCurPacketerNr)
{
    u32_t retValue=0;
    u16_t i;      
	
	
    if(uiCurSlotNr>32 && uiCurSlotNr<64)//slot from 6 to 32；36 to 62 
    {
			s_sApAckPacket[1].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
			s_sApAckPacket[1].sPhr.ucType = SNP_PACKET_TYPE_ACK;
			s_sApAckPacket[1].sPhr.ucSensorMode = 0;
			s_sApAckPacket[1].sPhr.uiDevId = sys_flash_param.ap_param.band_id;
			s_sApAckPacket[1].sPhr.uiFcf = 0x0080;
			
			
       i=uiCurSlotNr-33;
       retValue=(u32_t)1u<<i;  
       if((s_sApAckPacket[1].ulSlotBm & retValue) ==0)
       {
         s_sApAckPacket[1].ulSlotBm |=retValue;
				 if(uiIndex[1]<= 0x0F)
					s_sApAckPacket[1].aucSerNr[(uiIndex[1]&0x0F)]=ucCurPacketerNr;         
         uiIndex[1]++;
       }
    }
    else  if(uiCurSlotNr<32)//时间槽如果落在小于36的区间则回复检测器s_sApAckPacket[0]
    {
			s_sApAckPacket[0].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
			s_sApAckPacket[0].sPhr.ucType = SNP_PACKET_TYPE_ACK;
			s_sApAckPacket[0].sPhr.ucSensorMode = 0;
			s_sApAckPacket[0].sPhr.uiDevId = sys_flash_param.ap_param.band_id;
			s_sApAckPacket[0].sPhr.uiFcf = 0x0080;			
			
      i=uiCurSlotNr-3;  
      retValue =(u32_t)1u<<i; 
      if((s_sApAckPacket[0].ulSlotBm & retValue) ==0)
      {
        s_sApAckPacket[0].ulSlotBm |=retValue;
				if(uiIndex[0]<= 0x0F)
					s_sApAckPacket[0].aucSerNr[(uiIndex[0]& 0x0F)]=ucCurPacketerNr;        
        uiIndex[0]++;
      }
    }   
		
	
		
 		else if((uiCurSlotNr & 0X7F)>96)//slot from 70 to 96；100 to 126 
     {
			 s_sApAckPacket[3].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
				s_sApAckPacket[3].sPhr.ucType = SNP_PACKET_TYPE_ACK;
				s_sApAckPacket[3].sPhr.ucSensorMode = 0;
				s_sApAckPacket[3].sPhr.uiDevId = sys_flash_param.ap_param.band_id;
				s_sApAckPacket[3].sPhr.uiFcf = 0x0080;
			 
        i=uiCurSlotNr-97;
        retValue=(u32_t)1u<<i;  
        if((s_sApAckPacket[3].ulSlotBm & retValue) ==0)
        {
          s_sApAckPacket[3].ulSlotBm |=retValue;
					if(uiIndex[3]<= 0x0F)
						s_sApAckPacket[3].aucSerNr[(uiIndex[3]&0x0F)]=ucCurPacketerNr;         
          uiIndex[3]++;
        }
     }
     else if((uiCurSlotNr & 0X7F)<96 && (uiCurSlotNr & 0X7F)>66)
     {
			 s_sApAckPacket[2].sPhr.ucSize = sizeof(SNP_AP_ACK_PACKET_t);
				s_sApAckPacket[2].sPhr.ucType = SNP_PACKET_TYPE_ACK;
				s_sApAckPacket[2].sPhr.ucSensorMode = 0;
				s_sApAckPacket[2].sPhr.uiDevId = sys_flash_param.ap_param.band_id;
				s_sApAckPacket[2].sPhr.uiFcf = 0x0080;
			 
       i=uiCurSlotNr-67;  
       retValue =(u32_t)1u<<i; 
       if((s_sApAckPacket[2].ulSlotBm & retValue) ==0)
       {
         s_sApAckPacket[2].ulSlotBm |=retValue;
				 if(uiIndex[2]<= 0x0F)
					s_sApAckPacket[2].aucSerNr[(uiIndex[2]& 0x0F)]=ucCurPacketerNr;        
         uiIndex[2]++;
       }
     }   
		
		
}


struct _test_1000p_manage
{
	unsigned int last_test_packet_time;
	unsigned int rev_packet_count;
	signed int rssi;
}test_1000p_manage;


extern uint8_t rf_send_1000_p_enable;
void rf_rev_1000p_test(int index)
{
	
	uint32_t *packet_seq = (uint32_t *)&rf_rx_buff[index][1];	
	
	if(rf_send_1000_p_enable == 0)
		return;
	
	if(rf_send_1000_p_enable & (1<<index))
	{
		if(rf_rx_buff[index][0] == 15 && rf_rx_buff[index][5] == 5 && rf_rx_buff[index][6] == 6)
		{
			test_1000p_manage.rev_packet_count++;
			test_1000p_manage.rssi += (signed int)(rf_rx_buff[index][14] - 76);
			test_1000p_manage.last_test_packet_time = systerm_info.slot;
			sprintf(gprs_debug_buff,"rx_1000p_rf%d: rev=%d packet_seq=%d rssi=%d\r\n",index+1,test_1000p_manage.rev_packet_count,*packet_seq,(signed char)(rf_rx_buff[index][14] - 76));
			copy_string_to_double_buff(gprs_debug_buff);				
		}
		
	}
	

}

void make_result_test_1000p()
{
	if(test_1000p_manage.last_test_packet_time == 0)
		return;
	
	if(test_1000p_manage.last_test_packet_time && systerm_info.slot-test_1000p_manage.last_test_packet_time>1024)
	{
		sprintf(gprs_debug_buff,"test_1000p_result:rev=%d,right=%f%% rssi=%d\r\n",test_1000p_manage.rev_packet_count,(float)(test_1000p_manage.rev_packet_count)/(float)10,test_1000p_manage.rssi/(int)test_1000p_manage.rev_packet_count);
		copy_string_to_double_buff(gprs_debug_buff);		
		test_1000p_manage.last_test_packet_time = 0;
		test_1000p_manage.rev_packet_count = 0;
		test_1000p_manage.rssi = 0;
		
	}
	
}


#define AUTO_SET_BUFF_LENGH 64
uint16_t auto_data_buff[AUTO_SET_BUFF_LENGH];   //用于从队列中取出完整的一条命令或者数据 


int auto_set_queue_write = 0;               //写偏移
int auto_set_queue_read = 0;								//读偏移

/*
 * 功能：往队列中写入一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char auto_set_write_queue(uint16_t data)
{
	if((auto_set_queue_write+1)%AUTO_SET_BUFF_LENGH==auto_set_queue_read)
		return -1;
	auto_data_buff[auto_set_queue_write] = data;
	auto_set_queue_write = (auto_set_queue_write+1)%AUTO_SET_BUFF_LENGH;
	return 0;
}

/*
 * 功能：从队列中读取一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char auto_set_read_queue(uint16_t *pdata)
{
	if(auto_set_queue_write==auto_set_queue_read)
		return -1;		
	*pdata = auto_data_buff[auto_set_queue_read];
	auto_set_queue_read = (auto_set_queue_read+1)%AUTO_SET_BUFF_LENGH;
	return 0;
}






/*
*  func:用作生产工装使用  自动校准sensor 避免长时间等待sensor自校准
*  方法:通过识别Sensor入网发送的7FFF 然后把校准命令加入缓冲 等待秒点发送
*
*
*
*/
void auto_jiaozhun_sensor_add_sensorid(uint16_t sensor_id)
{
	auto_set_write_queue(sensor_id);
}




uint16_t auto_jiaozhun_sensor_send_cmd()
{
	uint16_t pdata;
	if(auto_set_read_queue(&pdata)!=0)
	{
		sensor_rp_param.ParaFram.uiCmd = 2;
		sensor_rp_param.ParaFram.uiPoll = pdata;
	}
}
















