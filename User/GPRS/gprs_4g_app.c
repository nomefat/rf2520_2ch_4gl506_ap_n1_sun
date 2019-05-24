#include "gprs_4g_app.h"
#include "ap_param.h"
#include "rf_data_handle.h"
#include "string.h"
#include "typedef_struct.h"
#include "gprs_comm.h"
#include "timer_and_realtime_data.h"
#include "flash.h"
#include "update_s_rp.h"






int8_t who_dtu_ack_r = -1;

uint32_t  timer_reboot_system ;
uint16_t battery_mv;
uint8_t cfg_lane_num = 0;

extern uint8_t rtdata_send_buff[2][1450];   //数据发送buff
extern uint8_t rtdata_send_buff_count[2]; //buff里面的数据条数
extern struct_gprs_one_cmd gprs_one_cmd;
extern uint8_t ap_param_write_flash_flag;
extern struct_systerm_info systerm_info;
extern int32_t add_4g_data_len(int8_t client,int16_t len);
extern uint8_t * get_4g_data_buff(int8_t client,uint16_t len);
unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len);
extern int32_t heart_timeout(int32_t client);
extern RTC_TimeTypeDef rtc_time;
extern RTC_DateTypeDef rtc_date;
extern RTC_HandleTypeDef hrtc;
extern struct_update_s_rp_manage update_s_rp_manage ;
extern char gprs_debug_buff[256];
extern struct_gprs_stat gprs_stat;
extern int gprs_print_rx_tx_data_enable;
extern ADC_HandleTypeDef hadc1;





void rev_4g_server_ap_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len);
void rev_4g_server_rp_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len);
void rev_4g_server_s_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len);

extern int copy_string_to_double_buff(char *pstr);
extern void read_firmware_rp_head();
extern void read_firmware_sensor_head();
extern void sensor_rp_updata_manage();


struct {
	uint32_t send_time;
	uint32_t send_flag;
	uint32_t send_times;
}send_heart_flag[CLIENT_NUM];

uint8_t wait_cfg_rp_num;
uint8_t wait_cfg_s_num;

struct_client_data_buff client_data_buff_0;
struct_client_data_buff client_data_buff_1;


struct_gprs_4g_task gprs_4g_task[CLIENT_NUM];
struct_auto_cfg auto_cfg_stat[CLIENT_NUM]; 







int32_t send_4g_sys_param_table(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_gprs_4g_packet_head *phead;
	uint16_t crc = 0;
	uint16_t size = 12+2+sizeof(struct_ap_param) + sizeof(struct_global_cfg_param) + 2 + 
		sizeof(s_sensor_pram)*sys_flash_param.sensor_num + sizeof(s_rp_pram)*sys_flash_param.rp_num ;
	uint16_t index = 0;
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;
	
	phead = (struct_gprs_4g_packet_head *)pdata_buff;
	phead->head = 0x584d5555;
	phead->ap_id = sys_flash_param.ap_param.ap_id;
	phead->len = size-12;
	phead->data[index++] = CMD1_4G_SYS_PARAM_TABLE;
	phead->data[index++] = CMD2_4G_SYS_PARAM_TABLE_WRITE;

	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);		
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);

	sys_flash_param.ap_param.rtc_date_time.year = rtc_date.Year+2000;
	sys_flash_param.ap_param.rtc_date_time.month = rtc_date.Month;
	sys_flash_param.ap_param.rtc_date_time.date =  rtc_date.Date;
	sys_flash_param.ap_param.rtc_date_time.hour = rtc_time.Hours;
	sys_flash_param.ap_param.rtc_date_time.minter = rtc_time.Minutes;
	sys_flash_param.ap_param.rtc_date_time.sec = rtc_time.Seconds;
	sys_flash_param.ap_param.ap_version = AP_VERSION;
	
	memcpy(&phead->data[index],&sys_flash_param.ap_param,sizeof(sys_flash_param.ap_param));
	index += sizeof(sys_flash_param.ap_param);
	
	memcpy(&phead->data[index],&sys_flash_param.global_cfg_param,sizeof(sys_flash_param.global_cfg_param)+2);
	index += sizeof(sys_flash_param.global_cfg_param)+2;	
	
	if(sys_flash_param.sensor_num > 0)
	{
		memcpy(&phead->data[index],&sys_flash_param.sensor[0],sizeof(s_sensor_pram)*sys_flash_param.sensor_num);
		index += sizeof(s_sensor_pram)*sys_flash_param.sensor_num;		
	}
	
	if(sys_flash_param.rp_num > 0)
	{
		memcpy(&phead->data[index],&sys_flash_param.rp[0],sizeof(s_rp_pram)*sys_flash_param.rp_num);
		index += sizeof(s_rp_pram)*sys_flash_param.rp_num;		
	}
	
	crc = crc16(0,(uint8_t *)&phead->ap_id,size-6);
	phead->data[index++] = (crc>>8)&0xff;
	phead->data[index++] = crc&0xff;
	
	add_4g_data_len(whitch_client,size);
	
	return 0;
}


int32_t send_4g_sensor_rp_updata_stat(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_gprs_4g_packet_head *phead;
	uint16_t crc = 0;
	uint16_t size;
	uint16_t index ,index1= 0;
	struct__sensor_updata_stat *pstat;
	
	// head+cmd2+sensor_num+rp_num
	size = 12+2+2+(sys_flash_param.sensor_num + sys_flash_param.rp_num)*sizeof(struct__sensor_updata_stat);
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;


	phead = (struct_gprs_4g_packet_head *)pdata_buff;
	phead->head = 0x584d5555;
	phead->ap_id = sys_flash_param.ap_param.ap_id;
	phead->len = size-12;
	phead->data[index++] = CMD1_4G_UPDATA_SENSOR_RP;
	phead->data[index++] = CMD2_4G_UPDATA_SENSOR_RP_STAT;
	phead->data[index++] = sys_flash_param.sensor_num;
	phead->data[index++] = sys_flash_param.rp_num;
	pstat = (struct__sensor_updata_stat*)&phead->data[index];
	
	for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
	{
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id != 0)
		{
			pstat[index1].sensor_id = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id;
			pstat[index1].timeout_sec = (systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot)/512;
			if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.sPhr.ucType == SNP_PACKET_TYPE_SEN_UF_STATE)
				pstat[index1].updata_seq = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.uiSubData;
			else
				pstat[index1].updata_seq = -1;
			pstat[index1].version = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.uiFwVer;
			index1++;
		}
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id != 0)
		{
			pstat[index1].sensor_id = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id;
			pstat[index1].timeout_sec = (systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].before.last_packet_time_slot)/512;
			if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.sensor_stat_packet.sPhr.ucType == SNP_PACKET_TYPE_SEN_UF_STATE)
				pstat[index1].updata_seq = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.uiSubData;
			else
				pstat[index1].updata_seq = -1;
			pstat[index1].version = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.sensor_stat_packet.uiFwVer;
			index1++;
		}		
	}
	
	for(index=0;index<sys_flash_param.rp_num;index++)
	{
		if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id != 0)
		{
			pstat[index1].sensor_id = lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id;
			pstat[index1].timeout_sec = (systerm_info.slot - lane_to_sensor_info_and_result.rp_cfg_and_stat[index].last_packet_time_slot)/512;
			if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rp_stat_packet.sPhr.ucType == SNP_PACKET_TYPE_RP_UF_STATE)
				pstat[index1].updata_seq = lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rp_stat_packet.uiSubData;
			else
				pstat[index1].updata_seq = -1;
			pstat[index1].version = lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rp_stat_packet.uiFwVer;
			index1++;
		}
	}

	crc = crc16(0,(uint8_t *)&phead->ap_id,size-6);
	phead->data[(sys_flash_param.sensor_num + sys_flash_param.rp_num)*sizeof(struct__sensor_updata_stat)+4]  = (crc>>8)&0xff;
	phead->data[(sys_flash_param.sensor_num + sys_flash_param.rp_num)*sizeof(struct__sensor_updata_stat)+5] = crc&0xff;
	
	add_4g_data_len(whitch_client,size);	
	return 0;
	
}



int32_t send_4g_heart_pacekt(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_gprs_4g_heart_packet *p_packet;
	uint16_t size;
	uint16_t crc;
	
	
	size = sizeof(struct_gprs_4g_heart_packet);
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	
	
	p_packet = (struct_gprs_4g_heart_packet *)pdata_buff;
	p_packet->head = 0x584d5555;
	p_packet->ap_id = sys_flash_param.ap_param.ap_id;
	p_packet->len = size-12;
	p_packet->cmd = 0x0101;
	p_packet->ap_live_time = systerm_info.slot/512;	
	p_packet->gps_n_e[0] = sys_flash_param.ap_param.gps_n_e[0];
	p_packet->gps_n_e[1] = sys_flash_param.ap_param.gps_n_e[1];
	p_packet->gprs_rssi = gprs_stat.csq;
	
	p_packet->chip_id.id1 = *((uint32_t *)0x1fff7a10);
	p_packet->chip_id.id2 = *((uint32_t *)0x1fff7a14);
	p_packet->chip_id.id3 = *((uint32_t *)0x1fff7a18);	
	
	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);	
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);
	
	p_packet->rtc_date_time.date = rtc_date.Date;
	p_packet->rtc_date_time.month = rtc_date.Month;	
	p_packet->rtc_date_time.year = rtc_date.Year+2000;	
	p_packet->rtc_date_time.hour = rtc_time.Hours;
	p_packet->rtc_date_time.minter = rtc_time.Minutes;
	p_packet->rtc_date_time.sec = rtc_time.Seconds;	
	
	p_packet->client0_send_bytes = gprs_stat.con_client[0].tx_len;
	p_packet->client1_send_bytes = gprs_stat.con_client[1].tx_len;	
	p_packet->battery_mv = battery_mv*6*3300/4096;
	
	
	crc = crc16(0,(uint8_t *)&p_packet->ap_id,size-6);
	p_packet->crc = 0;
	p_packet->crc |= ((crc>>8)&0xff);
	p_packet->crc |= ((crc<<8)&0xff00);	
	
	add_4g_data_len(whitch_client,size);
	
	return 0;
}

int32_t send_4g_sys_param_ack(int32_t whitch_client)
{
	struct_gprs_4g_packet_head *phead;
	uint8_t *pdata_buff;
	uint16_t crc;
	
	if(whitch_client > CLIENT_NUM)
		return -1;
	
	pdata_buff = get_4g_data_buff(0,15);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	

	phead = (struct_gprs_4g_packet_head *)pdata_buff;
	phead->head = 0x584d5555;
	phead->ap_id = sys_flash_param.ap_param.ap_id;
	phead->len = 3;
	phead->data[0] = CMD1_4G_SYS_PARAM_TABLE;
	phead->data[1] = CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK;	
	phead->data[2] = gprs_4g_task[whitch_client].sys_param_table_task_ack_value;	
	
	crc = crc16(0,(uint8_t *)&phead->ap_id,9);
	phead->data[3] = (crc>>8)&0xff;
	phead->data[4] = crc&0xff;	
	
	add_4g_data_len(0,15);
	
	return 0;
}


int32_t send_4g_timer_data_sensor(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_to_server_head *p_packet;
	struct_sensor_data_timer *p_data;
	uint16_t size;
	uint16_t crc;
	int8_t index;
	
	
	
	if(cfg_lane_num > 64)
		cfg_lane_num = 64;
	
	size = sizeof(struct_to_server_head) + cfg_lane_num*sizeof(struct_sensor_data_timer) + 2;
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	
	
	p_packet = (struct_to_server_head *)pdata_buff;
	p_packet->head = 0x584d5555;
	p_packet->ap_id = sys_flash_param.ap_param.ap_id;
	p_packet->len = size-12;
	p_packet->cmd = 0x0148;

	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);

	
	p_packet->year = rtc_date.Year;
	p_packet->month = rtc_date.Month;
	p_packet->day = rtc_date.Date;
	
	p_packet->hour = rtc_time.Hours;
	p_packet->min = rtc_time.Minutes;
	p_packet->sec = rtc_time.Seconds;
	p_packet->sensor_num = 0;
	
	p_data = (struct_sensor_data_timer *)(&p_packet->sensor_num + 1);
	
	
	for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
	{
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id == 0)
			continue;
		
		p_data[p_packet->sensor_num].car1_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].car_len_count[0];
		p_data[p_packet->sensor_num].car2_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].car_len_count[1];		
		p_data[p_packet->sensor_num].car3_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].car_len_count[2];		
		p_data[p_packet->sensor_num].car4_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].car_len_count[3];		
		p_data[p_packet->sensor_num].car5_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].car_len_count[4];

		p_data[p_packet->sensor_num].avg_car_car_time_distance = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].avg_car_car_time_distance;
		p_data[p_packet->sensor_num].avg_car_head_time_distance = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].avg_car_head_time_distance;
		p_data[p_packet->sensor_num].avg_car_length = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].avg_car_length;
		p_data[p_packet->sensor_num].avg_speed = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].avg_speed;
		
		p_data[p_packet->sensor_num].max_speed = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].max_speed;
		p_data[p_packet->sensor_num].min_speed = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].min_speed;
		p_data[p_packet->sensor_num].occupancy = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_event.t[1].occupancy;
		p_data[p_packet->sensor_num].sensor_id = (lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id>>8) | (lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id<<8);
		p_packet->sensor_num++;
	}
	
	size = sizeof(struct_to_server_head) + p_packet->sensor_num*sizeof(struct_sensor_data_timer);
	p_packet->len = size-10;
	crc = crc16(0,(uint8_t *)&p_packet->ap_id,size-4);
	
	((uint8_t *)(&p_data[p_packet->sensor_num].sensor_id))[0] = (crc>>8)&0xff;
	((uint8_t *)(&p_data[p_packet->sensor_num].sensor_id))[1] = crc;	
	
	add_4g_data_len(whitch_client,size+2);
	
	return 0;	
}

int32_t send_4g_timer_stat_sensor(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_to_server_head *p_packet;
	struct_sensor_stat_timer *p_sensor_stat;
	struct_rp_stat_timer *p_rp_stat;
	struct_ap_stat_timer *p_ap_stat;
	uint16_t size;
	uint16_t crc;
	int8_t index;
	uint8_t *rp_num;
	
	
	if(lane_to_sensor_info_and_result.has_lane_sensor_num > 100)
		lane_to_sensor_info_and_result.has_lane_sensor_num = 100;
	
	size = sizeof(struct_to_server_head) + lane_to_sensor_info_and_result.has_lane_sensor_num*sizeof(struct_sensor_stat_timer) + 1+
		lane_to_sensor_info_and_result.cfg_rp_num*sizeof(struct_rp_stat_timer) + sizeof(struct_ap_stat_timer) + 2;
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	
	
	p_packet = (struct_to_server_head *)pdata_buff;
	p_packet->head = 0x584d5555;
	p_packet->ap_id = sys_flash_param.ap_param.ap_id;
	p_packet->len = size-12;
	p_packet->cmd = 0x014A;


	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);
	
	p_packet->year = rtc_date.Year;
	p_packet->month = rtc_date.Month;
	p_packet->day = rtc_date.Date;
	
	p_packet->hour = rtc_time.Hours;
	p_packet->min = rtc_time.Minutes;
	p_packet->sec = rtc_time.Seconds;
	p_packet->sensor_num = 0;
	
	p_sensor_stat = (struct_sensor_stat_timer *)(&p_packet->sensor_num + 1);
	
	
	for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
	{
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id != 0)
		{
			p_sensor_stat[p_packet->sensor_num].sensor_id = (lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id>>8) | (lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id<<8);
			p_sensor_stat[p_packet->sensor_num].battay = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.sensor_stat_packet.ucVolt*20;
			p_sensor_stat[p_packet->sensor_num].lost_bit = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.lost_rate;
			p_sensor_stat[p_packet->sensor_num].rssi = (int8_t)lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.avg_rssi1;
			p_sensor_stat[p_packet->sensor_num].packet_count = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.packet_count;
			
			if(p_sensor_stat[p_packet->sensor_num].lost_bit < 10)
				p_sensor_stat[p_packet->sensor_num].stat = 0;
			else if(p_sensor_stat[p_packet->sensor_num].lost_bit > 10)
			{
				p_sensor_stat[p_packet->sensor_num].stat = 0;		
				if(p_sensor_stat[p_packet->sensor_num].packet_count * p_sensor_stat[p_packet->sensor_num].lost_bit/100 > 10)
					p_sensor_stat[p_packet->sensor_num].stat = 2;
				else
					p_sensor_stat[p_packet->sensor_num].stat = 1;
			}
			if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].before.last_packet_time_slot > 7200*512)
				p_sensor_stat[p_packet->sensor_num].stat = 4;
			
			p_packet->sensor_num++;
		}
	
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id != 0)
		{
			p_sensor_stat[p_packet->sensor_num].sensor_id = (lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id>>8) | (lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id<<8);
			p_sensor_stat[p_packet->sensor_num].battay = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.ucVolt*20;
			p_sensor_stat[p_packet->sensor_num].lost_bit = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.lost_rate;
			p_sensor_stat[p_packet->sensor_num].rssi = (int8_t)lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.avg_rssi1;
			p_sensor_stat[p_packet->sensor_num].packet_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.packet_count;
			
			if(p_sensor_stat[p_packet->sensor_num].lost_bit < 10)
				p_sensor_stat[p_packet->sensor_num].stat = 0;
			else if(p_sensor_stat[p_packet->sensor_num].lost_bit > 10)
			{
				p_sensor_stat[p_packet->sensor_num].stat = 0;		
				if(p_sensor_stat[p_packet->sensor_num].packet_count * p_sensor_stat[p_packet->sensor_num].lost_bit/100 > 10)
					p_sensor_stat[p_packet->sensor_num].stat = 2;
				else
					p_sensor_stat[p_packet->sensor_num].stat = 1;
			}
			if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot > 7200*512)
				p_sensor_stat[p_packet->sensor_num].stat = 4;
			
			p_packet->sensor_num++;
		}		
	}
	rp_num = (uint8_t *)&p_sensor_stat[p_packet->sensor_num].sensor_id;
	*rp_num = 0;
	
	p_rp_stat = (struct_rp_stat_timer *)((uint32_t)&p_sensor_stat[p_packet->sensor_num].sensor_id + 1);
	
	for(index=0;index<RP_MAX_COUNT;index++)
	{
		if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id != 0)
		{
			p_rp_stat[*rp_num].rp_id = (lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id>>8) | (lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id<<8);
			p_rp_stat[*rp_num].battay = lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rp_stat_packet.ucVolt*20;
			p_rp_stat[*rp_num].rssi = (int8_t)lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rssi;
			if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot > 7200*512)
				p_rp_stat[*rp_num].stat = 4;	
			else if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot > 1800*512)
				p_rp_stat[*rp_num].stat = 3;	
			else if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot > 600*512)
				p_rp_stat[*rp_num].stat = 3;	
			else 
				p_rp_stat[*rp_num].stat = 0;	

			(*rp_num)++;
		}			
	}
	
	p_ap_stat = (struct_ap_stat_timer *)(&p_rp_stat[*rp_num].rp_id);
	p_ap_stat->ap_stat = battery_mv*6*3300/4096;
	p_ap_stat->a1 = 0;
	p_ap_stat->a2 = 0;
	
	
	size = sizeof(struct_to_server_head) + p_packet->sensor_num*sizeof(struct_sensor_stat_timer) + 1 + (*rp_num)*sizeof(struct_rp_stat_timer) + 
		sizeof(struct_ap_stat_timer);
	p_packet->len = size-12;
	crc = crc16(0,(uint8_t *)&p_packet->ap_id,size-4);
	
	((uint8_t *)(&p_ap_stat[1].ap_stat))[0] = (crc>>8)&0xff;
	((uint8_t *)(&p_ap_stat[1].ap_stat))[1] = crc;	
	
	add_4g_data_len(whitch_client,size+2);
	
	return 0;	
}


int32_t send_4g_timer_stat_qianfang(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_qianfang_stat_data_head *p_packet;
	struct_sensor_stat_timer *p_sensor_stat;
	uint16_t size;
	uint16_t crc;
	int8_t index;

	
	
	if(lane_to_sensor_info_and_result.has_lane_sensor_num > 100)
		lane_to_sensor_info_and_result.has_lane_sensor_num = 100;
	
	size = sizeof(struct_qianfang_stat_data_head) + lane_to_sensor_info_and_result.has_lane_sensor_num*sizeof(struct_sensor_stat_timer);
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	
	
	p_packet = (struct_qianfang_stat_data_head *)pdata_buff;
	p_packet->head = 0x584d5555;
	p_packet->n1_id = sys_flash_param.ap_param.ap_id;
	p_packet->len = size-12;
	p_packet->serial_number++;
	p_packet->cmd = 3;

	HAL_RTC_GetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);

	p_packet->year = rtc_date.Year;
	p_packet->month = rtc_date.Month;
	p_packet->day = rtc_date.Date;
	
	p_packet->hour = rtc_time.Hours;
	p_packet->min = rtc_time.Minutes;
	p_packet->sec = rtc_time.Seconds;
	p_packet->sensor_num = 0;
	
	p_sensor_stat = (struct_sensor_stat_timer *)(&p_packet->sensor_num + 1);
	
	
	for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
	{
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id != 0)
		{
			p_sensor_stat[p_packet->sensor_num].sensor_id = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id;
			p_sensor_stat[p_packet->sensor_num].battay = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.sensor_stat_packet.ucVolt;
			p_sensor_stat[p_packet->sensor_num].lost_bit = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.lost_rate;
			p_sensor_stat[p_packet->sensor_num].rssi = (int8_t)lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.avg_rssi;
			p_sensor_stat[p_packet->sensor_num].packet_count = lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.packet_count;
			
			if(p_sensor_stat[p_packet->sensor_num].lost_bit < 10)
				p_sensor_stat[p_packet->sensor_num].stat = 0;
			else if(p_sensor_stat[p_packet->sensor_num].lost_bit > 10)
			{
				p_sensor_stat[p_packet->sensor_num].stat = 0;		
				if(p_sensor_stat[p_packet->sensor_num].packet_count * p_sensor_stat[p_packet->sensor_num].lost_bit/100 > 10)
					p_sensor_stat[p_packet->sensor_num].stat = 2;
				else
					p_sensor_stat[p_packet->sensor_num].stat = 1;
			}
			if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].before.last_packet_time_slot > 7200*512)
				p_sensor_stat[p_packet->sensor_num].stat = 4;
			
			p_packet->sensor_num++;
		}
	
		if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id != 0)
		{
			p_sensor_stat[p_packet->sensor_num].sensor_id = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id;
			p_sensor_stat[p_packet->sensor_num].battay = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.ucVolt;
			p_sensor_stat[p_packet->sensor_num].lost_bit = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.lost_rate;
			p_sensor_stat[p_packet->sensor_num].rssi = (int8_t)lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.avg_rssi;
			p_sensor_stat[p_packet->sensor_num].packet_count = lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.packet_count;
			
			if(p_sensor_stat[p_packet->sensor_num].lost_bit < 10)
				p_sensor_stat[p_packet->sensor_num].stat = 0;
			else if(p_sensor_stat[p_packet->sensor_num].lost_bit > 10)
			{
				p_sensor_stat[p_packet->sensor_num].stat = 0;		
				if(p_sensor_stat[p_packet->sensor_num].packet_count * p_sensor_stat[p_packet->sensor_num].lost_bit/100 > 10)
					p_sensor_stat[p_packet->sensor_num].stat = 2;
				else
					p_sensor_stat[p_packet->sensor_num].stat = 1;
			}
			if(systerm_info.slot - lane_to_sensor_info_and_result.lane_and_sensor[index].after.last_packet_time_slot > 7200*512)
				p_sensor_stat[p_packet->sensor_num].stat = 4;
			
			p_packet->sensor_num++;
		}		
	}
	
	
	size = sizeof(struct_qianfang_stat_data_head) + p_packet->sensor_num*sizeof(struct_sensor_stat_timer) ;

	
	add_4g_data_len(whitch_client,size);
	
	return 0;	
}







//把sensor  rp 等系统参数加载到使用数据结构中
void load_sys_param_to_use_table()
{
	uint8_t i = 0;
	uint8_t index = 0;
	
	lane_to_sensor_info_and_result.has_lane_sensor_num = 0;
	lane_to_sensor_info_and_result.cfg_rp_num = 0;
	
	cfg_lane_num = 0;
	memset(&lane_to_sensor_info_and_result.lane_and_sensor[0],0,sizeof(lane_to_sensor_info_and_result.lane_and_sensor));
	for(i=0;i<sys_flash_param.sensor_num;i++)
	{
		index = sys_flash_param.sensor[i].sensor_param1.m_ucLaneId & 0x1f;
		index--;
		lane_to_sensor_info_and_result.has_lane_sensor_num++;
		if(sys_flash_param.sensor[i].sensor_param1.m_ucPosition) //后置
		{
			cfg_lane_num++;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id = sys_flash_param.sensor[i].sensor_param1.m_usDotId;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.lane_direction = sys_flash_param.sensor[i].sensor_param2.m_ucDirection;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.ccex = sys_flash_param.sensor[i].sensor_param1.m_ucCcexChannel;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.lane_type = sys_flash_param.sensor[i].sensor_param2.m_ucLaneType;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.m_innum = sys_flash_param.sensor[i].sensor_param2.m_innum;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.m_vLaneID = sys_flash_param.sensor[i].sensor_param2.m_vLaneID;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.rf_ch = sys_flash_param.sensor[i].sensor_param2.m_ucChannel;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.road_end_or_in = sys_flash_param.sensor[i].sensor_param1.m_ucSectionId;
		  lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.sensor_before_to_after_distance = sys_flash_param.sensor[i].sensor_param1.m_usDistance;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.up_rp_id = sys_flash_param.sensor[i].sensor_param2.m_ucpid;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.work_mode = sys_flash_param.sensor[i].sensor_param2.m_ucmode;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.slot = sys_flash_param.sensor[i].slot;
			lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_cfg.level = sys_flash_param.sensor[i].level;		
		}
		else
		{
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id = sys_flash_param.sensor[i].sensor_param1.m_usDotId;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.lane_direction = sys_flash_param.sensor[i].sensor_param2.m_ucDirection;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.ccex = sys_flash_param.sensor[i].sensor_param1.m_ucCcexChannel;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.lane_type = sys_flash_param.sensor[i].sensor_param2.m_ucLaneType;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.m_innum = sys_flash_param.sensor[i].sensor_param2.m_innum;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.m_vLaneID = sys_flash_param.sensor[i].sensor_param2.m_vLaneID;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.rf_ch = sys_flash_param.sensor[i].sensor_param2.m_ucChannel;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.road_end_or_in = sys_flash_param.sensor[i].sensor_param1.m_ucSectionId;
		  lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.sensor_before_to_after_distance = sys_flash_param.sensor[i].sensor_param1.m_usDistance;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.up_rp_id = sys_flash_param.sensor[i].sensor_param2.m_ucpid;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.work_mode = sys_flash_param.sensor[i].sensor_param2.m_ucmode;	
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.slot = sys_flash_param.sensor[i].slot;
			lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_cfg.level = sys_flash_param.sensor[i].level;
		}
		
		
	}
	memset(&lane_to_sensor_info_and_result.rp_cfg_and_stat[0],0,sizeof(lane_to_sensor_info_and_result.rp_cfg_and_stat));
	for(i=0;i<sys_flash_param.rp_num;i++)
	{
		if(i>=16)
			break;
		lane_to_sensor_info_and_result.cfg_rp_num++;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_id = sys_flash_param.rp[i].rp_param.m_usRpId;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.lane_direction = sys_flash_param.rp[i].rp_param.m_ucDirection;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.m_innum = sys_flash_param.rp[i].rp_param.m_innum;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.rf_ch = sys_flash_param.rp[i].rp_param.m_ucChannel;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.up_rp_id = sys_flash_param.rp[i].rp_param.m_ucpid;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.paraA.uiChannel = sys_flash_param.rp[i].paraA.uiChannel;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.paraA.uiGrade = sys_flash_param.rp[i].paraA.uiGrade;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.paraB.uimySlot = sys_flash_param.rp[i].paraB.uimySlot;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.paraB.uiSlotStateE = sys_flash_param.rp[i].paraB.uiSlotStateE;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.uiSlotStateH = sys_flash_param.rp[i].uiSlotStateH;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.uiSlotStateL = sys_flash_param.rp[i].uiSlotStateL;
		lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_cfg.uiSlotStateM = sys_flash_param.rp[i].uiSlotStateM;
		memset(&lane_to_sensor_info_and_result.rp_cfg_and_stat[i].rp_stat.rp_stat_packet,0,sizeof(SNP_STATE_PACKET_SENSOR_t));
	}

}


int32_t rev_sys_param(void *pdata,uint16_t len)
{
	
	struct_sys_flash_param *pd = (struct_sys_flash_param *)pdata;
	
	if(pd->ap_param.ap_version != AP_VERSION)
		return SYS_PARAM_TABLE_WRITE_ACK_F1;
	if(pd->sensor_num > 64 || pd->rp_num >16)
		return SYS_PARAM_TABLE_WRITE_ACK_F2;

	if(pd->global_cfg_param.realtime_data_type[0] != sys_flash_param.global_cfg_param.realtime_data_type[0])
		init_realtime_data_queue(0);

	if(pd->global_cfg_param.realtime_data_type[1] != sys_flash_param.global_cfg_param.realtime_data_type[1])
		init_realtime_data_queue(1);
	
	sys_flash_param.ap_param.ap_id = pd->ap_param.ap_id;
	memcpy(&sys_flash_param.ap_param.band_id,&pd->ap_param.band_id,22+sizeof(struct_global_cfg_param));
	memcpy(&sys_flash_param.sensor[0],&(pd->sensor[0]),pd->sensor_num*sizeof(s_sensor_pram));
	memcpy(&sys_flash_param.rp[0],(void *)((uint32_t)&(pd->sensor[0])+pd->sensor_num*sizeof(s_sensor_pram)),pd->rp_num*sizeof(s_rp_pram));
		
	ap_param_write_flash_flag = 1;
	
	load_sys_param_to_use_table();
	return SYS_PARAM_TABLE_WRITE_ACK_F0;
}



void start_updata_sensor(uint8_t *data,uint8_t data_len)
{
	uint8_t index,i;
#pragma pack(1)
	struct _sensor
	{
		uint8_t len;
		uint16_t sensor_id[1];
	}*p_sensor = (struct _sensor*)data;
#pragma pack()
	update_s_rp_manage.updata_s_rp_sendcmd_enable = 2;
	update_s_rp_manage.updata_s_rp_sendcmd_timeout = systerm_info.slot;
	
	if(p_sensor->len*2 == data_len)
	{
		for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
		{
			if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id != 0)
			{
				lane_to_sensor_info_and_result.lane_and_sensor[index].after.updata_enable = 0;
				for(i=0;i<p_sensor->len;i++)
				{
					if(p_sensor->sensor_id[i] == lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id)
					{
						lane_to_sensor_info_and_result.lane_and_sensor[index].after.updata_enable = 1;
						break;
					}
				}
			}
			if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id != 0)
			{
				lane_to_sensor_info_and_result.lane_and_sensor[index].before.updata_enable = 0;
				for(i=0;i<p_sensor->len;i++)
				{
					if(p_sensor->sensor_id[i] == lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id)
					{
						lane_to_sensor_info_and_result.lane_and_sensor[index].before.updata_enable = 1;
						break;
					}
				}
			}		
		}		
	}
	
	
}


void start_updata_rp(uint8_t *data,uint8_t data_len)
{
	uint8_t index,i;
#pragma pack(1)	
	struct _sensor
	{
		uint8_t len;
		uint16_t sensor_id[1];
	}*p_sensor = (struct _sensor*)data;
#pragma pack()	
	update_s_rp_manage.updata_s_rp_sendcmd_enable = 1;
	update_s_rp_manage.updata_s_rp_sendcmd_timeout = systerm_info.slot;	
	
	if(p_sensor->len*2 == data_len)
	{
		for(index=0;index<RP_MAX_COUNT;index++)
		{
			if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id != 0)
			{
				lane_to_sensor_info_and_result.rp_cfg_and_stat[index].updata_enable = 0;
				for(i=0;i<p_sensor->len;i++)
				{
					if(p_sensor->sensor_id[i] == lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id)
					{
						lane_to_sensor_info_and_result.rp_cfg_and_stat[index].updata_enable = 1;
						break;
					}
				}
			}
			
		}		
	}	
}


/*
解析4G服务器下发数据
	该函数接收到的数据为服务器下发的数据，可能包含多条或者不足一条

*/
int32_t server_4g_data_hanle(int32_t whitch_client)
{
	uint16_t crc,crc1;
	uint16_t index;
	struct_gprs_4g_packet_head *phead;
	uint8_t data_from_witch_client = -1;
	struct_client_data_buff *client_buff[3] = {&client_data_buff_0,&client_data_buff_1};
	uint8_t dtu_ack_r[9] = "dtu_ack_r";
	struct_dezhou_realtime_data *p_dz = 0;
	uint32_t i = 0;
	uint32_t *seq = 0;
	
	
	if(whitch_client<0 || whitch_client>1)
		return -1;
	while(1)
	{	
		for(index=0;index<client_buff[whitch_client]->index;index++)
		{
			phead = (struct_gprs_4g_packet_head *)&(client_buff[whitch_client]->buff[index]);
			if(phead->head == 0x584d5555)   //从数据buff中找协议头
			{
				if(phead->len+12 > GPRS_DATA_BUFF_LENGH) //协议中长度超过范围 数据作废
				{
					client_buff[whitch_client]->index = 0;
					return -2;					
				}
				else if(phead->len+12 <= client_buff[whitch_client]->index-index)  //一条协议数据长度小于总数据长度，说明存在一条完整的协议数据
				{
					index += phead->len+12;
					goto work;
				}
				else   //没有一条完整的协议数据，继续接收数据
				{
					if(index > 0)  //消除前面的废弃数据
					{
						memcpy(&(client_buff[whitch_client]->buff[0]),&(client_buff[whitch_client]->buff[index]),client_buff[whitch_client]->index-index);
					}
					return -1;
				}
			}
			if(memcmp(dtu_ack_r,&(client_buff[whitch_client]->buff[index]),9)== 0)
			{
				gprs_one_cmd.index = 0;
				memset(gprs_one_cmd.data,0,256);
				for(;index<client_buff[whitch_client]->index;index++)
				{
					gprs_one_cmd.data[gprs_one_cmd.index] = client_buff[whitch_client]->buff[index];
					gprs_one_cmd.index++;
					if(gprs_one_cmd.index>255)
					{
						gprs_one_cmd.index = 0;
						memset(gprs_one_cmd.data,0,256);
						break;
					}
				
					if(gprs_one_cmd.data[gprs_one_cmd.index-1] == '\n')
					{
						who_dtu_ack_r = whitch_client;
						dtu_cmd_param((char *)&gprs_one_cmd.data[0],gprs_one_cmd.index);
						memset(gprs_one_cmd.data,0,256);
						gprs_one_cmd.index = 0;			
					}	
				}				
			}
		}
		//没有找到协议头
		client_buff[whitch_client]->index = 0;
		return -1;
		


work:	
		crc = (phead->data[phead->len]<<8) + phead->data[phead->len+1];
		crc1 = crc16(0,(uint8_t *)&phead->ap_id,phead->len+6);
		if(gprs_print_rx_tx_data_enable)
		{
			sprintf(gprs_debug_buff,"4g_hanle_data: len=%d %d\r\n",index,crc-crc1);
			copy_string_to_double_buff(gprs_debug_buff);
		}
		if(crc == crc1)  //crc校验
		{		
			switch(phead->data[0]) //cmd 1
			{
				case 1:
					send_heart_flag[whitch_client].send_time = systerm_info.slot;
					send_heart_flag[whitch_client].send_flag = 0;
					send_heart_flag[whitch_client].send_times = 0;
					break;
				case 2:
						if(phead->len == 1)
							HAL_NVIC_SystemReset();
					break;
				case CMD1_4G_SYS_PARAM_TABLE:      //系统参数命令
					switch(phead->data[1])  //cmd 2
					{
						case CMD2_4G_SYS_PARAM_TABLE_WRITE:
							gprs_4g_task[whitch_client].sys_param_table_task_ack_value = rev_sys_param(&phead->data[2],phead->len-2);
							gprs_4g_task[whitch_client].sys_param_table_task = CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK;      //建立ack任务
							gprs_4g_task[whitch_client].sys_param_table_task_resend_times = 0;
							gprs_4g_task[whitch_client].sys_param_table_task_resend_timeout = 0;					
							break;
						case CMD2_4G_SYS_PARAM_TABLE_READ:
								gprs_4g_task[whitch_client].sys_param_table_task = CMD2_4G_SYS_PARAM_TABLE_WRITE;      //建立write任务
								gprs_4g_task[whitch_client].sys_param_table_task_resend_times = 0;
								gprs_4g_task[whitch_client].sys_param_table_task_resend_timeout = 0;					
							break;
						case CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK:   //收到服务器返回 系统参数表的ack
							if(gprs_4g_task[whitch_client].sys_param_table_task == CMD2_4G_SYS_PARAM_TABLE_WRITE)   //有发送系统参数表的任务
							{
								gprs_4g_task[whitch_client].sys_param_table_task = 0;      //任务完成 清掉标志
								gprs_4g_task[whitch_client].sys_param_table_task_resend_times = 0;
								gprs_4g_task[whitch_client].sys_param_table_task_resend_timeout = 0;
							}
							break;
						
					}
					break;
				case CMD1_4G_AUTO_CFG:         //自动配置命令集
					switch(phead->data[1])  //cmd 2
					{
						case CMD2_4G_START_AUTO_CFG:	 //启动自动配置			
								auto_cfg_stat[whitch_client].auto_cfg_switch++;
								auto_cfg_stat[whitch_client].auto_cfg_time = 0;
								phead = (struct_gprs_4g_packet_head *)&auto_cfg_stat[whitch_client].to_4g_data[0];

								phead->head = 0x584d5555;
								phead->ap_id = sys_flash_param.ap_param.ap_id;
								phead->len = 2;
								phead->data[0] = CMD1_4G_AUTO_CFG;
								phead->data[1] = CMD2_4G_START_ACK_BEGIN;
								crc = crc16(0,(uint8_t *)&phead->ap_id,8);
								phead->data[2] = (crc>>8)&0xff;
								phead->data[3] = crc&0xff;
								auto_cfg_stat[whitch_client].re_send_times = 0;
								auto_cfg_stat[whitch_client].send_to_4g_time = 0;
						break;
						case CMD2_4G_AUTO_CFG_SERVER_ACK: //收到服务器发送的ack  清除数据缓冲
								auto_cfg_stat[whitch_client].re_send_times = 0;
								auto_cfg_stat[whitch_client].send_to_4g_time = 0;
								auto_cfg_stat[whitch_client].to_4g_data[0] = 0;
						break;
						default: break;
					}
					break;
					case 0x41:   //德州协议服务器回复ack
						if(phead->data[1] == 1)
						{
							if(sys_flash_param.global_cfg_param.realtime_data_type[whitch_client] == RDT_DZ) //千方的数据长度
								{
									seq = (uint32_t *)(&(phead->data[2]));
									if(rtdata_send_buff_count[whitch_client]>0)
									{
										p_dz = (struct_dezhou_realtime_data *)rtdata_send_buff[whitch_client];
										if(rtdata_send_buff_count[whitch_client] > DZ_ARRAY_MAX_COUNT)  //长度异常  直接清零缓冲区
										{
											rtdata_send_buff_count[whitch_client] = 0;	
											return -2;
										}			
										for(i=0;i<rtdata_send_buff_count[whitch_client];i++)
										{
											if(p_dz[i].seq == *seq)
											{
												p_dz[i] = p_dz[rtdata_send_buff_count[whitch_client]-1];
												rtdata_send_buff_count[whitch_client]--;
												break;
											}
										}
									}							
								}	
						}
					break;
					case 0x4a:   //状态包 ack
						if(phead->data[1] == 1)
						{
							gprs_4g_task[whitch_client].timer_stat_task = 0;
						}
					break;
					case 0x48:   //流量包 ack
						if(phead->data[1] == 1)
						{
							gprs_4g_task[whitch_client].timer_data_task = 0;
						}
					break;	
					case CMD1_4G_DOWNLOAD_FIRMWARE:  //服务器下发 AP RP S固件
						switch(phead->data[1])
						{
							case CMD2_4G_AP_FIRMWARE:
								rev_4g_server_ap_firmware(whitch_client,&phead->data[2],phead->len-2);
								break;
							case CMD2_4G_RP_FIRMWARE:
								rev_4g_server_rp_firmware(whitch_client,&phead->data[2],phead->len-2);
								break;	
							case CMD2_4G_S_FIRMWARE:
								rev_4g_server_s_firmware(whitch_client,&phead->data[2],phead->len-2);
								break;		
							default: break;
						}
						break;
					case 0xab:    //服务器获取sensor rp 的升级状态信息
						send_4g_sensor_rp_updata_stat(whitch_client);
						break;
					case 0xa7:
						start_updata_sensor(&phead->data[1],phead->len-2);
						send_4g_sensor_rp_updata_stat(whitch_client);
						break;
					case 0xa8:
						start_updata_rp(&phead->data[1],phead->len-2);
					send_4g_sensor_rp_updata_stat(whitch_client);
						break;
				default: break;
			}
		}
		if(client_buff[whitch_client]->index-index > 0) //还有未处理完的数据
		{
			memcpy(&client_buff[whitch_client]->buff[0],&client_buff[whitch_client]->buff[index],client_buff[whitch_client]->index-index);
			client_buff[whitch_client]->index = client_buff[whitch_client]->index-index;
		}
		else
			client_buff[whitch_client]->index = 0;
	}
}


int32_t send_4g_firmware_ack(int32_t whitch_client,int32_t who_ack,uint32_t firmware_len,uint32_t now_len)
{
	struct_gprs_4g_packet_head *phead;
	struct_gprs_4g_firmware_data *p_firmware;
	uint8_t *pdata_buff;
	uint16_t crc;
	
	if(whitch_client > CLIENT_NUM)
		return -1;
	
	pdata_buff = get_4g_data_buff(0,22);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	

	phead = (struct_gprs_4g_packet_head *)pdata_buff;
	phead->head = 0x584d5555;
	phead->ap_id = sys_flash_param.ap_param.ap_id;
	phead->len = 10;
	phead->data[0] = CMD1_4G_DOWNLOAD_FIRMWARE;
	phead->data[1] = who_ack;	
	p_firmware = (struct_gprs_4g_firmware_data *)(&phead->data[2]);
	p_firmware->firmware_len = firmware_len;
	p_firmware->firmware_now_len = now_len;
	
	crc = crc16(0,(uint8_t *)&phead->ap_id,16);
	p_firmware->data[0] = (crc>>8)&0xff;
	p_firmware->data[1] = crc&0xff;	
	
	add_4g_data_len(0,22);
	
	return 0;
}


void rev_4g_server_ap_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len)
{
	struct_gprs_4g_firmware_data *p_firmware = (struct_gprs_4g_firmware_data *)pdata;
	static uint32_t now_firmware_len = 0;   //记录当前flash写了的固件长度
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	uint16_t crc;
	struct _ap_firmware
	{
		uint32_t head;
		uint32_t firmware_len;
		uint16_t version;
		uint16_t crc;
		uint8_t data[1];
	}*p_ap_firmware = (struct _ap_firmware*)FLASH_AP_FIRMWARE_BEGIN;
	
	if(p_firmware->firmware_now_len == 0)  //服务器下发第0包   如果本地不是第0包 则需要执行Flash擦除
	{
		//__disable_irq() ;  //关总中断
		HAL_FLASH_Unlock();
		EraseInit.Sector = FLASH_SECTOR_19;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);
		EraseInit.Sector = FLASH_SECTOR_20;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);		
		HAL_FLASH_Lock();
		//__enable_irq() ; //开总中断	
		now_firmware_len = 0;		
	}
	
	if(p_firmware->firmware_now_len == now_firmware_len)  //服务器下发的固件位置等于本地记录的位置  才会写flash
	{
		if(len < 1500)
		{
			if(write_bin_flash(FLASH_AP_FIRMWARE_BEGIN+now_firmware_len,&p_firmware->data[0],len-8) == 0) //写正确
			{
				now_firmware_len += len-8;
			}
			else
			{
				now_firmware_len = 0;
			}			
		}
	}
	
	
	//返回服务器ack   包含本地已经写到的固件位置， 便于服务器确认固件下发的对错，及时调整固件位置
	send_4g_firmware_ack(whitch_client,CMD2_4G_AP_FIRMWARE_ACK,p_firmware->firmware_len,now_firmware_len);	

	if(p_firmware->firmware_len == now_firmware_len)
	{
		crc = crc16(0,p_ap_firmware->data,p_ap_firmware->firmware_len);
		if(crc == p_ap_firmware->crc)
		{
			timer_reboot_system = systerm_info.slot + 5*512;
		}
	}	
	
}

void rev_4g_server_rp_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len)
{
	struct_gprs_4g_firmware_data *p_firmware = (struct_gprs_4g_firmware_data *)pdata;
	static uint32_t now_firmware_len = 0;	
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;	
	
	
	if(p_firmware->firmware_now_len == 0)  //服务器下发第0包   如果本地不是第0包 则需要执行Flash擦除
	{
		//__disable_irq() ;  //关总中断
		HAL_FLASH_Unlock();
		EraseInit.Sector = FLASH_SECTOR_17;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);
		HAL_FLASH_Lock();
		//__enable_irq() ; //开总中断			
		now_firmware_len = 0;
	}
	
	if(p_firmware->firmware_now_len == now_firmware_len)  //服务器下发的固件位置等于本地记录的位置  才会写flash
	{
		if(len < 1500)
		{
			if(write_bin_flash(FLASH_RP_FIRMWARE_BEGIN+now_firmware_len,&p_firmware->data[0],len-8) == 0) //写正确
			{
				now_firmware_len += len-8;
			}
			else
			{
				now_firmware_len = 0;
			}
		}
	}
	if(p_firmware->firmware_len == now_firmware_len)
	{
		read_firmware_rp_head();
	}	
	//返回服务器ack   包含本地已经写到的固件位置， 便于服务器确认固件下发的对错，及时调整固件位置
	send_4g_firmware_ack(whitch_client,CMD2_4G_RP_FIRMWARE_ACK,p_firmware->firmware_len,now_firmware_len);
	
}

void rev_4g_server_s_firmware(int32_t whitch_client,uint8_t *pdata, uint32_t len)
{
	struct_gprs_4g_firmware_data *p_firmware = (struct_gprs_4g_firmware_data *)pdata;
	static uint32_t now_firmware_len = 0;	
	FLASH_EraseInitTypeDef EraseInit;
	uint32_t SectorError	;
	
	
	if(p_firmware->firmware_now_len == 0)  //服务器下发第0包   如果本地不是第0包 则需要执行Flash擦除
	{
		//__disable_irq() ;  //关总中断
		HAL_FLASH_Unlock();
		EraseInit.Sector = FLASH_SECTOR_18;
		EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInit.Banks = FLASH_BANK_2;
		EraseInit.NbSectors = 1;		
		EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		HAL_FLASHEx_Erase(&EraseInit,&SectorError);
		HAL_FLASH_Lock();
		//__enable_irq() ; //开总中断	
		now_firmware_len = 0;
	}
	
	if(p_firmware->firmware_now_len == now_firmware_len)  //服务器下发的固件位置等于本地记录的位置  才会写flash
	{
		if(len < 1500)
		{
			if(write_bin_flash(FLASH_SENSOR_FIRMWARE_BEGIN+now_firmware_len,&p_firmware->data[0],len-8) == 0) //写正确
			{
				now_firmware_len += len-8;
			}
			else
			{
				now_firmware_len = 0;
			}
		}
	}
	if(p_firmware->firmware_len == now_firmware_len)
	{
		read_firmware_sensor_head();
	}
	
	//返回服务器ack   包含本地已经写到的固件位置， 便于服务器确认固件下发的对错，及时调整固件位置
	send_4g_firmware_ack(whitch_client,CMD2_4G_S_FIRMWARE_ACK,p_firmware->firmware_len,now_firmware_len);	
}





int32_t send_4g_timer_data(int32_t whitch_client)
{
	uint8_t *pdata_buff;
	struct_gprs_4g_heart_packet *p_packet;
	uint16_t size;
	uint16_t crc;
	
	
	size = sizeof(struct_gprs_4g_heart_packet);
	
	pdata_buff = get_4g_data_buff(whitch_client,size);
	if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
		return -1;	
	
	p_packet = (struct_gprs_4g_heart_packet *)pdata_buff;
	p_packet->head = 0x584d5555;
	p_packet->ap_id = sys_flash_param.ap_param.ap_id;
	p_packet->len = size-12;
	p_packet->cmd = 0x0101;
	p_packet->ap_live_time = systerm_info.slot/512;	
	p_packet->gps_n_e[0] = sys_flash_param.ap_param.gps_n_e[0];
	p_packet->gps_n_e[1] = sys_flash_param.ap_param.gps_n_e[1];

	crc = crc16(0,(uint8_t *)&p_packet->ap_id,size-6);
	p_packet->crc = 0;
	p_packet->crc |= (crc>>8)&0xff;
	p_packet->crc |= (8<<crc)&0xff00;	
	
	add_4g_data_len(whitch_client,size);
	
	return 0;
}







//轮训任务  用来处理4G关于系统参数表的任务
void poll_sysparam_4G_task()
{
	int i;
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		switch(gprs_4g_task[i].sys_param_table_task)
		{
			case CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK:			
				if(0 == send_4g_sys_param_ack(i))
				{
					gprs_4g_task[i].sys_param_table_task_resend_times--;
					if(gprs_4g_task[i].sys_param_table_task_resend_times<=0)
						gprs_4g_task[i].sys_param_table_task = 0;
				}		
				break;
			case CMD2_4G_SYS_PARAM_TABLE_WRITE:
				if(0 == send_4g_sys_param_table(i))
				{
					gprs_4g_task[i].sys_param_table_task_resend_times--;
					if(gprs_4g_task[i].sys_param_table_task_resend_times<=0)
						gprs_4g_task[i].sys_param_table_task = 0;
				}
				break;
			case CMD2_4G_SYS_PARAM_TABLE_READ:
				gprs_4g_task[i].sys_param_table_task = 0;
				break;
			default:
				break;		
		}
		if(gprs_4g_task[i].timer_data_task > 0 && systerm_info.slot - gprs_4g_task[i].timer_data_task_timeslot > 10*512)
		{
			if(send_4g_timer_data_sensor(i) == 0)
				gprs_4g_task[i].timer_data_task_timeslot = systerm_info.slot;
		}
		if(gprs_4g_task[i].timer_stat_qianfang_task > 0 && systerm_info.slot - gprs_4g_task[i].timer_stat_qianfang_task_timeslot > 10*512)
		{
			if(send_4g_timer_stat_qianfang(i) == 0)
				gprs_4g_task[i].timer_stat_qianfang_task_timeslot = systerm_info.slot;			
		}		
		if(gprs_4g_task[i].timer_stat_task > 0 && systerm_info.slot - gprs_4g_task[i].timer_stat_task_timeslot > 10*512)
		{
			if(send_4g_timer_stat_sensor(i) == 0)
				gprs_4g_task[i].timer_stat_task_timeslot = systerm_info.slot;			
		}		
	}
}



int32_t cmp_sensor_cgfgparam_and_realparam(uint16_t *pid)
{
	uint8_t j = 0;
	uint8_t choose_rp_index = 0xff;
	uint8_t zero_rp_index = 0xff;
	struct_lane_and_sensor *p_sensor;
	struct_lane_and_sensor *chose_p_sensor = 0;
	struct_lane_and_sensor *zero_p_sensor = 0;
	uint32_t time_slot = 0xffffffff;
	uint16_t set_param_times = 0xffff;
	uint8_t id_count = 1;
	
	wait_cfg_rp_num = 0;
	wait_cfg_s_num = 0;
	
	for(j=0;j<RP_MAX_COUNT;j++)
	{
		if(lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_id!= 0 && lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.rf_ch && (lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.paraA.uiChannel != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiChannel||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.paraA.uiGrade != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiGrade||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.rf_ch != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.sPhr.ucSensorMode||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.paraB.uimySlot != (lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiSlot & 0xff) ||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.paraB.uiSlotStateE != ((lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiSlot>>8) & 0xff) ||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.uiSlotStateH != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiSlotStateH ||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.uiSlotStateM != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiSlotStateM ||	
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_cfg.uiSlotStateL != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiSlotStateL ||	
			sys_flash_param.ap_param.band_id != lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_stat.rp_stat_packet.uiBindId ||
			lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot == 0))
		{
			wait_cfg_rp_num++;
			if(lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot < time_slot && 
				lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot != 0) //上来过，记录最早上来的索引
			{
				time_slot = lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot;
				choose_rp_index = j;
			}
			else if(lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot == 0 &&
				lane_to_sensor_info_and_result.rp_cfg_and_stat[j].set_param_times < set_param_times) //没有上来过 记录最少设置次数
			{
				set_param_times = lane_to_sensor_info_and_result.rp_cfg_and_stat[j].set_param_times;
				zero_rp_index = j;
			}
			if(pid != 0)
			{
				pid[id_count++] = lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_id;
			}
		}			
	}
	
	for(j=0;j<LANE_SENSOR_MAX_COUNT;j++)
	{
		p_sensor = &lane_to_sensor_info_and_result.lane_and_sensor[j].after;
		if(p_sensor->sensor_id != 0)
		{						
			if(p_sensor->sensor_cfg.level != p_sensor->sensor_stat.sensor_stat_packet.uiGrade ||
				p_sensor->sensor_cfg.rf_ch != p_sensor->sensor_stat.sensor_stat_packet.uiChannel||
				sys_flash_param.ap_param.band_id != p_sensor->sensor_stat.sensor_stat_packet.uiBindId ||
				p_sensor->sensor_cfg.work_mode != p_sensor->sensor_stat.sensor_stat_packet.sPhr.ucSensorMode||
				p_sensor->sensor_cfg.slot != p_sensor->sensor_stat.sensor_stat_packet.uiSlot||
				p_sensor->last_packet_time_slot == 0)
			{//参数不同 需要配置
				wait_cfg_s_num++;
				if(pid != 0)
				{
					pid[id_count++] = p_sensor->sensor_id;
				}
				if(p_sensor->last_packet_time_slot != 0) //上来过
				{
					if(chose_p_sensor == 0) //初始化 sensor指针
						chose_p_sensor = p_sensor;
					              //已经初始化过的   找到最先上来的
					if(p_sensor->last_packet_time_slot < chose_p_sensor->last_packet_time_slot)
						chose_p_sensor = p_sensor;
					
					if(chose_p_sensor->last_packet_time_slot < time_slot)
					{
						choose_rp_index = 0xff;
						time_slot = chose_p_sensor->last_packet_time_slot;						
					}
					
				}
				else  		//没有上来过			
				{
					if(zero_p_sensor == 0) //初始化 sensor指针
						zero_p_sensor = p_sensor;
					if(p_sensor->set_param_times < zero_p_sensor->set_param_times)
						zero_p_sensor = p_sensor;
					if(zero_p_sensor->set_param_times < set_param_times) //找到设置次数最小的 rp or sensor
					{
						zero_rp_index = 0xff;						
						set_param_times = zero_p_sensor->set_param_times;
					}
				}
			}
		}
		p_sensor = &lane_to_sensor_info_and_result.lane_and_sensor[j].before;
		if(p_sensor->sensor_id != 0)
		{			
			
			if(p_sensor->sensor_cfg.level != p_sensor->sensor_stat.sensor_stat_packet.uiGrade ||
				p_sensor->sensor_cfg.rf_ch != p_sensor->sensor_stat.sensor_stat_packet.uiChannel||
				sys_flash_param.ap_param.band_id != p_sensor->sensor_stat.sensor_stat_packet.uiBindId ||
				p_sensor->sensor_cfg.work_mode != p_sensor->sensor_stat.sensor_stat_packet.sPhr.ucSensorMode||
				p_sensor->sensor_cfg.slot != p_sensor->sensor_stat.sensor_stat_packet.uiSlot)
				{//参数不同 需要配置
					wait_cfg_s_num++;
					if(pid != 0)
					{
						pid[id_count++] = p_sensor->sensor_id;
					}					
					if(p_sensor->last_packet_time_slot != 0) //上来过
					{
					if(chose_p_sensor == 0) //初始化 sensor指针
						chose_p_sensor = p_sensor;
					              //已经初始化过的   找到最先上来的
					if(p_sensor->last_packet_time_slot < chose_p_sensor->last_packet_time_slot)
						chose_p_sensor = p_sensor;
					
					if(chose_p_sensor->last_packet_time_slot < time_slot)
					{
						choose_rp_index = 0xff;
						time_slot = chose_p_sensor->last_packet_time_slot;						
					}
						
					}
					else  		//没有上来过			
					{
						if(zero_p_sensor == 0) //初始化 sensor指针
							zero_p_sensor = p_sensor;
						if(zero_p_sensor->set_param_times < p_sensor->set_param_times)
							zero_p_sensor = p_sensor;
						if(zero_p_sensor->set_param_times < set_param_times) //找到设置次数最小的 rp or sensor
						{
							zero_rp_index = 0xff;						
							set_param_times = zero_p_sensor->set_param_times;
						}
					}
				}
		}		
			
	}	
	if(pid != 0)
	{
		pid[0] = id_count-1;
	}			
	
	if(choose_rp_index < RP_MAX_COUNT)  // RP上来过  且是最早的
	{
		sensor_rp_param.ParaFram.uiPoll = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_id;
		sensor_rp_param.ParaFram.uiCmd = 11;
		sensor_rp_param.ParaFram.uiBindId = sys_flash_param.ap_param.band_id;
		sensor_rp_param.ParaFram.uiSlotStateL = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.uiSlotStateL;
		sensor_rp_param.ParaFram.uiSlotStateM = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.uiSlotStateM;
		sensor_rp_param.ParaFram.uiSlotStateH = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.uiSlotStateH;
		sensor_rp_param.ParaFram.paraA.uiGrade = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.paraA.uiGrade;
		sensor_rp_param.ParaFram.paraA.uiChannel = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.paraA.uiChannel;
		sensor_rp_param.ParaFram.paraB.uimySlot = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.paraB.uimySlot;	
		sensor_rp_param.ParaFram.paraB.uiSlotStateE = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.paraB.uiSlotStateE;
		sensor_rp_param.ucSensorMode = lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].rp_cfg.rf_ch;	
		lane_to_sensor_info_and_result.rp_cfg_and_stat[choose_rp_index].last_packet_time_slot = systerm_info.slot;		
	}	
	else if(chose_p_sensor != 0)   //sensor上来过  且是最早的
	{
		sensor_rp_param.ParaFram.uiPoll = chose_p_sensor->sensor_id;
		sensor_rp_param.ParaFram.uiCmd = 11;
		sensor_rp_param.ParaFram.uiBindId = sys_flash_param.ap_param.band_id;	
		sensor_rp_param.ParaFram.paraA.uiGrade = chose_p_sensor->sensor_cfg.level;
		sensor_rp_param.ParaFram.paraA.uiChannel = chose_p_sensor->sensor_cfg.rf_ch;
		sensor_rp_param.ParaFram.paraB.uimySlot = chose_p_sensor->sensor_cfg.slot;
		sensor_rp_param.ucSensorMode = chose_p_sensor->sensor_cfg.work_mode;
		chose_p_sensor->last_packet_time_slot = systerm_info.slot;		
	}
	else if(zero_rp_index < RP_MAX_COUNT) //rp s 都没有上来过的  RP设置次数最少
	{
		sensor_rp_param.ParaFram.uiPoll = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_id;
		sensor_rp_param.ParaFram.uiCmd = 11;
		sensor_rp_param.ParaFram.uiBindId = sys_flash_param.ap_param.band_id;
		sensor_rp_param.ParaFram.uiSlotStateL = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.uiSlotStateL;
		sensor_rp_param.ParaFram.uiSlotStateM = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.uiSlotStateM;
		sensor_rp_param.ParaFram.uiSlotStateH = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.uiSlotStateH;
		sensor_rp_param.ParaFram.paraA.uiGrade = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.paraA.uiGrade;
		sensor_rp_param.ParaFram.paraA.uiChannel = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.paraA.uiChannel;
		sensor_rp_param.ParaFram.paraB.uimySlot = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.paraB.uimySlot;	
		sensor_rp_param.ParaFram.paraB.uiSlotStateE = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.paraB.uiSlotStateE;
		sensor_rp_param.ucSensorMode = lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].rp_cfg.rf_ch;	
		lane_to_sensor_info_and_result.rp_cfg_and_stat[zero_rp_index].set_param_times++;				
	}
	else if(zero_p_sensor != 0) //rp s都没有上来过的了  Sensor配置次数最少
	{
		sensor_rp_param.ParaFram.uiPoll = zero_p_sensor->sensor_id;
		sensor_rp_param.ParaFram.uiCmd = 11;
		sensor_rp_param.ParaFram.uiBindId = sys_flash_param.ap_param.band_id;	
		sensor_rp_param.ParaFram.paraA.uiGrade = zero_p_sensor->sensor_cfg.level;
		sensor_rp_param.ParaFram.paraA.uiChannel = zero_p_sensor->sensor_cfg.rf_ch;
		sensor_rp_param.ParaFram.paraB.uimySlot = zero_p_sensor->sensor_cfg.slot;
		sensor_rp_param.ucSensorMode = zero_p_sensor->sensor_cfg.work_mode;
		zero_p_sensor->set_param_times++;			
	}
	else
	{
		//实际参数和配置参数相同   配置完毕
		return -1;
	}
	return 0;
}



void auto_cfg_poll(void)
{
	uint8_t i = 0;
	uint8_t j = 0;
	int8_t flag = 0;
	uint8_t timeout = 0;
	struct_gprs_4g_packet_head *phead;
	uint16_t crc = 0;
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		if(auto_cfg_stat[i].auto_cfg_switch > 0)
		{
			if(auto_cfg_stat[i].auto_cfg_time == 0)  //首次开始自动配置 初始化超时时间
			{
				auto_cfg_stat[i].auto_cfg_time = systerm_info.slot;
				for(j=0;j<LANE_SENSOR_MAX_COUNT;j++)
				{
					if(lane_to_sensor_info_and_result.lane_and_sensor[j].after.sensor_id != 0)
					{
						lane_to_sensor_info_and_result.lane_and_sensor[j].after.last_packet_time_slot = 0;
						lane_to_sensor_info_and_result.lane_and_sensor[j].after.set_param_times = 0;
					}
					if(lane_to_sensor_info_and_result.lane_and_sensor[j].before.sensor_id != 0)
					{
						lane_to_sensor_info_and_result.lane_and_sensor[j].before.last_packet_time_slot = 0;
						lane_to_sensor_info_and_result.lane_and_sensor[j].before.set_param_times = 0;
					}					
				}
				for(j=0;j<RP_MAX_COUNT;j++)
				{
					if(lane_to_sensor_info_and_result.rp_cfg_and_stat[j].rp_id != 0)
					{
						lane_to_sensor_info_and_result.rp_cfg_and_stat[j].last_packet_time_slot = 0;
						lane_to_sensor_info_and_result.rp_cfg_and_stat[j].set_param_times = 0;					
					}
				}
			}
			flag++;
			
			if(systerm_info.slot - auto_cfg_stat[i].auto_cfg_time >  (AUTO_CFG_TIMEOUT * 512))
				timeout++;
		}
	}
	if(flag == 0)
		return;
	
	
	
	if(sensor_rp_param.ParaFram.uiCmd !=0)  //有新的snesor 或者 rp参数
		return;
	
	if(timeout)
	{
		for(i=0;i<CLIENT_NUM;i++)
		{
			if(auto_cfg_stat[i].auto_cfg_switch > 0)
			{
				phead = (struct_gprs_4g_packet_head *)&auto_cfg_stat[i].to_4g_data[0];

				phead->head = 0x584d5555;
				phead->ap_id = sys_flash_param.ap_param.ap_id;
				phead->len = 2;
				phead->data[0] = CMD1_4G_AUTO_CFG;
				phead->data[1] = CMD2_4G_AUTO_CFG_TIMEOUT;
				cmp_sensor_cgfgparam_and_realparam((uint16_t *)(&phead->data[2]));
				phead->len += ((uint16_t *)(&phead->data[2]))[0]*2 + 2;
				crc = crc16(0,(uint8_t *)&phead->ap_id,phead->len+6);
				phead->data[phead->len] = (crc>>8)&0xff;
				phead->data[phead->len+1] = crc&0xff;
				auto_cfg_stat[i].auto_cfg_switch = 0;
				auto_cfg_stat[i].re_send_times = 0;
				auto_cfg_stat[i].send_to_4g_time = 0;
				
			}
		}
	}
	else if(0 == cmp_sensor_cgfgparam_and_realparam(0))
		return;
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		if(auto_cfg_stat[i].auto_cfg_switch > 0)
		{
			phead = (struct_gprs_4g_packet_head *)&auto_cfg_stat[i].to_4g_data[0];

			phead->head = 0x584d5555;
			phead->ap_id = sys_flash_param.ap_param.ap_id;
			phead->len = 2;
			phead->data[0] = CMD1_4G_AUTO_CFG;
			phead->data[1] = CMD2_4G_AUTO_CFG_END;
			crc = crc16(0,(uint8_t *)&phead->ap_id,phead->len+6);
			phead->data[phead->len] = (crc>>8)&0xff;
			phead->data[phead->len+1] = crc&0xff;
			auto_cfg_stat[i].auto_cfg_switch = 0;
			auto_cfg_stat[i].re_send_times = 0;
			auto_cfg_stat[i].send_to_4g_time = 0;			
		}
	}	
}

void poll_auto_cfg_4G_task()
{
	uint32_t i;
	struct_gprs_4g_packet_head *phead;	
	uint16_t crc;
	uint8_t *pdata_buff;
	
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		phead = (struct_gprs_4g_packet_head *)&auto_cfg_stat[i].to_4g_data[0];
		if(phead->head == 0x584d5555 && (systerm_info.slot - auto_cfg_stat[i].send_to_4g_time) > (5*512) )
		{
			crc = (phead->data[phead->len]<<8) + phead->data[phead->len+1];			
			if(crc != crc16(0,(uint8_t *)&phead->ap_id,phead->len+6))
			{
				phead->head = 0;
				continue;
			}
			pdata_buff = get_4g_data_buff(i,phead->len+12);
			if(pdata_buff == (uint8_t *)0 || pdata_buff == (uint8_t *)0xffffffff)
				continue;
			memcpy(pdata_buff,&auto_cfg_stat[i].to_4g_data[0],phead->len+12);
			add_4g_data_len(i,phead->len+12);
			auto_cfg_stat[i].send_to_4g_time = systerm_info.slot;
			auto_cfg_stat[i].re_send_times++;
			if(auto_cfg_stat[i].re_send_times > 20)
			{
				phead->head = 0;
				auto_cfg_stat[i].re_send_times = 0;
				auto_cfg_stat[i].send_to_4g_time = 0;
			}
		}
	}
}


void gprs_4g_heart()
{
	uint8_t i;
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		if(send_heart_flag[i].send_flag != 1)
		{
			if(0 == heart_timeout(i))
			{
				send_heart_flag[i].send_flag = 1;
				send_heart_flag[i].send_time = 0;
			}
		}
		if(((send_heart_flag[i].send_flag == 1) && (systerm_info.slot - send_heart_flag[i].send_time > 10*512)) || (systerm_info.slot - send_heart_flag[i].send_time>300*512))
		{
			send_heart_flag[i].send_time = systerm_info.slot;
			send_4g_heart_pacekt(i);
			send_heart_flag[i].send_times++;
			if(send_heart_flag[i].send_times > 3)
			{
				send_heart_flag[i].send_time = systerm_info.slot;
				send_heart_flag[i].send_flag = 0;
				send_heart_flag[i].send_times = 0;
			}
		}
	}
	
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	battery_mv = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}

void gprs_4g_task_poll()
{
	static uint32_t timeslot = 0;
	poll_sysparam_4G_task();
	poll_auto_cfg_4G_task();
	auto_cfg_poll();                   //自动配置轮训
	gprs_4g_heart();                   //轮训心跳包

	poll_to_4g_realtime_data();        //轮训发送实时数据


	sensor_data_and_stat_timer_task(); //结算定时数据 使能4G发送标志  内部自动实现1秒调用一次
	sensor_rp_updata_manage();
}
