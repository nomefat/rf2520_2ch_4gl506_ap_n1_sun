#include "stm32f4xx_hal.h"
#include "snp.h"
#include "typedef_struct.h"
#include "ap_param.h"
#include "rf_hal.h"
#include "string.h"
#include "to_n1.h"
#include "eeprom.h"
#include "debug.h"
#include "update_s_rp.h"
#include "typedef_struct.h"
#include "debug_uart.h"




extern struct_rf_stat rf_stat[4];
extern SNP_SYNC_PACKET_t syn_packet;
extern struct_systerm_info systerm_info;




#define FIRM_RP        0x01  
#define FIRM_SENSOR    0x02




struct_update_s_rp_manage update_s_rp_manage ;


void rf_send_updata_packet(uint8_t s_or_rp,uint16_t flash_seq);


//记录和更新已经进入更新模式的sensor或者rp
void get_s_rp_input_update_stat(uint16_t dev_id, uint16_t flash_times)
{
	uint8_t i = 0;
	
	if(update_s_rp_manage.dev_num>=64)
		return;

//	if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq == 0)
//		update_s_rp_manage.upadate_s_rp_enable = 1;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)     //如果设备已经在表中了，更新设备参数
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_id == dev_id)
		{
			update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = flash_times;
			return;
		}
	}
	
	//设备没有在记录中
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_id = dev_id;
	update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num].dev_packet_seq = flash_times;
	update_s_rp_manage.dev_num++;
	

}


int32_t check_update_s_rp_list(void)
{
	int i = 0;
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq < update_s_rp_manage.now_upadate_packet_seq) //设备上传的包序号小于AP的包序号，说明设备已经掉队
		{
			if(update_s_rp_manage.dev_num>1) //大于一个设备 把末尾的设备放在这里
			{
				update_s_rp_manage.update_s_rp_list[i].dev_id = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_id ;
				update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = update_s_rp_manage.update_s_rp_list[update_s_rp_manage.dev_num-1].dev_packet_seq ;
				update_s_rp_manage.dev_num--;
			}
			else //等于1个设备 直接清零
			{
				update_s_rp_manage.update_s_rp_list[i].dev_id = 0 ;
				update_s_rp_manage.update_s_rp_list[i].dev_packet_seq = 0 ;				
				update_s_rp_manage.dev_num = 0;
				update_s_rp_manage.now_upadate_packet_seq = 0;
				update_s_rp_manage.now_send_times = 0;
			}
		}
	}
	
	for(i=0;i<update_s_rp_manage.dev_num;i++)
	{
		if(update_s_rp_manage.update_s_rp_list[i].dev_packet_seq <= update_s_rp_manage.now_upadate_packet_seq) //设备上传的包序号小于AP的包序号，说明设备已经掉队
		{
				return -1; //有一个没有完成ack
		}
	}
	
	return 0;  // 所有的都完成了ack
}

extern int enable_print_updata_flag;
extern char gprs_debug_buff[];
extern SNP_UF_DATA_PACKET_t upadate_packet;
extern int rf1_flush_rx ;
extern int rf2_flush_rx ;
extern int rf3_flush_rx ;
extern int rf4_flush_rx ;
int enable_test_send_updata_packet = 0;
/*
	函数：
	功能： 更新时间槽调用该函数 ，负责发送更新RP SENSOR的固件

*/
void rf_send_update_packet(void)
{

	
	if(update_s_rp_manage.upadate_s_rp_enable !=0) //如果已经使能更新
	{
		if(update_s_rp_manage.dev_num == 0 && enable_test_send_updata_packet == 0)     //如果已经使能更新　但是没有设备进入，取消更新使能
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.upadate_s_rp_enable = 0;
			update_s_rp_manage.now_upadate_packet_seq = 0;
			return;
		}
		if(check_update_s_rp_list()==0 || enable_test_send_updata_packet != 0) //全部收到ａｃｋ　进行下一包
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq++;
		}
		//发送一包数据
		rf_send_updata_packet(update_s_rp_manage.upadate_s_rp_enable,update_s_rp_manage.now_upadate_packet_seq);
		
		if(enable_print_updata_flag)
		{
			sprintf(gprs_debug_buff,"updata: %d %d %d %d seq=%d addr=%08X times=%d\r\n",rf1_flush_rx,rf2_flush_rx,rf3_flush_rx,rf4_flush_rx,update_s_rp_manage.now_upadate_packet_seq,upadate_packet.uiAddress,update_s_rp_manage.now_send_times);
			copy_string_to_double_buff(gprs_debug_buff);
	
		}
		
		update_s_rp_manage.now_send_times++;   //发送次数增加
		
		if(update_s_rp_manage.now_send_times>ONE_PACKET_SEND_TIMES_MAX)    //大于指定发送次数　发送包需要增加
		{
			update_s_rp_manage.now_send_times = 0;
			update_s_rp_manage.now_upadate_packet_seq++;
		}
	}

}





void enable_sensor_update()
{
	update_s_rp_manage.now_send_times = 0;
	update_s_rp_manage.upadate_s_rp_enable = FIRM_SENSOR;
	update_s_rp_manage.now_upadate_packet_seq = 0;	
}


void enable_rp_update()
{
	update_s_rp_manage.now_send_times = 0;
	update_s_rp_manage.upadate_s_rp_enable = FIRM_RP;
	update_s_rp_manage.now_upadate_packet_seq = 0;	
	
}



void sensor_rp_updata_manage()
{
	uint8_t index;
	uint8_t send_times_min_id_index = 0xff;
	uint8_t before_or_after = 0xff;
	uint8_t sec;
	uint8_t dev_sec = 0;
	uint8_t *p_char = 0;
	uint8_t flag = 0;
	
	sec = syn_packet.ucCurSecNr + 1;
	if(sec > 29)
		sec = 0;
	
	if(update_s_rp_manage.updata_s_rp_sendcmd_enable == 2)
	{

		if(sensor_rp_param.ParaFram.uiCmd ==0)
		{
			for(index=0;index<LANE_SENSOR_MAX_COUNT;index++)
			{
				if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id != 0 && lane_to_sensor_info_and_result.lane_and_sensor[index].after.updata_enable>0 &&
					lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_stat.sensor_stat_packet.sPhr.ucType != SNP_PACKET_TYPE_SEN_UF_STATE)
				{
					flag++;
					p_char = (uint8_t *)&lane_to_sensor_info_and_result.lane_and_sensor[index].after.sensor_id;
					dev_sec = (p_char[0] + p_char[1])%30;
					if(sec == dev_sec)
					{
						if(send_times_min_id_index == 0xff)
						{
							send_times_min_id_index = index;
							before_or_after = 0;
						}
						else
						{
							if(before_or_after == 0)
							{
								if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.updata_enable < lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].after.updata_enable)
									send_times_min_id_index = index;
							}	
							else
							{
								if(lane_to_sensor_info_and_result.lane_and_sensor[index].after.updata_enable < lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].before.updata_enable)
								{
									send_times_min_id_index = index;	
									before_or_after = 0;
								}									
							}
						}
					}
				}
				
				if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id != 0 && lane_to_sensor_info_and_result.lane_and_sensor[index].before.updata_enable>0 &&
					lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_stat.sensor_stat_packet.sPhr.ucType != SNP_PACKET_TYPE_SEN_UF_STATE)
				{
					flag++;
					p_char = (uint8_t *)&lane_to_sensor_info_and_result.lane_and_sensor[index].before.sensor_id;
					dev_sec = (p_char[0] + p_char[1])%30;
					if(sec == dev_sec)
					{
						if(send_times_min_id_index == 0xff)
						{
							send_times_min_id_index = index;
							before_or_after = 1;
						}
						else
						{
							if(before_or_after == 1)
							{
								if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.updata_enable < lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].before.updata_enable)
									send_times_min_id_index = index;
							}	
							else
							{
								if(lane_to_sensor_info_and_result.lane_and_sensor[index].before.updata_enable < lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].after.updata_enable)
								{
									send_times_min_id_index = index;	
									before_or_after = 1;
								}									
							}
						}
					}
				}				
				
			}
			if(systerm_info.slot - update_s_rp_manage.updata_s_rp_sendcmd_timeout > 250*512 || flag ==0)
			{
				update_s_rp_manage.updata_s_rp_sendcmd_enable = 0;
				enable_sensor_update();
				return;
			}			
			if(send_times_min_id_index!=0xff)
			{
				if(before_or_after ==0)
				{
					sensor_rp_param.ParaFram.uiCmd = 7;
					sensor_rp_param.ParaFram.uiPoll = lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].after.sensor_id;
					lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].after.updata_enable++;
				}
				else
				{
					sensor_rp_param.ParaFram.uiCmd = 7;
					sensor_rp_param.ParaFram.uiPoll = lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].before.sensor_id;	
					lane_to_sensor_info_and_result.lane_and_sensor[send_times_min_id_index].before.updata_enable++;					
				}
			}
		}
	}
	else if(update_s_rp_manage.updata_s_rp_sendcmd_enable == 1)
	{

		if(sensor_rp_param.ParaFram.uiCmd ==0)
		{
			for(index=0;index<RP_MAX_COUNT;index++)
			{
				if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id != 0 && lane_to_sensor_info_and_result.rp_cfg_and_stat[index].updata_enable>0 &&
					lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_stat.rp_stat_packet.sPhr.ucType != SNP_PACKET_TYPE_RP_UF_STATE)
				{			
					flag++;
					p_char = (uint8_t *)&lane_to_sensor_info_and_result.rp_cfg_and_stat[index].rp_id;
					dev_sec = (p_char[0] + p_char[1])%30;
					if(sec == dev_sec)
					{
						if(send_times_min_id_index == 0xff)
						{
							send_times_min_id_index = index;
						}
						else
						{
							if(lane_to_sensor_info_and_result.rp_cfg_and_stat[index].updata_enable < lane_to_sensor_info_and_result.rp_cfg_and_stat[send_times_min_id_index].updata_enable)
								send_times_min_id_index = index;
						}
					}
				}
			}
			if(systerm_info.slot - update_s_rp_manage.updata_s_rp_sendcmd_timeout > 250*512|| flag ==0)
			{
				update_s_rp_manage.updata_s_rp_sendcmd_enable = 0;
				enable_rp_update();
				return;
			}			
			if(send_times_min_id_index!=0xff)
			{
				sensor_rp_param.ParaFram.uiCmd = 7;
				sensor_rp_param.ParaFram.uiPoll = lane_to_sensor_info_and_result.rp_cfg_and_stat[send_times_min_id_index].rp_id;
				lane_to_sensor_info_and_result.rp_cfg_and_stat[send_times_min_id_index].updata_enable++;
			}
			
		}
	}
		
}












