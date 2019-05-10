#ifndef _UPDATE_S_RP_H_
#define _UPDATE_S_RP_H_

#include "stm32f4xx_hal.h"






#define S_RP_COUNT 64                       //最大管理64个设备同时升级
#define ONE_PACKET_SEND_TIMES_MAX 50        //同一个更新包重发次数  

typedef struct _update_s_rp_list
{
	uint16_t dev_id;                      //设备ID
	uint16_t dev_packet_seq;              //设备更新到第几包了
}struct_update_s_rp_list;            



typedef struct _update_s_rp_manage
{
	uint8_t updata_s_rp_sendcmd_enable;
	uint32_t updata_s_rp_sendcmd_timeout;
	uint8_t upadate_s_rp_enable;
	uint16_t now_upadate_packet_seq;    //现在升级到第几包
	uint16_t now_send_times;            //一个升级包发送的次数
	uint8_t dev_num;
	struct_update_s_rp_list update_s_rp_list [S_RP_COUNT];  //进入升级状态的S或者RP

}struct_update_s_rp_manage;




void enable_sensor_update();
void enable_rp_update();


#endif


