#ifndef GPRS_4G_APP_H_
#define GPRS_4G_APP_H_


#include "stm32f4xx_hal.h"


#define CLIENT_NUM 2
#define GPRS_DATA_BUFF_LENGH (1024 + 800)      //用于从队列中取出完整的一条命令或者数据 (经过实际测试 可能含有几条命令)

#pragma pack(1)


struct _gps_date
{
	uint16_t year;
	uint8_t month;
	uint8_t date;
	uint8_t hour;
	uint8_t minter;
	uint8_t sec;
};

typedef struct _gprs_4g_packet
{
		uint32_t head;
		uint32_t ap_id;
		uint16_t len;
		uint8_t data[1];
}struct_gprs_4g_packet_head;

typedef struct _gprs_4g_heart_packet
{
		uint32_t head;
		uint32_t ap_id;
		uint16_t len;
		uint16_t cmd;                         //命令字
		float gps_n_e[2];                  //gps
		struct _gps_date rtc_date_time;                    //rtc时间
		uint32_t ap_live_time;                //AP存活时间
		uint32_t client0_send_bytes;
	  uint32_t client1_send_bytes;
		int8_t gprs_rssi;                    //4G信号强度
		struct
		{
			uint32_t id1;
			uint32_t id2;
			uint32_t id3;
		}chip_id;
		uint16_t battery_mv;
		uint16_t crc;
}struct_gprs_4g_heart_packet;

typedef struct _gprs_4g_firmware_data
{
	uint32_t firmware_len;         //固件总长度
	uint32_t firmware_now_len;     //固件当前包的长度
	uint8_t data[1];
}struct_gprs_4g_firmware_data;



#pragma pack()




typedef struct {
	uint32_t index;
	uint8_t buff[GPRS_DATA_BUFF_LENGH];   //用于从队列中取出完整的一条命令或者数据 
}struct_client_data_buff;



#define  CMD1_4G_SYS_PARAM_TABLE             0XB0     //4g传输命令字1  系统参数表
#define  CMD2_4G_SYS_PARAM_TABLE_WRITE       0X01     //4g传输命令字2  写系统参数表
#define  CMD2_4G_SYS_PARAM_TABLE_READ        0X02     //4g传输命令字2  读系统参数表
#define  CMD2_4G_SYS_PARAM_TABLE_WRITE_ACK   0x03     //4g传输命令字2  写表ACK
#define  SYS_PARAM_TABLE_WRITE_ACK_F1        -1
#define  SYS_PARAM_TABLE_WRITE_ACK_F2        -2
#define  SYS_PARAM_TABLE_WRITE_ACK_F0        0

#define CMD1_4G_AUTO_CFG                      0XB1      //4G命令字1 自动配置命令集
#define CMD2_4G_START_AUTO_CFG                0X01      //4G命令字2 启动自动配置  (服务器发送)
#define CMD2_4G_START_ACK_BEGIN               0X02      //4G命令字2 开启自动配置 (AP发送)
#define CMD2_4G_AUTO_CFG_END                  0X03      //4G命令字2 配置完成
#define CMD2_4G_AUTO_CFG_TIMEOUT              0X04      //4G命令字2 配置超时未完成
#define CMD2_4G_AUTO_CFG_SERVER_ACK           0XF0      //4G命令字2 服务器收到数据之后发个ACK


#define CMD1_4G_DOWNLOAD_FIRMWARE             0XB2      //4G命令字1 服务器下发固件命令集
#define CMD2_4G_AP_FIRMWARE                   0X01      //4G命令字2 AP固件  (服务器发送)
#define CMD2_4G_RP_FIRMWARE                   0X02      //4G命令字2 RP固件   (服务器发送)
#define CMD2_4G_S_FIRMWARE                    0X03      //4G命令字2 Sensor固件 (服务器发送)
#define CMD2_4G_AP_FIRMWARE_ACK               0X11      //4G命令字2 ap固件ack (服务器发送)
#define CMD2_4G_RP_FIRMWARE_ACK               0X12      //4G命令字2 ap固件ack (服务器发送)
#define CMD2_4G_S_FIRMWARE_ACK                0X13      //4G命令字2 ap固件ack (服务器发送)


#define CMD1_4G_UPDATA_SENSOR_RP              0XB3      //4G命令字1 Sensor RP 固件升级命令集
#define CMD2_4G_UPDATA_SENSOR_RP_STAT         0X01      //4G命令字2 发送S RP升级状态




#define GPRS_4G_RESEND_TIME  5000  //毫秒

typedef struct _gprs_4g_task
{
	int8_t sys_param_table_task;
	int8_t sys_param_table_task_resend_times;
	int8_t sys_param_table_task_resend_timeout; //5s 重发间隔
	int8_t sys_param_table_task_ack_value; 
	
	uint16_t timer_data_task;  //定时流量任务
	uint16_t timer_stat_task;  //定时状态任务
	uint16_t timer_stat_qianfang_task; //定时千方状态任务
	
	uint32_t timer_data_task_timeslot;  //定时流量任务发送时间点
	uint32_t timer_stat_task_timeslot;  //定时状态任务发送时间点
	uint32_t timer_stat_qianfang_task_timeslot; //定时千方状态任务	发送时间点
	
}struct_gprs_4g_task;








typedef struct _auto_cfg
{
	uint8_t auto_cfg_switch;   //自动配置开关
	uint32_t auto_cfg_time;    //自动配置计时
	uint32_t send_to_4g_time;  //记录发送的时间点 用作超时重发
	uint16_t re_send_times;    //重发计数
	#define AUTO_CFG_TIMEOUT    600  //  超时时间  秒
	uint8_t to_4g_data[156];
	
}struct_auto_cfg;




extern struct_client_data_buff client_data_buff_0;
extern struct_client_data_buff client_data_buff_1;

int32_t server_4g_data_hanle(int32_t whitch_client);
void gprs_4g_task_poll();
void load_sys_param_to_use_table();

int32_t send_4g_heart_pacekt(int32_t whitch_client);



#endif


