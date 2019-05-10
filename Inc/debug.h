#ifndef _DEBUG_H_
#define _DEBUG_H_




#define SENSOR_NUM       256



typedef struct _sensor_event_list
{
	
	unsigned short sensor_id;          //设备ＩＤ
	unsigned char slot;                  //时间槽
	signed char rssi;                  //信号强度
	unsigned char syn;                 //包序号
	unsigned char last_repeat_times;
	unsigned int all_packet_count;        //总包数
	unsigned int receive_packet_count;    //接收到的包数
	unsigned int repeat_packet_count;     //重复包数
	unsigned char repeat_packet_times_count[10];
	
}struct_sensor_event_list;




void debug_insert_sensor_event(unsigned short id,unsigned char syn,signed char rssi,unsigned char slot,unsigned char repeat_times);



void re_start_sensor_event_record();

void debug_sensor_event_to_str();

int copy_string_to_double_buff(char *pstr);



#endif

