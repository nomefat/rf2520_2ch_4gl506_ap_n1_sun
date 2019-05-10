#include "debug.h"
#include "string.h"





struct_sensor_event_list sensor_event_list[SENSOR_NUM];

char debug_sensor_event_str[SENSOR_NUM][43+50];

void debug_insert_sensor_event(unsigned short id,unsigned char syn,signed char rssi,unsigned char slot,unsigned char repeat_times)
{
	int i = 0;
	signed char p1 = 0;
	unsigned short sid = 0;
	
	sid = id>>8;
	sid += id<<8;
		
	for(i=0;i<SENSOR_NUM;i++)
	{
		 //列表中找到该ＩＤ的数据
		if(sid == sensor_event_list[i].sensor_id || sensor_event_list[i].sensor_id == 0 )
			break;
	}
	
	if(i >= SENSOR_NUM) //列表已经满了
		return;
	
	sensor_event_list[i].sensor_id = sid;
	sensor_event_list[i].rssi = rssi;
	sensor_event_list[i].slot = slot;
	
	if(repeat_times == 0)  //当前sensor表示重发次数是０　此时结算上一包的重发次数　表示上一包数据重发了多少次
	{
		if(sensor_event_list[i].last_repeat_times > 0)
		{
			if(sensor_event_list[i].last_repeat_times<10) //重发10次以内的对应累加　大于１０次的放一起累加
			{
				sensor_event_list[i].repeat_packet_times_count[sensor_event_list[i].last_repeat_times-1]++;
			}
			else
				sensor_event_list[i].repeat_packet_times_count[9]++;
		}
	}
	sensor_event_list[i].last_repeat_times = repeat_times;
	
	if(sensor_event_list[i].all_packet_count != 0)
	{
		p1 = syn - sensor_event_list[i].syn;
		if(p1 <0)
			p1 += 0xff;
	}
	else
		p1 = 1;

	if(sensor_event_list[i].syn == syn)
		sensor_event_list[i].repeat_packet_count++;
	
	sensor_event_list[i].syn = syn;
	sensor_event_list[i].receive_packet_count +=1;
	sensor_event_list[i].all_packet_count += p1;
	
	
}





void re_start_sensor_event_record()
{
	memset(sensor_event_list,0,sizeof(struct_sensor_event_list)*SENSOR_NUM);
}


void debug_sensor_event_to_str()
{
	int i = 0;
	int j = 0;
	int index = 0;
	
	char hex_str[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	
	memcpy(&debug_sensor_event_str[0][0]	,"s_id slot rssi all_count rev_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10\r\n",sizeof("s_id slot rssi all_count rev_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10\r\n"));
	index += sizeof("s_id slot rssi all_count rev_count repeat r_01 r_02 r_03 r_04 r_05 r_06 r_07 r_08 r_09 r_10\r\n");
	
	for(i=0;i<SENSOR_NUM;i++)
	{
		if(sensor_event_list[i].sensor_id == 0)
			break;
		
		debug_sensor_event_str[i+1][0] = hex_str[(sensor_event_list[i].sensor_id>>12)];
		debug_sensor_event_str[i+1][1] = hex_str[(sensor_event_list[i].sensor_id>>8) & 0x0f];		
		debug_sensor_event_str[i+1][2] = hex_str[(sensor_event_list[i].sensor_id &0xff)>>4];
		debug_sensor_event_str[i+1][3] = hex_str[(sensor_event_list[i].sensor_id & 0x000f)];			
		debug_sensor_event_str[i+1][4] = ' ';

		debug_sensor_event_str[i+1][5] = ' ';
		debug_sensor_event_str[i+1][6] = hex_str[sensor_event_list[i].slot/100];
		debug_sensor_event_str[i+1][7] = hex_str[sensor_event_list[i].slot%100/10];		
		debug_sensor_event_str[i+1][8] = hex_str[sensor_event_list[i].slot%10];	
		debug_sensor_event_str[i+1][9] = ' ';	
		
		debug_sensor_event_str[i+1][10] = '-';
		debug_sensor_event_str[i+1][11] = hex_str[-sensor_event_list[i].rssi/100];
		debug_sensor_event_str[i+1][12] = hex_str[-sensor_event_list[i].rssi%100/10];		
		debug_sensor_event_str[i+1][13] = hex_str[-sensor_event_list[i].rssi%10];	
		debug_sensor_event_str[i+1][14] = ' ';	

		debug_sensor_event_str[i+1][15] = hex_str[sensor_event_list[i].all_packet_count/100000000];
		debug_sensor_event_str[i+1][16] = hex_str[sensor_event_list[i].all_packet_count%100000000/10000000];
		debug_sensor_event_str[i+1][17] = hex_str[sensor_event_list[i].all_packet_count%10000000/1000000];		
		debug_sensor_event_str[i+1][18] = hex_str[sensor_event_list[i].all_packet_count%1000000/100000];	
		debug_sensor_event_str[i+1][19] = hex_str[sensor_event_list[i].all_packet_count%100000/10000];		
		debug_sensor_event_str[i+1][20] = hex_str[sensor_event_list[i].all_packet_count%10000/1000];
		debug_sensor_event_str[i+1][21] = hex_str[sensor_event_list[i].all_packet_count%1000/100];		
		debug_sensor_event_str[i+1][22] = hex_str[sensor_event_list[i].all_packet_count%100/10];	
		debug_sensor_event_str[i+1][23] = hex_str[sensor_event_list[i].all_packet_count%10];
		debug_sensor_event_str[i+1][24] = ' ';			

		debug_sensor_event_str[i+1][25] = hex_str[sensor_event_list[i].receive_packet_count/100000000];
		debug_sensor_event_str[i+1][26] = hex_str[sensor_event_list[i].receive_packet_count%100000000/10000000];
		debug_sensor_event_str[i+1][27] = hex_str[sensor_event_list[i].receive_packet_count%10000000/1000000];		
		debug_sensor_event_str[i+1][28] = hex_str[sensor_event_list[i].receive_packet_count%1000000/100000];	
		debug_sensor_event_str[i+1][29] = hex_str[sensor_event_list[i].receive_packet_count%100000/10000];		
		debug_sensor_event_str[i+1][30] = hex_str[sensor_event_list[i].receive_packet_count%10000/1000];
		debug_sensor_event_str[i+1][31] = hex_str[sensor_event_list[i].receive_packet_count%1000/100];		
		debug_sensor_event_str[i+1][32] = hex_str[sensor_event_list[i].receive_packet_count%100/10];	
		debug_sensor_event_str[i+1][33] = hex_str[sensor_event_list[i].receive_packet_count%10];
		debug_sensor_event_str[i+1][34] = ' ';	
		
		debug_sensor_event_str[i+1][35] = hex_str[sensor_event_list[i].repeat_packet_count%1000000/100000];	
		debug_sensor_event_str[i+1][36] = hex_str[sensor_event_list[i].repeat_packet_count%100000/10000];		
		debug_sensor_event_str[i+1][37] = hex_str[sensor_event_list[i].repeat_packet_count%10000/1000];
		debug_sensor_event_str[i+1][38] = hex_str[sensor_event_list[i].repeat_packet_count%1000/100];		
		debug_sensor_event_str[i+1][39] = hex_str[sensor_event_list[i].repeat_packet_count%100/10];	
		debug_sensor_event_str[i+1][40] = hex_str[sensor_event_list[i].repeat_packet_count%10];
		
		for(j=0;j<10;j++)
		{
			debug_sensor_event_str[i+1][41+j*5] = ' ';
			debug_sensor_event_str[i+1][42+j*5] = hex_str[sensor_event_list[i].repeat_packet_times_count[j]%10000/1000];
			debug_sensor_event_str[i+1][43+j*5] = hex_str[sensor_event_list[i].repeat_packet_times_count[j]%1000/100];		
			debug_sensor_event_str[i+1][44+j*5] = hex_str[sensor_event_list[i].repeat_packet_times_count[j]%100/10];	
			debug_sensor_event_str[i+1][45+j*5] = hex_str[sensor_event_list[i].repeat_packet_times_count[j]%10];
		}
		debug_sensor_event_str[i+1][91] = '\r';	
		debug_sensor_event_str[i+1][92] = '\n';	
	}
	
	debug_sensor_event_str[i+1][0] = 0;

}

