#include "stm32f4xx_hal.h"
#include "to_n1.h"
#include "string.h"
#include "math.h"
#include "gprs_hal.h"
#include "debug_uart.h"
#include "ap_param.h"
#include "rf_data_handle.h"
#include "gprs_4g_app.h"
#include "typedef_struct.h"
#include "gprs_comm.h"




#define L506_4G 
//#define M35_2G

typedef struct{
	int head;
	int n1_id;
	unsigned short length;
	unsigned char data[1];
}strcut_n1_data_head;




#define GPRS_DEBUG_PRINT

#define GPRS_STR_QUEUE_LENGH 4096      //字符串队列长度，用于存储gprs模块返回的字符串

#define GPRS_SEND_DATA_BUFF_LENGH 256


#define GPRS_CONNECT_FAIL_TIMEOUT   2

#define debug(str) copy_string_to_double_buff(str)

uint8_t n1_to_server_data[256];         //N1发送给gprs 服务器的数据


uint8_t gprs_str_queue[GPRS_STR_QUEUE_LENGH];  //用于存储gprs模块返回的字符串

uint8_t gprs_data_buff[GPRS_DATA_BUFF_LENGH];   //用于从队列中取出完整的一条命令或者数据 

struct_gprs_one_cmd gprs_one_cmd;


extern uint8_t ap_param_write_flash_flag;
extern struct_systerm_info systerm_info;


int gprs_str_queue_write = 0;               //写偏移
int gprs_str_queue_read = 0;								//读偏移

int gprs_receive_packet_flag = 0;


int gprs_cmd_param_num = 0;
char gprs_cmd_param[10][50];  //用来存储字符串中的参数

uint8_t gprs_sec_flag = 0;         //秒点标志 由同步包中断置1

char gprs_debug_buff[256];

char at_cmd_conn[50];
uint8_t gps_read_flag = 0;

int gprs_print_rx_tx_data_enable = 0;   //使能gprs收发数据打印功能



struct _gps_date gps_date;

struct_gprs_stat gprs_stat;

extern int32_t dtu_ack_r_fun(uint32_t seq,uint32_t sec_or_lane);
int32_t gps_set_rtc(struct _gps_date *p_date);

#define GPRS_CMD_OK 0
#define GPRS_CMD_CGREG 1
#define GPRS_CMD_CSQ 2
#define GPRS_CMD_SEND_READY 3
#define GPRS_CMD_CREG 4
#define GPRS_CMD_REVISION 5
#define GPRS_CMD_CONNECT_OK_0 6
#define GPRS_CMD_CONNECT_OK_1 7
#define GPRS_CMD_CONNECT_OK_2 8
#define GPRS_CMD_CONNECT_FAIL_0 9
#define GPRS_CMD_CONNECT_FAIL_1 10
#define GPRS_CMD_CONNECT_FAIL_2 11
#define GPRS_CMD_CONNECT_CLOSED_0 12
#define GPRS_CMD_CONNECT_CLOSED_1 13
#define GPRS_CMD_CONNECT_CLOSED_2 14
#define GPRS_CMD_REBOOT   15
#define GPRS_CMD_QISTAT   16
#define GPRS_CMD_QISACK   17
#define GPRS_CMD_RECEIVE  18
#define GPRS_CMD_CEREG  19
#define _4G_CMD_CIPOPEN 20
#define _4G_CMD_NETOPEN 21
#define _4G_CMD_CIPSEND 22
#define _4G_CMD_DISCONN 23
#define _4G_CMD_ERROR 24
#define _4G_CMD_SERVERDISCONN 25
#define _4G_CMD_RX 26
#define _4G_CMD_CIPSTAT 27
#define DTU_ACK_R 28
#define _GPS_CMD_CGPSINFO 29


char *gprs_str_cmd[] = {

	"OK",
	"+CGREG",
	"+CSQ",
	">",
	"+CREG",
	"Revision",
	"0, CONNECT OK",
	"1, CONNECT OK",
	"2, CONNECT OK",
	"0, CONNECT FAIL",
	"1, CONNECT FAIL",
	"2, CONNECT FAIL",
	"0, CLOSED",
	"1, CLOSED",
	"2, CLOSED",	
	"Call Ready",
	"+QISTATE",
	"+QISACK",
	"+RECEIVE",
	"+CEREG",
	"+CIPOPEN",      //+CIPOPEN:SUCCESS,1
	"+NETOPEN" ,      //:SUCCESS
	"+CIPSEND",    //:SUCCESS,1,20,4
	"+NETWORK DISCONNECTED",  //:0
	"ERROR",
	"+SERVER DISCONNECTED" ,  //:0	
	"+CIPRXGET",          //:SUCCESS,0,0,19,
	"+CIPSTAT",
	"dtu_ack_r",
	"+CGPSINFO",
};




#define AT_CMD_ATI             "ATI\r\n"          //AT指令 判断GSM模块是否启动
#define AT_CMD_AT_CSQ         "AT+CSQ\r\n"      //信号强度
#define AT_CMD_AT_CREG  			"AT+CREG?\r\n"    //GSM网络是否注册    +CREG: 0,1  // <stat>=1,GSM网络已经注册上
#define AT_CMD_AT_CGREG       "AT+CGREG?\r\n"   //GPRS网络是否注册   +CGREG: 0,1    // <stat>=1,GPRS网络已经注册上
#define AT_CMD_AT_CEREG       "AT+CEREG?\r\n" 
#define AT_CMD_AT_SET_BAUD    "AT+IPR=230400&W\r\n"   //配置固定波特率
#define AT_CMD_AT_QIMUX       "AT+QIMUX=1\r\n"   //配置多链接模式
#define AT_CMD_AT_QISTAT      "AT+QISTATE\r\n"
#define AT_CMD_AT_CLOSE_CONNECT "AT+QICLOSE=%d\r\n"    //关闭指定的tcp链接
#define AT_CMD_AT_QISACK      "AT+QISACK=%d\r\n"        //查询发送的数据
#define AT_CMD_AT_SEND         "AT+QISEND=%d,%d\r\n"       //发送数据  连接编号 数量
#define AT_CMD_4GAT_NETOPEN      "AT+NETOPEN\r\n"         //打开网络
#define AT_CMD_4GAT_NETCLOSE     "AT+NETCLOSE\r\n"         //打开网络
#define AT_CMD_4GAT_SET_TIMEOUT   "AT+CIPTIMEOUT=10000,10000,10000,10000\r\n"    //(netopen cipopen cipsend dns_cipopen)  timeout
#define AT_CMD_4GAT_CIPOPEN        "AT+CIPOPEN=%d,\"TCP\",\"%d.%d.%d.%d\",%d,0\r\n"    //连接服务器
#define AT_CMD_4GAT_CIPSEND      "AT+CIPSEND=%d,%d\r\n"       //发送数据  连接编号 数量   4G
#define AT_CMD_4GAT_CIPSTAT       "AT+CIPSTAT=%d\r\n"
#define AT_CMD_4GAT_CGPS_ON       "AT+CGPS=1,2\r\n"
#define AT_CMD_4GAT_CGPS_OFF       "AT+CGPS=0\r\n"
#define AT_CMD_4GAT_CGPSINFO_5       "AT+CGPSINFO=5\r\n"
#define AT_CMD_4GAT_CGPSINFO_0       "AT+CGPSINFO=0\r\n"


extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef rtc_time;
extern RTC_DateTypeDef rtc_date;

extern UART_HandleTypeDef huart2;
void send_gprs_200_();




/*
 * 功能：往队列中写入一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char gprs_str_write_queue(uint8_t data)
{
	if((gprs_str_queue_write+1)%GPRS_STR_QUEUE_LENGH==gprs_str_queue_read)
		return -1;
	gprs_str_queue[gprs_str_queue_write] = data;
	gprs_str_queue_write = (gprs_str_queue_write+1)%GPRS_STR_QUEUE_LENGH;
	return 0;
}

/*
 * 功能：从队列中读取一个字节
 * 失败：返回-1
 * 成功：返回0
*/
char gprs_str_read_queue(uint8_t *pdata)
{
	if(gprs_str_queue_write==gprs_str_queue_read)
		return -1;		
	*pdata = gprs_str_queue[gprs_str_queue_read];
	gprs_str_queue_read = (gprs_str_queue_read+1)%GPRS_STR_QUEUE_LENGH;
	return 0;
}

/*
 * 功能：将串口dma接收完的一包数据 copy到环形队列
 * 失败：返回-1
 * 成功：返回0
*/
void gprs_str_copy_to_queue(unsigned short len,char* p_data)
{
	int i = 0;

//	gprs_str_write_queue(len);
//	gprs_str_write_queue(len>>8);	
	for(i=0;i<len;i++)
	{
		gprs_str_write_queue(*p_data++);	
	
	}
	//gprs_receive_packet_flag++;  //每次完成一个数据包的copy 此变量++
}



int gprs_str_to_int(char *pstr)
{
	char *p_str = pstr;
	int lengh = 0;
	int offset = 0;
	int ret = 0;
	int i = 0;
	
	while(1)
	{
		if(*p_str == ' ')
			offset++;
		if(*p_str == 0 || *p_str == '.')
			break;
		p_str++;
		lengh++;
	}
	p_str = pstr+offset;
	lengh = lengh - offset;
	
	for(i=0;i<lengh;i++)
	{
	 ret += (*(p_str+i)-'0')*pow(10,(lengh-i-1));	
	}
	return ret;
}


void gprs_uart_send_string(char *pstr)
{
	int num = 0;
	char *ptr = pstr;
	while(*ptr++)
	{
		num++;
		if(num>5000)return;
	}
	
	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)pstr,num);
	
}

/*
	说明：GSM网络正常后 调用这个函数查询 gprs网络是否正常

*/
void gprs_at_get_CGREG(void)
{
	gprs_uart_send_string(AT_CMD_AT_CGREG);	  //查询gsm网络

}

/*
	说明：查询那个链接的状态     gprs模块支持同时保存多个tcp连接

*/
void gprs_at_get_tcp_online(int num)
{
	gprs_stat.con_client[num].connect_ok = 0;


}

/*
	说明： 每次上电 后调用ATI 返回revision字符串后 执行这个函数

*/
void gprs_at_init(void)
{
	int i;
//	gprs_uart_send_string(AT_CMD_AT_SET_BAUD);   //设置波特率 
//	i = 200000;
//	while(i--);
//	gprs_uart_send_string(AT_CMD_AT_QIMUX);	  //配置为多链接模式
//	i = 200000;
//	while(i--);
gprs_uart_send_string("ATE0\r\n");	
}

void gprs_reboot(void)
{
	
	gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;
/*	gprs_off();
	i = 100000;
	while(i--);
	gprs_on();
	memset(&gprs_stat,0,sizeof(struct struct_gprs_stat));	
	gprs_stat.send_data_id = -1; //表示没有连接要发送数据
	*/
}

char at_cmd_conn[50];
void gprs_at_tcp_conn(int num,unsigned long ip,unsigned short port)
{

	memset(at_cmd_conn,0,50);
#ifdef M35_2G	
	sprintf(at_cmd_conn,"AT+QIOPEN=%d,\"TCP\",\"%d.%d.%d.%d\",%d\r\n",num,ip&0xff,(ip>>8)&0xff,(ip>>16)&0xff,ip>>24,port);
#else
	sprintf(at_cmd_conn,AT_CMD_4GAT_CIPOPEN,num,ip&0xff,(ip>>8)&0xff,(ip>>16)&0xff,ip>>24,port);
#endif
	
	gprs_uart_send_string(at_cmd_conn);

}

void gprs_at_tcp_close(int num)
{
	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,"AT+QICLOSE=%d\r\n" ,num);
	gprs_uart_send_string(at_cmd_conn);

}

//每次有新数据从N1产生就调用一次该函数，
int gprs_send_data_flag(int num, unsigned short data_num,unsigned char * data_buff)
{
//	if(gprs_stat.send_data_id >=0)        //有还没发送的数据
//		return -1;
	
	if(num > 2)        //有还没发送的数据
		return -2;	
	
	memset(at_cmd_conn,0,50);
	sprintf(at_cmd_conn,AT_CMD_AT_SEND,num,data_num);
	gprs_uart_send_string(at_cmd_conn);                    //发送一次发送数据请求的命令
	gprs_stat.con_client[num].send_no_timeout = 0;
//	gprs_stat.send_data_id = num;
//	gprs_stat.send_data_num = data_num;
//	gprs_stat.send_data_buff = data_buff;
}







void gprs_send_data(int num, unsigned short data_num,unsigned char * data_buff)
{
	if(gprs_stat.con_client[num].connect_ok == 0)
		return;
//	memset(at_cmd_conn,0,50);
//	sprintf(at_cmd_conn,AT_CMD_AT_SEND,num,data_num);
//	gprs_uart_send_string(at_cmd_conn);
//	gprs_stat.send_data_id = num;
//	gprs_stat.send_data_num = data_num;
//	gprs_stat.send_data_buff = data_buff;
}

int gprs_str_cmd_handle(char *pstr)
{
		int str_cmd_num;
		int i;
		unsigned short *data_len;
	  int lengh = 0;
	
		int32_t delay = 100000;
	
	
		str_cmd_num = sizeof(gprs_str_cmd)/4;
		for(i=0;i<str_cmd_num;i++)
		{
			if(strcmp(pstr,gprs_str_cmd[i])==0)
				break;	
		}
		
		if(i == str_cmd_num)
			return 0;
		
		switch(i)
		{
			case GPRS_CMD_OK: break;
			case GPRS_CMD_CREG: 
				if(gprs_cmd_param[1][0] == '0'+1 || gprs_cmd_param[1][0] == '0'+5) 
				{
					gprs_stat.creg_ok = 1;
					gprs_at_get_CGREG();
				}		
				break;		
			case GPRS_CMD_CGREG: 
				if(gprs_cmd_param[1][0] == '0'+1 ||gprs_cmd_param[1][0] == '0'+5) 			
				{
					gprs_stat.cgreg_ok = 1;			
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:cgreg ok\r\n");

#endif					
				}	
				else if(gprs_cmd_param[1][0] == '0'+1 ||gprs_cmd_param[1][0] == '0'+3) 	
				{
#ifdef GPRS_DEBUG_PRINT					
					debug("gprs:cgreg 0,3\r\n");

#endif
				}
			
			break;
			case GPRS_CMD_CEREG:
				if(gprs_cmd_param[1][0] == '0'+1 ||gprs_cmd_param[1][0] == '0'+5) 			
				{
					gprs_stat.cereg_ok = 1;			
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:cereg ok\r\n");

#endif					
				}	
				else if(gprs_cmd_param[1][0] == '0'+3) 	
				{
#ifdef GPRS_DEBUG_PRINT					
					debug("gprs:cereg 0,3\r\n");

#endif
				}
			
			break;	
			case _4G_CMD_ERROR:
				break;
			case _4G_CMD_NETOPEN:
				if(strcmp(gprs_cmd_param[0],"SUCCESS") == 0)
				{
					gprs_stat.netopen = 1;
				}
				break;
			case GPRS_CMD_CSQ: gprs_stat.csq =  (gprs_cmd_param[0][0]-'0')*10 + gprs_cmd_param[0][1]-'0' ;
#ifdef GPRS_DEBUG_PRINT

		sprintf(gprs_debug_buff,"gprs:rssi %d\r\n",gprs_stat.csq);
		debug(gprs_debug_buff);

#endif	
				break;
				
			case	_4G_CMD_CIPOPEN:   //链接SUCCESS or FAIL
				if(strcmp(gprs_cmd_param[0],"SUCCESS") == 0)
				{
					if(gprs_cmd_param[1][0] == '0')
					{
						gprs_stat.con_client[0].connect_ok = 1;
						gprs_stat.con_client[0].send_cmd_timeout = 0;
						gprs_stat.con_client[0].send_cmd_times = 0;	
						send_4g_heart_pacekt(0);
					}
					else if(gprs_cmd_param[1][0] == '1')
					{
						gprs_stat.con_client[1].connect_ok = 1;
						gprs_stat.con_client[0].send_cmd_timeout = 0;
						gprs_stat.con_client[0].send_cmd_times = 0;
						send_4g_heart_pacekt(1);						
					}
				}
				else  //FAIL
				{
					if(gprs_cmd_param[1][0] == '0')
					{
						gprs_uart_send_string("AT+CIPCLOSE=0\r\n");	 
						gprs_stat.con_client[0].connect_fail_times++;
					}
					else if(gprs_cmd_param[1][0] == '1')
					{
						gprs_uart_send_string("AT+CIPCLOSE=1\r\n");	
						gprs_stat.con_client[1].connect_fail_times++;
					}		
					if(gprs_stat.con_client[0].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT && gprs_stat.con_client[1].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)
					{
						gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;
					}
				}
				break;
			case	_4G_CMD_CIPSEND:        //Send 
				if(strcmp(gprs_cmd_param[0],"SUCCESS") == 0)
				{
					if(gprs_cmd_param[1][0] == '0') 
					{
						gprs_stat.send_steps = GPRS_SEND_OK;
						gprs_stat.con_client[0].send_no_timeout = systerm_info.slot;
						gprs_stat.con_client[0].send_len = 0;	
								
						gprs_stat.now_who_working = -1;
						gprs_stat.send_steps = 0;							
					}
					else if(gprs_cmd_param[1][0] == '1') 
					{
						gprs_stat.send_steps = GPRS_SEND_OK;
						gprs_stat.con_client[1].send_no_timeout = systerm_info.slot;
						gprs_stat.con_client[1].send_len = 0;	
								
						gprs_stat.now_who_working = -1;
						gprs_stat.send_steps = 0;							
					}
				}
				else
				{
					if(gprs_cmd_param[1][0] == '0') 
					{
						
					}
					else if(gprs_cmd_param[1][0] == '1') 
					{
						
					}					
				}
				break;
			case _GPS_CMD_CGPSINFO:
				if(gprs_cmd_param[0][0] == 0)
					break;
				sscanf(&gprs_cmd_param[0][0],"%f",&sys_flash_param.ap_param.gps_n_e[0]);
				sscanf(&gprs_cmd_param[2][0],"%f",&sys_flash_param.ap_param.gps_n_e[1]);
				ap_param_write_flash_flag = 1;
				gps_date.date = (gprs_cmd_param[4][0]-'0')*10 + gprs_cmd_param[4][1]-'0';
				gps_date.month = (gprs_cmd_param[4][2]-'0')*10 + gprs_cmd_param[4][3]-'0';
				gps_date.year = (gprs_cmd_param[4][6]-'0')*10 + gprs_cmd_param[4][7]-'0';
				gps_date.hour = (gprs_cmd_param[5][0]-'0')*10 + gprs_cmd_param[5][1]-'0' + 8;
				if(gps_date.hour >= 24)
					gps_date.hour -= 24;
				gps_date.minter = (gprs_cmd_param[5][2]-'0')*10 + gprs_cmd_param[5][3]-'0';
				gps_date.sec = (gprs_cmd_param[5][4]-'0')*10 + gprs_cmd_param[5][5]-'0';
				if(gprs_print_rx_tx_data_enable >0)
				{
					sprintf(gprs_debug_buff,"gps: [%d-%d] %d-%d-%d %d:%d:%d\r\n",sys_flash_param.ap_param.gps_n_e[0],sys_flash_param.ap_param.gps_n_e[1],
					gps_date.year,gps_date.month,gps_date.date,gps_date.hour,gps_date.minter,gps_date.sec);
					debug(gprs_debug_buff);
				}	
				if(0 == gps_set_rtc(&gps_date))
				{
					gprs_uart_send_string(AT_CMD_4GAT_CGPSINFO_0);
					while(delay--);
					gprs_uart_send_string(AT_CMD_4GAT_CGPS_OFF);
				}
				break;
			case _4G_CMD_DISCONN:
				if(gprs_cmd_param[0][0] == '0') 
				{
					gprs_stat.netopen = 0;
					gprs_stat.con_client[0].connect_ok = 0;
				}					
				else if(gprs_cmd_param[0][0] == '1') 
				{
					gprs_stat.netopen = 0;
					gprs_stat.con_client[1].connect_ok = 0;
				}
				break;
			case	_4G_CMD_SERVERDISCONN:
				if(gprs_cmd_param[0][0] == '0') 
				{
					gprs_stat.con_client[0].connect_ok = 0;
				}					
				else if(gprs_cmd_param[0][0] == '1') 
				{
					gprs_stat.con_client[1].connect_ok = 0;
				}
				break;			
			case _4G_CMD_RX:
				if(strcmp(gprs_cmd_param[0],"SUCCESS") == 0)
				{
					if(gprs_cmd_param[2][0] == '0') 
					{
						gprs_stat.now_who_rx = 1;
						gprs_stat.now_rx_len = gprs_str_to_int(&gprs_cmd_param[3][0]);
					}
					else if(gprs_cmd_param[2][0] == '1') 
					{
						gprs_stat.now_who_rx = 2;
						gprs_stat.now_rx_len = gprs_str_to_int(&gprs_cmd_param[3][0]);						
					}
				}				
				break;
			case _4G_CMD_CIPSTAT:
				gprs_stat.con_client[gprs_stat.now_read_who_txrx].tx_len += gprs_str_to_int(&gprs_cmd_param[0][0]);
				gprs_stat.con_client[gprs_stat.now_read_who_txrx].rx_len += gprs_str_to_int(&gprs_cmd_param[1][0]);
				break;
			case GPRS_CMD_SEND_READY:             //发送数据
				if(gprs_stat.now_who_working < CLIENT_NUM && gprs_stat.now_who_working > -1)   //链接0
				{
					gprs_stat.send_steps = GPRS_SEND_WAIT_SENDOK;
					gprs_stat.con_client[gprs_stat.now_who_working].send_no_timeout = systerm_info.slot;
					HAL_UART_Transmit_DMA(&huart2,gprs_stat.con_client[gprs_stat.now_who_working].buf,gprs_stat.con_client[gprs_stat.now_who_working].send_len);
				}
				if(gprs_print_rx_tx_data_enable >0)
				{
					sprintf(gprs_debug_buff,"gprs: client_0 send data %d\r\n",gprs_stat.now_who_working);
					debug(gprs_debug_buff);
				}
				break;
			case GPRS_CMD_REVISION:
				gprs_stat.ati_ok = 1;
				gprs_at_init();
				break;
			case GPRS_CMD_CONNECT_OK_0 :
				gprs_stat.con_client[0].connect_ok = 1; 	
				gprs_stat.con_client[0].connect_fail_times = 0;
//				gprs_send_data(0,gprs_ap_data_buff[0],gprs_ap_data_buff); 
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:0,connect ok\r\n");

#endif						
				break;
			case GPRS_CMD_CONNECT_OK_1 :gprs_stat.con_client[1].connect_ok = 1;break;
			case GPRS_CMD_CONNECT_OK_2 :gprs_stat.con_client[2].connect_ok = 1;break;
			case GPRS_CMD_CONNECT_FAIL_0 :
				gprs_stat.con_client[0].connect_fail_times += 1; 
			  gprs_at_tcp_close(0);

			  if(gprs_stat.con_client[0].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)
				{
					debug("gprs:0,connect fail timeout reboot\r\n");
					gprs_stat.con_client[0].connect_fail_times = 0;
					gprs_reboot();
					break;
				}
				debug("gprs:0,connect fail\r\n");				
				break;
			case GPRS_CMD_CONNECT_FAIL_1 :gprs_stat.con_client[1].connect_fail_times += 1; gprs_at_tcp_close(1);if(gprs_stat.con_client[1].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)gprs_reboot();break;
			case GPRS_CMD_CONNECT_FAIL_2 :gprs_stat.con_client[2].connect_fail_times += 1; gprs_at_tcp_close(2);if(gprs_stat.con_client[2].connect_fail_times>GPRS_CONNECT_FAIL_TIMEOUT)gprs_reboot();break;
			case GPRS_CMD_CONNECT_CLOSED_0 : 
				gprs_stat.con_client[0].connect_ok = 0;  
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:0,closed\r\n");

#endif						
				break;
			case GPRS_CMD_CONNECT_CLOSED_1 : gprs_stat.con_client[1].connect_ok = 0;  break;
			case GPRS_CMD_CONNECT_CLOSED_2 : gprs_stat.con_client[2].connect_ok = 0;  break;
			
			case GPRS_CMD_REBOOT	:

				gprs_at_init();
#ifdef GPRS_DEBUG_PRINT


					debug("gprs:reinit\r\n");

#endif						
				break;

			case GPRS_CMD_QISACK :
#ifdef GPRS_DEBUG_PRINT

					sprintf(gprs_debug_buff,"gprs:send_data=%d ack_data=%d noack_data=%d\r\n",gprs_str_to_int(&gprs_cmd_param[0][0]),gprs_str_to_int(&gprs_cmd_param[1][0]),gprs_str_to_int(&gprs_cmd_param[2][0]));
					debug(gprs_debug_buff);

#endif					
				if(gprs_cmd_param[2][0]!='0')
				{
							
					gprs_stat.con_client[0].gprs_send_error_timeout++;
					if(gprs_stat.con_client[0].gprs_send_error_timeout>2)
					{
						gprs_stat.con_client[0].gprs_send_error_timeout = 0;
						gprs_stat.con_client[0].connect_ok = 0;
					}
				}
				else
				{
					gprs_stat.con_client[0].gprs_send_error_timeout = 0;
				}
			break;
			
			case GPRS_CMD_QISTAT:
				if(gprs_cmd_param[0][0]=='0')
				{
					lengh = lengh;
				}
			
			break;
			case	GPRS_CMD_RECEIVE:
				lengh = gprs_str_to_int(&gprs_cmd_param[1][0]);

				if(gprs_print_rx_tx_data_enable >0)
				{
					sprintf(gprs_debug_buff,"gprs:receive data %d\r\n",lengh);
					debug(gprs_debug_buff);
				}
			
				return lengh;
			case DTU_ACK_R:
				
				dtu_ack_r_fun(gprs_str_to_int(&gprs_cmd_param[0][0]),gprs_str_to_int(&gprs_cmd_param[1][0]));
				if(gprs_print_rx_tx_data_enable >0)
				{
					sprintf(gprs_debug_buff,"gprs:dtu_ack_r %d %d\r\n",gprs_str_to_int(&gprs_cmd_param[0][0]),gprs_str_to_int(&gprs_cmd_param[1][0]));
					debug(gprs_debug_buff);
				}				
			break;
			default:break;

		}
		return 0;
}


/*
 * 功能：解析取得的字符串 分割成命令和参数
 * 失败：返回-1
 * 成功：返回0
*/
void gprs_get_cmd_param(char *pstr , unsigned short length)
{
	int i,j;
	int flag_param = 0;
	int n1_data_length = 0;
	int strat_string = 0;

	memset(gprs_cmd_param,0,500);
	for(i=0;i<length;i++)
	{		
		
		if(pstr[i] == '\r' || pstr[i] == '\n' || pstr[i] == '>')  //索引到字符串结束  将该字节置0 作为字符串结束标志
		{
			if(pstr[i] != '>')
				pstr[i] = 0;
			if(pstr[i] == '>')
				strat_string = i;			
			if(pstr[i+1] == '\n' || pstr[i+1] == ' ' )
				pstr[++i] = 0;

			n1_data_length = gprs_str_cmd_handle(&pstr[strat_string]);

			gprs_cmd_param_num = 0;
			flag_param = 0;
			strat_string = i+1;
		}

		if(pstr[i] == ':')	//表示带有参数	
		{
			pstr[i++] = 0;   //把这个： 跨过去 以免写进参数里
			gprs_cmd_param_num++;
			flag_param = 1;
			j = 0;			
		}
		
		if(pstr[i] == ',' && flag_param == 1)
		{
			pstr[i++] = 0;   //把这个, 跨过去 以免写进参数里
			gprs_cmd_param_num++;		
			j = 0;
		}
		
		if(flag_param == 1)
		{
			if(j!=0 || pstr[i]!=0x20)
			{	
				gprs_cmd_param[gprs_cmd_param_num-1][j++] = pstr[i];
				pstr[i] = 0;		
			}				
		}

	}	
}


/*
 * 功能：解析取得的字符串 分割成命令和参数
 * 失败：返回-1
 * 成功：返回0
*/
void dtu_cmd_param(char *pstr , unsigned short length)
{
	int i,j;
	int flag_param = 0;
	int n1_data_length = 0;
	int strat_string = 0;

	memset(gprs_cmd_param,0,500);
	for(i=0;i<length;i++)
	{		
		
		if(pstr[i] == '\r' || pstr[i] == '\n' )  //索引到字符串结束  将该字节置0 作为字符串结束标志
		{
		
			if(pstr[i+1] == '\n' || pstr[i+1] == ' ' )
				pstr[++i] = 0;

			n1_data_length = gprs_str_cmd_handle(&pstr[strat_string]);

			gprs_cmd_param_num = 0;
			flag_param = 0;
			strat_string = i+1;
		}

		if(pstr[i] == ' ')	//表示带有参数	
		{
			pstr[i++] = 0;   //把这个： 跨过去 以免写进参数里
			gprs_cmd_param_num++;
			flag_param = 1;
			j = 0;			
		}
		
		if(pstr[i] == ' ' && flag_param == 1)
		{
			pstr[i++] = 0;   //把这个, 跨过去 以免写进参数里
			gprs_cmd_param_num++;		
			j = 0;
		}
		
		if(flag_param == 1)
		{
			if(j!=0 || pstr[i]!=0x20)
			{	
				gprs_cmd_param[gprs_cmd_param_num-1][j++] = pstr[i];
				pstr[i] = 0;		
			}				
		}

	}	
}



uint32_t get_one_data_from_queue(uint8_t *p_data)
{
	int i;
	uint32_t ret;
	unsigned short length = 0;
	uint8_t data = 0;	
	
	//判断是否有新数据
	if(gprs_receive_packet_flag == 0)
		return 0;
	
	gprs_receive_packet_flag--; //有可能积攒了多条数据
	
	for(i=0;i<2;i++)
	{
		ret = gprs_str_read_queue(&data);
		if(ret!=0)
			return 0;
		length |= data<<(i*8);
	}
	
	if(length<=0)
		return 0;
	
	if(length > GPRS_DATA_BUFF_LENGH)
		length = GPRS_DATA_BUFF_LENGH;
	
	
	for(i=0;i<length;i++)
	{
		ret = gprs_str_read_queue(p_data++);
		if(ret!=0)
			return 0;			
	}	
	
	return length;
}


/*
* 功能 : main函时调用 来处理gprs模块的一些请求和状态查询等任务 * 成功：返回0
*/
void gprs_data_handle(void)
{
	unsigned short length = 0;
	uint8_t data = 0;
	int i;
	int begin;
	char ret = 0;
	uint8_t *p_gprs_data_buff = gprs_data_buff;
	strcut_n1_data_head *p_n1_data_head;
	int32_t timeout = 0;
	struct_client_data_buff *client_buff[3] = {0,&client_data_buff_0,&client_data_buff_1};
	
	while(1)
	{
		if(gprs_stat.now_who_rx > 0 && gprs_stat.now_who_rx < 3)
		{
			if(0!=gprs_str_read_queue(&client_buff[gprs_stat.now_who_rx]->buff[client_buff[gprs_stat.now_who_rx]->index]))
			{
				timeout++;
				if(timeout > 1000000)
				{
					
					client_buff[gprs_stat.now_who_rx]->index = 0;
					gprs_stat.now_who_rx = 0;
					return;
				}
				continue;
			}
			timeout = 0;
			client_buff[gprs_stat.now_who_rx]->index++;
			gprs_stat.now_rx_len--;
			if(client_buff[gprs_stat.now_who_rx]->index>=GPRS_DATA_BUFF_LENGH)
			{
				server_4g_data_hanle(gprs_stat.now_who_rx-1);
			}
			if(gprs_stat.now_rx_len == 0)
			{				
				server_4g_data_hanle(gprs_stat.now_who_rx-1);
				gprs_stat.now_who_rx = 0;
			}
			continue;
		}
		
		if(0!=gprs_str_read_queue(&gprs_one_cmd.data[gprs_one_cmd.index]))
			return;
		
		gprs_one_cmd.index++;
		if(gprs_one_cmd.index>255)
		{
			gprs_one_cmd.index = 0;
			memset(gprs_one_cmd.data,0,256);
			return;
		}
		
		if(gprs_one_cmd.data[gprs_one_cmd.index-1] == '>')
		{
			gprs_get_cmd_param((char *)&gprs_one_cmd.data[gprs_one_cmd.index-1],1);
			memset(gprs_one_cmd.data,0,256);
			gprs_one_cmd.index = 0;
		}
		if(gprs_one_cmd.data[gprs_one_cmd.index-1] == '\n')
		{
			gprs_get_cmd_param((char *)&gprs_one_cmd.data[0],gprs_one_cmd.index);
			memset(gprs_one_cmd.data,0,256);
			gprs_one_cmd.index = 0;			
		}
	}

	
}

void close_tcp_conn(int index)
{

	sprintf(at_cmd_conn,AT_CMD_AT_CLOSE_CONNECT ,index);				
	gprs_uart_send_string(at_cmd_conn);	
	gprs_stat.con_client[index].connect_ok = 0;
}

//由于异常或者数据发送完毕 需要关闭
void poweroff_gprs_4g_mode()
{
	
}



//由于异常需要重启模块 需要关闭
void restart_gprs_4g_mode()
{
	
}

int fuck = 1;

/*
	说明： main主循环调用

*/
void gprs_main_call()
{
	static int timeout = 0;
	int i = 0;
	static int get_link_stat = 0;
	static int which_client = 0;
	static int sec_gps_no_read = 0;
	int delay = 0;
	gprs_data_handle();
	
	for(i=0;i<CLIENT_NUM;i++)
	{
		if(gprs_stat.con_client[i].connect_ok ==1)                //判断是否联网状态
		{
			if(gprs_stat.con_client[i].send_len > 0)     //有数据需要发送
			{	
				
				if(gprs_stat.now_who_working == -1) //没有链接正在发送数据  
				{
					gprs_stat.now_who_working = i;  //,占用发送通道
					memset(at_cmd_conn,0,50);
	#ifdef M35_2G
					sprintf(at_cmd_conn,AT_CMD_AT_SEND,i,gprs_stat.con_client[i].send_len);
	#else
					sprintf(at_cmd_conn,AT_CMD_4GAT_CIPSEND,i,gprs_stat.con_client[i].send_len);
	#endif
					gprs_uart_send_string(at_cmd_conn);  //启动发送
					gprs_stat.send_steps =  GPRS_SEND_WAIT_S;  //切换到等待>状态
					gprs_stat.con_client[i].send_no_timeout = systerm_info.slot;  //超时清零
					return;				
				}
				else if(gprs_stat.now_who_working == i)  //本链接正在使用发送功能
				{
					gprs_stat.con_client[i].last_data_send_time = systerm_info.slot;		
					if(gprs_stat.send_steps == GPRS_SEND_WAIT_S)  //正在等待>
					{			
						if(systerm_info.slot - gprs_stat.con_client[i].send_no_timeout > 10*512)
						{
							gprs_stat.con_client[0].connect_ok = 0;
							gprs_stat.con_client[1].connect_ok = 0;
							gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;  //重启模块
							memset(at_cmd_conn,0,50);
							sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",0);
							gprs_uart_send_string(at_cmd_conn);	 //关闭链接
							timeout = 10000;
							while(timeout--);		
							memset(at_cmd_conn,0,50);
							sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",1);
							gprs_uart_send_string(at_cmd_conn);	 //关闭链接
							timeout = 10000;
							while(timeout--);	
							gprs_uart_send_string(AT_CMD_4GAT_NETCLOSE); 
						}
						else
							return;
					}
					else if(gprs_stat.send_steps == GPRS_SEND_WAIT_SENDOK)  //等待返回发送结果
					{

						if(systerm_info.slot - gprs_stat.con_client[i].send_no_timeout > 10*512)
						{
							gprs_stat.con_client[i].send_no_timeout = systerm_info.slot;
							gprs_stat.con_client[i].connect_ok = 0;
							gprs_stat.con_client[i].send_cmd_timeout = 0;
							gprs_stat.con_client[i].send_cmd_times = 0;							
							memset(at_cmd_conn,0,50);
							sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",0);
							gprs_uart_send_string(at_cmd_conn);	 //关闭链接
							timeout = 10000;
							while(timeout--);		
							memset(at_cmd_conn,0,50);
							sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",1);
							gprs_uart_send_string(at_cmd_conn);	 //关闭链接
							timeout = 10000;
							while(timeout--);							
							gprs_stat.now_who_working = -1;
							gprs_stat.send_steps = 0;	
							gprs_stat.con_client[0].connect_ok = 0;
							gprs_stat.con_client[1].connect_ok = 0;
							gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;  //重启模块	
							gprs_uart_send_string(AT_CMD_4GAT_NETCLOSE); 
						}		
						else
							return;
					}
					else if(gprs_stat.send_steps == GPRS_SEND_OK)  //收到发送结果，表示发送成功
					{
						gprs_stat.con_client[i].send_no_timeout = systerm_info.slot;
						gprs_stat.con_client[i].send_len = 0;	
								
						gprs_stat.now_who_working = -1;
						gprs_stat.send_steps = 0;			
							
					}
					return;
				}
			}			
		}
	}
	
	
	
	if(gprs_sec_flag >= 1) //1sec进入下面一次
	{	
		gprs_sec_flag = 0;
//		if(fuck == 0)
//			grps_power_off();
//		else
//			grps_power_on();
		
		if(gprs_stat.reboot_flag == GPRS_REBOOT_SETP0) //关GSM电源
		{
			grps_power_off();
			gprs_stat.reboot_flag++;
			gprs_stat.ati_ok = 0;
			gprs_stat.creg_ok = 0;
			gprs_stat.cgreg_ok = 0;
			gprs_stat.cereg_ok = 0;
			gprs_stat.netopen = 0;
			gprs_stat.set_timeout = 0;
			gprs_stat.now_who_rx = 0;
			gprs_stat.now_who_working = -1;
			gprs_stat.send_steps = 0;
			gps_read_flag = 0;
			for(i=0;i<2;i++)
			{
				gprs_stat.con_client[i].connect_fail_times = 0;
				gprs_stat.con_client[i].connect_ok = 0;
				gprs_stat.con_client[i].send_cmd_timeout = 0;
				gprs_stat.con_client[i].send_no_timeout = 0;
				gprs_stat.con_client[i].send_cmd_times = 0;
			}
			return;
		}
		else if(gprs_stat.reboot_flag == GPRS_REBOOT_SETP1)  //开GSM电源
		{
			grps_power_on();
			gprs_stat.reboot_flag++;
			timeout = 0;	
			return;		
		}
		HAL_GPIO_WritePin(G4_on_off_GPIO_Port,G4_on_off_Pin,GPIO_PIN_SET);
		if(gprs_stat.ati_ok == 0)                         //命令1
		{
			timeout++;
			if(timeout>90)
				gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;      //关GSM电源
			gprs_uart_send_string(AT_CMD_ATI);  
			 
			return;
		}
		if(gprs_stat.creg_ok == 0)                           //命令2
		{
			timeout++;
			if(timeout>90)
				gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;		//关GSM电源
			gprs_uart_send_string(AT_CMD_AT_CREG);  
			return;
		}	
		if(gprs_stat.cgreg_ok == 0)                            //命令3
		{
			timeout++;
			if(timeout>90)
				gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;			//关GSM电源
			gprs_uart_send_string(AT_CMD_AT_CGREG);  
			return;
		}	
		
	#ifdef L506_4G 	
		if(gprs_stat.cereg_ok == 0)                            //读一下4G网络注册情况 不是必须条件 读一次 不管有没有成功都跳过
		{
			timeout++;
			gprs_stat.cereg_ok = -1;			//关GSM电源
			gprs_uart_send_string(AT_CMD_AT_CEREG);  
			return;
		}
		if(gprs_stat.set_timeout == 0)                            //设置超时 不存在失败 
		{
			timeout++;
			gprs_stat.set_timeout = 1;			//关GSM电源
			gprs_uart_send_string(AT_CMD_4GAT_SET_TIMEOUT);  
			return;
		}	
		if(gps_read_flag == 0)
		{
			gps_read_flag++;
			gprs_uart_send_string(AT_CMD_4GAT_CGPS_ON); 		
			return;		
		}
		if(gps_read_flag == 1)
		{
			gps_read_flag++; 
			gprs_uart_send_string(AT_CMD_4GAT_CGPSINFO_5); 		
			return;		
		}	
		if(gps_read_flag == 2) //已经完成过一次打开GPS动作 这里计时 用作下一次开启GPS  24小时打开一次
		{
			sec_gps_no_read++;
			if((rtc_time.Hours == 3 && rtc_time.Minutes == 3 && rtc_time.Seconds == 3) || sec_gps_no_read > (3600+100))
			{
				sec_gps_no_read = 0;
				gps_read_flag = 0; 
				
			}	
		}			
		if(gprs_stat.netopen == 0)                            //打开网络
		{
			timeout++;
			if(timeout>90)
				gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;			//关GSM电源
			gprs_uart_send_string(AT_CMD_4GAT_NETOPEN);  
			return;
		}	
	#endif	
	
		for(i=0;i<CLIENT_NUM;i++)
		{
			if(gprs_stat.con_client[i].connect_ok !=1)                //判断是否联网状态
			{
				if(gprs_stat.con_client[i].send_cmd_timeout>0)
					gprs_stat.con_client[i].send_cmd_timeout--;
				
				if(gprs_stat.con_client[i].send_cmd_timeout == 0 && sys_flash_param.global_cfg_param.server_ip[i] != 0)
				{
					if(gprs_stat.con_client[i].send_cmd_times%5 == 0)
					{
						sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",i);
						gprs_uart_send_string(at_cmd_conn);	 //关闭链接
						timeout = 10000;
						while(timeout--);
					}
					else
						gprs_at_tcp_conn(i,sys_flash_param.global_cfg_param.server_ip[i],sys_flash_param.global_cfg_param.server_port[i]);      //发送一次命令后  20秒超时后才可以再次发送
					gprs_stat.con_client[i].send_cmd_timeout = 5;
					gprs_stat.con_client[i].send_cmd_times++;
					if((gprs_stat.con_client[0].send_cmd_times>50 && gprs_stat.con_client[0].connect_ok !=1) || (gprs_stat.con_client[1].connect_ok !=1 && gprs_stat.con_client[1].send_cmd_times>50)||
						(gprs_stat.con_client[0].send_cmd_times>5 && gprs_stat.con_client[1].connect_ok !=1) || (gprs_stat.con_client[0].connect_ok !=1 && gprs_stat.con_client[1].send_cmd_times>5))       //连续链接了6次 都没有成功
					{
						gprs_stat.con_client[1].send_cmd_times = 0;
						gprs_stat.con_client[0].send_cmd_times = 0;
						gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;			//关GSM电源		
						memset(at_cmd_conn,0,50);
						sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",0);
						gprs_uart_send_string(at_cmd_conn);	 //关闭链接
						timeout = 10000;
						while(timeout--);		
						memset(at_cmd_conn,0,50);
						sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",1);
						gprs_uart_send_string(at_cmd_conn);	 //关闭链接
						timeout = 10000;
						while(timeout--);								
						gprs_uart_send_string(AT_CMD_4GAT_NETCLOSE);  						
					}
					return;
				}
			}	
			else
			{
				get_link_stat++;
				if(get_link_stat == 15)
				{	
					gprs_uart_send_string(AT_CMD_AT_CSQ);
					return;
				}
				if(get_link_stat>30)
				{
					if(gprs_stat.now_read_who_txrx == 1)
						gprs_stat.now_read_who_txrx = 0;
					else
						gprs_stat.now_read_who_txrx = 1;					
					get_link_stat = 0;
					memset(at_cmd_conn,0,50);
	#ifdef M35_2G				
					sprintf(at_cmd_conn,AT_CMD_AT_QISACK ,0);	
	#else				
					sprintf(at_cmd_conn,AT_CMD_4GAT_CIPSTAT ,gprs_stat.now_read_who_txrx);					
	#endif								

					gprs_uart_send_string(at_cmd_conn);
					return;
				}
			}			
		
		}
		
	}
		
	
	

/*		else     //联网状态
		{
			if(gprs_stat.con_client[i].send_len > 0)     //有数据需要发送
			{	
				
				if(gprs_stat.now_who_working == -1) //没有链接正在发送数据  
				{
					gprs_stat.now_who_working = i;  //,占用发送通道
					memset(at_cmd_conn,0,50);
	#ifdef M35_2G
					sprintf(at_cmd_conn,AT_CMD_AT_SEND,i,gprs_stat.con_client[i].send_len);
	#else
					sprintf(at_cmd_conn,AT_CMD_4GAT_CIPSEND,i,gprs_stat.con_client[i].send_len);
	#endif
					gprs_uart_send_string(at_cmd_conn);  //启动发送
					gprs_stat.send_steps =  GPRS_SEND_WAIT_S;  //切换到等待>状态
					gprs_stat.con_client[i].send_no_timeout = 0;  //超时清零
					return;				
				}
				else if(gprs_stat.now_who_working == i)  //本链接正在使用发送功能
				{
					gprs_stat.con_client[i].last_data_send_time = systerm_info.slot;		
					if(gprs_stat.send_steps == GPRS_SEND_WAIT_S)  //正在等待>
					{
						gprs_stat.con_client[i].send_no_timeout++;
						if(gprs_stat.con_client[i].send_no_timeout>10)
						{
							gprs_stat.reboot_flag = GPRS_REBOOT_SETP0;  //重启模块
						}
					}
					else if(gprs_stat.send_steps == GPRS_SEND_WAIT_SENDOK)  //等待返回发送结果
					{

						if(gprs_stat.con_client[i].send_no_timeout>10)
						{
							gprs_stat.con_client[i].send_no_timeout = 0;
							gprs_stat.con_client[i].connect_ok = 0;
							gprs_stat.con_client[i].send_cmd_timeout = 0;
							gprs_stat.con_client[i].send_cmd_times = 0;							
							memset(at_cmd_conn,0,50);
							sprintf(at_cmd_conn,"AT+CIPCLOSE=%d\r\n",i);
							gprs_uart_send_string(at_cmd_conn);	 //关闭链接
							
							gprs_stat.now_who_working = -1;
							gprs_stat.send_steps = 0;		
						}					
					}
					else if(gprs_stat.send_steps == GPRS_SEND_OK)  //收到发送结果，表示发送成功
					{
						gprs_stat.con_client[i].send_no_timeout = 0;
						gprs_stat.con_client[i].send_len = 0;	
								
						gprs_stat.now_who_working = -1;
						gprs_stat.send_steps = 0;			
							
					}
					return;
				}
			}
			else
			{
				get_link_stat++;
				if(get_link_stat == 15)
				{	
					gprs_uart_send_string(AT_CMD_AT_CSQ);
					return;
				}
				if(get_link_stat>30)
				{
					get_link_stat = 0;
					memset(at_cmd_conn,0,50);
	#ifdef M35_2G				
					sprintf(at_cmd_conn,AT_CMD_AT_QISACK ,0);	
	#else				
					sprintf(at_cmd_conn,AT_CMD_4GAT_CIPSTAT ,0);					
	#endif								
					gprs_stat.now_read_who_txrx = 0;
					gprs_uart_send_string(at_cmd_conn);
					return;
				}
			}
			
		}
	}
	*/
	
}

// data[0] 本包数据长度  data[1]总包数   data[2345] N1ID data[6] 当前包序号 data[7] 开始是透传数据
// 组包
void send_gprs_data(void *pdata,int len)
{
	unsigned char *pd = (unsigned char *)pdata;
	
//	if(gprs_stat.send_data_id != -1 && gprs_stat.send_data_num >0)  //缓冲里有数据
//		return;
	
//	if(pd[6] == 0)  //收到第0包 是个新的开始 刷新包数据
//	{
//		gprs_stat.packet_seq = 0;
//		gprs_stat.send_data_num = 0;		
//	}
//	else if(pd[6] != gprs_stat.packet_seq) //N1来的包序号 对不上已经保存的包序号 不接受组包
//	{
//		return;
//	}
//	//第0包  或者 N1发来的包序号等于需要的包序号
//	memcpy(n1_to_server_data+gprs_stat.send_data_num,&pd[7],pd[0]-8);
//	gprs_stat.send_data_num += pd[0]-8;
//	gprs_stat.packet_seq++;
//	
//	if(gprs_stat.packet_seq == pd[1]) //已经是完整的包
//	{
//		gprs_stat.send_data_id = 0;
//		gprs_stat.send_data_buff = n1_to_server_data;
//			
//	}


}





void send_gprs_200_()
{
//	memset(n1_to_server_data,0x55,200);
//	gprs_stat.send_data_id = 0;
//	gprs_stat.send_data_buff = n1_to_server_data;
//	gprs_stat.send_data_num = 200;
}

char* make_gprs_stat()
{
	sprintf(gprs_debug_buff,"gprs:ati=%d creg=%d cgreg=%d tcp=%d:%d csq=%d\r\n",gprs_stat.ati_ok,gprs_stat.creg_ok,gprs_stat.cgreg_ok,gprs_stat.con_client[0].connect_ok,gprs_stat.con_client[1].connect_ok,gprs_stat.csq);
	return gprs_debug_buff;
}


/*

client:   0-> 0链接 公司后台     1-> 1链接  甲方平台

ret  :    ret == 0    写入成功
          ret <  0    正在忙  写入失败
				  ret >  0    写入超出范围  返回最大可写字节数

*/
int8_t send_to_4g(int8_t client,void *pdata,int16_t len)
{
	if(client < 0 || client > 1)
		return -2;   //无此链接
	
	if(gprs_stat.now_who_working != client) //空闲中
	{
		if((_4G_BUFF_LEN - gprs_stat.con_client[client].send_len) > len)  //还有空间
		{
			memcpy( &gprs_stat.con_client[client].buf[len],pdata,len);
			gprs_stat.con_client[client].send_len += len;
			return 0;
		}
		else
			return (_4G_BUFF_LEN - gprs_stat.con_client[client].send_len);  //空间不够  返回可写大小
	}

	return -1;  //正在发送 返回不能写
}



uint8_t * get_4g_data_buff(int8_t client,uint16_t len)
{
	if(client < 0 || client > 1)
		return (uint8_t *)-2;   //无此链接
	
	if(gprs_stat.now_who_working != client) //空闲中
	{
		if((_4G_BUFF_LEN - gprs_stat.con_client[client].send_len) > len)  //还有空间
		{
			return &gprs_stat.con_client[client].buf[gprs_stat.con_client[client].send_len];
		}
		else
			return (uint8_t *)-1;;  //空间不够  返回可写大小
	}

	return (uint8_t *)0;  //正在发送 返回不能写	
}

uint8_t * get_4g_data_buff_max(int8_t client,uint16_t *len)
{
	if(client < 0 || client > 1)
		return (uint8_t *)-2;   //无此链接
	
	if(gprs_stat.now_who_working != client) //空闲中
	{
		if(_4G_BUFF_LEN - gprs_stat.con_client[client].send_len)  //还有空间
		{
			*len = _4G_BUFF_LEN - gprs_stat.con_client[client].send_len;
			return &gprs_stat.con_client[client].buf[gprs_stat.con_client[client].send_len];
		}
		else
			return (uint8_t *)-1;;  //空间不够  返回可写大小
	}

	return (uint8_t *)0;  //正在发送 返回不能写	
}



int32_t add_4g_data_len(int8_t client,int16_t len)
{
	if(client < 0 || client > 1)
		return -2;   //无此链接
	
	if(gprs_stat.now_who_working != client) //空闲中
	{
		if((_4G_BUFF_LEN - gprs_stat.con_client[client].send_len) > len)  //还有空间
		{
			gprs_stat.con_client[client].send_len += len;
			return len;
		}
		else
			return -1;  //空间不够  返回可写大小
	}

	return 0;  //正在发送 返回不能写		
}


int32_t heart_timeout(int32_t client)
{
	if(client < 0 || client > 1)
		return -2;   //无此链接
	
	if(gprs_stat.con_client[client].connect_ok != 1)
		return -3;
	
	if((systerm_info.slot - gprs_stat.con_client[client].last_data_send_time) > (180*512))
		return 0;
	else
		return -1;
}


int32_t gps_set_rtc(struct _gps_date *p_date)
{
	
	if(p_date->year < 19 || p_date->month > 12 || p_date->date > 31 || p_date->hour > 24 || p_date->minter > 59 || p_date->sec > 59)
		return -1;
	
	rtc_date.Year = p_date->year;
	rtc_date.Month = p_date->month;
	rtc_date.Date =  p_date->date;
	
	rtc_time.Hours = p_date->hour;
	rtc_time.Minutes = p_date->minter;
	rtc_time.Seconds = p_date->sec;
	
	HAL_RTC_SetDate(&hrtc,&rtc_date,RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc,&rtc_time,RTC_FORMAT_BIN);
	return 0;
}
