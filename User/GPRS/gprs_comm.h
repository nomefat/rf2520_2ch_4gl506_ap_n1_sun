#ifndef GPRS_COMM_H_
#define GPRS_COMM_H_

#include "stm32f4xx_hal.h"

#include "gprs_4g_app.h"


typedef struct struct_gprs_stat{
	signed char ati_ok;
	signed char creg_ok;       
	signed char cgreg_ok;  	
	signed char cereg_ok;          //4G网络注册标志位
	signed char set_timeout;         //设置超时时间标志位
	signed char netopen;             //打开网络标志位
	signed char now_who_rx;         //最后一条数据是哪个链接收到的
	unsigned short now_rx_len;      //最后一条数据的长度
	signed char now_read_who_txrx;    //现在在读取哪个链接的收发数据长度
	volatile signed char now_who_working;  //指示现在那个链接正在发送数据
	volatile signed char send_steps;       //指示发送数据的状态   
	#define GPRS_SEND_WAIT_S 1           //已经发送 cipsend命令，等待>
	#define GPRS_SEND_WAIT_SENDOK 2      //已经发送数据  等待模块返回send ok  +cipsend:success,1,20,4
	#define GPRS_SEND_OK            3     //发送完成
 	char csq;              //信号质量
	char reboot_flag;
#define GPRS_REBOOT_SETP0 0               //执行关闭GSM模块电源指令
#define GPRS_REBOOT_SETP1 1               //执行打开GSM模块电源指令
#define GPRS_REBOOT_SETP2 4               //GSM已经上电 不执行操作	
	struct {
		char connect_ok;
		uint8_t connect_fail_times;	//连接失败的次数
		uint8_t send_cmd_timeout; //发送连接命令的超时
		uint8_t send_cmd_times;    //发送连接命令的次数
		uint32_t send_no_timeout;            //发送后直接没有收到 > 超时
		uint8_t send_no_times;
		int gprs_send_error_timeout;   // tcp发送数据没有成功计数	
		uint32_t tx_len;              //发送了多少字节
		uint32_t rx_len;  		        //接收了多少字节
		uint16_t send_len;
		uint32_t last_data_send_time;
		#define _4G_BUFF_LEN 1400
		uint8_t buf[_4G_BUFF_LEN];
	}con_client[CLIENT_NUM];

}struct_gprs_stat;


typedef struct _gprs_one_cmd{
	uint8_t index;
	uint8_t data[256]; //用于从队列中取出完整的一条命令
}struct_gprs_one_cmd;


void dtu_cmd_param(char *pstr , unsigned short length);
void gprs_get_cmd_param(char *pstr , unsigned short length);

#endif


