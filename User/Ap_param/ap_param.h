#ifndef _AP_PARAM_H_
#define _AP_PARAM_H_


#include "stm32f4xx_hal.h"
#include "rf_data_handle.h"
#include "gprs_4g_app.h"


//#define AP_VERSION  0X8888                 //作为工装使用   增加自动校准sensor
//#define AP_VERSION  0X8889               //打印接收syn的工装  可以设置rssi过滤
#define AP_VERSION  0X9228



//0006 更改4个射频通道跟N1 WEB显示颠倒的bug
//0007 RF -18db
//0008 增加判断rf同步包发送失败重启RF rf120s没有接收到数据重启RF
//0009 晶振更改为26M
//000A 和0009程序一样 测试升级使用
//000c 26m 增加每帧复位RF
//000d 26m 增加打印sensor事件包 使能功能
//000e 26m 增加LED crc 显示3路串口接收
//000F 26M 增加了3路串口接收中断 溢出处理
//0010 26M RF -18DB
//0011 26M RF -4DB
//0012 26M 通道大于29 关闭rf
//0013 26m 升级包最有发送FFFF标志的时候 重复发送20次
//0003 8M 等同 0013
//0014 26M 增加rf包大小限制 避免异常包 增加D模式
//00A4 26M 不过滤事件包时间槽 AB 47
//0015 26M RP过滤字节取低6位
//0016 26M 针对只用2路RF做过修改
//0017 rp sensor 固件判断有没有重复行
//0118 针对升级RP 丢失状态包情况 增加rf接收判断溢出并清理  收到数据crc错误fulsh rx
//0119 升级包中的CRC_NUM 改用和N1通信的crc函数
//011A 设置RP参数的时候把sensor_mode字段也修改了
//011b 增加控制同步包发送的指令
//011c 出厂默认通道改为31

//0X9228   修正4G模块电源无法关闭的bug


#pragma pack(1)



typedef struct _ap_param{

	uint32_t ap_id;                                                           //     N1只读
	uint16_t ap_version;                                                      //     N1只读
	uint16_t rp_version;                //eeprom 中rp固件版本号                       N1只读
	uint16_t sensor_version;						   //eeprom 中sensor固件版本号                N1只读	
	struct _gps_date rtc_date_time; 
	uint16_t band_id;	
	uint32_t ap_channel;
	uint8_t ap_syn_param[6];
	float gps_n_e[2];
	
}struct_ap_param;



typedef struct{
	
 uint8_t uiCmd;  //命令2-检测器重校准 3-检测器模式设置  11 -参数全配置 
 uint16_t uiPoll;   //目的ID
 
 uint16_t uiBindId; //绑定ID
 struct{

  uint16_t uimySlot:8,     //8位-本机时间槽
     uiSlotStateE:8; //8位-开关时间槽扩展
  
 }paraB; 
 uint16_t uiSlotStateL;//开关时间槽低16
 uint16_t uiSlotStateM;//开关时间槽中16
 uint16_t uiSlotStateH;//开关时间槽高16
 struct{  
  uint8_t uiGrade:3, //跟的同步包级别0-3
    uiChannel:5;//设置的通道0-31
 }paraA;
 
}rp_param;




typedef struct{

 uint8_t ucSensorMode;
 rp_param ParaFram;
 
}struct_sensor_rp_param;



extern struct_sensor_rp_param sensor_rp_param ;



typedef	struct{
		_dot_info sensor_param1;
		_dot_info_ext sensor_param2;
		uint8_t level;
		uint8_t slot;
}s_sensor_pram;

	
typedef	struct{
	 _RP_info rp_param;
	 struct{
		uint16_t uimySlot:8,     //8位-本机时间槽
			 uiSlotStateE:8; //8位-开关时间槽扩展
		
	 }paraB; 
	 uint16_t uiSlotStateL;//开关时间槽低16
	 uint16_t uiSlotStateM;//开关时间槽中16
	 uint16_t uiSlotStateH;//开关时间槽高16
	 struct{  
		uint8_t uiGrade:3, //跟的同步包级别0-3
			uiChannel:5;//设置的通道0-31   down
	 }paraA;		
}s_rp_pram;

typedef struct _sys_flash_param
{
	struct_ap_param ap_param;
	struct_global_cfg_param global_cfg_param;  //配置参数表
	uint8_t sensor_num;
	uint8_t rp_num;	
	s_sensor_pram sensor[64];
	s_rp_pram rp[16];
}struct_sys_flash_param;






typedef struct _sensor_rp_updata_stat
{
	uint16_t sensor_id;
	uint16_t version;
	uint16_t timeout_sec;
	int16_t updata_seq;		
}struct__sensor_updata_stat;








#pragma pack()



extern struct_sys_flash_param sys_flash_param ;

void init_ap_param(void);



#endif


