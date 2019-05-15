#ifndef RF_DATA_HANDLE_H_
#define RF_DATA_HANDLE_H_

#include "stm32f4xx_hal.h"
#include "snp.h"



#define LANE_SENSOR_MAX_COUNT 64         //有车道的sensor最大数量
#define NOLANE_SENSOR_MAX_COUNT 64      //无车道的sensor最大数量
#define RP_MAX_COUNT 24                  //RP列表
#define EVENT_MAX 10                    //列表中最大缓存的事件个数

#define BEFORE 1
#define AFTER 2

#define CAR_STAT_ON 1
#define CAR_STAT_OFF 0

#define SENSOR_EVENT_PACKET_HEAD_SIZE 10    //事件包除了事件的长度，用来计算事件个数

#define ROW_MAX 64
#define LINE_MAX 200


#pragma pack(1)


typedef struct 
{
	unsigned short m_usDotId;		/*检测器ID*/
	unsigned char  m_ucLaneId;		/*车道号*/
	unsigned char  m_ucPosition;	/*位置前置0-后置1*/
	unsigned short m_usDistance;	/*间隔距离*/
	unsigned char  m_ucSectionId;	/*0 路口 1路段*/
	unsigned char  m_ucCcexChannel; /*ccex*/
	
}_dot_info;

typedef struct
{
	unsigned char  m_ucLaneType;  /*车道类型 1左转 2直行3右转4掉头可自由分配*/
	unsigned char  m_ucDirection;  /*方向*/
	unsigned char  m_ucChannel;   /* RF通道 */
	unsigned short m_ucpid;      /*父ID */
	unsigned char  m_ucmode;     /*工作模式*/
	unsigned int   m_innum;		/*排列序号*/
	unsigned char  m_vLaneID;	/*虚拟车道号 对应web下发的车道号*/
	
}_dot_info_ext;

typedef struct
{
	unsigned short m_usRpId;              /*Rp id*/
	unsigned char  m_ucDirection;         /*方向*/
	unsigned char  m_ucChannel;           /*RF通道   up*/  
	unsigned short m_ucpid;				  /*父ID*/
	unsigned int   m_innum;					/*排列序号*/
	
}_RP_info;


typedef struct _global_cfg_param
{
	uint16_t data_save_timer_time;       //定时数据定时时间   秒
	uint16_t m_usCarLimit;								/*分车阈值 ms*/
	uint16_t m_usOndelay;									/*ON delay*/
	uint16_t m_usDelayTime;                               /*延时时间*/
	uint16_t m_usMinThroughTime;							/*最小通过时间ms*/
	uint16_t m_arDelimiter[4];							/*分车型 mm*/	
	uint8_t dev_reset_switch;             //系统定时复位开关
	uint8_t dev_reset_hour;
	uint8_t dev_reset_min;          //定时复位时间点
	uint32_t server_ip[2];          //公司服务器ip
	uint16_t server_port[2];        //公司服务器端口
	uint8_t realtime_data_type[2];          //千方 和 德州
	#define RDT_QF  1
	#define RDT_DZ  2
	uint8_t realtime_data_switch[2];        //实时数据开关
	uint8_t timer_data_switch[2];          //定时数据开关
	uint8_t timer_stat_switch[2];          //定时状态开关
}struct_global_cfg_param;

#pragma pack()






typedef struct _sensor_cfg
{

	uint16_t sensor_before_to_after_distance;        //前后置检测器距离
	uint8_t road_end_or_in;      //路口或者路段
		#define ROAD_END 0
		#define ROAD_IN 1
	uint8_t ccex;
	uint8_t lane_type;          //车道类型
		#define LANE_LEFT 1        //左转
		#define LANE_DIRECT 2      //直行
		#define LANE_RIGHT 4       //右转
		#define LANE_TURN_ROUND 8   //掉头
	uint8_t lane_direction;    //车道方向
	uint8_t rf_ch;
	uint16_t up_rp_id;
	uint8_t work_mode;
	uint32_t   m_innum;		/*排列序号*/
	uint8_t  m_vLaneID;	/*虚拟车道号 对应web下发的车道号*/	
	uint8_t slot;
	uint8_t level;
}struct_sensor_cfg;


typedef struct _rp_cfg
{
	uint8_t lane_direction;    //车道方向
	uint8_t rf_ch;
	uint16_t up_rp_id;
	uint32_t  m_innum;					/*排列序号*/	
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
}struct_rp_cfg;





typedef	struct _event_and_info{
		SNP_EVENT_t event;
		uint32_t event_rev_time_slot;   //事件收到的时间 系统时间槽计数
		uint8_t event_valid;    //事件是有效的
			#define EVENT_HANDLE_ZERO 0      //未处理的
			#define EVENT_HANDLE_ONE 1      //处理过一次 需要根据下一个事件结算的
			#define EVENT_HANDLE_END 2      //最终结果
}struct_event_and_info;

typedef struct _sensor_event
{
	struct_event_and_info event[10];
	
	volatile uint8_t event_count;          //缓存sensor event个数
	volatile uint8_t last_event_packet_seq;   //当前事件包序号	
	volatile int8_t has_event_no_handle;     //指示还有未处理的sensor事件
	uint8_t last_packet_event_count;   //上一包的事件个数
	uint8_t car_zhou;   //sensor 上报的车轴数
	struct{   //计算需要使用的过程数据
		SNP_EVENT_t last_pcket_event[16];   //上一包的事件                  
		int8_t now_on_off;                 //当前的ON OFF状态
		uint16_t off_to_on_timeslot; //off-on
		uint16_t on_to_off_timeslot; //on-off
		struct_event_and_info on_event_info;   //用来计算车头时距
		SNP_EVENT_t car_onoff_event[2];  //一辆车的on off 事件记录
	};
	struct{       //实时数据
		uint16_t speed;                 //车速  mm/S
		uint16_t car_length;             //车长 MS
		uint32_t car_count;              //车辆计数
		uint16_t car_head_time_distance; //车头时距 MS
		uint16_t car_car_time_distance;   //车间距 
	};
	struct{
		uint32_t car_len_count[6];  //5种车型流量统计 5表示总数
		uint32_t occupancy;         //占有率
		uint32_t avg_speed;
		uint32_t max_speed;
		uint32_t min_speed;
		uint32_t avg_car_length;             //车长 MS
		uint32_t avg_car_head_time_distance; //车头时距 MS
		uint32_t avg_car_car_time_distance;   //车间距 		
		}t[2];  // 0:临时数据   1：统计数据
	
	uint8_t last_resend_times;		
	uint16_t resend[10]; //重传统计
	uint32_t all_rev_event_count;
	uint32_t all_lost_event_count;	
}struct_sensor_event;

typedef struct _sensor_stat
{
	int8_t new_7fff_flag;
	uint32_t timeslot_7fff_event;
	int8_t rssi;
	int8_t luzl;
	uint8_t slot;
	int8_t avg_rssi1;
	int32_t avg_rssi;
	int32_t avg_volt;
	uint8_t lost_rate;
	uint16_t packet_count;  //总包数 长期保持 
	uint32_t timer_lost_packet_num;   //定时时间内丢了的报数
	uint32_t timer_packet_num;	   //定时时间内包数
	uint16_t t_resend[10];  //定时时间内重传统计
	SNP_STATE_PACKET_SENSOR_t sensor_stat_packet;
	
}struct_sensor_stat;

typedef struct _rp_stat
{
	int8_t new_7fff_flag;
	uint32_t timeslot_7fff_event;
	int8_t rssi;
	int8_t luzl;
	uint8_t slot;
	int32_t avg_rssi;
	int32_t avg_volt;
	uint32_t timer_packet_num;    //定时时间内包数
	uint32_t event_rev_time_slot;   //事件收到的时间 系统时间槽计数
	SNP_STATE_PACKET_RP_t rp_stat_packet;
	
}struct_rp_stat;


typedef struct _lane_and_sensor{
			uint16_t sensor_id;
			uint32_t last_packet_time_slot;
			uint16_t set_param_times;
			uint8_t updata_enable;
			struct_sensor_cfg sensor_cfg;          //sensor配置信息
			struct_sensor_event sensor_event;
			struct_sensor_stat sensor_stat;
}struct_lane_and_sensor;


/*sensor 车道信息 事件 以及计算过车结果*/
typedef struct _lane_to_sensor_info_and_result
{
	uint8_t has_lane_sensor_num;   //配置的sensor个数
	uint8_t no_lane_sensor_num;    //没有配置的个数
	uint8_t cfg_rp_num;           //配置的rp个数
	uint8_t rp_num;        //上来的个数
	
	struct{
		struct_lane_and_sensor before;                                 //前置sensor
		struct_lane_and_sensor after;		                              //后置sensor
	}lane_and_sensor[LANE_SENSOR_MAX_COUNT];                        //有配置的sensor
	
	struct{
		uint16_t rp_id;
		uint32_t last_packet_time_slot;
		uint16_t set_param_times;
		uint8_t updata_enable;
		struct_rp_cfg rp_cfg;
		struct_rp_stat rp_stat;
	}rp_cfg_and_stat[RP_MAX_COUNT];      //rp
	
	
	struct{
		uint16_t sensor_id;
		uint32_t last_packet_time_slot;
		struct_sensor_event sensor_event;
		struct_sensor_stat sensor_stat;
	}no_lane_sensor[NOLANE_SENSOR_MAX_COUNT];                  //没有配置的sensor
	
}struct_lane_to_sensor_info_and_result;





extern int32_t print_event_handle_guocheng ;     //打印ON OFF过滤详细过程

extern int32_t print_one_car;   //打印实时过车结果

int insert_sensor_event(SNP_SEN_MODE_B_PACKET_t *ptr_s_event,int8_t rssi,uint8_t luzl,uint8_t slot);
void sensor_event_and_stat_hanle();
void insert_sensor_stat_packet(SNP_STATE_PACKET_SENSOR_t *pstat,int8_t rssi,uint8_t luzl,uint8_t slot);
int32_t insert_rp_stat_packet(SNP_STATE_PACKET_RP_t *pstat,int8_t rssi,uint8_t luzl,uint8_t slot);
void add_sensor_cfg(uint8_t lane,uint16_t sid,uint8_t before_or_after);
void make_timer_statistics_data(uint32_t timer_time_ms);

extern struct_lane_to_sensor_info_and_result lane_to_sensor_info_and_result;


#endif


