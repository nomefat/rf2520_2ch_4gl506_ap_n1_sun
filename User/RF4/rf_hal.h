#ifndef _RF_HAL_H_
#define _RF_HAL_H_

#include "stm32f4xx_hal.h"

//定义射频模块默认处于正常工作  单载波 调制波 或者 空闲状态
#define RF_DEFAULT_MODE RF_WORK



#define RF1 &hspi4
#define RF2 &hspi1
#define RF3 &hspi5
#define RF4 &hspi3



#define EN_RF1 (1<<0)
#define EN_RF2 (1<<1)
#define EN_RF3 (1<<2)
#define EN_RF4 (1<<3)


#define RF_WORK                         0
#define ENABLE_NOMODULATE_CARRIER       1
#define ENABLE_MODULATE_CARRIER         2
#define RF_IDLE                         3

#define RF_POWER_ON    1
#define RF_POWER_OFF   0
#define RF_POWER_WORK  2

typedef enum 
{	
		RF_SPI_ERROR  =  -1,
		RF_NO_GET_0X84 = -2,
		RF_REG_INIT_OK =  1
}RF_init_stat_typedef;
	
	
	
typedef struct _rf_stat
{
	char rf_power_stat;
	RF_init_stat_typedef reg_init_stat;
	unsigned char mode;
	unsigned int rf_no_data_sec;
	unsigned int rf_no_data_reboot_times;
	unsigned int rf_send_error_reboot_times;
}struct_rf_stat;

typedef struct _test_1000p_data
{
	uint8_t len;
	uint32_t head;
	uint16_t pakcet_count;
	uint16_t packet_seq;
	uint8_t data[16];
}struct_test_1000p_data;


void rf_satt_init();
void rf_io_tx_4();

void rf_scan_channel();





HAL_StatusTypeDef CC2520_send_cmd(SPI_HandleTypeDef* hspi,uint8_t data);



















#endif


