#ifndef EE_PROM_H_
#define EE_PROM_H_


#include "stm32f4xx_hal.h"




#define EE_DEV_ADDRESS  0XA0






typedef struct _ee_task
{
	int8_t read_write;
#define EE_WRITE 1
#define EE_READ  2
	uint32_t ee_address;
	uint16_t size;
  int8_t timeout;		
	uint8_t fail_times;
}struct_ee_task;




extern struct_ee_task ee_task;








#define EEPROM_AP_PARAM_BEGIN 0                     //AP参数起始地址
#define EEPROM_DEV_LIST_BEGIN 1024                  //设备表起始地址
#define EEPROM_RP_FIRMWARE_BEGIN 10*1024            //RP固件起始地址
#define EEPROM_SENSOR_FIRMWARE_BEGIN 110*1024       //sensor固件起始地址
#define EEPROM_ELSE_BEGIN 210*1024                  //其他起始地址



HAL_StatusTypeDef ee_read_no(uint32_t ee_address,uint16_t size);
HAL_StatusTypeDef ee_write_no(uint32_t ee_address,uint16_t size);

HAL_StatusTypeDef ee_write(uint32_t ee_address,uint16_t size);
HAL_StatusTypeDef ee_read(uint32_t ee_address,uint16_t size);



#endif


