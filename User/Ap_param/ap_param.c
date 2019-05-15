#include "stm32f4xx_hal.h"
#include "ap_param.h"
#include "eeprom.h"
#include "string.h"






extern CRC_HandleTypeDef hcrc;


//extern uint8_t ee_data_write[256];

//extern uint8_t ee_data_read[256];


struct_sys_flash_param sys_flash_param ;

struct_sensor_rp_param sensor_rp_param ;



void init_ap_param(void)
{
	
	sys_flash_param.ap_param.ap_id = *(uint32_t *)0x0800001c;
	if(sys_flash_param.ap_param.ap_id == 0)
		sys_flash_param.ap_param.ap_id = HAL_CRC_Calculate(&hcrc,(uint32_t *)0x1fff7a10,3);
	sys_flash_param.ap_param.ap_version = AP_VERSION;
	sys_flash_param.ap_param.band_id = sys_flash_param.ap_param.ap_id;
	sys_flash_param.ap_param.ap_channel = 0x1f1f1f1f;
	sys_flash_param.ap_param.ap_syn_param[0] = 0x10;
	
	sys_flash_param.global_cfg_param.server_ip[0] = (219 | (239<<8) | (83<<16) | (74<<24));
	sys_flash_param.global_cfg_param.server_port[0] = 40010;
	
	sys_flash_param.global_cfg_param.data_save_timer_time = 300;
	sys_flash_param.global_cfg_param.dev_reset_hour = 4;
	sys_flash_param.global_cfg_param.dev_reset_min = 1;
	sys_flash_param.global_cfg_param.dev_reset_switch = 0;
	sys_flash_param.global_cfg_param.m_usCarLimit = 420;
	sys_flash_param.global_cfg_param.m_arDelimiter[0] = 5500;
	sys_flash_param.global_cfg_param.m_arDelimiter[1] = 8000;
	sys_flash_param.global_cfg_param.m_arDelimiter[2] = 12000;
	sys_flash_param.global_cfg_param.m_arDelimiter[3] = 16000;

}


//void read_ap_param(void)
//{
//	int32_t delay;
//	int32_t times = 0;
//	
//	while(1)
//	{
//		HAL_GPIO_TogglePin(general_led_2_GPIO_Port,general_led_2_Pin);
//		if(HAL_OK==ee_read_no(EEPROM_AP_PARAM_BEGIN,sizeof(struct_ap_param)))
//		{
//			if(ee_data_read[0] != 0xff && ee_data_read[0] != 0)
//			{
//				memcpy(&ap_param,ee_data_read,sizeof(struct_ap_param));
//				ap_param.ap_version = AP_VERSION;
//				return;
//			}
//			else
//			{
//				init_ap_param();
//				memcpy(ee_data_write,&ap_param,sizeof(struct_ap_param));
//				if(HAL_OK==ee_write_no(EEPROM_AP_PARAM_BEGIN,sizeof(struct_ap_param)))
//				{					
//					continue;
//				}
//			}
//		}
//		delay = 100000;
//		while(delay--);
//		times++;
//	}
//}

