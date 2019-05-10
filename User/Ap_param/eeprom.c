#include "stm32f4xx_hal.h"
#include "eeprom.h"
#include "string.h"
#include "ap_param.h"
#include "to_n1.h"


extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

void MX_I2C1_Init(void)
{}
	
extern void MX_I2C1_Init(void);

extern char gprs_debug_buff[256];
extern void copy_string_to_double_buff(char *pstr);



extern int16_t ee_write_packet_syn ;
extern uint32_t ee_write_addr_add ;





uint8_t ee_data_write[256];

uint8_t ee_data_read[256] = {0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55,0x55};


struct_ee_task ee_task;


/*
1.eeprom 地址分配
	0-1024 ：  AP_param
	1024-10*1024 dev_list
	10*1024 -110*1204 rp_firmware
	110*1024-210*1024 sensor_firmware

*/






HAL_StatusTypeDef ee_read(uint32_t ee_address,uint16_t size)
{
	uint16_t dev_address;

	dev_address = EE_DEV_ADDRESS;
	dev_address |= ((ee_address>>16)&0x06);
	dev_address |= 0x0001;
	dev_address |= 0x0008;
	
	return HAL_I2C_Mem_Read_DMA(&hi2c1,dev_address,ee_address,I2C_MEMADD_SIZE_16BIT,ee_data_read,size);


}


HAL_StatusTypeDef ee_write(uint32_t ee_address,uint16_t size)
{
	uint16_t dev_address;

	dev_address = EE_DEV_ADDRESS;
	dev_address |= ((ee_address>>16)&0x06);
	dev_address &= 0xfffe;
	dev_address |= 0x0008;
	
	return HAL_I2C_Mem_Write_DMA(&hi2c1,dev_address,ee_address,I2C_MEMADD_SIZE_16BIT,ee_data_write,size);


}

HAL_StatusTypeDef ee_write_no(uint32_t ee_address,uint16_t size)
{
	uint16_t dev_address;

	dev_address = EE_DEV_ADDRESS;
	dev_address |= ((ee_address>>16)&0x06);
	dev_address &= 0xfffe;
	dev_address |= 0x0008;
	
	return HAL_I2C_Mem_Write(&hi2c1,dev_address,ee_address,I2C_MEMADD_SIZE_16BIT,ee_data_write,size,25);


}

HAL_StatusTypeDef ee_read_no(uint32_t ee_address,uint16_t size)
{
	uint16_t dev_address;

	dev_address = EE_DEV_ADDRESS;
	dev_address |= ((ee_address>>16)&0x06);
	dev_address |= 0x0001;
	dev_address |= 0x0008;
	
	return HAL_I2C_Mem_Read(&hi2c1,dev_address,ee_address,I2C_MEMADD_SIZE_16BIT,ee_data_read,size,25);


}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(ee_task.read_write == EE_READ)
	{
		if(ee_task.ee_address == 0)
		{
			memcpy(&sys_flash_param.ap_param,ee_data_read,sizeof(sys_flash_param));
		}
		memset(&sys_flash_param.ap_param,0,sizeof(sys_flash_param));		
		ee_task.ee_address = 0;
		ee_task.fail_times = 0;
		ee_task.read_write = 0;
		ee_task.size = 0;
		ee_task.timeout = 0;
	}
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	uint8_t data[2];
	
	if(ee_task.read_write == EE_WRITE)
	{
		
		if(ee_task.ee_address > EEPROM_RP_FIRMWARE_BEGIN)
		{
			ee_write_addr_add += ee_task.size;
			ee_write_packet_syn++;
			data[0] = 0;
			data[1] = ee_write_packet_syn;
			insert_to_n1_buff(data,2,1);
			sprintf(gprs_debug_buff,"eeprom:write ok packet_syn = %d\r\n",ee_write_packet_syn);
			copy_string_to_double_buff(gprs_debug_buff);				
		}
		
		ee_task.ee_address = 0;
		ee_task.fail_times = 0;
		ee_task.read_write = 0;
		ee_task.size = 0;
		ee_task.timeout = 0;		
	}	
}



void ee_task_poll(void)
{
	if(ee_task.read_write > 0)   //有读写ee的任务
	{
		if(ee_task.timeout>10)  //任务超时没有完成
		{
			ee_task.timeout = 0;
			ee_task.fail_times ++;
			if(ee_task.fail_times>30)
			{
				ee_task.ee_address = 0;
				ee_task.fail_times = 0;
				ee_task.read_write = 0;
				ee_task.size = 0;
				ee_task.timeout = 0;		
				return;
			}
			if(ee_task.read_write == EE_WRITE)
			{
				if(ee_write(ee_task.ee_address,ee_task.size) != HAL_OK)
				{
					copy_string_to_double_buff("eeprom: read error reinit\r\n");
					MX_I2C1_Init();
				}
				else
				{
					sprintf(gprs_debug_buff,"eeprom:read addr=%d fail=%d\r\n",ee_task.ee_address,ee_task.fail_times);
					copy_string_to_double_buff(gprs_debug_buff);					
				}
				
			}
			else if(ee_task.read_write == EE_READ)
			{
				if(ee_read(ee_task.ee_address,ee_task.size) != HAL_OK)
				{
					copy_string_to_double_buff("eeprom: write error reinit\r\n");
					MX_I2C1_Init();
				}	
				else
				{
					sprintf(gprs_debug_buff,"eeprom:write addr=%d fail=%d\r\n",ee_task.ee_address,ee_task.fail_times);
					copy_string_to_double_buff(gprs_debug_buff);					
				}
								
			}
		}
	}
}





