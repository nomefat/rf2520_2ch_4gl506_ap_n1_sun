#include "stm32f4xx_hal.h"
#include "typedef_struct.h"
#include "rf_hal.h"
#include "hal_cc2520.h"
#include "debug_uart.h"
#include "ap_param.h"
#include "debug_uart.h"
#include "stdio.h"
#include "gprs_4g_app.h"


#define INCLUDE_PA

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi3;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;

int rf_io_tx(SPI_HandleTypeDef* hspi);
void rf_cmd_tx(SPI_HandleTypeDef* hspi);
void rf_rx(SPI_HandleTypeDef* hspi);
void rf_cs_off(SPI_HandleTypeDef* hspi);
void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel );
void rf_power_reset(SPI_HandleTypeDef* hspi);


extern struct_auto_cfg auto_cfg_stat[CLIENT_NUM]; 
extern void rf_rx_data_handle(int index);
extern struct_systerm_info systerm_info;

struct_rf_stat rf_stat[4];

SPI_HandleTypeDef* rf_index[] = {RF4,RF3,RF2,RF1};

uint8_t rf_sec_flag = 0;         //Ãëµã±êÖ¾ ÓÉÍ¬²½°üÖÐ¶ÏÖÃ1

uint8_t rf_scan_channel_enable = 0;  //µ¥ÔØ²¨É¨ÃèÍ¨µÀÊ¹ÄÜ

uint8_t rf_send_1000_p_enable = 0;   //·¢ËÍ1000°üÊ¹ÄÜ

unsigned char rf_send_data[15] = {0x0f,0x00,0x00,0x00,0x00,0x05,0x06,0x07,0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e};

uint8_t rf_rx_buff[4][256];

uint32_t syn_send_error = 0;


struct_test_1000p_data test_1000p_data=
{
	sizeof(struct_test_1000p_data),
	0xaaaa5555,
	1000,
	0,
};



extern char gprs_debug_buff[256];




int spi_to_index(SPI_HandleTypeDef* hspi)
{
	if( hspi == &hspi3) 
		return 0;
	else if(hspi == &hspi5) 
		return 1;	
	else if(hspi == &hspi1)  
		return 2;
	else if(hspi == &hspi4) 
		return 3;	
	else
		return 0;
}




HAL_StatusTypeDef CC2520_set_reg(SPI_HandleTypeDef* hspi,uint16_t addr,uint8_t data)
{

	HAL_StatusTypeDef ret;
	uint8_t cmd[3];
	cmd[0] = (addr>>8) | CC2520_INS_MEMWR;
	cmd[1] = addr &0x00ff;
	cmd[2] = data;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_Transmit(hspi,cmd,3,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return ret;

}

HAL_StatusTypeDef CC2520_send_cmd(SPI_HandleTypeDef* hspi,uint8_t data)
{

	HAL_StatusTypeDef ret;


	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_Transmit(hspi,&data,1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return ret;

}

uint8_t CC2520_get_data(SPI_HandleTypeDef* hspi,uint8_t cmd)
{

	HAL_StatusTypeDef ret;
	uint8_t data;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_TransmitReceive(hspi,&cmd,&data,1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
	
	
	return data;

}




HAL_StatusTypeDef CC2520_get_reg(SPI_HandleTypeDef* hspi,uint16_t addr,uint8_t * pdata)
{

	HAL_StatusTypeDef ret;
	uint8_t cmd[3];
	
	cmd[0] = (addr>>8) | CC2520_INS_MEMRD;
	cmd[1] = addr &0x00ff;
	cmd[2] = 0xff;

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	
	ret = HAL_SPI_TransmitReceive(hspi,cmd,pdata,3,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);		
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
		

	return ret;	


}


void CC2520_Reg_Init(SPI_HandleTypeDef* hspi)
{
	HAL_StatusTypeDef stat;
	uint8_t data_[3];
  #ifdef INCLUDE_PA

  stat = CC2520_set_reg(hspi,CC2520_TXPOWER,     0x81);
  stat = CC2520_set_reg(hspi,CC2520_TXCTRL,      0xC1);
  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x16);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL4,   0x46);                                                                        
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL5,   0x47);
	//CC2520_MEMWR8(CC2520_GPIOCTRL5,   CC2520_GPIO_LOW);
  stat = CC2520_set_reg(hspi,CC2520_GPIOPOLARITY,0x0F);
    
  #else
  stat = CC2520_set_reg(hspi,CC2520_TXPOWER,     0x32);
  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x11);
  #endif

		  
  stat = CC2520_set_reg(hspi,CC2520_CCACTRL0,    0xF8);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL0,    0x83);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL1,    0x14);
  stat = CC2520_set_reg(hspi,CC2520_RXCTRL,      0x3F);
  stat = CC2520_set_reg(hspi,CC2520_FSCTRL,      0x5A);
  stat = CC2520_set_reg(hspi,CC2520_FSCAL1,      0x03);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST0,    0x10);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST1,    0x0E);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST2,    0x03);  
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0,    0x40);
  stat = CC2520_set_reg(hspi,CC2520_EXTCLOCK,    0x00); 
//  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL3,   CC2520_GPIO_SFD); 
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL0,   CC2520_GPIO_FIFO);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL3,   0x88);     //SFD CCA ¸ÄÎª TX RFOFF ¹Ü½Å
//	stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL2,   0x8e);
  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL1,   CC2520_GPIO_FIFOP);  
//  stat = CC2520_set_reg(hspi,CC2520_GPIOCTRL2,   CC2520_GPIO_SAMPLED_CCA);
  stat = CC2520_set_reg(hspi,CC2520_FRMFILT1,    0x18);   
  stat = CC2520_set_reg(hspi,CC2520_FRMFILT0,    0x00); 
  

	if(rf_stat[spi_to_index(hspi)].mode == RF_WORK)
	{
		stat = CC2520_get_reg(hspi,CC2520_TXPOWER,data_);
		return;
	}

//ENABLE_NOMODULATE_CARRIER	
	if(rf_stat[spi_to_index(hspi)].mode == ENABLE_NOMODULATE_CARRIER)	{

  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0 , 0x43); 
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL1 , 0x00); 
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST0 , 0x65);
 
	}
   
//  ENABLE_MODULATE_CARRIER
	if(rf_stat[spi_to_index(hspi)].mode == ENABLE_MODULATE_CARRIER)	{

  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL0 ,   0x43); 
  stat = CC2520_set_reg(hspi,CC2520_FRMCTRL1 ,   0x00); 
//  stat = CC2520_set_reg(hspi,CC2520_GPIOPOLARITY,0x00);
  stat = CC2520_set_reg(hspi,CC2520_CCACTRL0,    0xF8);
  stat = CC2520_set_reg(hspi,CC2520_EXTCLOCK,    0x00);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL0,    0x85);
  stat = CC2520_set_reg(hspi,CC2520_MDMCTRL1,    0x14);
  stat = CC2520_set_reg(hspi,CC2520_RXCTRL,      0x3F);
  stat = CC2520_set_reg(hspi,CC2520_FSCTRL,      0x5A);
  stat = CC2520_set_reg(hspi,CC2520_FSCAL1,      0x2B);
//  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL1,    0x11);  
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST0,    0x10);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST1,    0x0E);
  stat = CC2520_set_reg(hspi,CC2520_ADCTEST2,    0x03);  

 
//  stat = CC2520_set_reg(hspi,CC2520_AGCCTRL2,  0x00);
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST0 , 0x05);
  stat = CC2520_set_reg(hspi,CC2520_MDMTEST1 , 0x08);
//  
	}

	
}


RF_init_stat_typedef rf_init(SPI_HandleTypeDef* hspi)
{
	uint32_t tickstart = 0U;
	uint32_t Timeout ;
	uint8_t data_[3];
	uint8_t data_1[3];
	uint8_t data_2[3];
	uint8_t data_3[3];
	uint8_t data_4[3];
	uint8_t data_5[3];
	HAL_StatusTypeDef stat;
	
	
	if( hspi == &hspi3) 
	{
						
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);		
		
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms

		HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_11) == GPIO_PIN_RESET )          //ÅÐ¶Ïspi¹Ü½ÅÊÇ·ñ±»³õÊ¼»¯
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //Æô¶¯Ê§°Ü
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;

	}
	else if( hspi == &hspi1) 
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);
		
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms

		HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==GPIO_PIN_RESET )          //ÅÐ¶Ïspi¹Ü½ÅÊÇ·ñ±»³õÊ¼»¯
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //Æô¶¯Ê§°Ü
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		


	}
	else if(hspi == &hspi4)  
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);		
		
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms

		HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();

		while( HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13) == GPIO_PIN_RESET )          //ÅÐ¶Ïspi¹Ü½ÅÊÇ·ñ±»³õÊ¼»¯
		{
			if((HAL_GetTick()-tickstart) > 250)
			{				
				return RF_SPI_ERROR;                              //Æô¶¯Ê§°Ü				
			}
		
		}
		HAL_GPIO_WritePin(general_led_5_GPIO_Port,general_led_5_Pin,GPIO_PIN_SET);
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		
	}
	else if(hspi == &hspi5)  
	{
		tickstart = HAL_GetTick();
		HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_RESET);	
	  HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);			
		
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms

		HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_SET);	
	  HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
		
		tickstart = HAL_GetTick();
		while( HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_8) == GPIO_PIN_RESET )          //ÅÐ¶Ïspi¹Ü½ÅÊÇ·ñ±»³õÊ¼»¯
		{
			if((HAL_GetTick()-tickstart) > 250)
				return RF_SPI_ERROR;                              //Æô¶¯Ê§°Ü
		}
		
		tickstart = HAL_GetTick();
		while((HAL_GetTick()-tickstart) < 1);      //ÑÓÊ±1ms
		
		stat = CC2520_get_reg(hspi,0X40,data_);
		if(data_[2] != 0x84)
			return RF_NO_GET_0X84;
		
	}


	
	CC2520_Reg_Init(hspi);
	
	stat = CC2520_get_reg(hspi,CC2520_TXPOWER,     data_1);
  stat = CC2520_get_reg(hspi,CC2520_TXCTRL,      data_2);
  stat = CC2520_get_reg(hspi,CC2520_AGCCTRL1,    data_3);
  stat = CC2520_get_reg(hspi,CC2520_GPIOCTRL4,   data_4);                                                                        
  stat = CC2520_get_reg(hspi,CC2520_GPIOCTRL5,   data_5);
	sprintf(gprs_debug_buff,"rf_reg: %X %X %X %X %X \r\n",data_1[2],data_2[2],data_3[2],data_4[2],data_5[2]);
	copy_string_to_double_buff(gprs_debug_buff);	
	
	CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
	
	CC2520_send_cmd(hspi,CC2520_INS_SRXON);	
	
	return RF_REG_INIT_OK;
}


void rf_fast_init()
{
	
	uint32_t tickstart = 0U;
	int32_t Timeout = 0 ;
	uint8_t data_[3];
	HAL_StatusTypeDef stat;
	SPI_HandleTypeDef* hspi;
	uint8_t ch[4] = {0,0,0,0};
	uint8_t *p_chanel = (uint8_t *)&sys_flash_param.ap_param.ap_channel;
	
	static int index = 0;

	if(systerm_info.slot < 1024)
		return;
	
	hspi = rf_index[index];
	index++;
	if(index > 3)
		index = 0;
	
	if(auto_cfg_stat[0].auto_cfg_switch > 0 || auto_cfg_stat[1].auto_cfg_switch > 0)
		p_chanel = ch;
	else
		p_chanel = (uint8_t *)&sys_flash_param.ap_param.ap_channel;
	
	rf_power_reset(hspi);
	
	if( hspi == &hspi3) 
	{
		if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK) 
		{		
			tickstart = HAL_GetTick();
			HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);		
			
			Timeout = 500;
			while(Timeout--);      //ÑÓÊ±

			HAL_GPIO_WritePin(SPI3_rf_io_vreg_GPIO_Port,SPI3_rf_io_vreg_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);
			
			Timeout = 500;
			while(Timeout--);      //ÑÓÊ±			
			CC2520_Reg_Init(hspi);

			rf_set_channel(hspi,p_chanel[3]);	
			
			CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
			
			CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
		
		}

	}
	else if( hspi == &hspi1) 
	{
		if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK) 
		{		
			tickstart = HAL_GetTick();
			HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);
			
			Timeout = 500;
			while(Timeout--);      //ÑÓÊ±

			HAL_GPIO_WritePin(SPI1_rf_io_vreg_GPIO_Port,SPI1_rf_io_vreg_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI1_rf_io_reset_GPIO_Port,SPI1_rf_io_reset_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);
			
			Timeout = 500;
			while(Timeout--);      //ÑÓÊ			
			CC2520_Reg_Init(hspi);

			rf_set_channel(hspi,p_chanel[1]);
			
			CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
			
			CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
		}
	}
	else if(hspi == &hspi4)  
	{
		if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK) 
		{
			tickstart = HAL_GetTick();
			HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);		
			
			Timeout = 500;
			while(Timeout--);      //ÑÓÊ±

			HAL_GPIO_WritePin(SPI4_rf_io_vreg_GPIO_Port,SPI4_rf_io_vreg_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI4_rf_io_reset_GPIO_Port,SPI4_rf_io_reset_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	

			Timeout = 500;
			while(Timeout--);      //ÑÓÊ			
			CC2520_Reg_Init(hspi);

			rf_set_channel(hspi,p_chanel[0]);		
			
			CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
			
			CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
		}

	}
	else if(hspi == &hspi5)  
	{

		if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK) 
		{		
			tickstart = HAL_GetTick();
			HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_RESET);	
			HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);			

			Timeout = 500;
			while(Timeout--);      //ÑÓÊ±

			HAL_GPIO_WritePin(SPI5_rf_io_vreg_GPIO_Port,SPI5_rf_io_vreg_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI5_rf_io_reset_GPIO_Port,SPI5_rf_io_reset_Pin,GPIO_PIN_SET);	
			HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	

			Timeout = 500;
			while(Timeout--);      //ÑÓÊ
			CC2520_Reg_Init(hspi);


			rf_set_channel(hspi,p_chanel[2]);	

			CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);

			CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
		}
	}


	

	
}


void rf_write_buff(SPI_HandleTypeDef* hspi,void *ptr,int len)
{
	HAL_StatusTypeDef ret;

	uint8_t d = CC2520_INS_TXBUF;
	
//	CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);
//	CC2520_send_cmd(hspi,CC2520_INS_STXON);	
	
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	ret = HAL_SPI_Transmit(hspi,&d,1,1);
	
	ret = HAL_SPI_Transmit(hspi,ptr,len+1,1);

	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	


//	CC2520_send_cmd(hspi,CC2520_INS_STXON);	
	

	
//	rf_io_tx(hspi);	
//	rf_cmd_tx(hspi);
}

void rf_start_send()
{


}

void rf_power_off(SPI_HandleTypeDef* hspi)
{
	HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_RESET);
	if( hspi == &hspi3)
	{		
		HAL_GPIO_WritePin(SPI3_rf_power_onoff_GPIO_Port,SPI3_rf_power_onoff_Pin,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	

	}
	else if(hspi == &hspi5) 
	{
		HAL_GPIO_WritePin(SPI5_rf_power_onoff_GPIO_Port,SPI5_rf_power_onoff_Pin,GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOF,GPIO_PIN_9,GPIO_PIN_RESET);
	}
	else if(hspi == &hspi1)  
	{
		HAL_GPIO_WritePin(SPI1_rf_power_onoff_GPIO_Port,SPI1_rf_power_onoff_Pin,GPIO_PIN_RESET);
	}
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_power_onoff_GPIO_Port,SPI4_rf_power_onoff_Pin,GPIO_PIN_RESET);
	}


}

void rf_power_on(SPI_HandleTypeDef* hspi)
{
	HAL_GPIO_WritePin(SPI3_rf_io_reset_GPIO_Port,SPI3_rf_io_reset_Pin,GPIO_PIN_SET);
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_power_onoff_GPIO_Port,SPI3_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_power_onoff_GPIO_Port,SPI5_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_power_onoff_GPIO_Port,SPI1_rf_power_onoff_Pin,GPIO_PIN_SET);
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_power_onoff_GPIO_Port,SPI4_rf_power_onoff_Pin,GPIO_PIN_SET);


}


int rf_io_tx(SPI_HandleTypeDef* hspi)
{
	uint8_t data[3];
	int32_t time = 0;
	uint32_t tickstart = 0U;	
	int ret = 0;
	
	if( hspi == &hspi3) 
	{
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi5) 
	{
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi1)  
	{
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_SET);
	}
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_SET);
	}
	time = SysTick->VAL;
	
	tickstart = HAL_GetTick();
	while( CC2520_get_data(hspi,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV ))
	{
		if(HAL_GetTick()-tickstart>0)
		{
			ret = -10;
			break;
		}
	}
	
	time = time - SysTick->VAL;
	time = time;
		
	CC2520_get_reg(hspi,CC2520_EXCFLAG0 ,data); 
	if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
	{
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHTX );
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHTX ); 
		ret -= 1;
	}

	if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
	{
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHRX );
		CC2520_send_cmd(hspi,CC2520_INS_SFLUSHRX ); 
		ret -= 2;
	}	
	
	return ret;
}

int rf1_flush_rx = 0;
int rf2_flush_rx = 0;
int rf3_flush_rx = 0;
int rf4_flush_rx = 0;

void rf_io_tx_4()
{
	uint8_t data[3];
	int32_t time = 0;
	uint32_t tickstart = 0U;	
	uint8_t wait_flag = 0;
	
	static uint32_t tme[4] = {0,0,0,0};
	

	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>24)&0xff)<32)	
	{
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI3_rf_tx_GPIO_Port,SPI3_rf_tx_Pin,GPIO_PIN_SET);
		wait_flag |= 1<<3;
	}

	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>16)&0xff)<32)
	{
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI5_rf_tx_GPIO_Port,SPI5_rf_tx_Pin,GPIO_PIN_SET);
		wait_flag |= 1<<2;
	}
	
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>8)&0xff)<32)
	{
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI1_rf_tx_GPIO_Port,SPI1_rf_tx_Pin,GPIO_PIN_SET);
		wait_flag |= 1<<1;
	}
	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK && (sys_flash_param.ap_param.ap_channel&0xff)<32)
	{
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SPI4_rf_tx_GPIO_Port,SPI4_rf_tx_Pin,GPIO_PIN_SET);
		wait_flag |= 1;
	}
	
	time = SysTick->VAL;
	
	tickstart = HAL_GetTick();
	
	while(1)
	{
		if(wait_flag & 1)
		{
			
			if(!(CC2520_get_data(RF1,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				wait_flag &= 0xfe;
					HAL_GPIO_WritePin(general_led_1_GPIO_Port,general_led_1_Pin,GPIO_PIN_SET);	
			}
		}
		if(wait_flag & (1<<1))
		{
			
			if(!(CC2520_get_data(RF2,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_2_GPIO_Port,general_led_2_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xfd;
			}
		}
		if(wait_flag & (1<<2))
		{
			
			if(!(CC2520_get_data(RF3,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_3_GPIO_Port,general_led_3_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xfb;
			}
		}
		if(wait_flag & (1<<3))
		{
			
			if(!(CC2520_get_data(RF4,CC2520_INS_SNOP) & ( CC2520_STB_TX_ACTIVE_BV )))
			{
				HAL_GPIO_WritePin(general_led_4_GPIO_Port,general_led_4_Pin,GPIO_PIN_SET);	
				wait_flag &= 0xf7;
			}
		}
			
		if(wait_flag == 0)
		{
			syn_send_error = 0;
			tme[0] = 0;
			tme[1] = 0;
			tme[2] = 0;
			tme[3] = 0;
			
			break;
		}
		
		if(HAL_GetTick()-tickstart>0 || SysTick->VAL < 1000)
		{
			syn_send_error++;
			if(wait_flag & 1)
			{
				tme[0]++;
				if(tme[0]>80)
				{
					tme[0] = 0;
					rf_stat[3].rf_power_stat = RF_POWER_OFF;
					rf_stat[3].rf_send_error_reboot_times++;
				}
			}
			if(wait_flag & (1<<1))
			{
				tme[1]++;
				if(tme[1]>80)
				{
					tme[1] = 0;
					rf_stat[2].rf_power_stat = RF_POWER_OFF;
					rf_stat[2].rf_send_error_reboot_times++;
				}				
			}
			if(wait_flag & (1<<2))
			{	
				tme[2]++;	
				if(tme[2]>80)
				{
					tme[2] = 0;
					rf_stat[1].rf_power_stat = RF_POWER_OFF;
					rf_stat[1].rf_send_error_reboot_times++;
				}				
			}			
			if(wait_flag & (1<<3))
			{
				tme[3]++;	
				if(tme[3]>80)
				{
					tme[3] = 0;
					rf_stat[0].rf_power_stat = RF_POWER_OFF;
					rf_stat[0].rf_send_error_reboot_times++;
				}				
			}				
			break;
		}
		
	}
	
	time = time - SysTick->VAL;
	time = time;

	if(rf_stat[3].mode == RF_WORK	&& rf_stat[3].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>24)&0xff)<32)	
	{	
		CC2520_get_reg(RF1,CC2520_EXCFLAG0 ,data); 
		if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX ); 
//			rf1_flush_tx++;
		}

		if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX ); 
			rf1_flush_rx++;			
		}	
	}
	
	if(rf_stat[2].mode == RF_WORK	&& rf_stat[2].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>24)&0xff)<32)	
	{	
		CC2520_get_reg(RF2,CC2520_EXCFLAG0 ,data); 
		if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX ); 
		}

		if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX ); 
			rf2_flush_rx++;					
		}
	}
	
	if(rf_stat[1].mode == RF_WORK	&& rf_stat[1].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>24)&0xff)<32)	
	{	
		CC2520_get_reg(RF3,CC2520_EXCFLAG0 ,data); 
		if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX ); 
		}

		if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX ); 	
			rf3_flush_rx++;		
		}
	}

	if(rf_stat[0].mode == RF_WORK	&& rf_stat[0].reg_init_stat == RF_REG_INIT_OK && ((sys_flash_param.ap_param.ap_channel>>24)&0xff)<32)	
	{	
		CC2520_get_reg(RF4,CC2520_EXCFLAG0 ,data); 
		if(data[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX ); 
		//	rf4_flush_tx++;
		}

		if(data[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX ); 
		rf4_flush_rx++;			
		}	
	
	}
	
}



void rf_cmd_tx(SPI_HandleTypeDef* hspi)
{
		CC2520_send_cmd(hspi,CC2520_INS_STXON);
}


void rf_power_reset(SPI_HandleTypeDef* hspi)
{
	int i = 0;
	rf_power_off(hspi);
	i = 1000;
	while(i--);
	rf_power_on(hspi);

}

void rf_set_channel(SPI_HandleTypeDef* hspi, uint16_t uiChannel )
{
    uint16_t   uiReg;
	
		CC2520_send_cmd(hspi,CC2520_INS_SRFOFF);	
		CC2520_send_cmd(hspi,CC2520_INS_SRXON);		
	
    if(uiChannel <16)  
			uiReg       =   uiChannel + ( uiChannel << 2u ) + 0x0Bu;		// uiChannel * 5u + 0x4165.
		else
		{
			uiChannel -= 16;
			uiReg       =   uiChannel + ( uiChannel << 2u ) + 0x0Bu + 2;		// uiChannel * 5u + 0x4165.
		}
    CC2520_set_reg(hspi,CC2520_FREQCTRL,uiReg);

		CC2520_send_cmd(hspi,CC2520_INS_STXON);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState bitstatus;
	
	if(GPIO_PIN_4 == GPIO_Pin)
	{
		if((GPIOF->IDR & (1<<4)) && (GPIOF->IDR & (1<<5)))
		{
			HAL_GPIO_WritePin(SPI5_led_rf_send_GPIO_Port,SPI5_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi5);
		}
	}
	else if(GPIO_PIN_2 == GPIO_Pin)
	{
		if((GPIOI->IDR & (1<<2)) && (GPIOI->IDR & (1<<3)))
		{
			HAL_GPIO_WritePin(SPI3_led_rf_send_GPIO_Port,SPI3_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi3);		
		}
	}
	else if(GPIO_PIN_10 == GPIO_Pin)
	{
		if((GPIOB->IDR & (1<<10)) && (GPIOB->IDR & (1<<11)))
		{
//			HAL_GPIO_WritePin(SPI4_led_rf_send_GPIO_Port,SPI4_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi4);
		}
	}
	else if(GPIO_PIN_12 == GPIO_Pin)
	{
		if((GPIOF->IDR & (1<<12)) && (GPIOF->IDR & (1<<11)))
		{
			HAL_GPIO_WritePin(SPI1_led_rf_send_GPIO_Port,SPI1_led_rf_send_Pin,GPIO_PIN_RESET);
			rf_rx(&hspi1);		
		}
	}	
}



void rf_rx(SPI_HandleTypeDef* hspi)
{
	HAL_StatusTypeDef ret;
	uint8_t data1[3];	
	uint8_t data = CC2520_INS_RXBUF;

	//spiÆ¬Ñ¡
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_RESET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_RESET);	
	
	ret = HAL_SPI_Transmit(hspi,&data,1,1); //·¢ËÍ¶Ábuf ÃüÁî
	ret = HAL_SPI_Receive(hspi,&data,1,1); //¶ÁÒ»¸ö×Ö½Ú£¬buf³¤¶È
	

	if(hspi == RF1)	
	{
		rf_rx_buff[0][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rf_rx_buff[0][1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[0][1],data,1);
		//´ÓÖÐ¶Ïµ½ÕâÀï(½ÓÊÕ±ðµÄAPµÄÍ¬²½°ü)  58US 
		rf_cs_off(hspi);		
		rf_rx_data_handle(0);
		//´ÓÖÐ¶Ïµ½ÕâÀï(½ÓÊÕ±ðµÄAPµÄÍ¬²½°ü)   65US
		CC2520_get_reg(RF1,CC2520_EXCFLAG0 ,data1); 
		if(data1[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHTX ); 
			//rf1_flush_tx++;
		}

		if(data1[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF1,CC2520_INS_SFLUSHRX ); 
			rf1_flush_rx++;			
		}			
//		HAL_GPIO_WritePin(SPI4_led_rf_send_GPIO_Port,SPI4_led_rf_send_Pin,GPIO_PIN_SET);	
	}
	else if(hspi == RF2)
	{
		rf_rx_buff[1][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rdata2[1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[1][1],data,1);	
		HAL_GPIO_WritePin(SPI1_led_rf_send_GPIO_Port,SPI1_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);		
		rf_rx_data_handle(1);

		CC2520_get_reg(RF2,CC2520_EXCFLAG0 ,data1); 
		if(data1[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHTX ); 
			//rf1_flush_tx++;
		}

		if(data1[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF2,CC2520_INS_SFLUSHRX ); 
			rf2_flush_rx++;			
		}				
	}
	else if(hspi == RF3)	
	{
		rf_rx_buff[2][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rf_rx_buff[2][1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[2][1],data,1);
		HAL_GPIO_WritePin(SPI5_led_rf_send_GPIO_Port,SPI5_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);	
		rf_rx_data_handle(2);		

		CC2520_get_reg(RF3,CC2520_EXCFLAG0 ,data1); 
		if(data1[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHTX ); 
			//rf1_flush_tx++;
		}

		if(data1[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF3,CC2520_INS_SFLUSHRX ); 
			rf3_flush_rx++;			
		}		
	}
	else if(hspi == RF4)
	{
		rf_rx_buff[3][0] = data;
//		HAL_SPI_Receive_DMA(hspi,&rdata2[1],data);	
		ret = HAL_SPI_Receive(hspi,&rf_rx_buff[3][1],data,1);	
		HAL_GPIO_WritePin(SPI3_led_rf_send_GPIO_Port,SPI3_led_rf_send_Pin,GPIO_PIN_SET);	
		rf_cs_off(hspi);
		rf_rx_data_handle(3);		
		CC2520_get_reg(RF4,CC2520_EXCFLAG0 ,data1); 
		if(data1[2] & (1u << CC2520_EXC_TX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_TX_OVERFLOW))
		{
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX );
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHTX ); 
			//rf4_flush_tx++;
		}

		if(data1[2] & (1u << CC2520_EXC_RX_UNDERFLOW) || data1[2] & (1u << CC2520_EXC_RX_OVERFLOW))
		{
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(RF4,CC2520_INS_SFLUSHRX ); 
			rf4_flush_rx++;			
		}		
	}
	
		
}

void rf_cs_off(SPI_HandleTypeDef* hspi)
{
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	

}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	
	if( hspi == &hspi3) 
		HAL_GPIO_WritePin(SPI3_rf_cs_GPIO_Port,SPI3_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi5) 
		HAL_GPIO_WritePin(SPI5_rf_cs_GPIO_Port,SPI5_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi1)  
		HAL_GPIO_WritePin(SPI1_rf_cs_GPIO_Port,SPI1_rf_cs_Pin,GPIO_PIN_SET);	
	else if(hspi == &hspi4) 
	{
		HAL_GPIO_WritePin(SPI4_rf_cs_GPIO_Port,SPI4_rf_cs_Pin,GPIO_PIN_SET);	
//		sprintf(debug_send_buff,"d1 %d %x %d %d %d %d %d %d\r\n",rdata1[0],rdata1[1],rdata1[2],rdata1[3],rdata1[4],rdata1[5],rdata1[6],rdata1[7]);
//		copy_string_to_double_buff(debug_send_buff);		
	}
}




extern uint8_t get_slot_num();


void rf_rx_voerflow_check()
{
	static unsigned char timeout[4] = {0,0,0,0};
	
	if((GPIOF->IDR & (1<<4)) ) //&& !(GPIOF->IDR & (1<<5)))
		{
			timeout[0]++;
			if(timeout[0]>3){
				timeout[0] = 0;
				CC2520_send_cmd(rf_index[1],CC2520_INS_SFLUSHRX );
				CC2520_send_cmd(rf_index[1],CC2520_INS_SFLUSHRX ); 
				sprintf(gprs_debug_buff,"rf3 rxflush:slot=%d\r\n",systerm_info.slot%get_slot_num());
				copy_string_to_double_buff(gprs_debug_buff);				
			}
		}	
		else
			timeout[0] = 0;


//	if((GPIOI->IDR & (1<<2))) //&& !(GPIOI->IDR & (1<<3)))
//		{
//			timeout[1]++;
//			if(timeout[1]>3){
//				timeout[1] = 0;			
//			CC2520_send_cmd(rf_index[0],CC2520_INS_SFLUSHRX );
//			CC2520_send_cmd(rf_index[0],CC2520_INS_SFLUSHRX ); 
//				sprintf(gprs_debug_buff,"rf4 rxflush:slot=%d\r\n",systerm_info.slot%get_slot_num());
//				copy_string_to_double_buff(gprs_debug_buff);					
//			}
//		}		
//		else
//			timeout[1] = 0;
		
//	if((GPIOB->IDR & (1<<10)) )//&& !(GPIOB->IDR & (1<<11)))
//		{
//			timeout[2]++;
//			if(timeout[2]>3){
//				timeout[2] = 0;			
//			CC2520_send_cmd(rf_index[3],CC2520_INS_SFLUSHRX );
//			CC2520_send_cmd(rf_index[3],CC2520_INS_SFLUSHRX ); 
//				sprintf(gprs_debug_buff,"rf1 rxflush:slot=%d\r\n",systerm_info.slot%get_slot_num());
//				copy_string_to_double_buff(gprs_debug_buff);					
//			}
//		}	
//		else
//			timeout[2] = 0;
		
	if((GPIOF->IDR & (1<<12))) //&& !(GPIOF->IDR & (1<<11)))
		{
			timeout[3]++;
			if(timeout[3]>3){
				timeout[3] = 0;				
			CC2520_send_cmd(rf_index[2],CC2520_INS_SFLUSHRX );
			CC2520_send_cmd(rf_index[2],CC2520_INS_SFLUSHRX ); 
				sprintf(gprs_debug_buff,"rf2 rxflush:slot=%d\r\n",systerm_info.slot%get_slot_num());
				copy_string_to_double_buff(gprs_debug_buff);					
			}
		}	
		else
			timeout[3] = 0;
}

void rf_satt_init()
{
	int i;
	for(i=0;i<4;i++)	
	{
		rf_stat[i].mode = RF_DEFAULT_MODE;		
	}
	
}
	 int channel = 0;
void rf_scan_channel()
{

	int i = 0;
	
	struct _led
	{
		GPIO_TypeDef * port;
		uint32_t pin;
	}struct_led[4] = {
	general_led_1_GPIO_Port,general_led_1_Pin,
	general_led_2_GPIO_Port,general_led_2_Pin,
	general_led_3_GPIO_Port,general_led_3_Pin,
	general_led_4_GPIO_Port,general_led_4_Pin};
	
  if(rf_scan_channel_enable != 1)
		return;
	
	if(systerm_info.slot%2048 == 1)
	{
		for(i=0;i<4;i++)
		{
			if(rf_stat[i].mode == ENABLE_NOMODULATE_CARRIER || rf_stat[i].mode == ENABLE_MODULATE_CARRIER)
			{
				rf_set_channel(rf_index[i],channel);
				HAL_GPIO_TogglePin(struct_led[i].port,struct_led[i].pin);
				sprintf(gprs_debug_buff,"rf scan ch=%d\r\n",channel);
				copy_string_to_double_buff(gprs_debug_buff);					
			}
		}
			
		channel++;
		if(channel >15)
			channel = 0;

		while(systerm_info.slot%2048 == 1);
	}

}


void rf_send_1000p()
{
	uint32_t *packet_seq = (uint32_t *)&rf_send_data[1];
	int error = 0;
	
	if(systerm_info.slot%32 == 1)
	{
		if(rf_send_1000_p_enable & EN_RF1 && rf_stat[3].mode == RF_IDLE && rf_stat[3].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF1,rf_send_data,rf_send_data[0]);
				
				//CC2520_send_cmd(RF1,CC2520_INS_STXON);
				error = rf_io_tx(RF1);
				HAL_GPIO_TogglePin(general_led_1_GPIO_Port,general_led_1_Pin);
				sprintf(gprs_debug_buff,"rf1_send seq=%d e=%d\r\n",*packet_seq,error);
				copy_string_to_double_buff(gprs_debug_buff);	
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF2 && rf_stat[2].mode == RF_IDLE && rf_stat[2].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF2,rf_send_data,rf_send_data[0]);
				
				//CC2520_send_cmd(RF2,CC2520_INS_STXON);
				rf_io_tx(RF2);
				HAL_GPIO_TogglePin(general_led_2_GPIO_Port,general_led_2_Pin);
				sprintf(gprs_debug_buff,"rf2_send seq=%d e=%d\r\n",*packet_seq,error);
				copy_string_to_double_buff(gprs_debug_buff);	
				(*packet_seq)++;
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF3 && rf_stat[1].mode == RF_IDLE && rf_stat[1].rf_power_stat == RF_POWER_WORK)
		{

			if((*packet_seq)< 1000)
			{
			
				rf_write_buff(RF3,rf_send_data,rf_send_data[0] );

				//CC2520_send_cmd(RF3,CC2520_INS_STXON);
				rf_io_tx(RF3);
				HAL_GPIO_TogglePin(general_led_3_GPIO_Port,general_led_3_Pin);
				sprintf(gprs_debug_buff,"rf3_send seq=%d e=%d\r\n",*packet_seq,error);
				copy_string_to_double_buff(gprs_debug_buff);		
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}
		else if(rf_send_1000_p_enable & EN_RF4 && rf_stat[0].mode == RF_IDLE && rf_stat[0].rf_power_stat == RF_POWER_WORK)
		{
			if((*packet_seq)< 1000)
			{
				
				rf_write_buff(RF4,rf_send_data,rf_send_data[0] );

				//CC2520_send_cmd(RF4,CC2520_INS_STXON);
				rf_io_tx(RF4);
				HAL_GPIO_TogglePin(general_led_4_GPIO_Port,general_led_4_Pin);
				sprintf(gprs_debug_buff,"rf4_send seq=%d e=%d\r\n",*packet_seq,error);
				copy_string_to_double_buff(gprs_debug_buff);				
				(*packet_seq)++;				
				while(systerm_info.slot%32 == 1);
			}
			
		}		
	}
}


void rf_manage()
{
	int i;
	
	uint8_t *p_chanel = (uint8_t *)&sys_flash_param.ap_param.ap_channel;

	rf_scan_channel();
	rf_send_1000p();
	rf_rx_voerflow_check();
	
	if(rf_sec_flag < 1) //1sec½øÈëÏÂÃæÒ»´Î
		return;

//	if(systerm_info.slot > 10000)
//		return;

	rf_sec_flag = 0;
	
	for(i=0;i<4;i++)	
	{
		rf_stat[i].rf_no_data_sec++;
		if(rf_stat[i].rf_no_data_sec > 120)
		{
			rf_stat[i].rf_no_data_sec = 0;
			rf_stat[i].rf_power_stat = RF_POWER_OFF;
			rf_stat[i].rf_no_data_reboot_times++;
		}
	}
	
	for(i=0;i<4;i++)
	{
		if(i==0 || i==3)
			continue;
		if(rf_stat[i].rf_power_stat != RF_POWER_WORK)    //µçÔ´Ã»ÔÚ¹¤×÷×´Ì¬
		{
			if(rf_stat[i].rf_power_stat == RF_POWER_ON)    //µçÔ´ÎªON×´Ì¬
			{
				rf_power_on(rf_index[i]);
				rf_stat[i].reg_init_stat = rf_init(rf_index[i]);
				if(rf_stat[i].reg_init_stat != RF_REG_INIT_OK)
					rf_stat[i].rf_power_stat = RF_POWER_OFF;
				else
				{
					rf_stat[i].rf_power_stat = RF_POWER_WORK;
					rf_set_channel(rf_index[i],p_chanel[3-i]);
					if(rf_stat[i].mode == ENABLE_NOMODULATE_CARRIER || rf_stat[i].mode == ENABLE_MODULATE_CARRIER )
						rf_cmd_tx(rf_index[i]);

				}
					

			}
			else                                           //µçÔ´ÎªOFF»òÕßÆäËûÒì³£×´Ì¬
			{
				rf_power_off(rf_index[i]);
				rf_stat[i].rf_power_stat = RF_POWER_ON;
			}
			
		}
		
	}
	
	
	
	
}







extern char gprs_debug_buff[256];

char *str_power[] = {"off","on","ok"};
char *str_init[] = {"reg_error","spi_error","ok"};
char *str_mode[] = {"work","on_MC","MC"};


char* make_rf_stat()
{
	
	
	sprintf(gprs_debug_buff,"RF1:power=%s init=%s mode=%s s_error=%d r_error=%d\r\nRF2:power=%s init=%s mode=%s s_error=%d r_error=%d\r\nRF3:power=%s init=%s mode=%s s_error=%d r_error=%d\r\nRF4:power=%s init=%s mode=%s s_error=%d r_error=%d\r\n",
																			str_power[rf_stat[3].rf_power_stat],str_init[rf_stat[3].reg_init_stat+2],str_mode[rf_stat[3].mode],rf_stat[3].rf_send_error_reboot_times,rf_stat[3].rf_no_data_reboot_times	,
																			str_power[rf_stat[2].rf_power_stat],str_init[rf_stat[2].reg_init_stat+2],str_mode[rf_stat[2].mode],rf_stat[2].rf_send_error_reboot_times,rf_stat[2].rf_no_data_reboot_times,
																			str_power[rf_stat[1].rf_power_stat],str_init[rf_stat[1].reg_init_stat+2],str_mode[rf_stat[1].mode],rf_stat[1].rf_send_error_reboot_times,rf_stat[1].rf_no_data_reboot_times,
																			str_power[rf_stat[0].rf_power_stat],str_init[rf_stat[0].reg_init_stat+2],str_mode[rf_stat[0].mode],rf_stat[0].rf_send_error_reboot_times,rf_stat[0].rf_no_data_reboot_times);
	return gprs_debug_buff;
}





