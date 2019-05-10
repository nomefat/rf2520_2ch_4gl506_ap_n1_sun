#include "stm32f4xx_hal.h"
#include "string.h"
#include "to_n1.h"


#define TO_N1_BUFF_LENGTH 50              
#define RE_SEND_TIMES  10            //重发次数


int to_n1_buff_insert_ptr = 0;               //写偏移
volatile int to_n1_buff_get_ptr = 0;								//读偏移

uint8_t send_to_n1_buff[TO_N1_BUFF_LENGTH][256]={0};          //发送给N1的环形缓冲buff          50个256大小的数组
uint8_t send_to_n1_data[256]={0};
uint8_t send_to_n1_ack[10]={0xaa,0x55,0,0,0,0,1,1,0xc1,0xc0};



int to_n1_lost_packet_count;

extern UART_HandleTypeDef huart3;
#define CLEAR_SEND_TO_N1_TIMEOUT() TIM2->CNT = 0
#define SEND_TO_N1_TIMEOUT_20MS() (TIM2->CNT>(20000))  //20ms
#define SEND_TO_N1_TIMEOUT_1MS() (TIM2->CNT>(1000*1))     //1ms

extern char gprs_debug_buff[256];
extern void copy_string_to_double_buff(char *pstr);

/*** CRC table for the CRC-16. The poly is 0x8005 (x^16 + x^15 + x^2 + 1) */
unsigned short const crc16_table[256] = {
	0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
	0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
	0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
	0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
	0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
	0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
	0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
	0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
	0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
	0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
	0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
	0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
	0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
	0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
	0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
	0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
	0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
	0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
	0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
	0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
	0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
	0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
	0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
	0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
	0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
	0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
	0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
	0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
	0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
	0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
	0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
	0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

unsigned short  crc16_byte(unsigned short crc, const unsigned char data)
{
	return (crc >> 8) ^ crc16_table[(crc ^ data) & 0xff];
}
/***
 * crc16 - compute the CRC-16 for the data buffer
 * @crc:	previous CRC value
 * @buffer:	data pointer
 * @len:	number of bytes in the buffer
 *
 * Returns the updated CRC value.
 */
unsigned short crc16(unsigned short crc, unsigned char const *buffer,int len)
{
	while (len--)
		crc = crc16_byte(crc, *buffer++);
	return crc;
}



/**********************************************************************************************
***func:将需要发给N1的数据（同步包、sesor数据状态等）写入环形缓冲区，等待发送	
***
***date: 2017/3/4
*** nome
***********************************************************************************************/
int insert_to_n1_buff(uint8_t *data,uint8_t len,uint8_t cmd)
{
	int i = 0;
	static uint32_t packet_seq = 0;
	int16_t uiCrcValue=0;
	
	if((to_n1_buff_insert_ptr+1)%TO_N1_BUFF_LENGTH==to_n1_buff_get_ptr)
		return -1;
	
	__disable_irq() ; 
	
	send_to_n1_buff[to_n1_buff_insert_ptr][0]=0xaa;
	send_to_n1_buff[to_n1_buff_insert_ptr][1]=0x55;
	send_to_n1_buff[to_n1_buff_insert_ptr][2]=(packet_seq>>24)&0xff;
	send_to_n1_buff[to_n1_buff_insert_ptr][3]=(packet_seq>>16)&0xff;
	send_to_n1_buff[to_n1_buff_insert_ptr][4]=(packet_seq>>8)&0xff;
	send_to_n1_buff[to_n1_buff_insert_ptr][5]= packet_seq & 0xff;
	send_to_n1_buff[to_n1_buff_insert_ptr][6]=len+1;
	send_to_n1_buff[to_n1_buff_insert_ptr][7]=cmd;
	
	packet_seq++;
	
	for(i=0;i<len;i++)
	{
		send_to_n1_buff[to_n1_buff_insert_ptr][i+8] = data[i];
	}	

  uiCrcValue = crc16(0, &send_to_n1_buff[to_n1_buff_insert_ptr][7], len+1);	
	send_to_n1_buff[to_n1_buff_insert_ptr][8+len] = uiCrcValue & 0xFF;
	send_to_n1_buff[to_n1_buff_insert_ptr][8+len+1] = (uiCrcValue>>8) & 0xFF;

	
	to_n1_buff_insert_ptr = (to_n1_buff_insert_ptr+1)%TO_N1_BUFF_LENGTH;
	
	__enable_irq() ; 
	return 0;	
	
}




/**********************************************************************************************
***func:q取出一条需要发给N1的数据（同步包、sesor数据状态等） 到指定的内存中
***
***date: 2017/3/4
*** nome
***********************************************************************************************/
int get_from_n1_buff(uint8_t *data)
{

	if(to_n1_buff_insert_ptr==to_n1_buff_get_ptr)
		return -1;		
	
	memcpy(data,&send_to_n1_buff[to_n1_buff_get_ptr],256);
	
	to_n1_buff_get_ptr = (to_n1_buff_get_ptr+1)%TO_N1_BUFF_LENGTH;

//	sprintf(gprs_debug_buff,"to_n1:v=%d seq=%d rptr=%d\r\n",v,send_to_n1_data[5]+(send_to_n1_data[4]<<8)+(send_to_n1_data[3]<<16),to_n1_buff_get_ptr);
//	copy_string_to_double_buff(gprs_debug_buff);	
	return 0;

}




/**********************************************************************************************
***func:q取出一条需要发给N1的数据（同步包、sesor数据状态等） 到指定的内存中
***
***date: 2017/3/4
*** nome
***********************************************************************************************/
void to_n1_buff_handle()
{
	static int Tx_conter  = 0;
	static int packet_seq = 0;
	int packet_seq_rep = 0;
	
	if(send_to_n1_ack[0] != 0 && SEND_TO_N1_TIMEOUT_1MS())  //优先发送ack
	{
		if(HAL_OK == HAL_UART_Transmit_DMA(&huart3,send_to_n1_ack,10))
		{
			CLEAR_SEND_TO_N1_TIMEOUT();
			send_to_n1_ack[0] = 0;
		}			
	}	
	else if(send_to_n1_data[0] == 0)    //从buff中读取的数据已经发送成功，可以重新读取buff
	{
		if(0==get_from_n1_buff(send_to_n1_data))   //读取buff
		{
			Tx_conter = 0;
		}			
	} 
	else  //第一个字节不等于0 来判断有数据需要发送
	{		
		if((Tx_conter<RE_SEND_TIMES && SEND_TO_N1_TIMEOUT_20MS()) || (Tx_conter == 0 && SEND_TO_N1_TIMEOUT_1MS()))        //发送次数小于重发次数 而且已经超时未收到ack 或者 第一次发送
		{
				HAL_GPIO_WritePin(general_led_5_GPIO_Port,general_led_5_Pin,GPIO_PIN_RESET);
				HAL_UART_Transmit_DMA(&huart3,send_to_n1_data,send_to_n1_data[6]+9);
				CLEAR_SEND_TO_N1_TIMEOUT();		
		
				TIM2->CNT = 0;
				Tx_conter++;
				packet_seq_rep = send_to_n1_data[5] + (send_to_n1_data[4]<<8) + (send_to_n1_data[3]<<16) + (send_to_n1_data[2]<<24);
			
				if(packet_seq - packet_seq_rep >1)
					packet_seq_rep = packet_seq_rep;
				
				packet_seq = packet_seq_rep;
				
			
				if(Tx_conter >= RE_SEND_TIMES)
				{
					to_n1_lost_packet_count++;
					Tx_conter = 0;
					while(TIM2->CNT < 1000);
					send_to_n1_data[0] = 0;         //丢掉该数据
				}
		}	
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
	{
		HAL_GPIO_WritePin(general_led_5_GPIO_Port,general_led_5_Pin,GPIO_PIN_SET);	
	}
}

