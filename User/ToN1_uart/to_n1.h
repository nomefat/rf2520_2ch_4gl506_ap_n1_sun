#ifndef _TO_N1_H_
#define _TO_N1_H_



#include "stm32f4xx_hal.h"

#define N1_ACK           1

#define N1_GET_AP_PARAM   2
#define N1_SET_AP_PARAM   3

#define N1_SEND_AP_FIRMWARE 6
#define N1_SEND_RP_FIRMWARE 7
#define N1_SEND_S_FIRMWARE  8

#define N1_SEND_2G_DATA     9

#define N1_SET_S_PARAM      10
#define N1_SET_RP_PARAM     11

#define N1_GET_2G_STAT       12

#define N1_SET_SENSOR_UPDATE_ENABLE 13

#define N1_SET_RP_UPDATE_ENABLE 14




#define AP_RF_DATA    				 50
#define AP_NOP        				 51
#define AP_GPRS_SERVER_DATA    52
#define AP_AP_PARAM   				 53
#define AP_GPRS_STAT           54

#define AP_SEND_RP_FIRMWARE    57
#define AP_SEND_S_FIRMWARE    58

int insert_to_n1_buff(uint8_t *data,uint8_t len,uint8_t cmd);










#endif





