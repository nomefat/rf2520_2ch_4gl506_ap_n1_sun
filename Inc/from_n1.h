#ifndef FROM_N1_H_
#define FROM_N1_H_





#pragma pack(1)

typedef struct _ap_n1_protocol
{
	unsigned short head;
	unsigned int packet_syn;
	unsigned char lengh;
	unsigned char cmd;
	unsigned char data[1];
	
}struct_ap_n1_protocol;



#pragma pack()












#endif


