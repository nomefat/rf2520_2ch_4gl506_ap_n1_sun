#ifndef _DEBUG_UART_
#define _DEBUG_UART_



#define DEBUG_DOUBLE_BUFF_LEN 2048

void debug_uart_send_string(char *pstr);
void print_version();


extern char debug_send_buff[];


void debug_send_double_buff_poll(void);


typedef struct _debug_double_buff
{
	unsigned int len;
	char data[DEBUG_DOUBLE_BUFF_LEN];
}struct_debug_double_buff;



int copy_string_to_double_buff(char *pstr);




#endif


