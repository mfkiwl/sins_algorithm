/*
     tony.jiang 2021 10 26
*/
#ifndef _BV_RS232_H_
#define _BV_RS232_H_
void bv_milliseconds_sleep(unsigned long mSec);
int  set_opt(int fd,int wSpeed, int wBits, char cEvent, int wStop);
int  bv_uart_open(int uart_port,int baud_rate);
int  bv_init_uart(int port_no, int band_rate);
void bv_uart_close(int uart_port);
void bv_uart_close_all();
int  bv_uart_read(int fd, char *buf, int len);
int  bv_uart_write(int fd, char* p_data_buf,int buf_size);


#endif //_BV_RS232_H_