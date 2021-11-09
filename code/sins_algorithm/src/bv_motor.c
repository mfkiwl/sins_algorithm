/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include "typedef.h"
#include "bv_sins.h"
#include "functions_sins.h"
#include "bv_navigation.h"
#include "gnss_paraser.h"
#include "bv_rs232.h"
#include "string.h"
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <syslog.h>
#include <dirent.h>
#include <errno.h>
#include <signal.h>
#include <semaphore.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <stddef.h>
#include <stdarg.h>
#include <malloc.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h> 
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <asm-generic/ioctls.h>
#include <linux/serial.h>
typedef enum {DISABLE = 0, ENABLE} RS485_ENABLE_t;
int uart_motor_fd = -1;
static  char s_sent[10]={0};
static  char s_recv[10]={0};

int motor_init_uart()
{
   uart_motor_fd=bv_init_uart(3,115200) ;
   return 0;
}


int rs485_enable(const int fd, const RS485_ENABLE_t enable)
{
        struct serial_rs485 rs485conf;
        int res;
 
        /* Get configure from device */
        res = ioctl(fd, TIOCGRS485, &rs485conf);
        if (res < 0) 
		{
                perror("Ioctl error on getting 485 configure:");
                close(fd);
                return res;
        }
 
        /* Set enable/disable to configure */
      //  if (enable) 
		//{   // Enable rs485 mode
          // rs485conf.flags |= SER_RS485_ENABLED;
       // } 
		//else 
		//{        // Disable rs485 mode
        rs485conf.flags &= ~(SER_RS485_ENABLED);
       // }
        rs485conf.flags |= SER_RS485_RTS_ON_SEND;           //设置rts，当逻辑电平为高的时候，为发送
        rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);    //设置rts，当发送完后逻辑电平为低
        long rs_485_wait_time=(int)10*9*1000/115200.0 +1;
        rs485conf.delay_rts_before_send =rs_485_wait_time;// 0x00000004;

		rs485conf.delay_rts_after_send =rs_485_wait_time;// 0x00000004;
	    rs485conf.flags |= SER_RS485_RX_DURING_TX;

 
        /* Set configure to device */
        res = ioctl(fd, TIOCSRS485, &rs485conf);
        if (res < 0) 
		{
                perror("Ioctl error on setting 485 configure:");
                close(fd);
        }
 
        return res;
}

unsigned char  check_bit(unsigned char *str)
{
    unsigned char rc=0;
	unsigned int rint=0;
	for(int i=0;i<6;i++)
	  rint=rint+str[i];
	rint=rint%256; 
	rc=(unsigned char)rint;
	return rc;
}
void  command_frame(unsigned char functiongcode,unsigned char domain_1,unsigned char domain_2,unsigned char domain_3,unsigned char domain_4)
{
   unsigned char checkstr[7]={0};
   memset(s_sent,0,sizeof(s_sent));
   s_sent[0]=0x68;
   s_sent[1]=checkstr[0]=0x01;
   s_sent[2]=checkstr[1]=functiongcode;
   s_sent[3]=checkstr[2]=domain_1;
   s_sent[4]=checkstr[3]=domain_2;
   s_sent[5]=checkstr[4]=domain_3;
   s_sent[6]=checkstr[5]=domain_4;
   s_sent[7]=check_bit(checkstr);  
   s_sent[8]=0x16;
//   printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",s_sent[0],s_sent[1],s_sent[2],s_sent[3],s_sent[4],s_sent[5],s_sent[6],s_sent[7],s_sent[8]);
   bv_uart_write(uart_motor_fd,s_sent,9);
   bv_milliseconds_sleep(300);//<<等待1ms 就好
   
}
int check_recv()
{
   unsigned char checkstr[7]={0};
   unsigned char checkbyte;
   if(s_recv[0]!=0x88) return -1;
   checkstr[0]=s_recv[1];
   checkstr[1]=s_recv[2];
   checkstr[2]=s_recv[3];
   checkstr[3]=s_recv[4];
   checkstr[4]=s_recv[5];
   checkstr[5]=s_recv[6];
   checkbyte=check_bit(checkstr);
   if(checkbyte!=s_recv[7]) return -1;
   if(s_recv[8]!=0x16) return -1;
   return 0;

}
int motor_left_init()
{
	int ret=-1;
	command_frame(0x04,0x00,0x00,0x00,0x00);//<<电机不同步
	bv_uart_read(uart_motor_fd,s_recv,9);
	//printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",s_recv[0],s_recv[1],s_recv[2],s_recv[3],s_recv[4],s_recv[5],s_recv[6],s_recv[7],s_recv[8]);
	//ret=check_recv();
	//if(ret!=0) printf("recv data error\n");
	command_frame(0x15,0x00,0x00,0x00,0x00);//串口 
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0x17,0x04,0x00,0x00,0x00);//占空比 
    bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0x19,0x00,0x00,0x00,0x00);//运转方向
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0x21,0x00,0x00,0x00,0x040);//设置允许电流 2A
	bv_uart_read(uart_motor_fd,s_recv,9);
	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",s_recv[0],s_recv[1],s_recv[2],s_recv[3],s_recv[4],s_recv[5],s_recv[6],s_recv[7],s_recv[8]);
	ret=check_recv();
	if(ret!=0) printf("recv data error\n");
	command_frame(0x1B,0x01,0x00,0x00,0x00);//允许输出
	bv_uart_read(uart_motor_fd,s_recv,9);

}
int motor_rightt_init()
{
	int ret=-1;
	//command_frame(0x04,0x00,0x00,0x00,0x00);//<<电机不同步
	//uart_read(s_recv,9);
//	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",s_recv[0],s_recv[1],s_recv[2],s_recv[3],s_recv[4],s_recv[5],s_recv[6],s_recv[7],s_recv[8]);
	//ret=check_recv();
	//if(ret!=0) printf("recv data error\n");
	command_frame(0xA5,0x00,0x00,0x00,0x00);//串口 
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0xA7,0x04,0x00,0x00,0x00);//占空比 
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0xA9,0x01,0x00,0x00,0x00);//运转方向
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0xB1,0x00,0x00,0x00,0x040);//设置允许电流 2A
	bv_uart_read(uart_motor_fd,s_recv,9);
	command_frame(0xAB,0x01,0x00,0x00,0x00);//允许输出
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_l_Duty_Ratio()
{
	command_frame(0x1D,0xFA,0x00,0x00,0x00);//占空比 25%
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_r_Duty_Ratio()
{
	command_frame(0xAD,0xFA,0x00,0x00,0x00);//占空比 25%
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_l_begin()
{
	command_frame(0x1B,0x01,0x00,0x00,0x00);
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_r_begin()
{
	command_frame(0xAB,0x01,0x00,0x00,0x00);
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_l_stop()
{
	command_frame(0x1B,0x00,0x00,0x00,0x00);
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_r_stop()
{
	command_frame(0xAB,0x00,0x00,0x00,0x00);
	bv_uart_read(uart_motor_fd,s_recv,9);
}
int motor_back()
{
	motor_l_stop();
	motor_r_stop();
	bv_milliseconds_sleep(300);
	command_frame(0x19,0x01,0x00,0x00,0x00);//运转方向
	command_frame(0xA9,0x00,0x00,0x00,0x00);//运转方向
	motor_l_begin();
	motor_r_begin();
}
int motor_front()
{
	motor_l_stop();
	motor_r_stop();
	bv_milliseconds_sleep(300);
	command_frame(0x19,0x00,0x00,0x00,0x00);//运转方向
	command_frame(0xA9,0x01,0x00,0x00,0x00);//运转方向
	motor_l_begin();
	motor_r_begin();
}
int motor_turn_right()
{
	motor_l_stop();
	motor_r_stop();
	bv_milliseconds_sleep(300);
	command_frame(0x19,0x00,0x00,0x00,0x00);//运转方向
	motor_l_begin();
}
int motor_turn_left()
{
	int ret=-1;
	motor_l_stop();
	motor_r_stop();
	bv_milliseconds_sleep(300);
	command_frame(0xA9,0x01,0x00,0x00,0x00);//运转方向
	bv_uart_read(uart_motor_fd,s_recv,9);
	printf("%02x %02x %02x %02x %02x %02x %02x %02x %02x\n",s_recv[0],s_recv[1],s_recv[2],s_recv[3],s_recv[4],s_recv[5],s_recv[6],s_recv[7],s_recv[8]);
	motor_r_begin();
}

