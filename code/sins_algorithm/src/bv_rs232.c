/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include <stdlib.h>
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

const int UART_PORT_NUM=5;
#define UART_MOTOR_BAND_RATE	115200	//设置odometry口的波特率
#define UART_DATA_BIT_8			8		//设置串口的数据位
#define UART_STOP_BIT_2			1		//设置串口的停止位
//#define UART_NAME		"/dev/ttyS3"
const char * uart_name[UART_PORT_NUM]={"/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS7"};
int gawFdUart[UART_PORT_NUM] = {-1, -1, -1, -1, -1};

void bv_milliseconds_sleep(unsigned long mSec)
{
    struct timeval tv;
    int err;


	tv.tv_sec = mSec/1000;
    tv.tv_usec = (mSec%1000)*1000;
	
    do{
       err = select(0,NULL,NULL,NULL,&tv);
    }while (err<0 && errno==EINTR);
}


int set_opt(int fd, int wSpeed, int wBits, char cEvent, int wStop)
{
    struct termios newtio, oldtio;

	/*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if (tcgetattr(fd,&oldtio) != 0)
	{
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));

	/*步骤一，设置字符大小*/
	newtio.c_cflag |= CLOCAL|CREAD;     //CLOCAL:忽略modem控制线  CREAD：打开接受者
    newtio.c_cflag &= ~CSIZE;           //字符长度掩码。取值为：CS5，CS6，CS7或CS8

	/*设置数据位*/
    switch(wBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

	/*设置奇偶校验位*/
    switch( cEvent )
    {
        case 'O'://奇数
            newtio.c_cflag |= PARENB; //允许输出产生奇偶信息以及输入到奇偶校验
            newtio.c_cflag |= PARODD;  //输入和输出是奇及校验
            newtio.c_iflag |= (INPCK|ISTRIP); // INPACK:启用输入奇偶检测；ISTRIP：去掉第八位
            break;
        case 'E'://偶数
            newtio.c_iflag |= (INPCK|ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N'://无奇偶校验位
            newtio.c_cflag &= ~PARENB;
            break;
    }
	
	/*设置波特率*/
    switch( wSpeed )
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 1500000:
            cfsetispeed(&newtio, B1500000);
            cfsetospeed(&newtio, B1500000);
            break;
        default:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
    }

	/*设置停止位*/

    if (wStop == 1)
        newtio.c_cflag &= ~CSTOPB; //CSTOPB:设置两个停止位，而不是一个
    else if (wStop == 2)
        newtio.c_cflag |= CSTOPB;

	/*设置等待时间和最小接收字符*/ 
    newtio.c_cc[VTIME] = 0; //VTIME:非cannoical模式读时的延时，以十分之一秒位单位
    newtio.c_cc[VMIN] = 0; //VMIN:非canonical模式读到最小字符数

	/*处理未接收字符*/
	tcflush(fd,TCIFLUSH); // 改变在所有写入 fd 引用的对象的输出都被传输后生效，所有已接受但未读入的输入都在改变发生前丢弃。
    if ((tcsetattr(fd,TCSANOW,&newtio))!=0) //TCSANOW:改变立即发生
    {
        printf("com set error");
        return -1;
    }
    printf("tc set attr done!\n");
    return 0;
}
int bv_uart_open(int uart_port,int baud_rate)
{    
	int rtn = 0;
	int fd = -1;

	/*O_NOCTTY:通知linux系统，这个程序不会成为这个端口的控制终端.  
	 O_NONBLOCK/O_NDELAY:通知linux系统不关心DCD信号线所处的状态(端口的另一端是否激活或者停止).
	 read不能从管道、FIFO或设备读得数据时O_NDELAY使read返回0，这与表示已读到文件尾端的返回值0相冲突，
	 应使用O_NONBLOCK代替O_NDELAY,read返回-1*/ 
	fd = open(uart_name[uart_port], O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY);

    if (fd < 0)
    {
        printf("uart%d open err!",uart_port);
        return -1;
    }

    /* 恢复串口为阻塞状态，用于等待串口数据                           
	if (fcntl(fd, F_SETFL, 0) < 0)
	{
		bv_loge("fcntl failed!\n");
		return FAILURE;
	}     
	else
	{
		bv_logi("fcntl=%d\n",fcntl(fd, F_SETFL,0));
	}
	*/

	/* 测试打开的文件描述符是否为终端设备，进一步确认串口是否正确打开 */
	if (0 == isatty(fd))
	{
		printf("fd%d is not a terminal device.\n",fd);
		return -1;
	}
	else
	{
		printf("isatty success!\n");
	}

	printf("fd-open=%d\n",fd);

	rtn = set_opt(fd, baud_rate, UART_DATA_BIT_8, 'N', UART_STOP_BIT_2);
	if (rtn < 0)
    {
        printf("set_opt  err!");
        return -1;
    }
	
	return fd;
}
int bv_init_uart(int port_no, int band_rate)
{
	int rtn = 0;
	int retry = 3;
	
	if((0 > port_no) || (band_rate <= 0))
	{
		printf("Input argument port_no[%d] or band_rate[%d] error.\n",port_no, band_rate);
		return -1;
	}
	
	/*init uart */
	while((--retry) >= 0)
	{
		rtn = bv_uart_open(port_no, band_rate);
		
		if (rtn > 0)
		{
			gawFdUart[port_no] = rtn;
			printf("uart%d open succeed!",port_no);
			break;
		}
		else if(0 == retry)
		{
			printf("uart%d open faled,retry %d times.\n",port_no,retry);
			return -1;
		}
	}
	
	return rtn;
}
void bv_uart_close(int uart_port)
{
	 int fd=-1;
	 
	 fd = gawFdUart[uart_port];
	 if (fd > 0) 
	 {
		 close(fd);
	 }
}
void bv_uart_close_all()
{
	 int fd = -1;
	 int i;

	 for (i=0; i<UART_PORT_NUM; i++)
	 {
		 fd = gawFdUart[i];
		 if (fd > 0) 
		 {
			 close(fd);
		 }
	 }
}
int bv_uart_read(int fd, char *buf, int len)
{
    fd_set 	fs_read;
	int 	tmp_len;
	char *	pbuf;
	int 	fs_sel;
	int 	recv_len = 0;
    struct timeval time;
	
	//1. 检查输入参数是否合法
	if ((fd < 0) || (NULL == buf) || (len <= 0))
	{
		printf("Input arguments illegal!!");
		return -1;
	}

	//2. 将文件描述符加入到可读写的集合中
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);

	//3. 设置延时等待时间
    time.tv_sec = 0;
    time.tv_usec = 300000;

	//4. 使用select实现串口的多路通信
    fs_sel = select(fd+1,&fs_read, NULL, NULL, &time);
	if (fs_sel)	//如果有数据产生
	{
		pbuf = buf;
		//如果com口有数据
		if (FD_ISSET(fd, &fs_read))	
		{
		    while (recv_len < len-1)
		    {
		       	bv_milliseconds_sleep(3);	//sleep 3毫秒，有可能串口缓存慢导致数据接收错误
				tmp_len = read(fd, pbuf, len-recv_len);
				
				if (tmp_len > 0)
		        {
		           	recv_len += tmp_len;
					pbuf += tmp_len;
					
					if(recv_len >= len-1) 
							break;
		        }
		        else 
		        {
					//tcflush(fd,TCIFLUSH);/*不需要刷新缓存，否则会丢失数据*/
		            break;
		    	}
		    }
	    }
	}
	else
	{
		recv_len = 0;
	} 
	
	return recv_len;
}
int bv_uart_write(int fd, char* p_data_buf, int data_len)
{
	int len;
	
	
	//bv_logd(">>>> UART%d:data_len=%d, data=%s",	uart_port,data_len, data_to_str(p_data_buf,  data_len));

	len = write(fd, p_data_buf, data_len);
	
	if(len == data_len)
	{
		return len;
	}     
	else   
	{
		tcflush(fd,TCOFLUSH);
		printf(">>>> UART: wirte failed!\n");
		
		return -1;
	}
}


/*
int main(void)
{
	int len;
	uart_open(115200);
	sleep(1);
	float ax=0.0,ay=0.0,az=0.0;
	float wx=0.0,wy=0.0,wz=0.0;
	float roll=0.0;
	float pitch=0.0;
	float yaw=0.0;
	short datax=0;
	short datay=0;
	short dataz=0;
	while(1)
	{
		uart_read(s_recv,11);
		for(int i=0;i<11;i++)
	    	printf("%c ",s_recv[i]);
		//sleep(1);
		/*
		if(s_recv[0]==0x55&&s_recv[1]==0x52) //角速度协议
		{
             datax=((short)s_recv[3]<<8)|s_recv[2];
			 datay=((short)s_recv[5]<<8)|s_recv[4];
			 dataz=((short)s_recv[7]<<8)|s_recv[6];
             wx=datax/32768.0*2000;
			 wy=datay/32768.0*2000;
			 wz=dataz/32768.0*2000;
		//	 printf("%f\t%f\t%f \n",wx,wy,wz);
		}
		if(s_recv[0]==0x55&&s_recv[1]==0x51) //加速度协议
		{
			//((AxH<<8)|AxL)/32768*16g
			//((short)DataH<<8)
			 datax=((short)s_recv[3]<<8)|s_recv[2];
			 datay=((short)s_recv[5]<<8)|s_recv[4];
			 dataz=((short)s_recv[7]<<8)|s_recv[6];
             ax=datax/32768.0*16*9.8;
			 ay=datay/32768.0*16*9.8;
			 az=dataz/32768.0*16*9.8;
			// printf("%f\t%f\t%f \n",ax,ay,az);
		}

		if(s_recv[0]==0x55&&s_recv[1]==0x53) //角度输出协议
		{
			//((AxH<<8)|AxL)/32768*16g
			//((short)DataH<<8)
			 datax=((short)s_recv[3]<<8)|s_recv[2];
			 datay=((short)s_recv[5]<<8)|s_recv[4];
			 dataz=((short)s_recv[7]<<8)|s_recv[6];
             roll=datax/32768.0*180;
			 pitch=datay/32768.0*180;
			 yaw=dataz/32768.0S*180;
			 printf("roll=%f\tpitch=%f\tyaw=%f \n",roll,pitch,yaw);
		}
		
	}
	
	return 0;
}

*/

