

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
int uart_imu_fd = -1;
double bv_Yaw=0.0;
enum coord_front
{
	F_X = 11,
	F_MinusX = 12,
	F_Y = 13,
	F_MinusY = 14,
	F_Z = 15,
	F_MinusZ = 16
};
enum coord_right
{
	R_X = 21,
	R_MinusX = 22,
	R_Y = 23,
	R_MinusY = 24,
	R_Z = 25,
	R_MinusZ = 26
};
enum coord_down
{
	D_X = 31,
	D_MinusX = 32,
	D_Y = 33,
	D_MinusY = 34,
	D_Z = 35,
	D_MinusZ = 36
};
static enum  coord_front  front_corrdinate;
static enum  coord_right  right_corrdinate;
static enum  coord_down   down_corrdinate;


struct imu_data_source
{
	short int  x_gyro;//<<陀螺仪角度
	short int  y_gyro;
	short int  z_gyro;
	short int  x_acc; //<<陀螺仪加速度
	short int  y_acc;
	short int  z_acc;

	short int  x_gyro_;//<<陀螺仪角度
	short int  y_gyro_;
	short int  z_gyro_;


};
static  char s_sent[11]={0};
static  char s_recv[11]={0};
 FILE *fp_imu=NULL;                /*定义一个文件指针*/
 int imu_data_ready=0;
int imu_init_uart()
{
   uart_imu_fd=bv_init_uart(4,9600) ;
   fp_imu=fopen("/home/imudata.txt", "at+"); 
   if(fp_imu==NULL)             /*判断文件是否打开成功*/
   {
        puts("File open error");/*提示打开不成功*/
        return -1;
   }
   imu_data_ready=1;
   front_corrdinate = F_X;//F_MinusX;
   right_corrdinate = R_MinusY;//R_MinusY;//R_Y;
   down_corrdinate = D_MinusZ;//D_MinusZ;
   return 0;
}
void read_imu_data(double imu_data2[],double dt)
{
	short int temp1,temp2;
    struct imu_data_source imu_data_source_local;
	bv_uart_read(uart_imu_fd,s_recv,11);
    float ax=0.0,ay=0.0,az=0.0;
	float wx=0.0,wy=0.0,wz=0.0;
	float roll=0.0;
	float pitch=0.0;
	float yaw=0.0;
	short datax=0;
	short datay=0;
	short dataz=0;
  
    bv_uart_read(uart_imu_fd,s_recv,11);
	if(s_recv[0]==0x55&&s_recv[1]==0x52) 
	{
             imu_data_source_local.x_gyro=((short)s_recv[3]<<8)|s_recv[2];
			 imu_data_source_local.y_gyro=((short)s_recv[5]<<8)|s_recv[4];
			 imu_data_source_local.z_gyro=((short)s_recv[7]<<8)|s_recv[6];
			 imu_data_ready=1;
          //    printf("imu_data_source_local.x_gyro=%f\n",imu_data_source_local.x_gyro);
	      //   printf("imu_data_source_local.y_gyro=%f\n",imu_data_source_local.y_gyro);
	     //  printf("imu_data_source_local.z_gyro=%f\n",imu_data_source_local.z_gyro);
	         imu_data2[1]= imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;
			 imu_data2[2]= imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;
			 imu_data2[3]= imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;
			
	}
	if(s_recv[0]==0x55&&s_recv[1]==0x51) 
	{
			 imu_data_source_local.x_acc=((short)s_recv[3]<<8)|s_recv[2];
			 imu_data_source_local.y_acc=((short)s_recv[5]<<8)|s_recv[4];
			 imu_data_source_local.z_acc=((short)s_recv[7]<<8)|s_recv[6];
			 imu_data2[4]= -imu_data_source_local.x_acc*dt*0.0011962890625f;
			 imu_data2[5]= -imu_data_source_local.y_acc*dt*0.0011962890625f;
			 imu_data2[6]= -imu_data_source_local.z_acc*dt*0.0011962890625f;
	}
	if(s_recv[0]==0x55&&s_recv[1]==0x53) 
	{
			 imu_data_source_local.x_gyro_=((short)s_recv[3]<<8)|s_recv[2];
			 imu_data_source_local.y_gyro_=((short)s_recv[5]<<8)|s_recv[4];
			 imu_data_source_local.z_gyro_=((short)s_recv[7]<<8)|s_recv[6];
			 bv_Yaw=imu_data_source_local.z_gyro_/32768.0;
	}
	/*
	switch(front_corrdinate)
	{
		case F_X:
		{
		imu_data2[1]= imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;
		imu_data2[4]= imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case F_MinusX:
		{
		imu_data2[1]= -imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;
		imu_data2[4]= -imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case F_Y:
		{
		imu_data2[1]= imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[4]= imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case F_MinusY:
		{
		imu_data2[1]= -imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;
		imu_data2[4]= -imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case F_Z:
		{
		imu_data2[1]= imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[4]= imu_data_source_local.z_acc*dt*0.0011962890625f;
		}
		case F_MinusZ:
		{
		imu_data2[1]= -imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[4]= -imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}
		default:
		{
		imu_data2[1]= imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[4]= imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}		
		}
		
		switch(right_corrdinate)
		{
		case R_X:
		{
		imu_data2[2]= imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case R_MinusX:
		{
		imu_data2[2]= -imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= -imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case R_Y:
		{
		imu_data2[2]= imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case R_MinusY:
		{
		imu_data2[2]= -imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= -imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case R_Z:
		{
		imu_data2[2]= imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}
		case R_MinusZ:
		{
		imu_data2[2]= -imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= -imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}
		default:
		{
		imu_data2[2]= imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[5]= imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}		
		}
		
		switch(down_corrdinate)
		{
		case D_X:
		{
		imu_data2[3]= imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[6]= imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case D_MinusX:
		{
		imu_data2[3]= -imu_data_source_local.x_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[6]= -imu_data_source_local.x_acc*dt*0.0011962890625f;
		break;
		}
		case D_Y:
		{
		imu_data2[3]= imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[6]= imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case D_MinusY:
		{
		imu_data2[3]= -imu_data_source_local.y_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;//rad
		imu_data2[6]= -imu_data_source_local.y_acc*dt*0.0011962890625f;
		break;
		}
		case D_Z:
		{
		imu_data2[3]= imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;
		imu_data2[6]= imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}
		case D_MinusZ:
		{
		imu_data2[3]= -imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;
		imu_data2[6]= -imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}
		default:
		{
		imu_data2[3]= imu_data_source_local.z_gyro*dt*0.01745329251994327777777777777778*0.00762939453125*8.0f;;
		imu_data2[6]= imu_data_source_local.z_acc*dt*0.0011962890625f;
		break;
		}		
	}
	*/
	
}
void *thread_getImuData(void* arg)
{
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
		 bv_uart_read(uart_imu_fd,s_recv,11);
		 
       //  fputs(s_recv,fp_imu);
		// fputs("\r\n",fp_imu);
		if(s_recv[0]==0x55&&s_recv[1]==0x52) //角速度协议
		{
             datax=((short)s_recv[3]<<8)|s_recv[2];
			 datay=((short)s_recv[5]<<8)|s_recv[4];
			 dataz=((short)s_recv[7]<<8)|s_recv[6];
             wx=datax/32768.0*2000;
			 wy=datay/32768.0*2000;
			 wz=dataz/32768.0*2000;
			 
			 fprintf(fp_imu,"wx=%f\twy=%f\twz=%f \n",wx,wy,wz);

		//	printf("wx=%f\twy=%f\twz=%f \n",wx,wy,wz);
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
			 fprintf(fp_imu,"ax=%f\tay=%f\taz=%f \n",ax,ay,az);
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
			 yaw=dataz/32768.0*180;
			 fprintf(fp_imu,"roll=%f\tpitch=%f\tyaw=%f \n",roll,pitch,yaw);
			 // printf("roll=%f\tpitch=%f\tyaw=%f \n",roll,pitch,yaw);
		}
		
	}
   
}