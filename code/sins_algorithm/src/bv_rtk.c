/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include <stddef.h>
#include "typedef.h"
#include "bv_sins.h"
#include "functions_sins.h"
#include "bv_navigation.h"
#include "gnss_paraser.h"
#include "bv_rs232.h"
#include "string.h"
#include "gnss_paraser.h"
#include "math.h"
static gnrmc_type gnrmc;
static gngga_type gngga;
int uart_rtk_fd = -1;
static  char s_recv[240]={0};
FILE *fp_rtk=NULL;                /*定义一个文件指针*/
int rtk_read=0;
struct gnss_data t_gnss_data;
int rtk_init()
{
   uart_rtk_fd=bv_init_uart(2,115200) ;
   fp_rtk=fopen("/home/rtkdata.txt", "at+"); 
   if(fp_rtk==NULL)             /*判断文件是否打开成功*/
   {
        puts("File open error");/*提示打开不成功*/
        return -1;
   }
        
   return 0;
}
double gps_data[13]={0.0};
double utc_to_gps(struct time utc_time)
{
  double gps_week=0;
  static long month_day[2][12] = {{0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334},
                                  {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335}};
  long yday, mjd, leap;
  double fmjd;

  leap = (utc_time.year%4 == 0);
  yday = month_day[leap][utc_time.month-1] + utc_time.day;

  mjd = ((utc_time.year-1901)/4)*1461 + ((utc_time.year-1901)%4)*365 + yday-1+15385;
  fmjd = ((utc_time.second/60.0 + utc_time.minute)/60.0 + utc_time.hour)/24.0;

  gps_week = (mjd-44244)/7;
  return ((mjd-44244) - gps_week*7 + fmjd )*86400.0 + ((double)utc_time.second_xiaoshu)/100.0;
}
void *thread_getRTKData(void* arg)
{
   struct gnss_data t_gnss_data;
   
   memset(&t_gnss_data,0,sizeof(t_gnss_data));
   while(1)
	{
		 int rec=bv_uart_read(uart_rtk_fd,s_recv,240);
       if(rec>0)
       {
          rtk_read=1;
       }
       NMEA_GNRMC_Analysis(&gnrmc, (uint8_t *)s_recv);
       NMEA_GNGGA_Analysis(&gngga, (uint8_t *)s_recv);
       struct gnss_data t_gnss_data;
       t_gnss_data.basic_llh[0]=gnrmc.latitude;
       t_gnss_data.basic_llh[1]=gnrmc.longitude;
       t_gnss_data.basic_llh[2]=gngga.msl;

        t_gnss_data.utc_time.hour = gnrmc.utc.hour;
        t_gnss_data.utc_time.minute = gnrmc.utc.min;
        t_gnss_data.utc_time.second = gnrmc.utc.sec;
        t_gnss_data.utc_time.second_xiaoshu = gnrmc.utc.pse;
          
        t_gnss_data.utc_time.day = gnrmc.utc.date;
        t_gnss_data.utc_time.month = gnrmc.utc.month;
        t_gnss_data.utc_time.year = gnrmc.utc.year;

        double temp=utc_to_gps(t_gnss_data.utc_time);
       if(temp - gps_data[0] > 0.05)
      {
            gps_data[0] = temp;
      }
          
      if(gps_data[0] < 86400.0)
            t_gnss_data.utc_time.weekday = 7;
      else
            t_gnss_data.utc_time.weekday = ceil(gps_data[0] / 86400.0) - 1;

      gps_data[1] =t_gnss_data.basic_llh[0]* 0.01745329251994327777777777777778;
      gps_data[2] =t_gnss_data.basic_llh[1]* 0.01745329251994327777777777777778;
      gps_data[3] =t_gnss_data.basic_llh[2];
      gps_data[4] =0;
      gps_data[5] =gnrmc.speed;
      gps_data[6] =0;
      printf("\r\n GNRMC gpsx->gps_data[1]=%.8f \r\n",        gps_data[1] );
     // printf("GNRMC gpsx->direction=%.8f \r\n",    gnrmc.direction );
       fputs(s_recv,fp_rtk);
   }
}

