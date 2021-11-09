/*
     tony.jiang 2021 10 26
*/
#ifndef __TYPEDEF_H_
#define __TYPEDEF_H_

#include <stdint.h>

struct position
{
  float x;
  float y;
};


struct point
{
  int32_t m;
  int32_t n;
};


struct time
{
  uint8_t month;
  uint8_t day;
  uint8_t weekday;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t second_xiaoshu;
  uint16_t year;
};


struct abline
{
  struct position a;
  struct position b;
};


struct navigation
{
  struct position pos;
  float velocity[3];
  float roll;
  float pitch;
  float head;
};


struct gnss_data
{
  uint8_t rtk;
  uint8_t rtk_delay;
  uint8_t num_satellite;
  struct time utc_time;
  struct position pos;
  double basic_llh[3];
  double position_xyz[3];
  float height;
};
/*
struct imu_data_source
{
	short int  x_gyro;//
	short int  y_gyro;
	short int  z_gyro;
	short int  x_acc; //
	short int  y_acc;
	short int  z_acc;
};
*/
struct mower_config
{
  uint8_t flag_map;
  uint8_t flag_reserve;
  uint8_t flag_imu;
  uint8_t flag_base_position;
  
  uint8_t flag_map_height1;
  uint8_t flag_map_height2;
  uint8_t work_type;
  uint8_t pad;
  
  uint8_t flag_charge_position;
  uint8_t flag_checkcode;
  uint8_t flag_work_in_rain;
  uint8_t gnss_type;
  
  uint8_t reserve_interval;
  uint8_t reserve_weekday[7];
  
  uint8_t reserve_hour;
  uint8_t reserve_minute;
  uint8_t checkcode[10];
  
  uint32_t version;
  
  double Gyo_bg[3];
  double Acc_ba[3];
  float  position_charge[2];
  double position_base[3];
};
extern int imu_data_ready;
#endif

