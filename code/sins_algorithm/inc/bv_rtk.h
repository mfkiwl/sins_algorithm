/*
     tony.jiang 2021 10 26
*/
#ifndef _BV_RTK_H_
#define _BV_RTK_H_
int  rtk_init();
double utc_to_gps(struct time utc_time);
void *thread_getRTKData(void* arg);
extern double gps_data[13];
extern  int rtk_read;

#endif //_BV_RTK_H_