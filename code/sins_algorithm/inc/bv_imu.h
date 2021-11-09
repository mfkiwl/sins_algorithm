/*
     tony.jiang 2021 10 26
*/
#ifndef _BV_IMU_H_
#define _BV_IMU_H_
int imu_init_uart();
void read_imu_data(double imu_data2[],double dt);
void *thread_getImuData(void* arg);

extern double bv_Yaw;

#endif //_BV_IMU_H_