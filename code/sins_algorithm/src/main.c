/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include<unistd.h>
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
#include "typedef.h"
#include "bv_sins.h"
#include "functions_sins.h"
#include "bv_navigation.h"
#include "gnss_paraser.h"
#include "bv_motor.h"
#include "bv_imu.h"
#include "bv_rtk.h"

int main(int argc ,char *argv[])
{
    pthread_t tid1, tid2,tid3;
    int rc1=0, rc2=0,rc3;
    motor_init_uart();
    imu_init_uart();
    rtk_init();
    sleep(1);
   /* motor_left_init();
	motor_rightt_init();
	motor_l_Duty_Ratio();
	motor_r_Duty_Ratio();
   */ 
	rc1 = pthread_create(&tid1, NULL, thread_getImuData, NULL);
	if(rc1 != 0)
		printf("%s: %d\n",__func__, strerror(rc1));
    rc2 = pthread_create(&tid2, NULL, thread_getRTKData, NULL);
	if(rc2 != 0)
		printf("%s: %d\n",__func__, strerror(rc2)); 
      rc3 = pthread_create(&tid3, NULL, bv_navigation_, NULL);
	if(rc3 != 0)
		printf("%s: %d\n",__func__, strerror(rc3));   

    pthread_join(tid1,NULL);
    pthread_join(tid2,NULL);
     pthread_join(tid3,NULL);
   /* 
	

    sleep(1800);
    
    motor_l_stop();
    motor_r_stop();
    */
    sleep(1800);
    
    return 0;
}
