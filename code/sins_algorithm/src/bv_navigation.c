/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include <sys/time.h>  
#include "typedef.h"
#include "bv_sins.h"
#include "functions_sins.h"
#include "bv_navigation.h"
#include "string.h"
#include "bv_imu.h"
#include "bv_rtk.h"
struct  timeval  lastTimetv;  
static  struct  timeval preTimetv;
extern double bv_Yaw;
extern double gps_data[13];
//<<thread
extern  int rtk_read;
void *bv_navigation_(void* arg)
{
     double gpsdata_pre[13]={0.0};
     struct position posi_nav_to_gns, nav_err_to_gns;
     double imu_data_pre[7]={0.0};
     double imu_data[7]={0.0};
     double basic_llh[3]={0.0};
     uint8_t navigation_ready=0;      
     double delta_tt = 0.02;//积分的时间
     struct bv_loose_cfg     loose_cfg_local;
     struct bv_loose_cal_var loose_cal_local;
     memset(&loose_cfg_local,0x00,sizeof(loose_cfg_local));
     memset(&loose_cal_local,0x00,sizeof(loose_cal_local));
     memset(&posi_nav_to_gns,0x00,sizeof(posi_nav_to_gns));
     memset(&nav_err_to_gns,0x00,sizeof(nav_err_to_gns));
     initial_imugps_cfg(&loose_cfg_local);
     re_ini_nav(&loose_cfg_local, &loose_cal_local, bv_Yaw);
     
     while(1)
      {
           if(imu_data_ready==1)
           {
               if(rtk_read==1)
               {
                  memcpy(gpsdata_pre,gps_data,sizeof(gps_data));
                  navigation_ready=initial_alignment(&loose_cfg_local, &loose_cal_local, &imu_data[0],&gps_data[0],&gpsdata_pre[0],delta_tt);
               }
              // printf("\n tony \n");
              if(navigation_ready)
              {
                    gettimeofday( &lastTimetv,NULL);
                    delta_tt = lastTimetv.tv_sec-preTimetv.tv_sec+ 0.01*(lastTimetv.tv_usec-preTimetv.tv_usec);
                    if(delta_tt>0.02) delta_tt=0.02;
                    gettimeofday( &preTimetv,NULL);
                    basic_llh[0]=gps_data[1];
                    basic_llh[1]=gps_data[2];
                    basic_llh[2]=gps_data[3];
                    
                    read_imu_data(&imu_data[0],delta_tt);//imu data
               //  for(int i=1;i<7;i++)
                    //    printf("\n%d=%f\n",i,imu_data[i]);

                    
                    ins_mech(&imu_data_pre[0],&imu_data[0],&loose_cal_local,delta_tt);
                    ins_kf_trn_psi_15state(BV_NSTATES,&loose_cal_local,loose_cfg_local.bg_model,loose_cfg_local.ba_model,delta_tt);
                    ekf_predict(&loose_cfg_local,&loose_cal_local,delta_tt);
					ins_ekf_inte(&loose_cfg_local,&loose_cal_local,&imu_data[0],&gps_data[0],&nav_err_to_gns, &basic_llh[0], delta_tt);

					rela_pos_cal(&basic_llh[0], loose_cal_local.imu2gnss_pos,&posi_nav_to_gns);	
                    nav_err_to_gns.x = nav_err_to_gns.x - posi_nav_to_gns.x;
                    nav_err_to_gns.y = nav_err_to_gns.y - posi_nav_to_gns.y;
              }
               	
           }
      }
}

