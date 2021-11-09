
/*
     tony.jiang 2021 10 26
*/
#ifndef __BV_SINS_H_
#define __BV_SINS_H_

#include "typedef.h"

#define BV_NSTATES 15
#define BV_A_Earth (6378137.0)		//re
#define BV_B_Earth (6356752.3142)
#define BV_F_Earth (1.0/298.257223563)		//fe
#define BV_WE (7.2921151467e-5)
#define BV_E2 (1-(BV_B_Earth/BV_A_Earth)*(BV_B_Earth/BV_A_Earth))
#define BV_GM_Earth 3986004.418e+8
#define BV_J2       1.082626683e-3
#define BV_J3       -2.5327e-6
#define BV_DEG2RAD  0.01745329251994327777777777777778
struct bv_loose_cfg
{
    uint32_t d_rate;
	double ini_pos[3];
	double ini_pos_var[3];
	double ini_vel[3];
	double ini_vel_var[3];
	double ini_att[3];
	double ini_att_var[3];
	double arm[3];
	double gps_pos_std_scale;
	double gps_vel_std_scale;
	double var_zupt;
	uint8_t opt_zuptA;
	uint8_t opt_Holo;
	double var_zuptA;	
	double var_holo[3];
	double c_bv[3][3];
	double var_odom[3];
	double sf_odom;
	double la_odom[3];
	double vrw[3];
	double arw[3];
	double bg[3];
	double ba[3];
	double sg[3];
	double sa[3];
	double bg_var[3];
	double ba_var[3];
	double bg_p0;
	double ba_p0;
	double bg_Tcor;
	double ba_Tcor;
	double bg_model[3];
	double ba_model[3];
	double q[BV_NSTATES][BV_NSTATES];
	double euler[3];
	double la_output[3];
};

struct bv_loose_cal_var
{
	
	double  gps_pos[3];
	double  gps_vel[3];
	double  att[3];
	double  imu2gnd_pos[3];
	double  imu2gnd_vel[3];
	double  imu2gnd_att[3];

	double  imu2gnss_pos[3];
	double  imu2gnss_vel[3];
	
	double  dv_n[3];
	double  q_bn[4];
	double  q_ne[4];
	double  c_bn[3][3];
	
	double  bg[3];
	double  ba[3];
	//Par
	double  rm;
	double  rn;
	double  w_ie[3];
	double  w_en[3];
	double  f_n[3];
	double  g[3];
	
	double  x[BV_NSTATES];
	double  p[BV_NSTATES][BV_NSTATES];
	double  phi[BV_NSTATES][BV_NSTATES];
	double  h[6][BV_NSTATES];

	double h_zupt[4][BV_NSTATES];
	double h_odom[3][BV_NSTATES];
	double h_holo[3][BV_NSTATES];
	double h_zuptA[1][BV_NSTATES];
};
//imu gps  configuration
void initial_imugps_cfg(struct bv_loose_cfg * loose_cfg_local);
//
void re_ini_nav(struct bv_loose_cfg* global_cfg,struct bv_loose_cal_var* P_imugps_globalvar,double head_rad);
//config phi state 
void ins_kf_trn_psi_15state(uint32_t n_states,struct bv_loose_cal_var* p_loose_cal_var,volatile double gb_model[],volatile double ab_model[],double dt);
//
void ekf_predict(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,double dt);
uint8_t initial_alignment(struct bv_loose_cfg* global_cfg, struct bv_loose_cal_var* p_imugps_var,volatile double imudata[],volatile double gpsdata[],volatile double gpsdata_pre[],double dt);
//
void kf_update_inte_6_6(struct bv_loose_cal_var* p_loose_cal_var,volatile double inno[],volatile double r[]);
//
uint8_t ins_ekf_inte(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,volatile double ins_data[],volatile double gps_data[],struct position* nav_err, double basic_llh[], double dt);
//
void kf_update_holo_3_3(struct bv_loose_cal_var* p_loose_cal_var,volatile double inno[],volatile double r[]);
//
uint8_t ins_ekf_holo_3(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,volatile double ins_data[],double dt, int16_t rpm_left, int16_t rpm_right);
//
void update_filter_solution(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,double imudata[], double dt,double nav_updata);
#endif