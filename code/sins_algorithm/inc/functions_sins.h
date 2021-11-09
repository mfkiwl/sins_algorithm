/*
     tony.jiang 2021 10 26
*/
#ifndef _FUNCTIONS_SINS_H_
#define _FUNCTIONS_SINS_H_

#include "typedef.h"
#include "bv_sins.h"

void euler2dcm(volatile double roll,volatile double pitch,volatile double heading,volatile double dcm[]);
int8_t norm(  int8_t m,  volatile double A[],volatile double* Val);
void matrix_multiply(uint8_t m1, uint8_t n1, uint8_t m2, uint8_t n2, volatile double M1[], volatile double M2[],volatile double M3[]);
void matrix_transpose(uint8_t m, uint8_t n,  volatile double M1[], volatile double MT[]);
void matrix_addition(uint8_t m, uint8_t n,  volatile double M1[],  volatile double M2[], volatile double M3[]);
int8_t matrix_inv(int8_t n,  volatile double a[], volatile double b[]);
void compensate(volatile double obs[],volatile double bias[],volatile double scale[],volatile double Corr[]);
void rc(double a,double e2,double lat,double* M,double *N);
void euler2quat(double roll,double pitch,double heading,volatile double q[]);
void pos2quat(double lat,double lon,volatile double q_ne[]);
void cross_product(volatile double a[],volatile double b[],volatile double c[]);
void dpos2rvec(double lat,double delta_lat,double delta_lon,volatile double rv[]);
void rvec2quat(volatile double rot_vec[],volatile double q[]);
void quat_prod(volatile double q[],volatile double p[],volatile double q1[]);
void quat2pos(volatile double q_ne[],volatile double* plat,volatile double* plon);
void cp_form(volatile double v[],volatile double VV[]);
void quat2dcm(volatile double q[],volatile double VV[]);
void geo2ecef(double e2,double Rn,volatile double geo[],volatile double r_e[]);
void pos2dcm(double lat,double lon,volatile double C_ne[]);
int8_t matrix_chol(volatile double input1_r[],int32_t input1_M,int32_t input1_N,volatile double output1_r[],int32_t output1_M,int32_t output1_N);
void dcm2euler(volatile double dc[],double* p_roll,double* p_pitch,double* p_heading);
void ins_mech(volatile double meas_pre[],volatile double meas[],struct bv_loose_cal_var* p_imugps_globalvar,double dt);
void rela_pos_cal(double basic_blh[3], double pos_llh[3],struct position *pos_blv);

#endif
