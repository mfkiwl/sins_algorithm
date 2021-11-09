/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include "bv_sins.h"
#include "functions_sins.h"
#include "mathmisc.h"

#define			DIA_wheel			0.22f
#define 		reduc_ratio		    3840.0f 
// coord

//imu gps  configuration
void initial_imugps_cfg(struct bv_loose_cfg * loose_cfg_local)
{
    volatile uint32_t i,j;
	volatile int32_t temp;
	volatile double temp_arm[3],c_vb[3][3];
	volatile double q_bg[3];
	volatile double q_ba[3];

	loose_cfg_local->d_rate = 50;
	

	
	
	loose_cfg_local->ini_pos[0]=0;
	loose_cfg_local->ini_pos[1]=0;
	loose_cfg_local->ini_pos[2]=0;
	loose_cfg_local->ini_pos_var[0]=0.15*0.15;
	loose_cfg_local->ini_pos_var[1]=0.15*0.15;
	loose_cfg_local->ini_pos_var[2]=0.15*0.15;
	
	loose_cfg_local->ini_vel[0]=0.0;
	loose_cfg_local->ini_vel[1]=0.0;
	loose_cfg_local->ini_vel[2]=0.0;
	loose_cfg_local->ini_vel_var[0]=0.2*0.2;
	loose_cfg_local->ini_vel_var[1]=0.2*0.2;
	loose_cfg_local->ini_vel_var[2]=0.2*0.2;

	loose_cfg_local->ini_att[0]=0.0;
	loose_cfg_local->ini_att[1]=0.0;
	loose_cfg_local->ini_att[2]=0.0;

	loose_cfg_local->ini_att_var[0]=(2.0*BV_DEG2RAD)*(2.0*BV_DEG2RAD);
	loose_cfg_local->ini_att_var[1]=(2.0*BV_DEG2RAD)*(2.0*BV_DEG2RAD);
	loose_cfg_local->ini_att_var[2]=(2.0*BV_DEG2RAD)*(2.0*BV_DEG2RAD);
	
	loose_cfg_local->gps_pos_std_scale=1;
	loose_cfg_local->gps_vel_std_scale=1;

	
	loose_cfg_local->var_zupt = 0.01*0.01;
	loose_cfg_local->opt_zuptA = 1;//
	loose_cfg_local->var_zuptA = (0.1*BV_DEG2RAD)*(0.1*BV_DEG2RAD);

	loose_cfg_local->var_holo[0]=0.06*0.06;//0.018*0.018;//0.040
	loose_cfg_local->var_holo[1]=0.02*0.02;//0.015*0.015;//0.035
	loose_cfg_local->var_holo[2]=0.02*0.02;//0.015*0.015;//0.035
	//NHC  
	loose_cfg_local->euler[0]=0.0;//-0.55;//-0.4;//1.5;
	loose_cfg_local->euler[1]=0.0;//-0.936;//-0.905;//1.475;
	loose_cfg_local->euler[2]=0.0;//-1.495;//-1.465;//2.137;

	loose_cfg_local->opt_Holo=1;
	euler2dcm(loose_cfg_local->euler[0]*BV_DEG2RAD,loose_cfg_local->euler[1]*BV_DEG2RAD,loose_cfg_local->euler[2]*BV_DEG2RAD,&loose_cfg_local->c_bv[0][0]);
	matrix_transpose(3,3,&loose_cfg_local->c_bv[0][0],&c_vb[0][0]);
	//
	loose_cfg_local->var_odom[0]=0.1*0.1;
	loose_cfg_local->var_odom[1]=loose_cfg_local->var_holo[0];
	loose_cfg_local->var_odom[2]=loose_cfg_local->var_holo[1];

	loose_cfg_local->sf_odom=1;
	//
	loose_cfg_local->la_odom[0]= 0.08;
	loose_cfg_local->la_odom[1]= 0.0;//88;
	loose_cfg_local->la_odom[2]= 0.056;
	//
	matrix_multiply(3,3,3,1,&c_vb[0][0],&loose_cfg_local->la_odom[0],&temp_arm[0]);
	loose_cfg_local->la_odom[0]=temp_arm[0];
	loose_cfg_local->la_odom[1]=temp_arm[1];
	loose_cfg_local->la_odom[2]=temp_arm[2];
	//
	loose_cfg_local->arm[0]= 0.27;
	loose_cfg_local->arm[1]= 0.0;
	loose_cfg_local->arm[2]= -0.038;
	//
	loose_cfg_local->la_output[0]=loose_cfg_local->arm[0];
	loose_cfg_local->la_output[1]=loose_cfg_local->arm[1];
	loose_cfg_local->la_output[2]=loose_cfg_local->arm[2];
	//
	matrix_multiply(3,3,3,1,&c_vb[0][0],&loose_cfg_local->arm[0],&temp_arm[0]);
	loose_cfg_local->arm[0]=temp_arm[0];
	loose_cfg_local->arm[1]=temp_arm[1];
	loose_cfg_local->arm[2]=temp_arm[2];

	//
	loose_cfg_local->vrw[0]=1.5/60.0;
	loose_cfg_local->vrw[1]=1.5/60.0;
	loose_cfg_local->vrw[2]=1.5/60.0;
	loose_cfg_local->arw[0]=10*BV_DEG2RAD/60;
	loose_cfg_local->arw[1]=10*BV_DEG2RAD/60;
	loose_cfg_local->arw[2]=10*BV_DEG2RAD/60;
	
	

	loose_cfg_local->bg[0]=-0.056;//-0.030409;//0.0;
	loose_cfg_local->bg[1]=-0.008;//0.03080;//0.0;
	loose_cfg_local->bg[2]=0.031;//-0.030037;//0.0;
	loose_cfg_local->ba[0]=-0.1572;//-0.05313;//0.0;
	loose_cfg_local->ba[1]=-0.1261;//-0.247879;//0.0;
	loose_cfg_local->ba[2]=-0.3496;//0.2079;//0.0;

	
	loose_cfg_local->sg[0]=0.0;
	loose_cfg_local->sg[1]=0.0;
	loose_cfg_local->sg[2]=0.0;
	loose_cfg_local->sa[0]=0.0;
	loose_cfg_local->sa[1]=0.0;
	loose_cfg_local->sa[2]=0.0;
	
	loose_cfg_local->bg_var[0]=(150*BV_DEG2RAD/3600)*(150*BV_DEG2RAD/3600); 
	loose_cfg_local->bg_var[1]=(150*BV_DEG2RAD/3600)*(150*BV_DEG2RAD/3600); 
	loose_cfg_local->bg_var[2]=(150*BV_DEG2RAD/3600)*(150*BV_DEG2RAD/3600); 
	
	loose_cfg_local->ba_var[0]=0.1*0.1;//0.0013*0.0013;
	loose_cfg_local->ba_var[1]=0.1*0.1;//0.001*0.001;
	loose_cfg_local->ba_var[2]=0.1*0.1;//0.001*0.001;
	
	loose_cfg_local->bg_p0=(150*BV_DEG2RAD/3600)*(150*BV_DEG2RAD/3600); 
	loose_cfg_local->ba_p0=0.1*0.1;
	
	loose_cfg_local->bg_Tcor=1.0*1600;//gyro bias correlation time [sec]
	loose_cfg_local->ba_Tcor=1.0*1600;//acc bias correlation time [sec]
	
	//calculate necessary parameters for 1st order GM process, based on given parameters
	loose_cfg_local->bg_model[0]=exp(-1.0/loose_cfg_local->bg_Tcor/loose_cfg_local->d_rate);
	loose_cfg_local->bg_model[1]=exp(-1.0/loose_cfg_local->bg_Tcor/loose_cfg_local->d_rate);
	loose_cfg_local->bg_model[2]=exp(-1.0/loose_cfg_local->bg_Tcor/loose_cfg_local->d_rate);

	loose_cfg_local->ba_model[0]=exp(-1.0/loose_cfg_local->ba_Tcor/loose_cfg_local->d_rate);
	loose_cfg_local->ba_model[1]=exp(-1.0/loose_cfg_local->ba_Tcor/loose_cfg_local->d_rate);
	loose_cfg_local->ba_model[2]=exp(-1.0/loose_cfg_local->ba_Tcor/loose_cfg_local->d_rate);

	q_bg[0] = 2*loose_cfg_local->bg_var[0]/loose_cfg_local->bg_Tcor;
	q_bg[1] = 2*loose_cfg_local->bg_var[1]/loose_cfg_local->bg_Tcor;
	q_bg[2] = 2*loose_cfg_local->bg_var[2]/loose_cfg_local->bg_Tcor;
	
	q_ba[0] = 2*loose_cfg_local->ba_var[0]/loose_cfg_local->ba_Tcor;
	q_ba[1] = 2*loose_cfg_local->ba_var[1]/loose_cfg_local->ba_Tcor;
	q_ba[2] = 2*loose_cfg_local->ba_var[2]/loose_cfg_local->ba_Tcor;
	
	for (i=0;i<15;i++)
	{
		for (j=0;j<15;j++)
		{
			loose_cfg_local->q[i][j] = 0.0;
		}
	}
	
	loose_cfg_local->q[3][3] = loose_cfg_local->vrw[0]*loose_cfg_local->vrw[0];
	loose_cfg_local->q[4][4] = loose_cfg_local->vrw[1]*loose_cfg_local->vrw[1];
	loose_cfg_local->q[5][5] = loose_cfg_local->vrw[2]*loose_cfg_local->vrw[2];

	loose_cfg_local->q[6][6] = loose_cfg_local->arw[0]*loose_cfg_local->arw[0];
	loose_cfg_local->q[7][7] = loose_cfg_local->arw[1]*loose_cfg_local->arw[1];
	loose_cfg_local->q[8][8] = loose_cfg_local->arw[2]*loose_cfg_local->arw[2];

	loose_cfg_local->q[9][9]	 = q_bg[0];
	loose_cfg_local->q[10][10] = q_bg[1];
	loose_cfg_local->q[11][11] = q_bg[2];

	loose_cfg_local->q[12][12] = q_ba[0];
	loose_cfg_local->q[13][13] = q_ba[1];
	loose_cfg_local->q[14][14] = q_ba[2];
	
}
void re_ini_nav(struct bv_loose_cfg* global_cfg,struct bv_loose_cal_var* P_imugps_globalvar,double head_rad)
{
   volatile uint8_t i,j;
   volatile double att[3];
    global_cfg->bg[0]=-0.056;//-0.030409;//0.0;
    global_cfg->bg[1]=-0.008;//0.03080;//0.0;
    global_cfg->bg[2]=0.031;//-0.030037;//0.0;
    global_cfg->ba[0]=-0.1572;//-0.05313;//0.0;
    global_cfg->ba[1]=-0.1261;//-0.247879;//0.0;
    global_cfg->ba[2]=-0.3496;//0.2079;//0.0;
  

	for (i=0;i<3;i++)
	{
		P_imugps_globalvar->bg[i] = global_cfg->bg[i];
		P_imugps_globalvar->ba[i] = global_cfg->ba[i];
	}

    att[0] = 0.0;
    att[1] = 0.0;
    att[2] = head_rad; //要得到航向角度 jiang
    //initial att matrix Cbn
    euler2dcm(att[0],att[1],att[2],&P_imugps_globalvar->c_bn[0][0]);    
    euler2quat(att[0],att[1],att[2],&P_imugps_globalvar->q_bn[0]);

    P_imugps_globalvar->p[6][6]=global_cfg->ini_att_var[0];
    P_imugps_globalvar->p[7][7]=global_cfg->ini_att_var[1];
    P_imugps_globalvar->p[8][8]=global_cfg->ini_att_var[2];
    //initial P0 better large 
    P_imugps_globalvar->p[9][9]=  global_cfg->bg_p0;//global_cfg->bg_var;
    P_imugps_globalvar->p[10][10]=global_cfg->bg_p0;//global_cfg->bg_var;
    P_imugps_globalvar->p[11][11]=global_cfg->bg_p0;//global_cfg->bg_var;

    P_imugps_globalvar->p[12][12]=global_cfg->ba_p0;//global_cfg->ba_var;
    P_imugps_globalvar->p[13][13]=global_cfg->ba_p0;//global_cfg->ba_var;
    P_imugps_globalvar->p[14][14]=global_cfg->ba_p0;//global_cfg->ba_var;

  
}
//Transition matrix for the psi-angle error model, 15 state KF
void ins_kf_trn_psi_15state(uint32_t n_states,struct bv_loose_cal_var* p_loose_cal_var,volatile double gb_model[],volatile double ab_model[],double dt)
{
    volatile uint32_t i,j;
	volatile double v1[3],v2[3][3];
	volatile double R;
	//dt=0.02001953125;
	//dt=IMU_DATA_CURR[0]-IMU_DATA_PREV[0];
	for (i=0;i<n_states;i++)
	{
		for (j=0;j<n_states;j++)
		{
			p_loose_cal_var->phi[i][j]=0.0f;//
		}
	}
	//Position error dynamics
	//Pos to Pos //找到 w_en
	v1[0]=-p_loose_cal_var->w_en[0];
	v1[1]=-p_loose_cal_var->w_en[1];
	v1[2]=-p_loose_cal_var->w_en[2];
	cp_form(&v1[0],&v2[0][0]);

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				p_loose_cal_var->phi[i][j]=1.0+v2[i][j]*dt;
				//pos to vel
				p_loose_cal_var->phi[i][j+3]=1.0*dt;
			}
			else
			{
				p_loose_cal_var->phi[i][j]=v2[i][j]*dt;
			}

		}
	}

	//Velocity error dynamics
	//vel to pos /rm rn 得到 jiang
	R=sqrt(p_loose_cal_var->rm*p_loose_cal_var->rn);
	p_loose_cal_var->phi[3][0]=-p_loose_cal_var->g[2]/(R+p_loose_cal_var->gps_pos[2])*dt;
	p_loose_cal_var->phi[4][1]=p_loose_cal_var->phi[3][0];
	p_loose_cal_var->phi[5][2]=-2*p_loose_cal_var->phi[3][0];

	//Vel to Vel //w_ie jiang
	v1[0]=-2*p_loose_cal_var->w_ie[0]-p_loose_cal_var->w_en[0];
	v1[1]=-2*p_loose_cal_var->w_ie[1]-p_loose_cal_var->w_en[1];
	v1[2]=-2*p_loose_cal_var->w_ie[2]-p_loose_cal_var->w_en[2];

	cp_form(&v1[0],&v2[0][0]);
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				p_loose_cal_var->phi[i+3][j+3]=1.0+v2[i][j]*dt;
			} 
			else
			{
				p_loose_cal_var->phi[i+3][j+3]=v2[i][j]*dt;
			}
		}
	}

	//Vel to Att
	cp_form(&p_loose_cal_var->f_n[0],&v2[0][0]);
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			p_loose_cal_var->phi[i+3][j+6]=v2[i][j]*dt;
		}
	}

	//Vel to Acc bias
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			p_loose_cal_var->phi[i+3][j+12]=p_loose_cal_var->c_bn[i][j]*dt;
		}
	}

	//Attitude error dynamics
	v1[0]=-p_loose_cal_var->w_ie[0]-p_loose_cal_var->w_en[0];
	v1[1]=-p_loose_cal_var->w_ie[1]-p_loose_cal_var->w_en[1];
	v1[2]=-p_loose_cal_var->w_ie[2]-p_loose_cal_var->w_en[2];
	cp_form(&v1[0],&v2[0][0]);
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				p_loose_cal_var->phi[i+6][j+6]=1.0+v2[i][j]*dt;
			} 
			else
			{
				p_loose_cal_var->phi[i+6][j+6]=v2[i][j]*dt;
			}
		}
	}
	
	//Att to gyro bias
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			p_loose_cal_var->phi[i+6][j+9]=-p_loose_cal_var->c_bn[i][j]*dt;
		}
	}

	//gyro bias
	p_loose_cal_var->phi[9][9]=gb_model[0];
	p_loose_cal_var->phi[10][10]=gb_model[1];
	p_loose_cal_var->phi[11][11]=gb_model[2];

	//acc bias
	p_loose_cal_var->phi[12][12]=ab_model[0];
	p_loose_cal_var->phi[13][13]=ab_model[1];
	p_loose_cal_var->phi[14][14]=ab_model[2];

}  

void ekf_predict(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,double dt)
{
	volatile uint32_t i,j;
	volatile double x1[BV_NSTATES];
	volatile double phit[BV_NSTATES][BV_NSTATES];
	volatile double phiq[BV_NSTATES][BV_NSTATES],qphit[BV_NSTATES][BV_NSTATES];
	volatile double qd[BV_NSTATES][BV_NSTATES];
	//dt=0.02001953125;
	//dt=IMU_DATA_CURR[0]-IMU_DATA_PREV[0];

	matrix_transpose(15,15,&p_loose_cal_var->phi[0][0],&phit[0][0]);
	
	matrix_multiply(15,15,15,15,&p_loose_cal_var->phi[0][0],&loose_cfg_local->q[0][0],&phiq[0][0]);
	matrix_multiply(15,15,15,15,&loose_cfg_local->q[0][0],&phit[0][0],&qphit[0][0]);

	for (i=0;i<BV_NSTATES;i++)
	{
		for (j=0;j<BV_NSTATES;j++)
		{
			qd[i][j]=0.5*(phiq[i][j]+qphit[i][j])*dt;
		}
	}

	matrix_multiply(15,15,15,15,&p_loose_cal_var->phi[0][0],&p_loose_cal_var->p[0][0],&phiq[0][0]);
	matrix_multiply(15,15,15,15,&phiq[0][0],&phit[0][0],&qphit[0][0]);

	for (i=0;i<BV_NSTATES;i++)
	{
		for (j=0;j<BV_NSTATES;j++)
		{
			p_loose_cal_var->p[i][j]=qphit[i][j]+qd[i][j];
		}
		p_loose_cal_var->x[i]=0.0;//x1[i];x1===0
	}

}



void kf_update_inte_6_6(struct bv_loose_cal_var* p_loose_cal_var,volatile double inno[],volatile double r[])
{


	volatile double pht[15][6];
	volatile double hpht[6][6];
	volatile double rhpht[6][6];
	volatile double ht[15][6];
	volatile double u[6][6],ut[6][6],uinv[6][6],uinvuinvt[6][6];
	volatile double k[15][6],kt[6][15];
	volatile double dx[15],x_up[15];
	volatile double ikh[15][15],ikht[15][15];

	volatile double v1[15][15],v2[15][15],v3[15][15];

	volatile uint8_t i,j;
	volatile int32_t reval;

	matrix_transpose(6,15,&p_loose_cal_var->h[0][0],&ht[0][0]);//H'
	matrix_multiply(15,15,15,6,&p_loose_cal_var->p[0][0],&ht[0][0],&pht[0][0]);//PH'

	matrix_multiply(6,15,15,6,&p_loose_cal_var->h[0][0],&pht[0][0],&hpht[0][0]);//HPH'

	matrix_addition(6,6,&r[0],&hpht[0][0],&rhpht[0][0]);//R+HPH'
	//////////////QC//////////////////////////
	for(i=0;i<6;i++)
	{
		if(fabs(inno[i])>20*sqrt(rhpht[i][i]))
			rhpht[i][i]=sqrt(rhpht[i][i])*fabs(inno[i]);
	}
	/////////////////////////////////////////

	for (i=0;i<6;i++)
	{
		for (j=0;j<6;j++)
		{
			u[i][j]=0.0;
		}
	}
	
	reval=matrix_chol(&rhpht[0][0],6,6,&ut[0][0],6,6);//
	matrix_transpose(6,6,&ut[0][0],&u[0][0]);

	if (reval==1)//Positive definete
	{
		matrix_inv(6,&u[0][0],&uinv[0][0]);//U=inv(U)
		matrix_transpose(6,6,&uinv[0][0],&u[0][0]);//U = U * U'; % eventually, U = inv(RHPHt)
		matrix_multiply(6,6,6,6,&uinv[0][0],&u[0][0],&uinvuinvt[0][0]);
	
		matrix_multiply(15,6,6,6,&pht[0][0],&uinvuinvt[0][0],&k[0][0]);//K = PHt * U;

		matrix_multiply(15,6,6,1,&k[0][0],&inno[0],&dx[0]);//dx = K * inno;

		matrix_addition(15,1,&p_loose_cal_var->x[0],&dx[0],&x_up[0]);
	
		matrix_multiply(15,6,6,15,&k[0][0],&p_loose_cal_var->h[0][0],&ikh[0][0]);//IKH = eye(length(x)) - K * H;
		for (i=0;i<15;i++)
		{
			for (j=0;j<15;j++)
			{
				if (i==j)
				{
					ikh[i][j]=1.0-ikh[i][j];
				}
				else
				{
					ikh[i][j]=-ikh[i][j];
				}
			}
		}
	
		matrix_transpose(15,15,&ikh[0][0],&ikht[0][0]);
		matrix_transpose(15,6,&k[0][0],&kt[0][0]);

		matrix_multiply(15,15,15,15,&ikh[0][0],&p_loose_cal_var->p[0][0],&v1[0][0]);
		matrix_multiply(15,15,15,15,&v1[0][0],&ikht[0][0],&v2[0][0]);

		matrix_multiply(15,6,6,6,&k[0][0],&r[0],&v1[0][0]);
		matrix_multiply(15,6,6,15,&v1[0][0],&kt[0][0],&v3[0][0]);

		matrix_addition(15,15,&v2[0][0],&v3[0][0],&p_loose_cal_var->p[0][0]);

		for (i=0;i<15;i++)
		{
			p_loose_cal_var->x[i]=x_up[i];
		//	printf("\n%f\n",p_loose_cal_var->x[i]);
		}	

		
	} 


}
uint8_t initial_alignment(struct bv_loose_cfg* global_cfg, struct bv_loose_cal_var* p_imugps_var,volatile double imudata[],volatile double gpsdata[],volatile double gpsdata_pre[],double dt)
{
	volatile uint8_t i,j,ini_finish_flag;
	volatile double delta_t=0,temp_head=0;
	volatile double ret_val;
	volatile double d_lat,d_lon,d_h;
	volatile double val1,val2,val3;
	volatile double ba[3],bg[3],sa[3],sg[3];
	volatile double bgt[3],bat[3];
	volatile double att[3];
	volatile double cbn_x_arm[3];
	volatile double std_heading;
	volatile double r_gps_e_pre[3],r_gps_e[3],delta_r_gps_e[3],d_gps[3],c_en[3][3],c_ne[3][3];
    
	
	for (i=0;i<3;i++)
	{
		ba[i]=global_cfg->ba[i]; 
		bg[i]=global_cfg->bg[i];
		sa[i]=global_cfg->sa[i];
		sg[i]=global_cfg->sg[i];
		p_imugps_var->bg[i]=global_cfg->bg[i];
		p_imugps_var->ba[i]=global_cfg->ba[i];
	}
	
	
		
     if(gpsdata_pre[1]!=0 )
	       ini_finish_flag=1;
		
		rc(BV_A_Earth,BV_E2,gpsdata_pre[1],&p_imugps_var->rm,&p_imugps_var->rn);
		pos2dcm(gpsdata_pre[1],gpsdata_pre[2],&c_ne[0][0]);
		matrix_transpose(3,3,&c_ne[0][0],&c_en[0][0]);
		geo2ecef(BV_E2,p_imugps_var->rn,&gpsdata_pre[1],&r_gps_e_pre[0]);
		geo2ecef(BV_E2,p_imugps_var->rn,&gpsdata[1],&r_gps_e[0]);
		delta_r_gps_e[0]=r_gps_e[0]-r_gps_e_pre[0];
		delta_r_gps_e[1]=r_gps_e[1]-r_gps_e_pre[1];
		delta_r_gps_e[2]=r_gps_e[2]-r_gps_e_pre[2];
		matrix_multiply(3,3,3,1,&c_en[0][0],&delta_r_gps_e[0],&d_gps[0]);
    

		delta_t=imudata[0]-gpsdata[0];

			if(delta_t<0 || delta_t>0.1)
				delta_t=0.02;
	
		rc(BV_A_Earth,BV_E2,gpsdata[1],&p_imugps_var->rm,&p_imugps_var->rn);
		d_lat=gpsdata[4]*delta_t/(p_imugps_var->rm+gpsdata[3]);
		d_lon=gpsdata[5]*delta_t/(p_imugps_var->rn+gpsdata[3])/cos(gpsdata[1]);
		d_h=-gpsdata[6]*delta_t;
		
		p_imugps_var->gps_pos[0]=gpsdata[1]+d_lat;
		p_imugps_var->gps_pos[1]=gpsdata[2]+d_lon;
		p_imugps_var->gps_pos[2]=gpsdata[3]+d_h;

		p_imugps_var->gps_vel[0]=gpsdata[4];
		p_imugps_var->gps_vel[1]=gpsdata[5];
		p_imugps_var->gps_vel[2]=-gpsdata[6];
		
		euler2dcm(att[0],att[1],att[2],&p_imugps_var->c_bn[0][0]);

		for (i=0;i<3;i++)
		{
			bgt[i]=bg[i]*dt;
			bat[i]=ba[i]*dt;
		}
		
		compensate(&imudata[1],&bgt[0],&sg[0],&imudata[1]);
		compensate(&imudata[4],&bat[0],&sa[0],&imudata[4]);

		rc(BV_A_Earth,BV_E2,p_imugps_var->gps_pos[0],&p_imugps_var->rm,&p_imugps_var->rn);

		matrix_multiply(3,3,3,1,&p_imugps_var->c_bn[0][0],&global_cfg->arm[0],&cbn_x_arm[0]);
		p_imugps_var->gps_pos[0]=p_imugps_var->gps_pos[0]-cbn_x_arm[0]/(p_imugps_var->rm+p_imugps_var->gps_pos[2]);
		p_imugps_var->gps_pos[1]=p_imugps_var->gps_pos[1]-cbn_x_arm[1]/(p_imugps_var->rn+p_imugps_var->gps_pos[2])/cos(p_imugps_var->gps_pos[0]);
		p_imugps_var->gps_pos[2]=p_imugps_var->gps_pos[2]+cbn_x_arm[2];
		
		euler2quat(att[0],att[1],att[2],&p_imugps_var->q_bn[0]);
	
		pos2quat(p_imugps_var->gps_pos[0],p_imugps_var->gps_pos[1],&p_imugps_var->q_ne[0]);
	
		for (i=0;i<BV_NSTATES;i++)
		{
			p_imugps_var->x[i]=0.0;
			for (j=0;j<BV_NSTATES;j++)
			{
				p_imugps_var->p[i][j]=0.0;
			}
		}

		p_imugps_var->p[0][0]=global_cfg->ini_pos_var[0];
		p_imugps_var->p[1][1]=global_cfg->ini_pos_var[1];
		p_imugps_var->p[2][2]=global_cfg->ini_pos_var[2];

		p_imugps_var->p[3][3]=global_cfg->ini_vel_var[0];
		p_imugps_var->p[4][4]=global_cfg->ini_vel_var[1];
		p_imugps_var->p[5][5]=global_cfg->ini_vel_var[2];

		p_imugps_var->p[6][6]=global_cfg->ini_att_var[0];
		p_imugps_var->p[7][7]=global_cfg->ini_att_var[1];
		p_imugps_var->p[8][8]=global_cfg->ini_att_var[2];
	
		p_imugps_var->p[9][9]=  global_cfg->bg_p0;
		p_imugps_var->p[10][10]=global_cfg->bg_p0;
		p_imugps_var->p[11][11]=global_cfg->bg_p0;

		p_imugps_var->p[12][12]=global_cfg->ba_p0;
		p_imugps_var->p[13][13]=global_cfg->ba_p0;
		p_imugps_var->p[14][14]=global_cfg->ba_p0;
		for (i=0;i<3;i++)
		{
			p_imugps_var->dv_n[i]=0.0;
		}
	
	return(ini_finish_flag);
}

uint8_t ins_ekf_inte(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,volatile double ins_data[],volatile double gps_data[],struct position* nav_err, double basic_llh[], double dt)
{



	volatile uint8_t i,j;
	volatile double val1;
	volatile double delta_t1;
	volatile double d_lon,d_lat,d_h;

	volatile double r_gps[3];
	volatile double r_gps_e[3],r_ins_e[3];
	volatile double v1[3][3],v2[3],v4[3],z[6],zv[3],c_en[3][3],c_ec[3][3],c_nc[3][3];

	volatile double r[6][6],v3[6],inno[6];
	volatile double w_ib[3];
	volatile double la_r[3];
	volatile double w_in[3];

	volatile double cp_la_r[3][3],cp_cfg_arm[3][3],cp_win[3][3],cp_arm_x_wib[3][3],cp_win_x_cp_cfg_arm[3][3],cbn_x_cp_cfg_arm[3][3];

  
	delta_t1 = ins_data[0]-gps_data[0];

	if(delta_t1<0 || delta_t1>0.1)
			delta_t1=0.05;
	
	
	rc(BV_A_Earth,BV_E2,gps_data[1],&p_loose_cal_var->rm,&p_loose_cal_var->rn);
	d_lat	=gps_data[4]*delta_t1/(p_loose_cal_var->rm+gps_data[3]);
	d_lon	=gps_data[5]*delta_t1/(p_loose_cal_var->rn+gps_data[3])/cos(gps_data[1]);
	d_h		=-gps_data[6]*delta_t1;
	
	gps_data[1]=gps_data[1]+d_lat;
	gps_data[2]=gps_data[2]+d_lon;
	gps_data[3]=gps_data[3]+d_h;
	
	rela_pos_cal(&basic_llh[0], (double *)&gps_data[1],nav_err);
	
	r_gps[0]=gps_data[1];
	r_gps[1]=gps_data[2];
	r_gps[2]=gps_data[3];
	// calculate the position difference in ECEF frame is better for high latitude area.
	rc(BV_A_Earth,BV_E2,r_gps[0],&p_loose_cal_var->rm,&p_loose_cal_var->rn);
	geo2ecef(BV_E2,p_loose_cal_var->rn,&r_gps[0],&r_gps_e[0]);
	geo2ecef(BV_E2,p_loose_cal_var->rn,&p_loose_cal_var->gps_pos[0],&r_ins_e[0]);
	
	pos2dcm(r_gps[0],r_gps[1],&v1[0][0]);
	matrix_transpose(3,3,&v1[0][0],&c_en[0][0]);

	matrix_multiply(3,3,3,1,&p_loose_cal_var->c_bn[0][0],&loose_cfg_local->arm[0],&la_r[0]);

	v2[0]=r_ins_e[0]-r_gps_e[0];
	v2[1]=r_ins_e[1]-r_gps_e[1];
	v2[2]=r_ins_e[2]-r_gps_e[2];
	
	matrix_multiply(3,3,3,1,&c_en[0][0],&v2[0],&z[0]);
	z[0]=z[0]+la_r[0];//position error
	z[1]=z[1]+la_r[1];
	z[2]=z[2]+la_r[2];		
	

	pos2dcm(p_loose_cal_var->gps_pos[0],p_loose_cal_var->gps_pos[1],&v1[0][0]);
	matrix_transpose(3,3,&v1[0][0],&c_ec[0][0]);
	
	matrix_transpose(3,3,&c_en[0][0],&v1[0][0]);
	matrix_multiply(3,3,3,3,&c_ec[0][0],&v1[0][0],&c_nc[0][0]);

	w_in[0]=p_loose_cal_var->w_ie[0]+p_loose_cal_var->w_en[0];
	w_in[1]=p_loose_cal_var->w_ie[1]+p_loose_cal_var->w_en[1];
	w_in[2]=p_loose_cal_var->w_ie[2]+p_loose_cal_var->w_en[2];
	
	w_ib[0]=ins_data[1]/dt;//*49.951219512195121951219512195122;//
	w_ib[1]=ins_data[2]/dt;//*49.951219512195121951219512195122;//
	w_ib[2]=ins_data[3]/dt;//*49.951219512195121951219512195122;//

	v2[0]=gps_data[4];
	v2[1]=gps_data[5];
	v2[2]=-gps_data[6];
					
	matrix_multiply(3,3,3,1,&c_nc[0][0],&v2[0],&zv[0]);//C_nc*[gps(5); gps(6); -gps(7)];
	zv[0]=-zv[0];
	zv[1]=-zv[1];
	zv[2]=-zv[2];
	
	zv[0]=zv[0]+p_loose_cal_var->gps_vel[0];
	zv[1]=zv[1]+p_loose_cal_var->gps_vel[1];
	zv[2]=zv[2]+p_loose_cal_var->gps_vel[2];

	cross_product(&w_in[0],&la_r[0],&v2[0]);//CrossProduct(w_in,la_r)
	zv[0]=zv[0]-v2[0];
	zv[1]=zv[1]-v2[1];
	zv[2]=zv[2]-v2[2];
	cross_product(&loose_cfg_local->arm[0],&w_ib[0],&v2[0]);//nav.C_bn*CrossProduct(CFG.arm,w_ib)
	matrix_multiply(3,3,3,1,&p_loose_cal_var->c_bn[0][0],&v2[0],&v4[0]);
	zv[0]=zv[0]-v4[0];
	zv[1]=zv[1]-v4[1];
	zv[2]=zv[2]-v4[2];
	
	z[3]=zv[0];//velocity error
	z[4]=zv[1];
	z[5]=zv[2];
	///////////////////////////set H//////////////////////////////////////
	for (i=0;i<6;i++)
	{
		for (j=0;j<BV_NSTATES;j++)
		{
			if (i==j)
			{
				p_loose_cal_var->h[i][j]=1.0;
			}
			else
			{
				p_loose_cal_var->h[i][j]=0.0;
			}
		}
	}
	cp_form(&la_r[0],&cp_la_r[0][0]);
	cp_form(&w_in[0],&cp_win[0][0]);
	cp_form(&loose_cfg_local->arm[0],&cp_cfg_arm[0][0]);
	cp_form(&v2[0],&cp_arm_x_wib[0][0]);
	
	matrix_multiply(3,3,3,3,&cp_win[0][0],&cp_cfg_arm[0][0],&cp_win_x_cp_cfg_arm[0][0]);
	matrix_multiply(3,3,3,3,&p_loose_cal_var->c_bn[0][0],&cp_cfg_arm[0][0],&cbn_x_cp_cfg_arm[0][0]);
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h[i][j+6]=cp_la_r[i][j];
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h[i+3][j+6]=-cp_win_x_cp_cfg_arm[i][j]-cp_arm_x_wib[i][j];	
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h[i+3][j+9]=cbn_x_cp_cfg_arm[i][j];	
	/////////////////////////////////////////////////////////////////
	//set R: control minimum values of the position std and the velocity
	v3[0]=loose_cfg_local->gps_pos_std_scale*gps_data[7];
	v3[1]=loose_cfg_local->gps_pos_std_scale*gps_data[8];
	v3[2]=loose_cfg_local->gps_pos_std_scale*gps_data[9];
	
	v3[3]=loose_cfg_local->gps_pos_std_scale*gps_data[10];
	v3[4]=loose_cfg_local->gps_pos_std_scale*gps_data[11];
	v3[5]=loose_cfg_local->gps_pos_std_scale*gps_data[12];
			
	for (i=0;i<6;i++)
	{
		for (j=0;j<6;j++)
		{
			if (i==j)
			{
				r[i][j]=v3[i]*v3[i];
			} 
			else
			{
				r[i][j]=0.0;
			}
		}
	}
				
	for (i=0;i<6;i++)
	{

		inno[i]=z[i];
	}
					
	kf_update_inte_6_6(p_loose_cal_var,&inno[0],&r[0][0]);
	
	return (1);
	
}


 void kf_update_holo_3_3(struct bv_loose_cal_var* p_loose_cal_var,volatile double inno[],volatile double r[])
{
	volatile double pht[15][3];
	volatile double hpht[3][3];
	volatile double rhpht[3][3];
	volatile double ht[15][3];
	volatile double u[3][3],ut[3][3],uinv[3][3],uinvuinvt[3][3];
	volatile double k[15][3],kt[3][15];
	volatile double dx[15],x_up[15];
	volatile double ikh[15][15],ikht[15][15];
	
	volatile double v1[15][15],v2[15][15],v3[15][15];
	
	volatile uint32_t i,j;
	volatile int32_t Reval;
	
	matrix_transpose(3,15,&p_loose_cal_var->h_holo[0][0],&ht[0][0]);
	matrix_multiply(15,15,15,3,&p_loose_cal_var->p[0][0],&ht[0][0],&pht[0][0]);
	
	matrix_multiply(3,15,15,3,&p_loose_cal_var->h_holo[0][0],&pht[0][0],&hpht[0][0]);
	
	matrix_addition(3,3,&r[0],&hpht[0][0],&rhpht[0][0]);
	//////////////QC//////////////////////////
	for(i=0;i<3;i++)
	{
		if(fabs(inno[i])>15*sqrt(rhpht[i][i]))
			rhpht[i][i]=sqrt(rhpht[i][i])*fabs(inno[i]);
	}
	/////////////////////////////////////////
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			u[i][j]=0.0;
		}
	}
	
	Reval=matrix_chol(&rhpht[0][0],3,3,&ut[0][0],3,3);
	matrix_transpose(3,3,&ut[0][0],&u[0][0]);
	
	
	if (Reval==1)//Positive definete
	{
		matrix_inv(3,&u[0][0],&uinv[0][0]);//U=inv(U)
		matrix_transpose(3,3,&uinv[0][0],&u[0][0]);
		matrix_multiply(3,3,3,3,&uinv[0][0],&u[0][0],&uinvuinvt[0][0]);
		
		matrix_multiply(15,3,3,3,&pht[0][0],&uinvuinvt[0][0],&k[0][0]);
		
		matrix_multiply(15,3,3,1,&k[0][0],&inno[0],&dx[0]);//dx = K * inno;
		
		matrix_addition(15,1,&p_loose_cal_var->x[0],&dx[0],&x_up[0]);
		
		matrix_multiply(15,3,3,15,&k[0][0],&p_loose_cal_var->h_holo[0][0],&ikh[0][0]);
		for (i=0;i<15;i++)
		{
			for (j=0;j<15;j++)
			{
				if (i==j)
				{
					ikh[i][j]=1.0-ikh[i][j];
				}
				else
				{
					ikh[i][j]=-ikh[i][j];
				}
			}
		}
		
		matrix_transpose(15,15,&ikh[0][0],&ikht[0][0]);
		matrix_transpose(15,3,&k[0][0],&kt[0][0]);
		
		matrix_multiply(15,15,15,15,&ikh[0][0],&p_loose_cal_var->p[0][0],&v1[0][0]);
		matrix_multiply(15,15,15,15,&v1[0][0],&ikht[0][0],&v2[0][0]);
		
		matrix_multiply(15,3,3,3,&k[0][0],&r[0],&v1[0][0]);
		matrix_multiply(15,3,3,15,&v1[0][0],&kt[0][0],&v3[0][0]);
		
		matrix_addition(15,15,&v2[0][0],&v3[0][0],&p_loose_cal_var->p[0][0]);
		
		for (i=0;i<15;i++)
		{
			p_loose_cal_var->x[i]=x_up[i];
		}	
	} 
}

uint8_t ins_ekf_holo_3(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,volatile double ins_data[],double dt, int16_t rpm_left, int16_t rpm_right)
{
	volatile uint8_t i,j;
	volatile double w_ib[3],w_in[3],c_nb_x_w_in[3],w_nb_b[3];
	volatile double r[3][3];
	volatile double c_nv[3][3],c_nb[3][3];
	volatile double cp_v[3][3],c_nv_x_cp_v[3][3],cp_la[3][3],c_bv_x_cp_la[3][3];
	volatile double v_v[3],v_v1[3],v_v2[3],v_v3[3],v_v4[3];
	volatile double inno[3];
	
	matrix_transpose(3,3,&p_loose_cal_var->c_bn[0][0],&c_nb[0][0]);
	matrix_multiply(3,3,3,3,&loose_cfg_local->c_bv[0][0],&c_nb[0][0],&c_nv[0][0]);
	
	w_ib[0]=ins_data[1]/dt;//*49.951219512195121951219512195122;
	w_ib[1]=ins_data[2]/dt;//*49.951219512195121951219512195122;
	w_ib[2]=ins_data[3]/dt;//*49.951219512195121951219512195122;
	
	w_in[0]=p_loose_cal_var->w_ie[0]+p_loose_cal_var->w_en[0];
	w_in[1]=p_loose_cal_var->w_ie[1]+p_loose_cal_var->w_en[1];
	w_in[2]=p_loose_cal_var->w_ie[2]+p_loose_cal_var->w_en[2];
	matrix_multiply(3,3,3,1,&c_nb[0][0],&w_in[0],&c_nb_x_w_in[0]);
	w_nb_b[0]=w_ib[0]-c_nb_x_w_in[0];
	w_nb_b[1]=w_ib[1]-c_nb_x_w_in[1];
	w_nb_b[2]=w_ib[2]-c_nb_x_w_in[2];
	
	cross_product(&w_nb_b[0],&loose_cfg_local->la_odom[0],&v_v1[0]);
	matrix_multiply(3,3,3,1,&loose_cfg_local->c_bv[0][0],&v_v1[0],&v_v2[0]);
	matrix_multiply(3,3,3,1,&c_nv[0][0],&p_loose_cal_var->gps_vel[0],&v_v3[0]);
	matrix_addition(3,1,&v_v2[0],&v_v3[0],&v_v[0]);
	
	cp_form(&p_loose_cal_var->gps_vel[0],&cp_v[0][0]);
	matrix_multiply(3,3,3,3,&c_nv[0][0],&cp_v[0][0],&c_nv_x_cp_v[0][0]);
	cp_form(&loose_cfg_local->la_odom[0],&cp_la[0][0]);
	matrix_multiply(3,3,3,3,&loose_cfg_local->c_bv[0][0],&cp_la[0][0],&c_bv_x_cp_la[0][0]);
	for(i=0;i<3;i++)
		for(j=0;j<15;j++)
			p_loose_cal_var->h_holo[i][j]=0;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h_holo[i][j+3]=c_nv[i][j];
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h_holo[i][j+6]=-c_nv_x_cp_v[i][j];
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			p_loose_cal_var->h_holo[i][j+9]=-c_bv_x_cp_la[i][j];

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			if(i==j)
				r[i][j]=loose_cfg_local->var_holo[i];
			else
				r[i][j]=0;
		}
	}
	
	float veh_velocity = ((float)(rpm_left+rpm_right))*0.5f*M_PI*DIA_wheel/reduc_ratio;
	matrix_multiply(3,15,15,1,&p_loose_cal_var->h_holo[0][0],&p_loose_cal_var->x[0],&v_v4[0]);
	inno[0]=v_v[0]-veh_velocity;
	inno[1]=v_v[1]-v_v4[1];
	inno[2]=v_v[2]-v_v4[2];				
	kf_update_holo_3_3(p_loose_cal_var,&inno[0],&r[0][0]);
	return 1;


}
void update_filter_solution(struct bv_loose_cfg* loose_cfg_local,struct bv_loose_cal_var* p_loose_cal_var,double imudata[], double dt,double nav_updata)
{
	volatile uint8_t i,j;
	volatile double d_theta_out[3],v1[3][3],v2[3],rn[3];
	volatile double qn_out[4];
	volatile double q_ne_out[4],q_e[4];
	volatile double c_cn_out[3][3];
	volatile double v_n_out[3];
	volatile double phi_ang_out[3];
	volatile double d_lat_out=0.0,d_lon_out=0.0;
	volatile double cp_phi_ang[3][3],c_pn[3][3],c_bn[3][3];
	volatile double cbn_x_laodom[3];
	volatile double w_ib[3],w_in[3],c_nb_x_w_in[3],w_nb_b[3];
	volatile double c_nb[3][3];
	volatile double v_v1[3],v_v2[3];
	volatile double c_vb[3][3],c_bn_x_c_vb[3][3];
	volatile double c_bv_x_c_nb[3][3],c_vn[3][3],v_vn[3];
	
	d_lat_out=p_loose_cal_var->x[0]/(p_loose_cal_var->rm+p_loose_cal_var->gps_pos[2]);
	d_lon_out=p_loose_cal_var->x[1]/(p_loose_cal_var->rn+p_loose_cal_var->gps_pos[2])/(cos(p_loose_cal_var->gps_pos[0]));
	dpos2rvec(p_loose_cal_var->gps_pos[0],d_lat_out,d_lon_out,&d_theta_out[0]);
	
	v2[0]=-d_theta_out[0];
	v2[1]=-d_theta_out[1];
	v2[2]=-d_theta_out[2];
		
	rvec2quat(&v2[0],&qn_out[0]);
	quat_prod(&p_loose_cal_var->q_ne[0],&qn_out[0],&q_ne_out[0]);//q_ne=q_ne X q_n
	//update filter position
	quat2pos(&q_ne_out[0],&rn[0],&rn[1]);
	rn[2]=p_loose_cal_var->gps_pos[2]+p_loose_cal_var->x[2];
	
	if(nav_updata>0)
	{
		p_loose_cal_var->gps_pos[0]=rn[0];
		p_loose_cal_var->gps_pos[1]=rn[1];
		p_loose_cal_var->gps_pos[2]=rn[2];
		p_loose_cal_var->q_ne[0]=q_ne_out[0];
		p_loose_cal_var->q_ne[1]=q_ne_out[1];
		p_loose_cal_var->q_ne[2]=q_ne_out[2];
		p_loose_cal_var->q_ne[3]=q_ne_out[3];
	}

	cp_form(&d_theta_out[0],&v1[0][0]);
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				c_cn_out[i][j]=1.0+v1[i][j];
			}
			else
			{
				c_cn_out[i][j]=v1[i][j];
			}
		}
	}
		
	v2[0]=p_loose_cal_var->gps_vel[0]-p_loose_cal_var->x[3];
	v2[1]=p_loose_cal_var->gps_vel[1]-p_loose_cal_var->x[4];
	v2[2]=p_loose_cal_var->gps_vel[2]-p_loose_cal_var->x[5];
	
	matrix_multiply(3,3,3,1,&c_cn_out[0][0],&v2[0],&v_n_out[0]);
	
	if(nav_updata>0)
	{
		p_loose_cal_var->gps_vel[0]=v_n_out[0];
		p_loose_cal_var->gps_vel[1]=v_n_out[1];
		p_loose_cal_var->gps_vel[2]=v_n_out[2];
	}
	//attitude correction
	phi_ang_out[0]=p_loose_cal_var->x[6]+d_theta_out[0];
	phi_ang_out[1]=p_loose_cal_var->x[7]+d_theta_out[1];
	phi_ang_out[2]=p_loose_cal_var->x[8]+d_theta_out[2];

	cp_form(&phi_ang_out[0],&cp_phi_ang[0][0]);
	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				c_pn[i][j]=1.0+cp_phi_ang[i][j];
			}
			else
			{
				c_pn[i][j]=cp_phi_ang[i][j];
			}
		}
	}

	matrix_multiply(3,3,3,3,&c_pn[0][0],&p_loose_cal_var->c_bn[0][0],&c_bn[0][0]);


	dcm2euler(&c_bn[0][0],&p_loose_cal_var->att[0],&p_loose_cal_var->att[1],&p_loose_cal_var->att[2]);

	if(nav_updata>0)
	{
		
		rvec2quat(&phi_ang_out[0],&q_e[0]);//q_e
		quat_prod(&q_e[0],&p_loose_cal_var->q_bn[0],&qn_out[0]);//q_bn=q_e X q_bn
		p_loose_cal_var->q_bn[0]=qn_out[0];
		p_loose_cal_var->q_bn[1]=qn_out[1];
		p_loose_cal_var->q_bn[2]=qn_out[2];
		p_loose_cal_var->q_bn[3]=qn_out[3];
		
		quat2dcm(&p_loose_cal_var->q_bn[0],&p_loose_cal_var->c_bn[0][0]);//C_bn

		for (i=0;i<9;i++)
		{
			p_loose_cal_var->x[i]=0.0;//status reset after filter
		}
				
		
		p_loose_cal_var->bg[0]=p_loose_cal_var->bg[0]+p_loose_cal_var->x[9];
		p_loose_cal_var->bg[1]=p_loose_cal_var->bg[1]+p_loose_cal_var->x[10];
		p_loose_cal_var->bg[2]=p_loose_cal_var->bg[2]+p_loose_cal_var->x[11];
	
		p_loose_cal_var->ba[0]=p_loose_cal_var->ba[0]+p_loose_cal_var->x[12];
		p_loose_cal_var->ba[1]=p_loose_cal_var->ba[1]+p_loose_cal_var->x[13];
		p_loose_cal_var->ba[2]=p_loose_cal_var->ba[2]+p_loose_cal_var->x[14];
			
		for (i=0;i<6;i++)
		{
			p_loose_cal_var->x[i+9]=0.0;
		}
		nav_updata=0;
	}
	

	#if 1
	rc(BV_A_Earth,BV_E2,p_loose_cal_var->gps_pos[0],&p_loose_cal_var->rm,&p_loose_cal_var->rn);
	matrix_transpose(3,3,&loose_cfg_local->c_bv[0][0],&c_vb[0][0]);
	matrix_multiply(3,3,3,3,&c_bn[0][0],&c_vb[0][0],&c_bn_x_c_vb[0][0]);
	
	matrix_multiply(3,3,3,1,&c_bn_x_c_vb[0][0],&loose_cfg_local->la_odom[0],&cbn_x_laodom[0]);

	p_loose_cal_var->imu2gnd_pos[0]=rn[0]+cbn_x_laodom[0]/(p_loose_cal_var->rm+p_loose_cal_var->gps_pos[2]);
	p_loose_cal_var->imu2gnd_pos[1]=rn[1]+cbn_x_laodom[1]/(p_loose_cal_var->rn+p_loose_cal_var->gps_pos[2])/cos(p_loose_cal_var->gps_pos[0]);
	p_loose_cal_var->imu2gnd_pos[2]=rn[2]-cbn_x_laodom[2];
	

	matrix_transpose(3,3,&p_loose_cal_var->c_bn[0][0],&c_nb[0][0]);
	w_ib[0]=imudata[1]/dt;
	w_ib[1]=imudata[2]/dt;
	w_ib[2]=imudata[3]/dt;
	
	w_in[0]=p_loose_cal_var->w_ie[0]+p_loose_cal_var->w_en[0];
	w_in[1]=p_loose_cal_var->w_ie[1]+p_loose_cal_var->w_en[1];
	w_in[2]=p_loose_cal_var->w_ie[2]+p_loose_cal_var->w_en[2];
	matrix_multiply(3,3,3,1,&c_nb[0][0],&w_in[0],&c_nb_x_w_in[0]);
	w_nb_b[0]=w_ib[0]-c_nb_x_w_in[0];
	w_nb_b[1]=w_ib[1]-c_nb_x_w_in[1];
	w_nb_b[2]=w_ib[2]-c_nb_x_w_in[2];

	cross_product(&w_nb_b[0],&loose_cfg_local->la_odom[0],&v_v1[0]);
	matrix_multiply(3,3,3,1,&c_bn[0][0],&v_v1[0],&v_v2[0]);
	v_vn[0]=v_n_out[0]+v_v2[0];
	v_vn[1]=v_n_out[1]+v_v2[1];
	v_vn[2]=v_n_out[2]+v_v2[2];

	matrix_transpose(3,3,&c_bn[0][0],&c_nb[0][0]);
	matrix_multiply(3,3,3,3,&loose_cfg_local->c_bv[0][0],&c_nb[0][0],&c_bv_x_c_nb[0][0]);
	matrix_multiply(3,3,3,1,&c_bv_x_c_nb[0][0],&v_vn[0],&p_loose_cal_var->imu2gnd_vel[0]);

	matrix_multiply(3,3,3,3,&c_bn[0][0],&c_vb[0][0],&c_vn[0][0]);
	dcm2euler(&c_vn[0][0],&p_loose_cal_var->imu2gnd_att[0],&p_loose_cal_var->imu2gnd_att[1],&p_loose_cal_var->imu2gnd_att[2]);
	#endif


	#if 0
	matrix_multiply(3,3,3,1,&c_bn_x_c_vb[0][0],&loose_cfg_local->la_output[0],&cbn_x_laodom[0]);
	p_loose_cal_var->imu2gnss_pos[0]=rn[0]+cbn_x_laodom[0]/(p_loose_cal_var->rm+p_loose_cal_var->gps_pos[2]);
	p_loose_cal_var->imu2gnss_pos[1]=rn[1]+cbn_x_laodom[1]/(p_loose_cal_var->rn+p_loose_cal_var->gps_pos[2])/cos(p_loose_cal_var->gps_pos[0]);
	p_loose_cal_var->imu2gnss_pos[2]=rn[2]-cbn_x_laodom[2];
	
	//w_nb_b=w_ib-C_nb*(w_ie+w_en) 
	//v_n =v_n+C_bn*CrossProduct(w_nb_b,CFG.la_output);
	cross_product(&w_nb_b[0],&loose_cfg_local->la_output[0],&v_v1[0]);
	matrix_multiply(3,3,3,1,&c_bn[0][0],&v_v1[0],&v_v2[0]);
	p_loose_cal_var->imu2gnss_vel[0]=v_n_out[0]+v_v2[0];
	p_loose_cal_var->imu2gnss_vel[1]=v_n_out[1]+v_v2[1];
	p_loose_cal_var->imu2gnss_vel[2]=v_n_out[2]+v_v2[2];
	#endif
}




