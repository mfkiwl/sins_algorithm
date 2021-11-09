
/*
     tony.jiang 2021 10 26
*/
#include <stdio.h>
#include "functions_sins.h"
#include "bv_sins.h"
#include "typedef.h"
#include "mathmisc.h"

void euler2dcm(volatile double roll,volatile double pitch,volatile double heading,volatile double dcm[])
{
	volatile double cr;
	volatile double cp;
	volatile double ch;

	volatile double sr;
	volatile double sp;
	volatile double sh;


	cr = cos(roll); cp = cos(pitch); ch = cos(heading);
	sr = sin(roll); sp = sin(pitch); sh = sin(heading);

	*(dcm+0*3+0)=cp*ch;
	*(dcm+0*3+1)=-cr*sh+sr*sp*ch;
	*(dcm+0*3+2)=sr*sh+cr*sp*ch;

	*(dcm+1*3+0)=cp*sh;
	*(dcm+1*3+1)=cr*ch+sr*sp*sh;
	*(dcm+1*3+2)=-sr*ch+cr*sp*sh;

	*(dcm+2*3+0)=-sp;
	*(dcm+2*3+1)=sr*cp;
	*(dcm+2*3+2)=cr*cp;
}


int8_t norm(  int8_t m,  volatile double A[],volatile double* Val)
{
  volatile int8_t RetVal;
	volatile double Sum;
	volatile int8_t i;

	Sum = 0.0;
	for( i=0; i<m; i++ )
	{
		Sum = Sum + *(A+i) * *(A+i);
	}
	if( Sum < 0.0 )
	{
		return(0);
	}
	else
	{
		*Val = sqrt( Sum );
		RetVal = 1;
	}	
    return (RetVal);
}


void matrix_multiply(uint8_t m1, uint8_t n1, uint8_t m2, uint8_t n2, 
                    volatile double M1[], volatile double M2[],volatile double M3[])
{    
    volatile uint8_t i, j, k;
    volatile double Sum;
	
	for(i=0;i<m1;i++) 
	{
		for(j=0;j<n2;j++)
		{
			Sum = 0.0;
			
			for ( k=0; k<n1; k++)   
			{
				Sum = Sum + *(M1+i*n1+k) * *(M2+k*n2+j);
			}
			
			*(M3+i*n2+j) = Sum; //<<保存到m3上
		}
	}
}

void matrix_transpose(uint8_t m, uint8_t n,  volatile double M1[], volatile double MT[])
{
	uint8_t i, j;

	for( i=0; i<m; i++ )
	{
		for( j=0; j<n; j++ )
		{
			*(MT+j*m+i) = *(M1+i*n+j);
		}
	}
}


void matrix_addition(uint8_t m, uint8_t n,  volatile double M1[],  volatile double M2[], volatile double M3[])
{
  volatile uint8_t i, j;
	for(i=0;i<m;i++)
	{
		for(j=0;j<n;j++)
		{
			*(M3+i*n+j) = *(M1+i*n+j) + *(M2+i*n+j);
		}
	}
}


int8_t matrix_inv(int8_t n,  volatile double a[], volatile double b[])
{
    volatile int8_t i,j,k,l,u,v,is[10],js[10];   /* matrix dimension <= 5 */
    volatile double  d, p;

    for(i=0;i<n;i++)
	{
        for(j=0;j<n;j++)
		{
            b[i*n+j]=a[i*n+j];
		}
	}

	for(k=0;k<n;k++)
	{
		d=0.0;
		for(i=k;i<n;i++)   
		{
			for(j=k;j<n;j++)
			{
				l=n*i+j;
				p = fabs(b[l]);
				if(p>d)
				{
					d=p;
					is[k]=i;
					js[k]=j;
				}
			}
		}

		if(d<0.0)  
		{
			return (0);
		}

		if( is[k]!=k )  
		{
			for(j=0;j<n;j++)
			{
				u=k*n+j;
				v=is[k]*n+j;
				p=b[u];
				b[u]=b[v];
				b[v]=p;
			}
		}

		if( js[k]!=k )  
		{
			for( i=0; i<n; i++ )
			{
				u=i*n+k;
				v=i*n+js[k];
				p=b[u];
				b[u]=b[v];
				b[v]=p;
			}
		}

		l=k*n+k;
		b[l]=1.0/b[l];  
		for( j=0; j<n; j++ )
		{
			if( j!=k )
			{
				u=k*n+j;
				b[u]=b[u]*b[l];
			}
		}
		for(i=0;i<n; i++)
		{
			if(i!=k)
			{
				for(j=0; j<n; j++ )
				{
					if( j!=k )
					{
						u=i*n+j;
						b[u]=b[u]-b[i*n+k]*b[k*n+j];
					}
				}
			}
		}
		for(i=0;i<n;i++)
		{
			if(i!=k)
			{
				u=i*n+k;
				b[u]=-b[u]*b[l];
			}
		}
	}

	for(k=n-1;k>=0;k--)  
	{
		if(js[k]!=k)
		{
			for(j=0;j<n;j++)
			{
				u=k*n+j;
				v=js[k]*n+j;
				p=b[u];
				b[u]=b[v];
				b[v]=p;
			}
		}
		if(is[k]!=k)
		{
			for(i=0;i<n;i++)
			{
				u=i*n+k;
				v=is[k]+i*n;
				p=b[u];
				b[u]=b[v];
				b[v]=p;
			}
		}
	}
 
	return (1);
}


//compensate errors for biases and scale factors
//function Corr = compensate(obs, bias, scale)


void compensate(volatile double obs[],volatile double bias[],volatile double scale[],volatile double Corr[])
{
	volatile double temp[3];
	volatile uint8_t i;

	for (i=0;i<3;i++)
	{
		temp[i]=obs[i]-bias[i];
	}

	Corr[0]=1/(1+scale[0])*temp[0];
	Corr[1]=1/(1+scale[1])*temp[1];
	Corr[2]=1/(1+scale[2])*temp[2];

}


//[par.Rm, par.Rn] = rc(WGS84.a, WGS84.e2, nav.r(1));
//Function to compute the radius of curvature in the meridian and the prime

void rc(double a,double e2,double lat,double* M,double *N)
{
	*M=a*(1-e2)/pow((1-e2*sin(lat)*sin(lat)),1.5);
	*N=a/sqrt(1-e2*sin(lat)*sin(lat));
	
}

//Convert Euler angles to quaternions

void euler2quat(double roll,double pitch,double heading,volatile double q[])
{
	volatile double c_r,c_p,c_h,s_r,s_p,s_h;

	c_r = cos(roll/2); 		s_r = sin(roll/2);
	c_p = cos(pitch/2); 		s_p = sin(pitch/2);
	c_h = cos(heading/2); 	s_h = sin(heading/2);

	q[0]= c_r * c_p * c_h + s_r * s_p * s_h;
	q[1] = s_r * c_p * c_h - c_r * s_p * s_h;
	q[2] = c_r * s_p * c_h + s_r * c_p * s_h;
	q[3] = c_r * c_p * s_h - s_r * s_p * c_h;
}

//Compute the quaternion (q_ne) representing the attitude of the navigation frame

void pos2quat(double lat,double lon,volatile double q_ne[])
{
	volatile double s1,c1,s2,c2;

	s1 = sin(lon/2);
	c1 = cos(lon/2);
	s2 = sin(-M_PI/4-lat/2);
	c2 = cos(-M_PI/4-lat/2);

	q_ne[0]=c1*c2;
	q_ne[1]=-s1*s2;
	q_ne[2]=c1*s2;
	q_ne[3]=c2*s1;

}

//Cross product of two vectors (c = a x b)
void cross_product(volatile double a[],volatile double b[],volatile double c[])
{
	c[0]=a[1]*b[2]-b[1]*a[2];
	c[1]=b[0]*a[2]-a[0]*b[2];
	c[2]=a[0]*b[1]-b[0]*a[1];
}

//Position error to rotation vector conversion
//function rv = dpos2rvec(lat, delta_lat, delta_lon)
void dpos2rvec(double lat,double delta_lat,double delta_lon,volatile double rv[])
{
	rv[0]=delta_lon*cos(lat);
	rv[1]=-delta_lat;
	rv[2]=-delta_lon*sin(lat);
}

//Rotation vector to quaternions conversion
//function q = rvec2quat(rot_vec)
void rvec2quat(volatile double rot_vec[],volatile double q[])
{
	volatile double mag2,mag,s_mag;
	volatile double c,s;

	mag2=rot_vec[0]*rot_vec[0]+rot_vec[1]*rot_vec[1]+rot_vec[2]*rot_vec[2];
	if (mag2<(M_PI*M_PI))
	{
		mag2=0.25*mag2;
		c=1.0-mag2/2.0*(1.0-mag2/12.0*(1.0-mag2/30.0));
		s=1.0-mag2/6.0*(1.0-mag2/20.0*(1.0-mag2/42.0));
		q[0]=c;
		q[1]=s*0.5*rot_vec[0];
		q[2]=s*0.5*rot_vec[1];
		q[3]=s*0.5*rot_vec[2];
	} 
	else
	{
		mag=sqrt(mag2);
		s_mag=sin(mag/2);
		
		q[0]=cos(mag/2);
		q[1]=rot_vec[0]*s_mag/mag;
		q[2]=rot_vec[1]*s_mag/mag;
		q[3]=rot_vec[2]*s_mag/mag;

		if(q[0]<0.0)
		{
			q[0]=-q[0];
			q[1]=-q[1];
			q[2]=-q[2];
			q[3]=-q[3];
		}
	}
}


//Product of Quaternions: q1 = q * p
//function q1 = quat_prod(q, p)
void quat_prod(volatile double q[],volatile double p[],volatile double q1[])
{
	volatile double qs,qv[3],ps,pv[3];
	volatile double v1[3];
	
	qs=q[0];
	qv[0]=q[1];
	qv[1]=q[2];
	qv[2]=q[3];

	ps=p[0];
	pv[0]=p[1];
	pv[1]=p[2];
	pv[2]=p[3];

	cross_product(&qv[0],&pv[0],&v1[0]);
	
	q1[0]=qs*ps-qv[0]*pv[0]-qv[1]*pv[1]-qv[2]*pv[2];
	q1[1]=qs*pv[0]+ps*qv[0]+v1[0];
	q1[2]=qs*pv[1]+ps*qv[1]+v1[1];
	q1[3]=qs*pv[2]+ps*qv[2]+v1[2];

	if (q1[0]<0.0)
	{
		q1[0]=-q1[0];
		q1[1]=-q1[1];
		q1[2]=-q1[2];
		q1[3]=-q1[3];
	}
}

//Compute latitude and longitude from the quaternion (q_ne)

void quat2pos(volatile double q_ne[],volatile double* plat,volatile double* plon)
{
	*plat=-2*atan(q_ne[2]/q_ne[0])-M_PI/2;
	*plon=2*atan2(q_ne[3], q_ne[0]);
}

//Computes Normal Gravity
static void normal_gravity(double latitude,double he,double* p_ng)
{
	volatile double a1,a2,a3,a4,a5,a6,s2,s4;

	a1 = 9.7803267715;
	a2 = 0.0052790414;
	a3 = 0.0000232718;
	a4 = -0.000003087691089;
	a5 = 0.000000004397731;
	a6 = 0.000000000000721;
	s2 = sin(latitude)*sin(latitude);
	s4 = s2 * s2;
	*p_ng = a1 * (1 + a2*s2 + a3*s4) + (a4 + a5*s2)*he + a6 * he * he;

}

//Calculates transport rate of navigation frame
static void trans_rate(volatile double r_n[],volatile double v_n[],double M,double N,volatile double w_en[])
{
	w_en[0]=v_n[1]/(N+r_n[2]);
	w_en[1]=-v_n[0]/(M+r_n[2]);
	w_en[2]=-v_n[1]*tan(r_n[0])/(N+r_n[2]);
}

//================================================
// cross product form of a 3-dimensional vector
//================================================
void cp_form(volatile double v[],volatile double VV[])
{
	*(VV+0*3+0)=0.0;
	*(VV+0*3+1)=-v[2];
	*(VV+0*3+2)=v[1];
	
	*(VV+1*3+0)=v[2];
	*(VV+1*3+1)=0.0;
	*(VV+1*3+2)=-v[0];
	
	*(VV+2*3+0)=-v[1];
	*(VV+2*3+1)=v[0];
	*(VV+2*3+2)=0.0;
}

//function to normalize a quaternion vector
static void norm_quat(volatile double q[],volatile double q_n[])
{
	volatile double tmp;

	tmp=(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]-1)*0.5;///2;
	tmp=1-tmp;
	
	q_n[0]=tmp*q[0];
	q_n[1]=tmp*q[1];
	q_n[2]=tmp*q[2];
	q_n[3]=tmp*q[3];
}

//Measure angular difference from ang1 to ang2
static double dist_ang(double ang1,double ang2)
{
	volatile double ang;

	ang=ang2-ang1;
	if (ang>M_PI)
	{
		ang=ang-2*M_PI;
	}
	else
	{
		if (ang<-M_PI)
		{
			ang=ang+2*M_PI;
		}
	}

	return (ang);
}

//Quaternion to Direction Cosine Matrix

void quat2dcm(volatile double q[],volatile double VV[])
{
	*(VV+0*3+0)=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
	*(VV+0*3+1)=2*(q[1]*q[2]-q[0]*q[3]);
	*(VV+0*3+2)=2*(q[1]*q[3]+q[0]*q[2]);
	
	*(VV+1*3+0)=2*(q[1]*q[2]+q[0]*q[3]);
	*(VV+1*3+1)=q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3];
	*(VV+1*3+2)=2*(q[2]*q[3]-q[0]*q[1]);
	
	*(VV+2*3+0)=2*(q[1]*q[3]-q[0]*q[2]) ;
	*(VV+2*3+1)=2*(q[2]*q[3]+q[0]*q[1]);
	*(VV+2*3+2)=q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
}

//Convert the geodetic coordinates into the e-frame coordiates
void geo2ecef(double e2,double Rn,volatile double geo[],volatile double r_e[])
{
	volatile double c_lat,s_lat,c_lon,s_lon;
	volatile double Rn_h;

	c_lat=cos(geo[0]);
	s_lat=sin(geo[0]);
	c_lon=cos(geo[1]);
	s_lon=sin(geo[1]);
	
	Rn_h=Rn+geo[2];
	
	r_e[0]=Rn_h*c_lat*c_lon;
	r_e[1]=Rn_h*c_lat*s_lon;
	r_e[2]=(Rn*(1-e2)+geo[2])*s_lat;
}

//Compute the DCM (C_ne) representing the attitude of the navigation frame from the latitude and longitude.
//function C_ne = pos2dcm(lat, lon)
void pos2dcm(double lat,double lon,volatile double C_ne[])
{
	volatile double c_lat,s_lat,c_lon,s_lon;

	s_lat = sin(lat);
	c_lat = cos(lat);
	s_lon = sin(lon);
	c_lon = cos(lon);

	*(C_ne+0*3+0)=-s_lat*c_lon;
	*(C_ne+0*3+1)=-s_lon;
	*(C_ne+0*3+2)=-c_lat*c_lon;
	
	*(C_ne+1*3+0)=-s_lat*s_lon;
	*(C_ne+1*3+1)=c_lon;
	*(C_ne+1*3+2)=-c_lat*s_lon;
	
	*(C_ne+2*3+0)=c_lat;
	*(C_ne+2*3+1)=0.0;
	*(C_ne+2*3+2)=-s_lat;
}

//
int8_t matrix_chol(volatile double input1_r[],int32_t input1_M,int32_t input1_N,volatile double output1_r[],int32_t output1_M,int32_t output1_N)
{
	volatile int8_t loop_i,loop_j,loop_k,loop_u,loop_l,loop_p,loop_q,matrix_n;
	
	matrix_n=input1_M;
	for (loop_p=0;loop_p<input1_M;loop_p++)
	{
		for(loop_q=0;loop_q<input1_N;loop_q++)
		{
			output1_r[loop_q+loop_p*input1_M]=input1_r[loop_q+loop_p*input1_M];
		}
	}
	
	if (input1_M!=input1_N)
	{
		return (-1);
	}
	if ((output1_r[0]+1.0==1.0)||(output1_r[0]<0.0))
	{
		return(-2);
	}
	
	output1_r[0]=sqrt(output1_r[0]);
	for (loop_i=1; loop_i<=matrix_n-1; loop_i++)
	{
		loop_u=loop_i*matrix_n; 
		output1_r[loop_u]=output1_r[loop_u]/output1_r[0];
	}
	for (loop_j=1; loop_j<=matrix_n-1; loop_j++)
	{
		loop_l=loop_j*matrix_n+loop_j;
		for (loop_k=0; loop_k<=loop_j-1; loop_k++)
		{ 
			loop_u=loop_j*matrix_n+loop_k; 
			output1_r[loop_l]=output1_r[loop_l]-output1_r[loop_u]*output1_r[loop_u];
		}
		if ((output1_r[loop_l]+1.0==1.0)||(output1_r[loop_l]<0.0))
		{
			return(-3);
		}
		output1_r[loop_l]=sqrt(output1_r[loop_l]);
		
		for (loop_i=loop_j+1; loop_i<=matrix_n-1; loop_i++)
		{
			loop_u=loop_i*matrix_n+loop_j;
			for (loop_k=0; loop_k<=loop_j-1; loop_k++)
			{
				output1_r[loop_u]=output1_r[loop_u]-output1_r[loop_i*matrix_n+loop_k]*output1_r[loop_j*matrix_n+loop_k];
			}
			output1_r[loop_u]=output1_r[loop_u]/output1_r[loop_l];
		}
	}
	for (loop_i=0; loop_i<=matrix_n-2; loop_i++)
	{
		for (loop_j=loop_i+1; loop_j<=matrix_n-1; loop_j++)
		{
			output1_r[loop_i*matrix_n+loop_j]=0.0;
		}
	}

	return(1);
}

//Convert a Direction Cosine Matrix to Euler angles - NED system
void dcm2euler(volatile double dc[],double* p_roll,double* p_pitch,double* p_heading)
{
	volatile double tmp[3][3];
	volatile uint32_t i,j;

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			tmp[i][j]=*(dc+i*3+j);
		}
	}


	*p_pitch=atan(-tmp[2][0]/sqrt(tmp[2][1]*tmp[2][1]+tmp[2][2]*tmp[2][2]));
	if ((tmp[2][0])<=(-0.99999))
	{
		*p_roll=-999.99;
		*p_heading=atan2((tmp[1][2]-tmp[0][1]),(tmp[0][2]+tmp[1][1]));
//		head_err++;
	}
	else if((tmp[2][0])>=0.99999)
	{
		*p_roll=-999.99;
		*p_heading=M_PI+atan2((tmp[1][2]+tmp[0][1]),(tmp[0][2]-tmp[1][1]));
//		head_err++;
	}
	else
	{
		*p_roll=atan2(tmp[2][1],tmp[2][2]);
		*p_heading=atan2l(tmp[1][0],tmp[0][0]);
	}
	
//	para_tmp_placed.C00 = (int8_t)(tmp[0][0]*100);

}

//INS Mechanization 
void ins_mech(volatile double meas_pre[],volatile double meas[],struct bv_loose_cal_var* p_imugps_globalvar,double dt)
{
	volatile uint8_t i,j;

	//volatile double dt;
	volatile double scul[3];
	volatile double v1[3],v2[3],v3[3],v4[4],v5[3][3];
	volatile double mid_r[3],mid_v[3],mid_q[4];
	volatile double d_lat,d_lon,d_theta[3],dv_f_n[3],dv_g_cor[3];
	volatile double zeta[3],halfzeta[3];
	volatile double Cn[3][3];
	volatile double dv_n[3],v[3],r[3];
	volatile double qn[4],xi[3],qe[4];
	volatile double q_ne[4];
	volatile double beta[3];
	volatile double qb[4];
	
	rc(BV_A_Earth,BV_E2,p_imugps_globalvar->gps_pos[0],&p_imugps_globalvar->rm,&p_imugps_globalvar->rn);
	
	printf("\np_imugps_globalvar->gps_pos[0]=%f\n",p_imugps_globalvar->gps_pos[0]);
//	printf("\np_imugps_globalvar->rm=%f\n",p_imugps_globalvar->rm);
//	printf("\np_imugps_globalvar->rn=%f\n",p_imugps_globalvar->rn);
	//Computes time increment
	
	cross_product(&meas_pre[1],&meas[1],&beta[0]);
	beta[0]=beta[0]*0.08333333333333333333333333333333;///12.0f;
	beta[1]=beta[1]*0.08333333333333333333333333333333;///12.0f;
	beta[2]=beta[2]*0.08333333333333333333333333333333;///12.0f;

	//Sculling Motion 
	cross_product(&meas[1],&meas[4],&v1[0]);
	cross_product(&meas_pre[1],&meas[4],&v2[0]);
	cross_product(&meas_pre[4],&meas[1],&v3[0]);

	for (i=0;i<3;i++)
	{
		scul[i]=0.5*v1[i]+(v2[i]+v3[i])*0.08333333333333333333333333333333;///12.0f;
	}

	//Velocity integration
	//extrapolate position  check whether it is necessary later.
	for (i=0;i<3;i++)
	{
		mid_r[i]=0.0;
	}
//	memset((void*)&mid_r[0],0x00,sizeof(mid_r));
	mid_r[0]=p_imugps_globalvar->gps_pos[0];
	mid_r[2]=p_imugps_globalvar->gps_pos[2]-0.5*p_imugps_globalvar->gps_vel[2]*dt;//Height
	
	d_lat=0.5*p_imugps_globalvar->gps_vel[0]*dt/(p_imugps_globalvar->rm+mid_r[2]);
	d_lon=0.5*p_imugps_globalvar->gps_vel[1]*dt/(p_imugps_globalvar->rn+mid_r[2])/cos(p_imugps_globalvar->gps_pos[0]);
	dpos2rvec(p_imugps_globalvar->gps_pos[0],d_lat,d_lon,&d_theta[0]);

	rvec2quat(&d_theta[0],&v4[0]);
	quat_prod(&p_imugps_globalvar->q_ne[0],&v4[0],&mid_q[0]);

	quat2pos(&mid_q[0],&mid_r[0],&mid_r[1]);

	p_imugps_globalvar->g[0]=0.0;
	p_imugps_globalvar->g[1]=0.0;
	normal_gravity(mid_r[0],mid_r[2],&p_imugps_globalvar->g[2]);

	for (i=0;i<3;i++)
	{
		mid_v[i]=p_imugps_globalvar->gps_vel[i]+0.5*p_imugps_globalvar->dv_n[i];
	}

	p_imugps_globalvar->w_ie[0]=BV_WE*cos(mid_r[0]);
	p_imugps_globalvar->w_ie[1]=0.0;
	p_imugps_globalvar->w_ie[2]=-BV_WE*sin(mid_r[0]);

	trans_rate(&mid_r[0],&mid_v[0],p_imugps_globalvar->rm,p_imugps_globalvar->rn,&p_imugps_globalvar->w_en[0]);

	for (i=0;i<3;i++)
	{
		zeta[i]=(p_imugps_globalvar->w_en[i]+p_imugps_globalvar->w_ie[i])*dt;
	}

	halfzeta[0]=0.5*zeta[0];
	halfzeta[1]=0.5*zeta[1];
	halfzeta[2]=0.5*zeta[2];
	cp_form(&halfzeta[0],&Cn[0][0]);

	for (i=0;i<3;i++)
	{
		for (j=0;j<3;j++)
		{
			if (i==j)
			{
				Cn[i][j]=1.0-Cn[i][j];
			}
			else
			{
				Cn[i][j]=-Cn[i][j];
			}
		}
	}

	matrix_multiply(3,3,3,3,&Cn[0][0],&p_imugps_globalvar->c_bn[0][0],&v5[0][0]);
	for (i=0;i<3;i++)
	{
		v1[i]=meas[i+4]+scul[i];
	}
	matrix_multiply(3,3,3,1,&v5[0][0],&v1[0],&dv_f_n[0]);

	for (i=0;i<3;i++)
	{
		p_imugps_globalvar->f_n[i]=dv_f_n[i]/dt;//*49.951219512195121951219512195122;
	}

	v1[0]=2*p_imugps_globalvar->w_ie[0]+p_imugps_globalvar->w_en[0];
	v1[1]=2*p_imugps_globalvar->w_ie[1]+p_imugps_globalvar->w_en[1];
	v1[2]=2*p_imugps_globalvar->w_ie[2]+p_imugps_globalvar->w_en[2];
	cross_product(&v1[0],&mid_v[0],&v2[0]);

	dv_g_cor[0]=(p_imugps_globalvar->g[0]-v2[0])*dt;
	dv_g_cor[1]=(p_imugps_globalvar->g[1]-v2[1])*dt;
	dv_g_cor[2]=(p_imugps_globalvar->g[2]-v2[2])*dt;

	dv_n[0]=dv_f_n[0]+dv_g_cor[0];//nav1.dv_n
	dv_n[1]=dv_f_n[1]+dv_g_cor[1];
	dv_n[2]=dv_f_n[2]+dv_g_cor[2]; //velocity increment, saved for next velocity extropolation.

	v[0]=p_imugps_globalvar->gps_vel[0]+dv_n[0];//nav1.v
	v[1]=p_imugps_globalvar->gps_vel[1]+dv_n[1];
	v[2]=p_imugps_globalvar->gps_vel[2]+dv_n[2];


	mid_v[0]=0.5*(p_imugps_globalvar->gps_vel[0]+v[0]);
	mid_v[1]=0.5*(p_imugps_globalvar->gps_vel[1]+v[1]);
	mid_v[2]=0.5*(p_imugps_globalvar->gps_vel[2]+v[2]);


	trans_rate(&mid_r[0],&mid_v[0],p_imugps_globalvar->rm,p_imugps_globalvar->rn,&p_imugps_globalvar->w_en[0]);

	zeta[0]=(p_imugps_globalvar->w_en[0]+p_imugps_globalvar->w_ie[0])*dt;
	zeta[1]=(p_imugps_globalvar->w_en[1]+p_imugps_globalvar->w_ie[1])*dt;
	zeta[2]=(p_imugps_globalvar->w_en[2]+p_imugps_globalvar->w_ie[2])*dt;

	rvec2quat(&zeta[0],&qn[0]);//tk-1->tk Qn

	xi[0]=0.0;
	xi[1]=0.0;
	xi[2]=-BV_WE*dt;

	rvec2quat(&xi[0],&qe[0]);//tk->tk-1 Qe

	quat_prod(&p_imugps_globalvar->q_ne[0],&qn[0],&v4[0]);//q_ne
	quat_prod(&qe[0],&v4[0],&q_ne[0]);
	
	norm_quat(&q_ne[0],&v4[0]);
//	memcpy(&q_ne[0],&v4[0],sizeof(double)*4);
	for (i=0;i<4;i++)
	{
		q_ne[i]=v4[i];
	}

	quat2pos(&q_ne[0],&r[0],&r[1]);//nav1.r
	r[2]=p_imugps_globalvar->gps_pos[2]-mid_v[2]*dt;



	v1[0]=meas[1]+beta[0];
	v1[1]=meas[2]+beta[1];
	v1[2]=meas[3]+beta[2];

	rvec2quat(&v1[0],&qb[0]);

	//interpolate position
	mid_r[0]=r[0]+0.5*dist_ang(r[0],p_imugps_globalvar->gps_pos[0]);
	mid_r[1]=r[1]+0.5*dist_ang(r[1],p_imugps_globalvar->gps_pos[1]);
	mid_r[2]=0.5*(r[2]+p_imugps_globalvar->gps_pos[2]);
	
	//computes rotation rates again
	p_imugps_globalvar->w_ie[0]=BV_WE*cos(mid_r[0]);
	p_imugps_globalvar->w_ie[1]=0.0;
	p_imugps_globalvar->w_ie[2]=BV_WE*(-sin(mid_r[0]));

	trans_rate(&mid_r[0],&mid_v[0],p_imugps_globalvar->rm,p_imugps_globalvar->rn,&p_imugps_globalvar->w_en[0]);

	zeta[0]=(p_imugps_globalvar->w_en[0]+p_imugps_globalvar->w_ie[0])*dt;
	zeta[1]=(p_imugps_globalvar->w_en[1]+p_imugps_globalvar->w_ie[1])*dt;
	zeta[2]=(p_imugps_globalvar->w_en[2]+p_imugps_globalvar->w_ie[2])*dt;

	v1[0]=-zeta[0];
	v1[1]=-zeta[1];
	v1[2]=-zeta[2];
	rvec2quat(&v1[0],&qn[0]);


	quat_prod(&p_imugps_globalvar->q_bn[0],&qb[0],&v4[0]);//q_bn
	quat_prod(&qn[0],&v4[0],&p_imugps_globalvar->q_bn[0]);

	norm_quat(&p_imugps_globalvar->q_bn[0],&v4[0]);
	for (i=0;i<4;i++)
	{
		p_imugps_globalvar->q_bn[i]=v4[i];
	}
	
	quat2dcm(&p_imugps_globalvar->q_bn[0],&p_imugps_globalvar->c_bn[0][0]);
	
	for (i=0;i<3;i++)
	{
		p_imugps_globalvar->gps_pos[i]=r[i];
		p_imugps_globalvar->gps_vel[i]=v[i];
		p_imugps_globalvar->dv_n[i]=dv_n[i];
	}
	

	for (i=0;i<4;i++)
	{
		p_imugps_globalvar->q_ne[i]=q_ne[i];
	}
}

void rela_pos_cal(double basic_blh[3], double pos_llh[3],struct position *pos_blv)  
{
  volatile double temp_xe,temp_yn,temp_zd;	
  double RM=6378137, RN=6378137;
  double  b_xe,b_yn,b_zd;

  rc(BV_A_Earth,BV_E2,basic_blh[0],&RM,&RN);
  b_xe=(RN+basic_blh[2])*cos(basic_blh[0])*cos(basic_blh[1]);
  b_yn=(RN+basic_blh[2])*cos(basic_blh[0])*sin(basic_blh[1]);
  b_zd=(RN*(1-BV_E2)+basic_blh[2])*sin(basic_blh[0]);
	
  rc(BV_A_Earth,BV_E2,pos_llh[0],&RM,&RN);
  temp_xe=(RN+pos_llh[2])*cos(pos_llh[0])*cos(pos_llh[1]);
  temp_yn=(RN+pos_llh[2])*cos(pos_llh[0])*sin(pos_llh[1]);
  temp_zd=(RN*(1-BV_E2)+pos_llh[2])*sin(pos_llh[0]);

  pos_blv->x = ((-1.0)*(temp_xe-b_xe)*sin(basic_blh[1])+(temp_yn-b_yn)*cos(basic_blh[1]));
  pos_blv->y =((-1.0)*(temp_xe-b_xe)*cos(basic_blh[1])*sin(basic_blh[0])-(temp_yn-b_yn)*sin(basic_blh[0])*sin(basic_blh[1])+(temp_zd-b_zd)*cos(basic_blh[0]));
	
}
