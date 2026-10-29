//============================================================================
// Name        : CommonFunction.cpp
// Author      : JUN
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>

#include <cstdio>	// printf()
#include <cstdlib>
#include <cmath>
#include <vector>
#include <cstring>

#include <stdio.h>	// printf()
#include <termios.h>	// termios{}, tcgetattr(), tcsetattr()
#include <unistd.h>	// STDIN_FILENO
#include <stdlib.h>	// exit()
#include <signal.h>	// signal(), SIGALRM
#include <unistd.h>	// alarm()
#include <sys/ioctl.h>	// ioctl()
#include <linux/kd.h>	// KDSKBMODE, K_RAW, K_MEDIUMRAW, K_XLATE
# include <algorithm>

#include "common_function.h"
#include "common_const.h"
//#include "common_struct.h"

using namespace std;

int SearchLabel(string str_line,string str_label,string* str_dst){

	int n=str_line.find(str_label);
	if(n>=0){
		*str_dst = str_line.substr(str_label.length());
		//cout << str_label <<"\t" << str_line << "\t" << str_dst << n << "\n";

		return str_dst->length();
	}
	else return 0;
}

int string_replace(string* pstr_target,string str_before,string str_after){

	int pos = pstr_target->find(str_before,0);
	pstr_target->replace(pos,pstr_target->size(),str_after);

	return pos;
}

//点(x1,y1)と,(xp,yp)を通って傾きqの直線との距離
double Dist_from_point_to_line(double x1,double y1,double xp,double yp,double qdeg){

	double dist=0;
	double tmp1,tmp2;

    double a = tan(qdeg*M_DEG2RAD);
	double b = -1;
	double c = yp - a*xp;

	tmp1 = a*x1+b*y1+c;
	tmp2 = sqrt(a*a+b*b);

	//if(fabs(tmp2) < VERY_SMALL_VAL) tmp2 = VERY_SMALL_VAL;
	dist = fabs(tmp1/tmp2);

	return dist;
}

//点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
double Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2){

	double dist=0;
	double tmp1,tmp2;
	double q_rad;

	if(fabs(x2-x1) < VERY_SMALL_VAL){
		return 9999;
	}

	q_rad = atan2(y2-y1,x2-x1);

	double a = tan(q_rad);
	double b = -1;
	double c = y1 - a*x1;

	tmp1 = a*xp+b*yp+c;
	tmp2 = sqrt(a*a+b*b);

	if(tmp2 < VERY_SMALL_VAL) tmp2 = VERY_SMALL_VAL;
	
    dist = fabs(tmp1/tmp2);

	return dist;
}

//点(xp,yp)と,(x0,y0),(x1,y1)を通る直線に直行する直線との距離
double Dist_from_point_to_crossline(double xp,double yp,double x0,double y0,double x1,double y1){

	double dist=0;
	// double tmp1,tmp2;
	// double q_rad;

	// if(fabs(x1-x0) < VERY_SMALL_VAL){
	// 	return 9999;
	// }

	// //q_rad = atan2(x1-x0,y1-y0);

	// double a = (y1-y0)/(x1-x0);
	// //double b = -1;
	// //double c = y1 - a*x1;

    // double A = 1/a;
    // double B = -1;
    // double C = y1-x1/a;
    
	// tmp1 = A*xp+B*yp+C;
	// tmp2 = sqrt(A*A+B*B);

	// if(tmp2 < VERY_SMALL_VAL) tmp2 = VERY_SMALL_VAL;
	
    // //dist = fabs(tmp1/tmp2);
    double dx = x1-x0;
    double dy = y1-y0;
    
    dist = fabs((x1-x0)*(xp-x1) + (y1-y0)*(yp-y1))/sqrt(dx*dx + dy*dy);

	return dist;
}
//2直線の交点
int Cross_point_2Line(double tan_q1,double x1,double y1,double tan_q2,double x2,double y2,double *xp,double *yp){

	double a1,b1,a2,b2;
	double xc,yc;

	a1 = tan_q1;
	a2 = tan_q2;

	b1 = y1-a1*x1;
	b2 = y2-a2*x2;

	if(fabs(a1- a2) < VERY_SMALL_VAL){
		return 0;
	}

	xc = (b2-b1)/(a1-a2);
	yc = a1*xc+b1;

	*xp = xc;
	*yp = yc;

	return 1;
}

//点と点の距離
double Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y){
  double dist =0;

  dist = (p1_x-p2_x)*(p1_x-p2_x) + (p1_y-p2_y)*(p1_y-p2_y);
  dist = sqrt(dist);

  return dist;
}

//点と点の二乗距離
double Dist2_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y){
	double dist =0;

	dist = (p1_x-p2_x)*(p1_x-p2_x) + (p1_y-p2_y)*(p1_y-p2_y);

	return dist;
}

//点と点の距離
double Dist_point_to_point3D(double p1_x,double p1_y,double p1_z,double p2_x,double p2_y,double p2_z){
	double dist =0;

	dist = (p1_x-p2_x)*(p1_x-p2_x) + (p1_y-p2_y)*(p1_y-p2_y) + (p1_z-p2_z)*(p1_z-p2_z);
	dist = sqrt(dist);

	return dist;
}


//線を越えたかどうかの判定
bool Judge_OverLine(double x,double y,double xp,double yp,double qp){

	bool judge = false;
    double qp_rad = qp*M_DEG2RAD;

	if((x-xp)*cos(qp_rad)+(y-yp)*sin(qp_rad) >= 0){
		judge = true;
	}

	return judge;
}

double Angle180(double q_org){

	double q_lim;

	if(q_org >= 0){
			q_lim = q_org - 360*(int)(q_org/360.0);
			if(q_lim > 180) q_lim = q_lim - 360;
		}
		else{
			q_lim = q_org - 360*(int)(q_org/360.0);
			if(q_lim <= -180) q_lim = q_lim + 360;
		}

		return q_lim;
}

double PosiAngle360(double q_org){

	double q_lim;

	if(q_org >= 0){
			q_lim = q_org - 360*(int)(q_org/360.0);
	}
	else{
		q_lim = 360+(q_org-360*(int)(q_org/360.0));
	}

	return q_lim;
}

double NegaAngle360(double q_org){

	double q_lim;

	if(q_org >= 0){
		q_lim = (q_org - 360*(int)(q_org/360.0))-360;

	}
	else{
		q_lim = (q_org-360*(int)(q_org/360.0));
	}

	return q_lim;
}


double Angle360(double q_org){

	if(q_org >= 360){
		while(q_org>= 360){
			q_org-=360;
		}
	}
	else if(q_org <= -360){
		while(q_org<= -360){
			q_org+=360;
		}
	}
	return q_org;
}


double Angle2PI(double q_org){

    double q_dst = q_org;
    double PI2 = 2.0*PI;
    if(q_dst >= PI2){
        while(q_dst>= PI2){
            q_dst-=PI2;
        }
    }
    else if(q_dst <= -PI2){
        while(q_dst<= -PI2){
            q_dst+=PI2;
        }
    }
    return q_dst;
}

//左回りか右回りの近い角度を返す
double NearestAngle360(double q_org,double q_ref){

	double q_comp[3];
	double diff_q = fabs(q_org - q_ref);
	double q_best = q_org;
	double tmp;

	q_comp[0] = q_org;
	q_comp[1] = q_org - 360;
	q_comp[2] = q_org + 360;

	for(int k=0;k < 3;k++){
		tmp = fabs(q_comp[k] - q_ref);
		if( diff_q > tmp) {
			q_best = q_comp[k];
			diff_q = tmp;
		}
	}

	return q_best;
}

//左回りか右回りの近い角度を返す
double NearestAngle360n(double q_org,double q_ref,int n){

	double q_comp[3];
	double diff_q = fabs(q_org - q_ref);
	double q_best = q_org;
	double tmp;

	for(int m=1;m < n;m++){

		q_comp[0] = q_org - 360*m;
		q_comp[1] = q_org + 360*m;

		for(int k=0;k < 2;k++){
			tmp = fabs(q_comp[k] - q_ref);
			if( diff_q > tmp) {
				q_best = q_comp[k];
				diff_q = tmp;
			}
		}
	}
	return q_best;
}

double NearestAngle180(double q_org,double q_ref){

	double q_comp;
	double q_best = q_org;

	if(q_org >= 0)q_comp = -180+q_org ;
	else q_comp = 180+q_org ;

	if(fabs(q_comp - q_ref) < fabs(q_org - q_ref)  ){
		q_best = q_comp;
	}

	return q_best;
}

//左回りか右回りの近い角度を返す
double Angle180_NearestAngle180(double q_org,double q_ref){

	double q_comp[3];
	double diff_q = 99999999;

	double q_best = q_org;
	double tmp;

	q_comp[0] = q_org;
	q_comp[1] = q_org - 180;
	q_comp[2] = q_org + 180;

	for(int k=0;k < 3;k++){
		tmp = fabs(q_comp[k] - NearestAngle180(q_ref,q_comp[k]));
		if( diff_q > tmp) {
			q_best = q_comp[k];
			diff_q = tmp;
		}
	}
	return q_best;
}

double ConteniousAngle(double qinitGL,double qrefGL){

	double qdstGL=0;

	if(fabs(qrefGL-qinitGL) > 180){
		if(qinitGL < 0 && qrefGL >= 0){
			qdstGL = qrefGL - 360*(int)(qrefGL/360.0);
			if(qdstGL > 0) qdstGL = qdstGL - 360;
		}
		else if(qinitGL >= 0 && qrefGL < 0 ){
			qdstGL = qrefGL + 360*(int)(qrefGL/360.0);
			if(qdstGL < 0) qdstGL = 360 + qdstGL;
		}
	}
	else qdstGL = qrefGL;

	return qdstGL;
}

double CompAngle360(double q_deg){

	double val;
    val = sin(q_deg*M_DEG2RAD);

	return val;

}


//角度計算　-180deg -> 180deg の範囲に入れる
double Calc_q_deg(double q_deg_src){

	double q_rad_src;
	double q_deg_dst=0;
	double tan_q;

	if(q_deg_src > 360) q_deg_src -= (int)(q_deg_src/360)*360.0;

	else if(q_deg_src <= -360) {
		q_deg_src +=(int)(-q_deg_src/360.0)*360;
	}

	if(q_deg_src >= 0){
        q_rad_src	= (q_deg_src-90)*M_DEG2RAD;
		tan_q		= tan(q_rad_src);
        q_deg_dst	= atan(tan_q)*M_RAD2DEG+90;

		if(q_deg_src/180 > 1) q_deg_dst -= 180;
	}

	else {

        q_rad_src	= (q_deg_src+90)*M_DEG2RAD;
		tan_q		= tan(q_rad_src);
        q_deg_dst	= atan(tan_q)*M_RAD2DEG-90;

		if(q_deg_src/(-180) > 1) q_deg_dst += 180;
	}

	return q_deg_dst;
}



//内積
double Calc_Innerproduct(double x1,double y1,double x2,double y2){
	return x1*x2+y1*y2;
}

double Calc_Innerproduct(double x1,double y1,double z1,double x2,double y2,double z2){
	return x1*x2+y1*y2+z1*z2;
}

//外積
int Calc_CrossProduct(double ax,double ay,double az,double bx,double by,double bz, double* px,double* py,double* pz){

	double tx,ty,tz,norm;

	tx = ay*bz-az*by;
	ty = az*bx-ax*bz;
	tz = ax*by-ay*bx;

	norm = tx*tx + ty*ty + tz*tz;
	if(fabs(norm) < VERY_SMALL_VAL ){
		return 0;
	}

	norm = sqrt(norm);

	tx /= norm;
	ty /= norm;
	tz /= norm;

	*px = fabs(tx);
	*py = fabs(ty);
	*pz = fabs(tz);

	return 1;
}

//三角形の面積(ヘロンの公式)
double TriangleArea(double A,double B,double C){

	double areaGL;
	double S = (A+B+C)/2;
	areaGL = sqrt(S*(S-A)*(S-B)*(S-C));

	return areaGL;
}

double qDeg_from_Innerproduct(double x0,double y0,double x1,double y1,double x2,double y2){

	double ip;
	double cos_q;
	double a,b;
	double deg;

	ip = Calc_Innerproduct(x1-x0,x2-x0,y1-y0,y2-y0);
	a = Dist_point_to_point(x0,y0,x1,y1);
	b = Dist_point_to_point(x0,y0,x2,y2);

	cos_q= ip/(a*b);
    deg =acos(cos_q)*M_RAD2DEG;

	 return deg;
}

//統計計算
int Calc_Statics(deque<double> &vt_data,double* ave,double* sd){

	int data_num = 0;
	double tmp_ave=0;
	double tmp_cov=0;

	for(int k=0;k < (int)vt_data.size();k++){
		if(fabs(vt_data[k]) > 0.001){
			tmp_ave += vt_data[k];
			data_num++;
		}
	}
	if(data_num == 0) return 0;

	//平均
	tmp_ave /= data_num;
	*ave = tmp_ave;

	for(int k=0;k < (int)vt_data.size();k++){
		if(fabs(vt_data[k]) > 0.001){
			tmp_cov += (vt_data[k]-tmp_ave)*(vt_data[k]-tmp_ave);
		}
	}
	//標準偏差
	tmp_cov /= data_num;
	*sd = sqrt(tmp_cov);

	return data_num;
}

//統計計算
int Calc_Statics_from_deque(deque<double> &dq_data,double* ave,double* sd){

	int data_num = 0;
	double tmp_ave=0;
	double tmp_cov=0;

	for(int k=0;k < (int)dq_data.size();k++){
		if(fabs(dq_data[k]) > 0.001){
			tmp_ave += dq_data[k];
			data_num++;
		}
	}
	if(data_num == 0) return 0;

	//平均
	tmp_ave /= data_num;
	*ave = tmp_ave;

	for(int k=0;k < (int)dq_data.size();k++){
		if(fabs(dq_data[k]) > 0.001){
			tmp_cov += (dq_data[k]-tmp_ave)*(dq_data[k]-tmp_ave);
		}
	}

	//標準偏差
	tmp_cov /= data_num;
	*sd = sqrt(tmp_cov);

	return data_num;
}

int Calc_Statics3(deque<double> &dq_data,double* ave,double* cov,double* sd){

	int data_num = 0;
	double tmp_ave=0;
	double tmp_cov=0;

	for(int k=0;k < (int)dq_data.size();k++){
		//if(fabs(dq_data[k]) > 0.001){
		tmp_ave += dq_data[k];
		data_num++;
		//}
	}
	if(data_num == 0) return 0;

	//平均
	tmp_ave /= data_num;
	*ave = tmp_ave;

	for(int k=0;k < (int)dq_data.size();k++){
		//if(fabs(dq_data[k]) > 0.001){
		tmp_cov += (dq_data[k]-tmp_ave)*(dq_data[k]-tmp_ave);
		//}
	}
	//分散
	*cov = tmp_cov;

	//標準偏差
	tmp_cov /= data_num;
	*sd = sqrt(tmp_cov);

	return data_num;
}

//相関計算
double Calc_Cos_q(double x0,double y0,double x1,double y1,double x2,double y2){

	double cos_q;
	double dist1 = Dist_point_to_point(x1,y1,x0,y0);
	double dist2 = Dist_point_to_point(x2,y2,x0,y0);

	if(dist1 < 0.001) dist1 = 0.001;
	if(dist2 < 0.001) dist2 = 0.001;

	cos_q = Calc_Innerproduct(x1-x0,y1-y0,x2-x0,y2-y0)/(dist1*dist2);
	return cos_q;
}

void msleep(int msec){
	usleep(1000*msec); // 0.001 秒 slee
}

void Voice(const char* str_wav){
	system(str_wav);
}


///////////////////////////////


cKeyInput::cKeyInput(void){
	counter = 0;	// Counter for "blank" key state
	key_mode = 1;	// Selected keyboard mode
}

void cKeyInput::sig_alarm(int sig) {
  int ret = 0;

  printf("\nI detected %d times of \"empty key\".\n", counter);

  // Restore the original "termios"

  ret = tcsetattr(STDIN_FILENO, TCSANOW, &tm);
  if (ret < 0) {
    printf("tcsetattr() failure!\n");
    ret = 1;
   }

  // Restore keyboard mode to "XLATE"

  if (key_mode != 1) {
  ret = ioctl(STDIN_FILENO, KDSKBMODE, K_XLATE);
    if (ret < 0) {
      printf("ioctl(KDSKBMODE) failure!\n");
      ret = 1;
     }
   }
  exit(ret);
 }

int cKeyInput::setup_stdin(void) {
  int ret;

  // Change keyboard mode

  if (key_mode != 1) {
    ret = ioctl(STDIN_FILENO, KDSKBMODE, key_mode);
    if (ret < 0) {
      printf("ioctl(KDSKBMODE) failure!\n");
      printf("Sorry, you might activate me on X-window.\n");
      printf("Please call me in a /dev/tty console!\n");
      return 1;
     }
   }

  // Read current "termios" parameter

  ret  = tcgetattr(STDIN_FILENO, &tm);
  ret |= tcgetattr(STDIN_FILENO, &ntm);
  if (ret < 0) {
    printf("tcgetattr() failure!\n");
    return 1;
   }

  // Setup realtime key scanning

  ntm.c_iflag &= ~(ISTRIP | INLCR | ICRNL | IGNCR | IXON | IXOFF);
  ntm.c_lflag &= ~(ECHO | ICANON | ISIG);
  ntm.c_cc[ VTIME ] = 0;	// We need no wait
  ntm.c_cc[ VMIN  ] = 0;	// Blank data is acceptable
  ret = tcsetattr(STDIN_FILENO, TCSANOW, &ntm);
  if (ret < 0) {
    printf("tcsetattr() failure!\n");
    return 1;
   }
  return 0;
 }

int testdest(int k){
	int val=k*2;
	printf("testdest:%d\n",k);
	return val;
}


///// HSV ///////
float Fraction( float v ) {
	return v - floor(v);
}

#if 1
stRGB ColorScaleBCGYR( double in_value )
{

    stRGB ret;
    int a = 255;    // alpha
    int r, g, b;    // RGB
    double  value = in_value;
    double  tmp_val = cos( 4 * M_PI * value );
    int     col_val = (int)( ( -tmp_val / 2 + 0.5 ) * 255 );

    if ( value >= ( 4.0 / 4.0 ) ) { r = 255;     g = 0;       b = 0;       }
    else if ( value >= ( 3.0 / 4.0 ) ) { r = 255;     g = col_val; b = 0;       }
    else if ( value >= ( 2.0 / 4.0 ) ) { r = col_val; g = 255;     b = 0;       }
    else if ( value >= ( 1.0 / 4.0 ) ) { r = 0;       g = 255;     b = col_val; }
    else if ( value >= ( 0.0 / 4.0 ) ) { r = 0;       g = col_val; b = 255;     }
    else {                               r = 0;       g = 0;       b = 255;     }

	ret.r =(double)r/255;
	ret.g =(double)g/255;
	ret.b =(double)b/255;

	//printf("r=%lf,g=%lf,b=%lf\n",ret.r,ret.g,ret.b);
	/*
	ret = (a&0x000000FF) << 24
        | (r&0x000000FF) << 16
        | (g&0x000000FF) <<  8
        | (b&0x000000FF);
    */

	return ret;
}
#endif


int Calc_PixelRGB(double val,stRGB* pixelRGB){

	stRGB tmpRGB={0};
	tmpRGB=ColorScaleBCGYR(val);
	*pixelRGB = tmpRGB;

	return 1;
}

double modAngle(double a) {
        
while (a < -M_PI)
	a += 2.0 * M_PI;
while (a > M_PI)
	a -= 2.0 * M_PI;
        
return a;
}