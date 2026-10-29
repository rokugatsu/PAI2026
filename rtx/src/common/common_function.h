/*
 * CommonFunction.h
 *
 *  Created on: 2010/03/02
 *      Author: jun
 */
#pragma once

#ifndef COMMONFUNCTION_H_
#define COMMONFUNCTION_H_

///// Common_Function.h ////
#include 	<iostream>
#include 	<cstdio>
#include 	<cstdlib>
#include 	<cstring>
#include 	<cmath>
#include 	<termios.h>	// termios{}, tcgetattr(), tcsetattr()
#include	<vector>
#include	<deque>

//#include	"./common_struct.h"

using namespace std;

//ラベル検索
int SearchLabel(string str_line,string str_label,string* str_dst);


//文字列置き換え
int string_replace(string* pstr_target,string str_before,string str_after);

//点(x1,y1)と,(xp,yp)を通り傾きqの直線との距離
double Dist_from_point_to_line(double x1,double y1,double xp,double yp,double qdeg);

//点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
double Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2);

double Dist_from_point_to_crossline(double xp,double yp,double x0,double y0,double x1,double y1);

//点と点の距離
double Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y);

//点と点の二乗距離
double Dist2_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y);

double Dist_point_to_point3D(double p1_x,double p1_y,double p1_z,double p2_x,double p2_y,double p2_z);

//線を越えたかどうかの判定
bool Judge_OverLine(double x,double y,double xp,double yp,double qp);

//2直線の交点
int Cross_point_2Line(double tan_q1,double x1,double y1,double tan_q2,double x2,double y2,double *xp,double *yp);


//角度計算　-180deg -> 180deg の範囲に入れる
double Calc_q_deg(double q_deg_src);

//音声ファイル再生
//bool Voice(CString cstr,bool flag_no_stop);

//内積
double Calc_Innerproduct(double x1,double y1,double x2,double y2);
double Calc_Innerproduct(double x1,double y1,double z1,double x2,double y2,double z2);

//外積
int Calc_CrossProduct(double ax,double ay,double az,double bx,double by,double bz,
		double* px,double* py,double* pz);

//統計計算 平均，分散
int Calc_Statics(deque<double> &vt_data,double* ave,double* sd);

//統計計算
int Calc_Statics_from_deque(deque<double> &dq_data,double* ave,double* sd);

//統計計算 平均,分散,標準偏差
int Calc_Statics3(deque<double> &dq_data,double* ave,double* cov,double* sd);

//相関計算
//double Calc_Cos_q(Position2D p_org,Position2D p1,Position2D p2);
double Calc_Cos_q(double x0,double y0,double x1,double y1,double x2,double y2);
double qDeg_from_Innerproduct(double x0,double y0,double x1,double y1,double x2,double y2);

//三角形の面積(ヘロンの公式)
double TriangleArea(double A,double B,double C);

double Angle180(double q_org);
double Angle360(double q_org);
double AnglePI(double q_org);
double Angle2PI(double q_org);
double PosiAngle360(double q_org);
double NegaAngle360(double q_org);
double ConteniousAngle(double qinitGL,double qrefGL);
double NearestAngle360(double q_org,double q_ref);
double NearestAngle360n(double q_org,double q_ref,int n);
double NearestAngle180(double q_org,double q_ref);
double Angle180_NearestAngle180(double q_org,double q_ref);
double CompAngle360(double q_deg);

double modAngle(double a);
 

void msleep(int msec);
int testdest(int k);
void Voice(const char* str_wav);




class cKeyInput{

public:
	cKeyInput(void);
	struct termios tm;	// "termios" structure holders
	struct termios ntm;	// "termios" structure holders
	int counter;	// Counter for "blank" key state
	int key_mode;	// Selected keyboard mode

	void sig_alarm(int sig) ;
	int setup_stdin(void) ;
};

/// HSV
typedef struct stRGB{
	double r;
	double g;
	double b;
	double a;
}stRGB;

stRGB ColorScaleBCGYR( double in_value );
int Calc_PixelRGB(double val,stRGB* pixelRGB);
#endif /* COMMON_FUNCTION_H_ */
