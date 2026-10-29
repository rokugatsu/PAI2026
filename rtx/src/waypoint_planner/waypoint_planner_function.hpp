#ifndef WAYPOINT_PLANNER_FUNCTION_HPP_
#define WAYPOINT_PLANNER_FUNCTION_HPP_

#include <iostream>
#include <vector>
#include <stdio.h>

using namespace std;

typedef struct time_val{
    double ts;
    double val;
}time_val;

/* 平均・標準偏差を計算するStatisticsクラス */
class Statistics{
public:
	/* 平均を計算 */
	time_val average(deque<time_val> &f){
		time_val ave = {0.0,0.0};
        int n = f.size();
		for(int i = 0; i < n; i++) {
            ave.ts += (f[i].ts-f[0].ts);
            ave.val += f[i].val;
        }
        ave.ts /= n;
        ave.val /= n;
        
        //printf("ave.ts=%lf,ave.val=%lf \n",ave.ts,ave.val);
		return ave;
	}

	/* 2乗の平均を計算 */
	time_val average2(deque<time_val> &f){
		time_val ave2 = {0.0,0.0};
        int n = f.size();
		for(int i = 0; i < n; i++){
            ave2.ts += (f[i].ts-f[0].ts)*(f[i].ts-f[0].ts);
            ave2.val += f[i].val*f[i].val;
        }
        ave2.ts /= n;
        ave2.val /= n;
        
        //printf("ave2.ts=%lf,ave2.val=%lf \n",ave2.ts,ave2.val);
		return ave2;
	}

	/* 標準偏差を計算 */
	time_val variance(deque<time_val> &f){
		time_val ave  = average(f);
		time_val ave2 = average2(f);
		time_val var;
        
        var.ts = ave2.ts - ave.ts*ave.ts;
        var.val = ave2.val - ave.ts*ave.val;
        
        //printf("var.ts=%lf,var.val=%lf \n",var.ts,var.val);
		return var;
	}

	/* 共分散を計算 */
	//double covariance(vector<double> &f, vector<double> &g, int N){
    double covariance(deque<time_val> &f){
		
        double cov = 0.0;
        int n = f.size();

		for(int i = 0; i < n; i++) cov += (f[i].ts-f[0].ts)*f[i].val/(double)n;
		//cov -= average(f, N) * average(g, N);
		cov -= average(f).ts * average(f).val;
		
        return cov;
	}
};

/* Statisticsクラスを継承したクラス */
/* Least_squares_method (最小二乗法) クラスで
 * 近似直線を計算 */

class Least_squares_method : public Statistics{

public:
	/* 傾きを計算 */
	//double slope(vector<double> &x, vector<double> &y, int N){
	double slope(deque<time_val> &tv){
		
        double a = 0.0;
		double x_var = variance(tv).ts;
		double cov   = covariance(tv);
		a = cov / x_var;
		return a;
	}

	/* 切片を計算 */
	double intercept(deque<time_val> &tv){
		double b = 0.0;
		double x_var = variance(tv).ts;
		double cov   = covariance(tv);
		double x_ave = average(tv).ts;
		double y_ave = average(tv).val;
		b = y_ave - (cov / x_var) * x_ave;
		return b;
	}
};

#endif