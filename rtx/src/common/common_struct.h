/*
 * CommonStruct.h
 *
 *  Created on: 2014/02/26
 *      Author: jun
 */

#ifndef COMMONSTRUCT_H_
#define COMMONSTRUCT_H_

// c++系
#include <iostream>
#include <iomanip>
#include <cmath>
#include "common_const.h"


typedef struct st_followpoint{
    int count;
    double xGL;
    double yGL;

	double xFS;
    double yFS;

    double qradGL;
    double qradFS;

	double dist_follow;
	double dist_front;
    double relvel_front_object;


}st_point;

typedef struct PositionBase{
    float dist;
    float x;
    float y;
    float z;
    float qrad;
}PositionBase;

typedef struct GPSVCL{
	double xgpsGL;
	double ygpsGL;
	double qgpsGL;
	double q_radgpsGL;

	double q_offsetGL;

	double xposGL;
	double yposGL;
	double qposGL;

    //void DEG2RAD(){
    void deg2rad(){
    	q_radgpsGL = qgpsGL*M_DEG2RAD;
	}

}GPSVCL;

////// 物理データ ////////
typedef struct Position2D{

	double distGL;
	double time;

	double xFS;
	double yFS;
	double qFS;

	double xGL;
	double yGL;
	double qGL;
	double q_offsetGL;

	double xOGL;
	double yOGL;
	double qOGL;
	double q_offsetOGL;

	//RAD
	double q_rad;
	double q_offset_rad;
	double qFS_rad;

    int transGL2OGL(){
    	xOGL = -yGL*FS2OGL;
    	yOGL = xGL*FS2OGL;
    	qOGL = -qGL;

    	return 1;
	}

    //void DEG2RAD(){
    int deg2rad(){
    	q_rad = qGL*M_DEG2RAD;
    	q_offset_rad = q_offsetGL*M_DEG2RAD;

    	qFS_rad = qFS*M_DEG2RAD;
    	return 1;
	}

	int dummy;
	//int search;

}Position2D;

typedef struct Position3D{

	short type;
	Position2D pos;
	double zGL;
	double zOGL;
	double val;
	int etc;

	void transGL2OGL(){
		pos.transGL2OGL();
		zOGL=zGL*FS2OGL;
	}

}Position3D;


typedef struct Velocity{
	double v;
	double w;
}Velocity;


typedef	struct Coord_Real{
	Position2D pos;

	Velocity vel;
}Coord_Real;

//三軸
typedef struct xy{
	double x;
	double y;
}xy;

//三軸
typedef struct xyz{
	double x;
	double y;
	double z;
}xyz;

//三軸+強度
typedef struct xyzi{
	double x;
	double y;
	double z;
	int i;
}xyzi;

//三軸
typedef struct xyqrad{
        double x;
        double y;
        double qrad;
}xyqrad;

//3Dmodel
typedef struct Model3D{

	xyz GL;
	xyz OGL;
	xyz FS;

	double qFS;

	//Coord3Type Sensor;

	double q_pitch;
	double wp_line_dist;
	double front_dist;
	double dist;

	bool flag_plane;
	bool flag_avoid;
	int WP_area;

	double term;
	int seg;
	int urg_num;
	int rbuf_num;
	int target;

	unsigned long intensity;
}Model3D;

typedef struct ScanPoint{

	Position3D coord;
	double dist;
	double term;
        int	urg_num;
	double	intensity;

}coord3D;


//統計
typedef struct StaticVal{
	double ave;
	double sd;
	int size;
}StaticVal;

//ジャイロオフセット
typedef struct GyroOffset{
	StaticVal offset;
	double q;
}GyroOffset;

typedef struct WayPoint{

	Position2D 		pos;
	long			num;
	double			stop;
	char			navi_mode;
	int 			map_change;
	int 			PFkey;
	double 			area;
	double			search;
        int                     type;
        double road_width;
}WayPoint;

typedef struct Target{
	Position2D pos;
	int type;
	int detect;
}Target;

typedef struct PointSet{
	int num;
	int wp;
	int flagOffset;
	double xGL;
	double yGL;
	double qGL;
}PointSet;

typedef struct TimeVal{
	double time;
	double val;
}TimeVal;

typedef struct StaticObject{
	Position2D pos;
	bool flagAlive;

	int poseCount;
	int reverseCount;
	int spinCount;

}StaticObject;

typedef struct VSSPData{

	double time;
	double x;
	double y;
	double z;
	int i;

}VSSPData;


typedef struct VehicleData{
   double v;
   double w;
   double dd;

   double xGL;
   double yGL;
   double qGL;
}VehicleData;

typedef struct st_Waypoint{
    uint num;
    float x0gl;
    float y0gl;		
    float xgl;
    float ygl;
    float xfs;
    float yfs;
    float qgl_deg;
    float qfs_rad;
    float l_xgl;
    float l_ygl;
    float l_qgl_deg;
    float refvel_cm;
    float dist;
    float stop;
    float road_width;
    uint type;
    std::string etc;
}st_Waypoint;
#endif /* COMMONSTRUCT_H_ */
