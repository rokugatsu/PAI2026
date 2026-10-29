#ifndef MOTION_COORDINATOR_CONST_H
#define MOTION_COORDINATOR_CONST_H

///ロボットの速度

#define VREF_MAX    60.0       //[cm/s]
#define VEL_VERYHIGH	85.0 	//[cm/sec]
/*
#define VEL_HIGH      75 	//[cm/sec]
#define VEL_MID       60 	//[cm/sec]
#define VEL_LOW       50 	//[cm/sec]
*/
#define VEL_HIGH      60.0 	//[cm/sec]
#define VEL_MID       50.0 	//[cm/sec]
#define VEL_LOW       40.0 	//[cm/sec]
#define VEL_LIGHT_LOW 30.0 	//[cm/sec]
#define VEL_VERY_LOW   20.0 	//[cm/sec]

#define VEL_EXTREAM_LOW   10.0 	//[cm/sec]
#define VEL_HILL      50.0 	//[cm/sec]

#define VEL_INIT	40.0
#define VEL_MAX		80.0

#define VEL_STEP		0.5

#define QRADFS_MAX  (M_PI/4.0)// [rad]

#define STOP_RADIUS		0.5 //[m]

#define STOP_DIST_QUEUE 0.5  //[m]

// Action variation
#define ACTION_NEUTRAL 		0
#define ACTION_POSE 		1
#define ACTION_FOLLOW 		2
#define ACTION_NAVI 		3

// Out of use 
//#define ACTION_CRUISE 		0
#define ACTION_APPROCAH 	4
#define ACTION_CURV 		5
#define ACTION_GOAWAY 		6
#define ACTION_THROUGH 		8
#define ACTION_ADJUST_GYRO 50


#define DIST_REVERSE			1.0		//[m]
#define DIST_AVOID_REVERSE      0.75

#define DIST_DEFAULT_FRONT		1.0
#define DIST_CRITICAL_FRONT		0.5
#define REVERSE_COUNT_DEFAULT   20 

#define DISTTIME 				3.0	//sec

#define DIST_SENSING_FOR_AVOID  2.0
#define DIST_MAX                3.0
#define COM_ARY_NUM             10


// Traffic Light
#define TL_GREEN    1
#define TL_RED      2

#endif // MOTION_COORDINATOR_CONST_H
