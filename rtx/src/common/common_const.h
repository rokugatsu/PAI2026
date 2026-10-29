/*
 * Common_Const.h
 *
 *  Created on: 2010/03/02
 *      Author: jun
 */

#ifndef COMMON_CONST_H_
#define COMMON_CONST_H_


// #define URG3D_MAX_DATA_SIZE    1000

// #define WPT_NORMAL      0
// #define WPT_CURV        1
// #define WPT_QUEUE       2
// #define WPT_FOLLOW      3
// #define WPT_TRAFFICLIGHT    4
// #define WPT_GOAL        10

// #define Z_WPT 0.0


//#define RATE_HZ 10 //[Hz]
//#define RATE_HZ 20 //[Hz]
#define RATE_HZ 50 //[Hz]

//設定ファイル
//#define CONFIG_PATH	"/home/jun/robo/config/RTX_config.txt"
#define LASTPOS_PATH "/home/jun/robo/data/lastpos.dat"


//    ///ロボットの速度

//    #define VEL_VERYHIGH	85 	//[cm/sec]
//    #define VEL_HIGH		75 	//[cm/sec]
//    #define VEL_MID		60 	//[cm/sec]
//    #define VEL_LOW		45 	//[cm/sec]
//    #define VEL_VERYLOW         20 	//[cm/sec]
//    #define VEL_HILL		50 	//[cm/sec]

//    #define VEL_INIT		40
//    #define VEL_MAX		80


//    #define VEL_VERYHIGH	50 	//[cm/sec]
//    #define VEL_HIGH		40 	//[cm/sec]
//    #define VEL_MID		30 	//[cm/sec]
//    #define VEL_LOW		25 	//[cm/sec]
//    #define VEL_VERYLOW	30 	//[cm/sec]
//    #define VEL_HILL		25 	//[cm/sec]

//    #define VEL_INIT		20
//    #define VEL_MAX		40

    // #define DIST_MAX 3.0   //[m]

    // #define GRIDMAP_RANGE 3.0
    // //#define GRIDMAP_RANGE 2.0
    // #define WP_LINE_WIDTH 0.75

    // //#define SELF_DOMAIN 0.35
    // #define EGO_DOMAIN 0.05
    // //#define EGO_DOMAIN 0.4

    // //RTX動作モード
    // #define	ROVING_MODE 			0
    // #define REPLAY_MODE 			1
    // #define	MAPMAKE_MODE 			2

    #define 	LEFT			0
    #define 	RIGHT		1
    #define	LEFT2		2
    #define	RIGHT2		3

    #define	BOTH		4
    #define	CENTER		10
    #define	NOSIDE		-1

    //ガイドウォール
    #define	LEFT_WALL	LEFT
    #define	RIGHT_WALL	RIGHT

    #define STR_SIZE		256

    //初期値
    #define INIT_VAL 9999

    //上限値
    #define DIFF_VEL_MAX_LIMIT	40.0		//[cm/sec]
    #define DIFF_POS_MAX_LIMIT	10.0		//[m]
    #define DIFF_DEG_MAX_LIMIT	90.0		//[deg]
    #define	DEG_EST_LIMIT		10000.0

    #define VEL_MAX_LIMIT	199.0			//[cm/sec]
    #define POS_MAX_LIMIT	10000.0			//[m]	-858993.460000
    #define DEG_MAX_LIMIT	99999999.0		//[deg]
    #define COV_MAX_LIMIT	99999999.0		//[deg]

    #define VERY_SMALL_VAL	0.00000001

    //過電流
    //#define OVER_CURRENT			2000 //[mA]
    //#define OVER_CURRENT			3000 //[mA]
    #define OVER_CURRENT			5000 //[mA]

    // ///Control Mode
    // #define READY_MODE              0
    // #define NAVI_MODE				1
    // #define POSE_MODE				2
    // #define SPIN_MODE				3
    // #define	AVOID_REVERSE_MODE      4
    // #define	RECOVERRY_MODE          5
    // #define	FINISH_MODE				6
    // #define	EMAGENCYSTOP_MODE		7
    // #define STAY_MODE				8
    // #define	MODE_ERROR				-1

    // //角度
    #define PI  (M_PI)

    #define M_RAD2DEG (180.0/M_PI)
    #define M_DEG2RAD (M_PI/180.0)

    //ロボットのサイズデータ
    #define	VCL_LENGTH			0.5	//[m]
    #define	VCL_WIDTH			0.5	//[m]
    #define	LU							0.5
    #define	LU2						0.5

    //ディスプレイサイズ
    #define DISPLAY_WIDTH	1080
    #define DISPLAY_HEIGHT	768

    // 画像サイズ
    #define AREA_OFFSET			60
    #define AREA_WIDTH			720 //360*2				//30[m]
    #define AREA_HEIGHT			((256+AREA_OFFSET))		//5[m]

    //OpenGL表示サイズ
    #define OGL_WINDOW_WIDTH	512
    #define OGL_WINDOW_HEIGHT	512

    //単位変換
    #define FS2OGL 0.05	//1mあたりの分解能
    #define OGL2FS 20		//1mあたりの分解能

    //#define FS2OGL 1	//1mあたりの分解能
    //#define OGL2FS 1		//1mあたりの分解能

    //長さ
    #define M2CM	100		//m -> cm
    #define M2MM	1000	//m -> mm
    #define CM2MM	10	//cm -> mm
    #define CM2M	0.01		//cm -> m
    #define MM2CM	0.1	//mm -> cm
    #define MM2M	0.001	//mm -> cm

    
    //Cos
    #define COS_0	1.0
    #define COS_5	0.996194698
    #define COS_10	0.984807753
    #define COS_15	0.96592582628
    #define COS_20	0.939692621
    #define COS_30	0.86602540378
    #define COS_45	0.70710678118
    #define COS_60	0.5
    #define COS_80	0.173648178
    #define COS_85	0.08715574274
    #define COS_90	0
    #define COS_95	-0.087155743
    #define COS_100 -0.173648178
    #define COS_110 -0.34202014332
    #define COS_120 -0.5
    #define COS_135 -0.70710678118
    #define COS_150 -0.86602540378
    #define COS_160 -0.939692621
    #define COS_170 -0.984807753
    #define COS_180 -1.0

    #define SIN_15	0.258819045

    #define URG_DEG15RAD 1.832490995


#endif /* COMMON_CONST_H_ */
