#define URG3D_MAX_DATA_SIZE    1000

#define WPT_NORMAL      0
#define WPT_CURV        1
#define WPT_QUEUE       2
#define WPT_FOLLOW      3
#define WPT_TRAFFICLIGHT    4
#define WPT_GOAL        10

#define Z_WPT 0.0

#define DIST_MAX 3.0   //[m]

#define GRIDMAP_RANGE 3.0
//#define GRIDMAP_RANGE 2.0
#define WP_LINE_WIDTH 0.75

//#define SELF_DOMAIN 0.35
#define EGO_DOMAIN 0.05
//#define EGO_DOMAIN 0.4

//RTX動作モード
#define	ROVING_MODE 			0
#define REPLAY_MODE 			1
#define	MAPMAKE_MODE 			2

///Control Mode
#define READY_MODE              0
#define NAVI_MODE				1
#define POSE_MODE				2
#define SPIN_MODE				3
#define	AVOID_REVERSE_MODE      4
#define	RECOVERRY_MODE          5
#define	FINISH_MODE				6
#define	EMAGENCYSTOP_MODE		7
#define STAY_MODE				8
#define	MODE_ERROR				-1

//角度
#define PI  (M_PI)

#define M_RAD2DEG (180.0/M_PI)
#define M_DEG2RAD (M_PI/180.0)