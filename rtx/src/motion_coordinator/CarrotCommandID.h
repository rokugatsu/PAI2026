//CarrotCommand ID
#define DELAY_TIME	0//[msec]

#define	GETA			100
#define	GETA2			1000
#define	GETA3			1000000

#define DATA_WIDTH_4			4
#define DATA_WIDTH_6			6
#define DATA_WIDTH_8			8

#define CARROT_INIT                     0

#define CARROT_SET_TUNING_MODE		1
#define CARROT_RESET_TUNING_MODE	2

#define	CARROT_GET_TUNING_DATA		3
#define	CARROT_SET_PWM                  4
#define	CARROT_SET_PID_GAIN				5
#define	CARROT_SET_LT_GAIN				6 //LineTrack Control
#define	CARROT_SET_LT_GAIN_LIMIT		7 //LineTrack Control
#define	CARROT_SET_WHEEL_VEL			8
#define	CARROT_CALC_GYRO_BIAS			9
#define	CARROT_SET_LT_VEL_GAIN		15 //LineTrack Control
        
#define 	CARROT_SET_VEL_CM		11
#define 	CARROT_STOP_Q				12
#define 	CARROT_STOP_FREE			13

#define 	CARROT_SET_GL					20
#define	CARROT_SET_EKF_GL				21
#define	CARROT_SET_EKF_COV_GL			22
#define	CARROT_SET_EKF_COV_VCL_GL		23
#define	CARROT_SET_EKF_COV_NOISE_GL	24
#define	CARROT_SET_HAND_LENGTH			25


#define 	CARROT_FOLLOW_GL					30
#define	CARROT_GOTO_GL						31
//#define	CARROT_RUN_GL						32
#define	CARROT_FOLLOWDIRECTION_GL				32

#define	CARROT_REACH_GL						33
#define	CARROT_ROVING_GL					34

#define	CARROT_SPIN_GL						40
#define	CARROT_VW_GL						50


#define	CARROT_ADJUST_ANGLE_GL				70

#define	CARROT_LINE_GL						80
#define	CARROT_APPROACH_GL					81
#define	CARROT_GO_ANGLE_GL					82

#define	CARROT_GET_EKF_GL					90
#define 	CARROT_GET_POS_GL		10
#define	CARROT_GET_EKF_COV_VCL_GL			91
#define	CARROT_GET_EKF_COV_NOISE_GL			92

#define	CARROT_COM_INIT						99

#define	CARROT_NUTRAL						999

//for DataLogger
#define	CARROT_DATALOG						100

//for Target Detection
#define	CARROT_IMAGE_CAPTURE				101
#define	CARROT_TRFFIC_SIGNAL_RECOGNITION	102



#define CARROT_COM_NONE                                         -1
#define CARROT_COM_LAST_NUM					-20
#define CARROT_COM_ERR							-100
#define CARROT_COM_FAULT						-10
