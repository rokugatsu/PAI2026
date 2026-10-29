#include 	<iostream>

#include 	<cstdio>
#include 	<cstdlib>
#include 	<cstring>

#include 	<GL/glut.h>
#include 	<pthread.h>

#include	"motion_coordinator_node.hpp"

#define	COUNT_NAVI				100
#define	COUNT_NAVI_INIT			50
#define	COUNT_AVOID				10

#define 	MINIMUM_STOP_TIME		50
#define 	POSE -10
#define 	DIST_REVERSE			1.0		//[m]
#define 	Q_RANGE_SPIN_MARGIN				10.0 	//[deg]
#define 	Q_RANGE_WALL_FOLLOW				10.0	//[deg]


#define WSUM_THERESHOLD_NORMAL 20

int Motion_coordinator::InitOdometry(void){

//PIDゲイン設定
//    mcom[0]= 1.0;		//P
//    mcom[1]= 0.1;   //I
//    mcom[2]= 0.0;		//D
//    CarrotCommand(CARROT_SET_PID_GAIN,3,mcom);
//    printf(" >>>>>>>> CARROT_SET_PID_GAIN >>>>>>>>>>>>\n");
//    msleep(50);

    mcom_[0]=cur_wpt_.l_xgl;
    mcom_[1]=cur_wpt_.l_ygl;
    mcom_[2]=cur_wpt_.l_qgl_deg;
    printf("MC:InitOdometry::SetPos:%lf, %lf, %lf \n",cur_wpt_.l_xgl,cur_wpt_.l_ygl,cur_wpt_.l_qgl_deg);
    CarrotCommand(CARROT_SET_GL,3,mcom_);
    //printf("MC:Init Odometry: %lf,%lf,%lf \n",mcom[0],mcom[1],mcom[2]);

    return 1;
}

int Motion_coordinator::Vel2StopTime(){

  int val = 0;

  if(Vel_ >0 ){
    //val = static_cast<int>(Vel_)*8;
    val = (int)(Vel_*0.01)*8;
  }

  return val;
}

int Motion_coordinator::PosingFunction(void){

    if(flag_pose_ == false){
        
        Voice(WAV_POSE);
        Stop_Mild_Until_Vel(10,100);
        printf("<DestWP=%d:mode=%d>:Control_Thread(1.3):[一時停止]Carrot_Stop_Free()\n",cur_wpt_.num,control_mode_);
            
        flag_pose_ = true;
        w_sum_=0;
        control_mode_ = POSE_MODE;
    }
    return 1;
}

int Motion_coordinator::EmergencyStopFunction(void){

    CarrotCommand(CARROT_STOP_Q,0,mcom_);
    
    w_sum_=0; //141104_2454

    return 1;
}

int Motion_coordinator::RestartFunction(void){

    flag_pose_ = false;
    control_mode_ = NAVI_MODE;

    RefVel_ = vel_high_;

    printf("<DestWP=%d:mode=%d>:Control_Thread(1.4):[再発進]\n", cur_wpt_.num,control_mode_);
    //flag_restart_ = true;

    w_sum_=0; //141104_2454

    Voice(WAV_RESTART);

    return 1;
}

int Motion_coordinator::InitStartFunction(void){

    //printf("<DestWP=%d:mode=%d> Control_Thread(0.2):[初期SubWP設定]:(%lf,%lf,%lf)\n",
    //   cur_wpt_.num,control_mode_,
    //   freespc_pt_.xGL,freespc_pt_.yGL,freespc_pt_.qradGL*M_RAD2DEG);

    printf("<DestWP=%d:mode=%d> Control_Thread(0.2):[初期SubWP設定]:(%lf,%lf,%lf)\n",
       cur_wpt_.num,
       control_mode_,
       dst_motion_cod_.xgl,dst_motion_cod_.ygl,dst_motion_cod_.qgl_rad*M_RAD2DEG);

    ///// 発進 //////
    RefVel_ =  vel_high_;

    printf("<DestWP=%d:mode=%d>:Control_Thread(1):[発進:SubWayPoint走行]Carrot_Goto_GL(%2.2lf,%4.2lf,%4.2lf)\n",
           cur_wpt_.num,control_mode_,
           RefVel_,
           dst_motion_cod_.xgl,
           dst_motion_cod_.ygl);
           
    w_sum_=0; //141104_2454

    msleep(20);

    return 1;
}

int Motion_coordinator::WaitStarting(void){

    if(key_start_<=0){
        return 1;
    }
    else{
        return 0;
    }
}

int Motion_coordinator::KeepStayingFunction(void){

    static bool flag_init = true;

    if(Vel_ > 0 && stay_count_ == 0){

        mcom_[0]= -0.5;
        mcom_[1]= 0;
        CarrotCommand(CARROT_VW_GL,2,mcom_);

        printf(">>>> Stay Here - \n");
        
        flag_stay_key_=true;
        stay_count_=100;
    }

    if(stay_count_ > 0){
        stay_count_--;
    }
    else{
        CarrotCommand(CARROT_STOP_Q,2,mcom_);
    }

    if(flag_init){
        InitOdometry();
        flag_init = false;
    }

    printf("stay_count=%d\n",stay_count_);

    return 1;
}

int Motion_coordinator::NaviControlFunction(bool flag_control){

    static double vref=0;
    static double wref=0;

    // const double kp_w = 0.005; //161029
    // const double kd_w = 0.005;//161029
    // const double ki_w = 0.00005;//161029

    const double kp_w = 0.005; //161029
    const double kd_w = 0.005;//161029
    //const double ki_w = 0.0000001;//161029
    const double ki_w = 0.00001;//161029


    double tmp_wref=0;
    double w = 0;
    int wrefError = 0;

    static double l_w = w;
    static int count_wsum=0;

    if(flag_control == false){
        RefVel_ = 0;
        RefVel_manual_ = 0;
        w_sum_ = 0;
    }
    
    if(manual_contol_){
        w = QradFS_manual_;
        w_sum_=0; //241109_tc5
    }
    else{
        //w = freespc_pt_.qradFS;
        w = dst_motion_cod_.qfs_rad;
    }

    //w cliping
//    if(vref > -0.001 && vref < 0.001 && w > -0.0005 && w < 0.0005){
//      w = 0;
//    }

    //----
    if(Vel_ > 0.5){
        w_sum_ += w;
    }

    if(fabs(w_sum_) > 0.1){
        w_sum_ = 0;
    }

    //printf("w_sum=%lf\n",w_sum_);

    //PID制御
    tmp_wref =  kp_w*(w) + kd_w*(w - l_w) + ki_w*(w_sum_);

    if(manual_contol_){
      vref = RefVel_manual_;
    }
    else{
      if(RefVel_ > vel_veryLow_){
        if(RefVel_ - Vel_ > 10 && vref < RefVel_){
          //vref += 2;
            vref += vel_step_;
        }
        else if(RefVel_ - Vel_ < -10 && vref > RefVel_){
          //vref -= 2;
          vref -= vel_step_;
        }
        else {
          vref = RefVel_;
        }
      }
      else{
        if(RefVel_ > 0 && RefVel_ < vel_veryLow_){
          vref = vel_veryLow_;
        }
        else if(RefVel_ > -vel_veryLow_ && RefVel_ < 0){
          vref = -vel_veryLow_;
        }
        else{
          vref = RefVel_;
        }
      }
    }

    //printf("vref=%lf\n",vref);

    //w cliping
    if(vref > -0.001 && vref < 0.001 && w > -0.0005 && w < 0.0005){
      w = 0;
    }
    //if(fabs(w) > 20) w=0;
    if(fabs(w) > 5.0) w=0;

    //正常値
    if(fabs(tmp_wref) <  0.02 ){
        wrefError=0;
        wref = tmp_wref;
    }
    //エラー発生！！
    else{
        wrefError=1;
        wref = 0; //!!!
    }

    l_w = w;

    wrefError_ = wrefError;

    //printf("MC::wref=%lf\n",wref);
    mcom_[0]=vref;
    mcom_[1]=wref;
    CarrotCommand(CARROT_VW_GL,2,mcom_);

    //printf("motion_coordinator::Carrot_VW(%lf,%lf)\n",mcom[0],mcom[1]);
    return 1;
}

int Motion_coordinator::FollowDirection(void){

    static double vref=0;
    //static double qref_degFS=0;
    static double qref_radFS=0;

    double w = 0;
    static double l_w = w;

    //float QdegFS = 0;
    float QradFS = 0;

    bool flag_update = false;

    if(manual_contol_){
        if(!(qref_radFS == QradFS_manual_ && vref == RefVel_manual_)){
          qref_radFS = QradFS_manual_;
          vref = RefVel_manual_;
          flag_update = true;
        }
        else{
          flag_update = false;
        }
    }
    else{
        //QradFS = freespc_pt_.qradFS;
        QradFS = dst_motion_cod_.qfs_rad;

        if(!(qref_radFS == QradFS && vref == RefVel_)){
          qref_radFS = QradFS;
          vref = RefVel_;

          flag_update = true;
        }
        else{
          flag_update = false;
        }
    }

    if(flag_update == false) return 0;

    //w cliping
    if(vref > -0.001 && vref < 0.001 && w > -0.001 && w < 0.001) w = 0;

    mcom_[0]=vref;
    mcom_[1]=qref_radFS;
    CarrotCommand(CARROT_FOLLOWDIRECTION_GL,2,mcom_);

    return 1;
}

int Motion_coordinator::SpinControlFunction(void){
    
    double qref_degFS = 0;
    
    control_mode_ = SPIN_MODE;

    //printf("motion_coordinator::SpinControlFunction(%lf)\n",QradFS_manual*M_RAD2DEG);
    if(manual_contol_){
       // printf(" --- SpinControlFunction() ------ >\n");
    }
    else{
        Stop_Until_Vel(30.0,100,Vel2StopTime());
        printf("Stop_Until_Vel: l 538\n");

        printf("<DestWP=%d:mode=%d>:Control_Thread(3.0):[その場回転] Spin()\n",
                                            cur_wpt_.num,control_mode_);
    }

    if(manual_contol_){
      qref_degFS = QradFS_manual_*M_RAD2DEG;
    }
    else{
      //qref_degFS = freespc_pt_.qradFS*M_RAD2DEG;
      qref_degFS = dst_motion_cod_.qfs_rad*M_RAD2DEG;
      
    }

    int val = Spin(qref_degFS,1.0,300);
    
    if(val < 0){
        w_sum_=0; //141104_2454

        flag_spin_ = false;
        control_mode_   = NAVI_MODE;
        action_mode_    = ACTION_NAVI;
        carrot_id_      =  CARROT_STOP_Q;

        printf("MP::SpinControlFunction finished \n");
    }
    
    return 1;
}


int Motion_coordinator::SpinControlLoop(void){
    
    double qref_degFS = 0;
    
    control_mode_ = SPIN_MODE;

    //printf("motion_coordinator::SpinControlFunction(%lf)\n",QradFS_manual*M_RAD2DEG);
    Stop_Until_Vel(30.0,100,Vel2StopTime());
    printf("Stop_Until_Vel: l 538\n");

    printf("<DestWP=%d:mode=%d>:Control_Thread(3.0):[その場回転] Spin()\n",
                                            cur_wpt_.num,control_mode_);

    //qref_degFS = freespc_pt_.qradFS*M_RAD2DEG;
    qref_degFS = dst_motion_cod_.qfs_rad*M_RAD2DEG;
      
    int val = SpinLoop(qref_degFS,1.0,300);
    
    if(val < 0){
        w_sum_=0; //141104_2454

        flag_spin_ = false;
        control_mode_   = NAVI_MODE;
        action_mode_    = ACTION_NAVI;
        carrot_id_      = CARROT_STOP_Q;

        printf("MP::SpinControlFunction finished \n");
    }
    
    return 1;
}

int Motion_coordinator::PoseControlFunction(void){

    clock_t current_time;
    double passed_time = 0;

    if(pose_time_ < 0){
        Stop_Until_Vel(5.0,100,Vel2StopTime());
        printf("Stop_Until_Vel: l 571\n");

    }
    else if(pose_time_ == 0){
        printf("<DestWP=%d:mode=%d>:Control_Thread(3.3):[一時停止<Q>] :%lf [sec]: %4.2lf[cm/s]\n",
            cur_wpt_.num,control_mode_,pose_time_,Vel_);

        Stop_Until_Vel(5.0,100,Vel2StopTime());
        printf("Stop_Until_Vel: l 579\n");
    }

    else{ //制限時間付き

        current_time = time(NULL);
        passed_time = current_time - pose_start_time_;

        if(passed_time > pose_time_){//POSE MODE 解除

            Voice(WAV_RESTART);

            Reverse(0.5,1000);

            //Spin(freespc_pt_.qradGL*M_RAD2DEG,5.0,1000);
            Spin(dst_motion_cod_.qgl_rad*M_RAD2DEG,5.0,50);
            
            printf("Stop_Until_Vel: l 596\n");

            control_mode_ = NAVI_MODE;
        }
        else{

            printf("<DestWP=%d:mode=%d>:Control_Thread(3.1):[一時停止]:Passed=%4.2lf/%4.2lf[sec] \n",
                                                                cur_wpt_.num,control_mode_,
                                                                passed_time,pose_time_);
            Stop_Until_Vel(5.0,50.0,Vel2StopTime());
             printf("Stop_Until_Vel: l 606\n");
        }
    }


    w_sum_ = 0; //141104_2454

    return 1;
}


int Motion_coordinator::ReverseFunction(void){

    double q_diffGL = 0;
    printf("%lf:<DestWP=%d:mode=%d>:Control_Thread(5.0):[後退]Reverse \n",cur_wpt_.num,control_mode_);

    w_sum_=0; //141104_2454
    Reverse(DIST_REVERSE,10);

    w_sum_=0; //141104_2454
    return 1;
}

int Motion_coordinator::Reverse_and_Spin(void){

    ReverseFunction();
    SpinControlLoop();
    
    return 1;
}

int Motion_coordinator::Stop_Until_Vel(double vel_stop,int limit,int time_msec_stop_mergin){

  int val = 0;

  CarrotCommand(CARROT_STOP_FREE,0,mcom_);
  msleep(time_msec_stop_mergin);

  CarrotCommand(CARROT_STOP_Q,0,mcom_);

  printf("MC:Stop_Until_Vel finished\n");

  return val;
}

int Motion_coordinator::Stop_Mild_Until_Vel(double vel_stop,int limit){

  int val = 0;
  int count = 0;

  printf("<MC>: Control_Thread(Stop_Mild_Until_Vel(%4.2lf,%d))\n",vel_stop,limit);

  CarrotCommand(CARROT_STOP_FREE,0,mcom_);

  while(1){

    if(fabs(Vel_) < vel_stop){
      val = 0;
      break;
    }
    else if(count > limit && count ){
      val = 1;
      break;
    }
    else if(key_pose_ > 0){
      val = POSE;
      break;
    }

    count++;
    msleep(100);
  }

  if(count < 10) msleep(50);

  CarrotCommand(CARROT_STOP_Q,0,mcom_);

  return val;
}

int Motion_coordinator::Reverse(double dist_refGL,int limit){

    int 		count = 0;
    int 		val = -1;
    bool 		flag_init = true;

    Position2D 	start;
    Position2D 	dstWP;

    int over_current=0;
    double dist_rvsGL = 0;

    control_mode_ = AVOID_REVERSE_MODE;

    start.xGL = xGL_;
    start.yGL = yGL_;
    start.qGL = qdegGL_;

    dstWP.qGL = qdegGL_ + 180;
    dstWP.deg2rad();
    dstWP.xGL = xGL_ + dist_refGL*cos(dstWP.q_rad);
    dstWP.yGL = yGL_ + dist_refGL*sin(dstWP.q_rad);

    printf("MC:Reverse(%4.2lf,%4.2lf,%4.2lf)\n",dstWP.xGL,dstWP.yGL,dstWP.qGL);

    Voice(WAV_REVERSE);

    CarrotCommand(CARROT_STOP_FREE,0,mcom_);

    msleep(100);

    while(1){

        dist_rvsGL = Dist_point_to_point(start.xGL,start.yGL,xGL_,yGL_);
        printf("MC:Reverse::%4.2lf[m],count=%d\n", dist_rvsGL,count);

        if(current_> OVER_CURRENT ){ //過電流

            over_current	=	 current_;

            printf("MC:Reverse:finish[過電流],count=%d,cur=%d,\n", count,over_current);
            val = -100;

            CarrotCommand(CARROT_STOP_Q,0,mcom_);

            msleep(100);

            break;
        }

        //if(flag_init == true && key_pose_ <= 0){
        if(key_pose_ <= 0){

            mcom_[0] = -40;
            mcom_[1] = 0;
            CarrotCommand(CARROT_VW_GL,2,mcom_);

            printf("MC:Reverse :CARROT_VW_GL(%lf,%lf)\n",mcom_[0],mcom_[1]);
        }

        if(key_pose_ > 0) {
            val = -10;

            CarrotCommand(CARROT_STOP_FREE,0,mcom_);
            msleep(10);
            break;
        }

        if( count > limit){
            val = 0;

            printf("MC:%lf:Reverse:finish[LIMIT],count=%d\n",count);
            break;
        }

        if( dist_rvsGL > dist_refGL){
            val = 2;

            printf("MC:Reverse:finish[DIST]::%4.2lf,count=%d\n", dist_rvsGL,count);

            break;

        }

        flag_init = false;
        count++;

        msleep(100);
    }

    if(key_pose_ > 0) {
        CarrotCommand(CARROT_STOP_FREE,0,mcom_);
    }
    else{
        CarrotCommand(CARROT_STOP_Q,0,mcom_);
    }
    msleep(500);

    control_mode_ = NAVI_MODE;

    return val;
}


int Motion_coordinator::Spin(double qdeg_refFS,double qdeg_range,int tmp_limit){

    int val = 0;
    static int count = 0;
    double qdeg_dstGL;
    int spin_dir = LEFT;

    static double qdeg_spin_start = 0;

    int over_current=0;
    int limit;
    //int rt = 0;
    
    double l_qdegGL = 0;
    static double l_qdeg_refFS = qdeg_refFS;

    limit = tmp_limit;

    control_mode_ = SPIN_MODE;

    if(qdeg_refFS > 0)spin_dir = LEFT;
    else spin_dir = RIGHT;

    //while(1){

        if(current_ > OVER_CURRENT  ){ //過電流

            over_current = current_;

            val = -100;
            CarrotCommand(CARROT_STOP_Q,0,mcom_);

            msleep(100);

            //break;  
            //return val;
        
        }

        //if(rt == 0 && (key_pose_ <= 0 || key_pose_ == 3)){
        if(flag_spin_init_ == true ){
            //mcom_[0] = min(qdeg_refFS,5.0);
            
            if(qdeg_refFS > 0) mcom_[0] = 180.0;
            else mcom_[0] = -180.0;
            
            qdeg_spin_start = Qdeg_Odom_; //200506
            flag_spin_init_ = false;  
            count = 0;

            printf("MC::Spin init \n");
        }
        
        CarrotCommand(CARROT_SPIN_GL,1,mcom_);
        
        double Qdeg_current = fabs(Qdeg_Odom_ - qdeg_spin_start);
        double target_threshold = fabs(qdeg_refFS) + qdeg_range;

        printf(" >>>>>>>>> CARROT_SPIN(qdeg_refFS=%lf, mcom=%lf),target=%lf,Qdeg_current = %lf, count = %d >>>>>>>>>>>>>>>\n",
            qdeg_refFS, mcom_[0], target_threshold, Qdeg_current,count);
        
        if(!manual_contol_){
            if(key_pose_ > 0 && key_pose_ < 3  ) {
                val = -10;
                CarrotCommand(CARROT_STOP_Q,0,mcom_);
                msleep(10);

                flag_spin_init_ = true;
                
                val = -10;
            }

            // if(control_mode_ == AVOID_REVERSE_MODE ){
            //     val = -1;
            //     //flag_avoid_reverse_ = true;
            //     //break;
            //     return val;
            // }
        }

       
        if( Qdeg_current > target_threshold  ){
  
            printf("CARROT_SPIN Reached target_threshold deg =%lf, limit = %d, count = %d >>>>>>>>>>>>>>>\n",target_threshold, limit,count);
            val = 1;
        } 

        if(count > limit){
            CarrotCommand(CARROT_STOP_Q,0,mcom_);
            printf("CARROT_SPIN Over limit=%d count = %d >>>>>>>>>>>>>>>\n",limit,count);
            //break;

            //return -1;
            val = 2;
        } 

        l_qdeg_refFS = qdeg_refFS;
        l_qdegGL = qdegGL_;
        //flag_init = false;
        count++;

        //if(mode_ == ROVING_MODE) 
        //msleep(750);
        
        msleep(500); //!!!
    //}

    if( val == -100) {
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Controller(Spin終了[過電流]:: cur=%d):\n",
                                                        cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_,over_current);
    }

    if( val == -10) {
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Spin終了[POSE]):\n",
            cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);
        //return val;
    }

    if(control_mode_ == AVOID_REVERSE_MODE || val < 0 ) {

        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Spin終了[AVOID_REVERSE_MODE]):\n",
            cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);

        //flag_avoid_reverse_ = true;

        val = -1;
        count = 0;
  
        CarrotCommand(CARROT_STOP_Q,0,mcom_);

        //return val;
    }

    if(val > 0){
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,qdegGL=%4.2lf)> Control_Thread(Spin終了[%d],count=%d): dir=%d,[qdeg_refFS]Spin(%lf),limit=%d\n",
                cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_,val,count,spin_dir,qdeg_refFS,limit);


        if(val==0 || count > 0){

            printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Stop)\n",cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);

        }
        
        flag_spin_init_ = true;
        count = 0;
  
        CarrotCommand(CARROT_STOP_Q,0,mcom_);
    }
       
    return val;
    
}


int Motion_coordinator::SpinLoop(double qdeg_refFS,double qdeg_range,int tmp_limit){

    int val = 0;
    static int count = 0;
    double qdeg_dstGL;//,qinitGL;
    int spin_dir = LEFT;

    static double qdeg_spin_start = 0;

    int over_current=0;
    int limit;
    
    double l_qdegGL = 0;
    static double l_qdeg_refFS = qdeg_refFS;

    limit = tmp_limit;

    control_mode_ = SPIN_MODE;

    if(qdeg_refFS > 0)spin_dir = LEFT;
    else spin_dir = RIGHT;

    while(1){

        if(current_ > OVER_CURRENT  ){ //過電流

            over_current = current_;

            val = -100;
            CarrotCommand(CARROT_STOP_Q,0,mcom_);

            msleep(100);

            break;  
        }

        if(flag_spin_init_ == true ){
            
            if(qdeg_refFS > 0) mcom_[0] = 180.0;
            else mcom_[0] = -180.0;
            
            qdeg_spin_start = Qdeg_Odom_; //200506
            flag_spin_init_ = false;  
            count = 0;

            printf("MC::Spin init \n");
        }
        
        CarrotCommand(CARROT_SPIN_GL,1,mcom_);
        
        double Qdeg_current = fabs(Qdeg_Odom_ - qdeg_spin_start);
        double target_threshold = fabs(qdeg_refFS) + qdeg_range;

        printf(" >>>>>>>>> CARROT_SPIN(qdeg_refFS=%lf, mcom=%lf),target=%lf,Qdeg_current = %lf, count = %d >>>>>>>>>>>>>>>\n",
            qdeg_refFS, mcom_[0], target_threshold, Qdeg_current,count);
        
        if(!manual_contol_){
            if(key_pose_ > 0 && key_pose_ < 3  ) {
                val = -10;
                CarrotCommand(CARROT_STOP_Q,0,mcom_);
                msleep(10);

                flag_spin_init_ = true;
                
                val = -10;
                
                printf("MC:: Spin Stop_Q\n");
                break;
            }

            // if(control_mode_ == AVOID_REVERSE_MODE ){
            //     val = -1;
            //     //flag_avoid_reverse_ = true;
            //     //break;
            //     return val;
            // }
        }

       
        if( Qdeg_current > target_threshold  ){
  
            printf("CARROT_SPIN Reached target_threshold deg =%lf, limit = %d, count = %d >>>>>>>>>>>>>>>\n",target_threshold, limit,count);
            val = 1;

            break;
        } 

        if(count > limit){
            CarrotCommand(CARROT_STOP_Q,0,mcom_);
            printf("CARROT_SPIN Over limit=%d count = %d >>>>>>>>>>>>>>>\n",limit,count);
            val = 2;
            
            break;
        } 

        l_qdeg_refFS = qdeg_refFS;
        l_qdegGL = qdegGL_;
        count++;
        
        msleep(500); //!!!
    }

    if( val == -100) {
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Controller(Spin終了[過電流]:: cur=%d):\n",
                                                        cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_,over_current);
    }

    else if( val == -10) {
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Spin終了[POSE]):\n",
            cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);
    }

    if(control_mode_ == AVOID_REVERSE_MODE || val < 0 ) {

        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Spin終了[AVOID_REVERSE_MODE]):\n",
            cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);

        val = -1;
        count = 0;
  
        CarrotCommand(CARROT_STOP_Q,0,mcom_);

    }

    if(val >= 0){
        printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,qdegGL=%4.2lf)> Control_Thread(Spin終了[%d],count=%d): dir=%d,[qdeg_refFS]Spin(%lf),limit=%d\n",
                cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_,val,count,spin_dir,qdeg_refFS,limit);


        if(val==0 || count > 0){

            printf("MC:<DestWP=%d:mode=%d:(%4.2lf,%4.2lf,%4.2lf)> Control_Thread(Stop)\n",cur_wpt_.num,control_mode_,xGL_,yGL_,qdegGL_);

        }
        
        flag_spin_init_ = true;
        count = 0;
  
        CarrotCommand(CARROT_STOP_Q,0,mcom_);
    }
       
    return val;
    
}

int Motion_coordinator::Set_PoseMode(double pose_time){

     control_mode_ = POSE_MODE;
     //printf("m_control_mode = POSE_MODE: l 375 \n");
     RefVel_ = 0;
     pose_time_ = pose_time;
     pose_start_time_ = time(nullptr);

     return 1;
 }


