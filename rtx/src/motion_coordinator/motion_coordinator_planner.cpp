#include 	<iostream>

#include 	<cstdio>
#include 	<cstdlib>
#include 	<cstring>

#include 	<GL/glut.h>
#include 	<pthread.h>

#include	"motion_coordinator_node.hpp"

//#define REVERSE_COUNT_DEFAULT 20 

int Motion_coordinator::SetBehavior(void){

    static double RefVelbyMode = vel_high_;
    static float l_min_dist_frontGL = 0;
    static float min_dist_frontGL = 0;
    float Diff_dist_frontGL = 0;

    
    if(cur_wpt_.type == WPT_GOAL && flag_goal_ == false){
      //Voice(WAV_GOAL_IN);
      flag_goal_ = true;
    }

    if(flag_goal_){

      Set_PoseMode(0);
      flag_goal_ = true;
      action_mode_ = ACTION_POSE;
      control_mode_ = ACTION_POSE;

      return 1;
    }

    // 歩行者信号 //241207
    if(flag_trafficLight_){
        
        if(key_pose_ ==2){
        
            //printf("Step_for_trafficLight_=%d, prev_Step_for_trafficLight_=%d\n",Step_for_trafficLight_,prev_Step_for_trafficLight_);
            
            if(Step_for_trafficLight_ == 0){
                ////Voice(WAV_STOP_TRAFFIC_SIGNAL);

                flag_pose_ = true;
                w_sum_=0;
                control_mode_ = POSE_MODE;
                Step_for_trafficLight_ = 1;

                printf("TrafficLight: -- Ready --\n");
            }
            //Wait for red
            else if(Step_for_trafficLight_ == 1){
                if(trafficLight_red_ > 0){
                    Step_for_trafficLight_ = 2;
                    if(Step_for_trafficLight_ > prev_Step_for_trafficLight_ ){
                        //Voice(WAV_RED_TS);
                        printf("TrafficLight:Red -- Stop --\n");
                    }
                }
            }
            //Wait for Green
            else if(Step_for_trafficLight_ == 2){
                if(trafficLight_green_ > 0 ){ //261016
                    
                    // Go
                    Step_for_trafficLight_ = 3;
                    ////Voice(WAV_GREEN_TS);
                    //Voice(  WAV_GREEN_TS_RESTART);
                    printf("TrafficLight:Green -- Restart --\n");

                    key_pose_ = 0;
                    flag_pose_ == true;                    
                }
                else{
                    //Stay
                    flag_pose_ = true;
                    w_sum_=0;
                    control_mode_ = POSE_MODE;
                }
            }
        }    

        prev_Step_for_trafficLight_ = Step_for_trafficLight_;
    }

    //追従走行
    l_min_dist_frontGL  = min_dist_frontGL;
    min_dist_frontGL    = freespc_pt_.dist_front;
    
    Diff_dist_frontGL   = fabs(min_dist_frontGL - l_min_dist_frontGL);

    if(control_mode_ == STAY_MODE|| manual_contol_){
        //printf("1 \n"); 
         return -1;
    }
    
    else if(cur_wpt_.type == WPT_QUEUE){
        
        control_mode_ = NAVI_MODE;          
        action_mode_ = ACTION_FOLLOW;
        RefVel_ = AdaptiveSpeed(min_dist_frontGL-Dist_stopQueue_,true);

        double diff_qrad_wpt2free = fabs(freespc_pt_.qradFS - cur_wpt_.qfs_rad);
        if(freespc_pt_.dist_follow <= 0.5 && diff_qrad_wpt2free < 10.0*M_DEG2RAD) action_mode_ = ACTION_NAVI;
        else action_mode_ = ACTION_FOLLOW;
        
    }
    
    else if( flag_spin_ || min_dist_frontGL < GRIDMAP_RANGE  ){
    
        //一時停止+その場回転で回避
        if(min_dist_frontGL < Dist_Front_Default_ || flag_spin_){
         
            //printf("2 \n");
            // Detect New Object
            if(NearestForwardObject_.poseCount == 0 && NearestForwardObject_.reverseCount == 0){ //1.0m

                NearestForwardObject_.poseCount     = ForwardObject_PoseCount_; //20
                if(scene_ != 10) NearestForwardObject_.reverseCount 	= Reverse_Count_Default_; //20
                
                //Voice(WAV_ENCOUNT_OBJECT);
            }

          
            //Step1: Slow down till pose
            if(NearestForwardObject_.poseCount > 0 ){

                RefVel_ -= 5;
                if(RefVel_ < 0) RefVel_ = 0;
                
                if( (RefVel_ <= vel_extreamLow_ && fabs(Vel_) <= vel_extreamLow_)){
                    //printf("5 \n");  
                    //printf("MC: min_dist_frontGL=%lf, RefVel_ <= vel_extreamLow_ && fabs(Vel) <= vel_extreamLow_: %d/%d ,Diff_dist_frontGL =%lf: %d\n",
                    //       min_dist_frontGL,mNearestForwardObject.poseCount,mNearestForwardObject.spinCount, Diff_dist_frontGL,static_cast<int>(flag_spin));
                    Set_PoseMode(-1.0); // m_control_mode = POSE_MODE;
                }

                //カウント
                NearestForwardObject_.poseCount--;
                if(NearestForwardObject_.poseCount < 0){
                    NearestForwardObject_.poseCount = 0;
                    printf("6 \n");
                }
            }

            //Step2 後退+回転
            else if(min_dist_frontGL < Dist_Avoid_Reverse_ && NearestForwardObject_.reverseCount > 0){
                

                control_mode_ = AVOID_REVERSE_MODE;
                action_mode_ = ACTION_NAVI;
                printf("7 \n");
                //カウント
                NearestForwardObject_.reverseCount--;
                if(NearestForwardObject_.reverseCount < 0){
                    NearestForwardObject_.reverseCount = 0;
                    printf("8 \n");
                }
            }

            else{

                if(scene_ == 20){ //他ロボット接近
                    RefVel_ = vel_high_; //251204
                }
                else if(scene_ == 10 || scene_ == 21){ //251123
                    if(min_dist_frontGL < Pedestrian_StopDist_) RefVel_ = 0.0; //近づきすぎたら停止
                    else RefVel_ = AdaptiveSpeed(min_dist_frontGL,false);
                }
                else{
                    RefVel_ = AdaptiveSpeed(min_dist_frontGL,false);
                }

                control_mode_   = NAVI_MODE;
                action_mode_  = ACTION_NAVI;
                if(flag_spin_ == true) flag_spin_ = false;
            }
        }

        //追従走行
        else {
            control_mode_ = NAVI_MODE;          
            action_mode_ = ACTION_NAVI;

            RefVel_ = AdaptiveSpeed(min_dist_frontGL,false);
        }     
    } //	<- 前方物体に対する行動＆速度決定
      
    else{
        AvoidCounterReset();
        action_mode_ = ACTION_NAVI;
        
        RefVel_ = AdaptiveSpeed(min_dist_frontGL,false);
        //printf("15 \n");
        
        if(control_mode_ != STAY_MODE){
          control_mode_ = NAVI_MODE;
          //printf("16 \n");
        }
    }

    return 0;
}

float Motion_coordinator::AdaptiveSpeed(double target_dist, bool flag_allowStop){

    float vel = 0;

    vel = target_dist*100/DistTime_;
    //printf("vel=%lf, target_dist = %lf, DistTime_=%lf\n",vel,target_dist,DistTime_);
    if(cur_wpt_.type == WPT_CURV){
        if(vel > vel_mid_) vel = vel_mid_;
    }
    else{
        if(vel > vel_high_) vel = vel_high_;
    }

    if(flag_allowStop){
        ;
    }
    else{        
        if(vel < vel_veryLow_){
            if(scene_ >=30) vel = vel_veryLow_;
        } 
    }

    if(fabs(cur_wpt_.qgl_deg - cur_wpt_.l_qgl_deg) > 10.0) {
        vel *= 0.8;
    }


    return vel;
}

//対策2：カウンタ初期化
void Motion_coordinator:: AvoidCounterReset(){
    NearestForwardObject_.poseCount 	= 0;
    NearestForwardObject_.reverseCount 	= 0;
    NearestForwardObject_.spinCount 	= 0;
}

int Motion_coordinator::SetSubWayPoint(void){

    //if(cur_wpt_.type == WPT_FOLLOW || cur_wpt_.type == WPT_QUEUE){
    if(cur_wpt_.type == WPT_QUEUE){
        Set_CurrentWayPoint_to_SubWayPoint();
    }
    else{  

        if(action_mode_ == ACTION_FOLLOW ){ 	//追従
            Set_CurrentWayPoint_to_SubWayPoint();
        }
        else{  // ACTION_NAVI           
            Set_SubWayPoint_in_FreeSpace();
        }
    }

    visDirection_.pose.position.x = xGL_;
    visDirection_.pose.position.y = yGL_;
    visDirection_.pose.position.z = Z_WPT*2;
   
    tf2::Quaternion q;
    q.setRPY(0, 0, dst_motion_cod_.qgl_rad);

    visDirection_.scale.x = dst_motion_cod_.dist*0.9;
    visDirection_.scale.y = visDirection_.scale.z = 0.1;

    visDirection_.pose.orientation.x = q.x();
    visDirection_.pose.orientation.y = q.y();
    visDirection_.pose.orientation.z = q.z();
    visDirection_.pose.orientation.w = q.w();

    visDirection_.header.stamp = this->get_clock()->now();

    pub_VisDirection_->publish(move(visDirection_));
    
    return 1;
}

int Motion_coordinator::Set_CurrentWayPoint_to_SubWayPoint(void){

    xy  tmpWpt;
    xy  WptFS;

    double tmp_qradGL = qdegGL_*M_DEG2RAD;
    double qrad0_FS=0;

    double cos_qGL = cos(tmp_qradGL);
    double sin_qGL = sin(tmp_qradGL);

 
    dst_motion_cod_.xgl  = cur_wpt_.xgl;
    dst_motion_cod_.ygl  = cur_wpt_.ygl;

    tmpWpt.x = cur_wpt_.xgl - xGL_;
    tmpWpt.y = cur_wpt_.ygl - yGL_;

    WptFS.x = (cos_qGL*tmpWpt.x + sin_qGL*tmpWpt.y);
    WptFS.y = (-sin_qGL*tmpWpt.x + cos_qGL*tmpWpt.y);
    qrad0_FS= atan2(WptFS.y,WptFS.x);

    dst_motion_cod_.xfs  = WptFS.x;
    dst_motion_cod_.yfs  = WptFS.y;

    cur_wpt_.qfs_rad = static_cast<float>(qrad0_FS);
    cur_wpt_.qgl_deg = qdegGL_ + cur_wpt_.qfs_rad*M_RAD2DEG;
    cur_wpt_.dist = Dist_point_to_point(cur_wpt_.xgl,cur_wpt_.ygl,xGL_,yGL_);
    
    dst_motion_cod_.qgl_rad = cur_wpt_.qgl_deg*M_DEG2RAD;
    dst_motion_cod_.qfs_rad = cur_wpt_.qfs_rad;
    dst_motion_cod_.dist    = Dist_point_to_point(dst_motion_cod_.xgl,dst_motion_cod_.ygl,xGL_,yGL_);
    
    qdegFS_motion_cod_ =cur_wpt_.qfs_rad*M_RAD2DEG;

    //printf("MC: cur_wpt_.distGL=%lf, dst_motion_cod_.qdegFS=%lf \n",cur_wpt_.distGL, dst_motion_cod_.qradFS*M_RAD2DEG);
    //printf("MC: %d: cur_wpt_.xGL=%lf, cur_wpt_.yGL=%lf, dst_motion_cod_.qdegFS=%lf \n",cur_wpt_.num ,cur_wpt_.xGL, cur_wpt_.yGL, dst_motion_cod_.qradFS*M_RAD2DEG);
    
    return 1;
 }


 int Motion_coordinator::Set_SubWayPoint_in_FreeSpace(void){
     
    dst_motion_cod_.xgl     = freespc_pt_.xGL;
    dst_motion_cod_.ygl     = freespc_pt_.yGL;
     
    dst_motion_cod_.xfs     = freespc_pt_.xFS;
    dst_motion_cod_.yfs     = freespc_pt_.yFS;
     
    dst_motion_cod_.qgl_rad = freespc_pt_.qradGL;
    dst_motion_cod_.qfs_rad = freespc_pt_.qradFS;
    dst_motion_cod_.dist    = Dist_point_to_point(dst_motion_cod_.xgl,dst_motion_cod_.ygl,xGL_,yGL_);
     
    qdegFS_motion_cod_ = freespc_pt_.qradFS*M_RAD2DEG;
    //printf("MC::qdegFS_motion_cod = %lf \n",qdegFS_motion_cod);

     return 1;
 }

