#include 	<iostream>

#include 	<cstdio>
#include 	<cstdlib>
#include 	<cstring>

#include 	<GL/glut.h>
#include 	<pthread.h>

#include	"motion_coordinator_node.hpp"

#define REVERSE_COUNT_DEFAULT 20 


int Motion_coordinator::SetBehavior(void){

    static double RefVelbyMode = vel_high_;
    static float l_min_dist_frontGL = 0;
    static float min_dist_frontGL = 0;
    float Diff_dist_frontGL = 0;

    
    if(cur_wpt_.type == WPT_GOAL && flag_goal_ == false){
      Voice(WAV_GOAL_IN);
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
    //if(flag_trafficLight_ && key_pose_ ==2){
    if(flag_trafficLight_){
        
        //static int Step_for_trafficLight = 0;
        //static int prev_Step_for_trafficLight = 0;
        //printf("key_pose_=%d\n",key_pose_);
        if(key_pose_ ==2){
        
            //printf("Step_for_trafficLight_=%d, prev_Step_for_trafficLight_=%d\n",Step_for_trafficLight_,prev_Step_for_trafficLight_);
            
            if(Step_for_trafficLight_ == 0){
                //Voice(WAV_STOP_TRAFFIC_SIGNAL);

                flag_pose_ = true;
                w_sum_=0;
                control_mode_ = POSE_MODE;
                Step_for_trafficLight_ = 1;
            }
            //Wait for red
            else if(Step_for_trafficLight_ == 1){
                if(trafficLight_red_ > 0){
                    Step_for_trafficLight_ = 2;
                    if(Step_for_trafficLight_ > prev_Step_for_trafficLight_ ){
                        Voice(WAV_RED_TS);
                    }
                }
            }
            //Wait for Green
            else if(Step_for_trafficLight_ == 2){
                if(trafficLight_green_ > 0){
                    
                    // Go
                    Step_for_trafficLight_ = 3;
                    if(Step_for_trafficLight_ > prev_Step_for_trafficLight_ ){
                        Voice(WAV_GREEN_TS);
                    }
                    
                    key_pose_ = 0;
                    flag_pose_ == true;

                    //Step_for_trafficLight = prev_Step_for_trafficLight = 0;
                    //flag_trafficLight_ = false;
                }
                else{
                    //Stay
                    flag_pose_ = true;
                    w_sum_=0;
                    control_mode_ = POSE_MODE;
                }
            }
        }    
        /*    
        else if(Step_for_trafficLight_ == 3){ //241207
            if(trafficLight_red_ > 0){
                key_pose_ = 2;
                flag_pose_ = true;
                w_sum_=0;
                control_mode_ = POSE_MODE;
                Step_for_trafficLight_ = 2;
                Voice(WAV_STOP);
            }
        }
        */
        prev_Step_for_trafficLight_ = Step_for_trafficLight_;
    }

    //追従走行
    l_min_dist_frontGL  = min_dist_frontGL;
    min_dist_frontGL    = freespc_pt_.dist_front;
    
    Diff_dist_frontGL   = fabs(min_dist_frontGL - l_min_dist_frontGL);

    //Judge_Spin(20.0);
    //Judge_Spin(60.0);
    //Judge_Spin(45.0);
    //printf("0 \n");

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
    
    //else if(flag_spin_ || min_dist_frontGL < GRIDMAP_RANGE  || minDist_Object_on_WPLineGL_ < WP_LINE_WIDTH ){
    else if(flag_spin_ || min_dist_frontGL < GRIDMAP_RANGE  ){
    
        //一時停止+その場回転で回避
        if(min_dist_frontGL < DIST_DEFAULT_FRONT || flag_spin_){
         
            //printf("2 \n");
            
            // Case for Spin
            if(flag_spin_ &&  NearestForwardObject_.poseCount <= 0 && NearestForwardObject_.spinCount <= 0){
                NearestForwardObject_.poseCount      = 20;
                NearestForwardObject_.reverseCount   = 0;
                NearestForwardObject_.spinCount      = 30;
            }

            // Detect New Object
            if(Diff_dist_frontGL > 0.5 && min_dist_frontGL < DIST_DEFAULT_FRONT ){
            //if(NearestForwardObject_.poseCount <= 0 && min_dist_frontGL < DIST_DEFAULT_FRONT ){
                NearestForwardObject_.poseCount     = 20;
                
                if(min_dist_frontGL < DIST_CRITICAL_FRONT){
                    NearestForwardObject_.reverseCount 	= REVERSE_COUNT_DEFAULT; 
                    
                    Voice(WAV_REVERSE);
                    //printf("3 \n");
                }
                else{
                    NearestForwardObject_.reverseCount 	= 0;
                    //printf("4 \n");
                    
                }

                if(flag_spin_){
                    NearestForwardObject_.spinCount     = 30;
                 }
                
                Voice(WAV_ENCOUNT_OBJECT);
            }

          
            //Step1: Slow down till pose
            if(NearestForwardObject_.poseCount > 0 ){

                RefVel_ -= 5;

                if( (RefVel_ <= vel_extreamLow_ && fabs(Vel_) <= vel_extreamLow_)){
                    //printf("5 \n");  
                    //RefVel_ = vel_extreamLow_;
                    //printf("MC: min_dist_frontGL=%lf, RefVel_ <= vel_extreamLow_ && fabs(Vel) <= vel_extreamLow_: %d/%d ,Diff_dist_frontGL =%lf: %d\n",
                    //       min_dist_frontGL,mNearestForwardObject.poseCount,mNearestForwardObject.spinCount, Diff_dist_frontGL,static_cast<int>(flag_spin));
                    Set_PoseMode(-1.0); // m_control_mode = POSE_MODE;
                }

                //カウント
                NearestForwardObject_.poseCount--;
                if(NearestForwardObject_.poseCount < 0){
                    NearestForwardObject_.poseCount = 0;
                    //printf("6 \n");
                }
            }

            //Step2 後退
            else if(min_dist_frontGL < DIST_AVOID_REVERSE && NearestForwardObject_.reverseCount > 0){
                
                if(NearestForwardObject_.reverseCount >= REVERSE_COUNT_DEFAULT){
                    Voice(WAV_REVERSE);
                }

                control_mode_ = AVOID_REVERSE_MODE;
                action_mode_ = ACTION_NAVI;
                //printf("7 \n");
                //カウント
                NearestForwardObject_.reverseCount--;
                if(NearestForwardObject_.reverseCount < 0){
                    NearestForwardObject_.reverseCount = 0;
                    //printf("8 \n");
                }
            }

            //Step3 回転
            //else if(NearestForwardObject_.poseCount <= 0 && NearestForwardObject_.spinCount > 0){
            else if(NearestForwardObject_.spinCount > 0){
            
                control_mode_ = SPIN_MODE;
                /*
                if(flag_spin_){
                    action_mode_  = ACTION_ROVING;
                }
                else{
                    action_mode_  = ACTION_NAVI;
                }
                */
                action_mode_  = ACTION_NAVI;
                //printf("9 \n");
                //カウント
                NearestForwardObject_.spinCount--;
                if(NearestForwardObject_.spinCount < 0){
                    NearestForwardObject_.spinCount = 0;
                }
            }

            //Step4 回避
            //else if(NearestForwardObject_.poseCount <= 0 && NearestForwardObject_.spinCount <= 0){
            // else{

            //     RefVel_ = vel_low_;
            //     control_mode_   = NAVI_MODE;
            //     if(flag_spin_){
            //         action_mode_  = ACTION_ROVING;
            //         //printf("10 \n");
            //     }
            //     else{
            //         action_mode_  = ACTION_NAVI;
            //         //printf("11 \n");
            //     }

            //     if(flag_spin_ == true) flag_spin_ = false;
            // }
            else{

                RefVel_ = vel_low_;
                control_mode_   = NAVI_MODE;
                /*
                if(flag_spin_){
                    action_mode_  = ACTION_ROVING;
                    //printf("10 \n");
                }
                else{
                    //action_mode_  = ACTION_AVOID;
                    action_mode_  = ACTION_NAVI;
                    
                    //printf("11 \n");
                }
                */
                action_mode_  = ACTION_NAVI;
                if(flag_spin_ == true) flag_spin_ = false;
            }
        }

        //追従走行
        else {

          control_mode_ = NAVI_MODE;          
          RefVel_ = AdaptiveSpeed(min_dist_frontGL,false);

          //double diff_qrad_wpt2free = fabs(freespc_pt_.qradFS - cur_wpt_.qfs_rad);
          //action_mode_ = ACTION_ROVING;
          action_mode_ = ACTION_NAVI;
          
          //if(freespc_pt_.dist_follow <= 0.5 && diff_qrad_wpt2free < 10.0*M_DEG2RAD) action_mode_ = ACTION_ROVING;
          //else action_mode_ = ACTION_FOLLOW;

        }     
    } //	<- 前方物体に対する行動＆速度決定
      
    else{
        AvoidCounterReset();
        //action_mode_ = ACTION_ROVING;
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

    //vel = target_dist*100/DISTTIME;
    vel = target_dist*100/DistTime_;

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
        if(vel < vel_veryLow_) vel = vel_veryLow_;
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

    if(cur_wpt_.type == WPT_FOLLOW || cur_wpt_.type == WPT_QUEUE){
        Set_CurrentWayPoint_to_SubWayPoint();
    }
    else{  
         //Set_SubWayPoint_in_FreeSpace();
          
        // if(action_mode_ == ACTION_NAVI){		//回避:Step1: 一時停止
        //     Set_SubWayPoint_in_FreeSpace();
        // }

        // else if(action_mode_ == ACTION_FOLLOW ){ 	//追従
        //     Set_CurrentWayPoint_to_SubWayPoint();
        // }
        // else{  // ACTION_ROVING           
        //     Set_SubWayPoint_in_FreeSpace();
        // }

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
    
    // char strc_control[32];
    // char strc_action[32];
        
    // if(control_mode_ == READY_MODE)     sprintf(strc_control,"READY");
    // else if(control_mode_ == NAVI_MODE) sprintf(strc_control,"NAVI");
    // else if(control_mode_ == POSE_MODE) sprintf(strc_control,"POSE");
    // else if(control_mode_ == SPIN_MODE) sprintf(strc_control,"SPIN");
    // else if(control_mode_ == AVOID_REVERSE_MODE)sprintf(strc_control,"REVERSE");
    // else if(control_mode_ == FINISH_MODE)       sprintf(strc_control,"READY");
    // else if(control_mode_ == EMAGENCYSTOP_MODE) sprintf(strc_control,"E-STOP");
    // else if(control_mode_ == STAY_MODE)         sprintf(strc_control,"STAY");
    // else  sprintf(strc_control,"control: ERROR");

    // //if(action_mode_ == ACTION_ROVING)       sprintf(strc_action,"ROVING");
    // if(action_mode_ == ACTION_NEUTRAL)       sprintf(strc_action,"NEUTRAL");
    // else if(action_mode_ == ACTION_POSE)    sprintf(strc_action,"POSE");
    // else if(action_mode_ == ACTION_FOLLOW)  sprintf(strc_action,"FOLLOW");
    // else if(action_mode_ == ACTION_NAVI)   sprintf(strc_action,"AVOID");

    // //printf("V:%2.1lf/%2.1lf,minDist:%2.2lf\n control:%s,action:%s \n Milage:%4.1lf\n",
    // //    Vel_,RefVel_,freespc_pt_.dist_front,strc_control,strc_action,travelMileage_);
    // // //printf("MC:Vel_=%lf\n",Vel_);

    // //sprintf(strc_text,"V:%2.1lf/%2.1lf,minDist:%2.2lf RelVel:%2.1lf\n control:%s,action:%s \n Milage:%4.1lf\n",
    // //            Vel_,RefVel_,freespc_pt_.dist_front,freespc_pt_.relvel_front_object*100,strc_control,strc_action,travelMileage_);
    
    
    // std::string str_mode;
    // if(manual_contol_ == true){
    //     str_mode = "Manual";
    // }
    // else{
    //     str_mode = "Auto";
    // }
#if 0
    sprintf(strc_text,"%s\nWPT:%d\nV[cm/s]:%2.1lf/%2.1lf\n\
        FrontDist:%2.1lf, RelVel:%2.1lf\nControl:%s, Action:%s \n\
        Milage[m]:%4.1lf\nCount::Pose:%d / Rev:%d / Spin:%d\n\
        Status::URG F:%d, R:%d, IMU:%d, Fix:%d \n",
                str_mode.c_str(),
                cur_wpt_.num,
                Vel_,
                RefVel_,
                freespc_pt_.dist_front,
                freespc_pt_.relvel_front_object*100,
                strc_control,
                strc_action,
                travelMileage_,
                NearestForwardObject_.poseCount,
                NearestForwardObject_.reverseCount,
                NearestForwardObject_.spinCount,
                URGstatus_[0],
                URGstatus_[1],
                IMU_FIX_Odom_Status_[0],
                IMU_FIX_Odom_Status_[1]);
 #endif

    //  char strc_text[512];
    //  sprintf(strc_text,"%s\nWPT:%d\nV[cm/s]:%2.1lf/%2.1lf\n\
    //     FrontDist:%2.1lf\n\
    //     Mileage[m]:%4.1lf\n\
    //     Control:%s, Action:%s \n\
    //     Count::Pose:%d / Rev:%d / Spin:%d\n\
    //     St::URG F:%d, R:%d, IMU:%d, Fix:%d, Odom:%d\n\
    //     St::JoyP:%d, Vcl:%d \n\
    //     Cur[mA]:%d/%d",
    //             str_mode.c_str(),
    //             cur_wpt_.num,
    //             Vel_,
    //             RefVel_,
    //             freespc_pt_.dist_front,
    //             travelMileage_,
    //             strc_control,
    //             strc_action,
    //             NearestForwardObject_.poseCount,
    //             NearestForwardObject_.reverseCount,
    //             NearestForwardObject_.spinCount,
    //             URGstatus_[0],
    //             URGstatus_[1],
    //             IMU_FIX_Odom_Status_[0],
    //             IMU_FIX_Odom_Status_[1],
    //             IMU_FIX_Odom_Status_[2],
    //             Joypad_Status_,
    //             Vehicle_Status_,
    //             current_,
    //             maxCurrentmA_);

    // //---
    // rviz_2d_overlay_msgs::msg::OverlayText visText;

    // visText.text = strc_text;
    // visText.width = 550;
    // visText.height = 320;
    // visText.text_size = 20;

    // //デバイス異常
    // if(current_ > maxCurrentmA_|| Vehicle_Status_ < 0){ //電流値
    //     visText.fg_color.r = 255/255.0;
    //     visText.fg_color.g = 0.0/255.0;
    //     visText.fg_color.b = 0.0/255.0;
    //     visText.fg_color.a = 1.0;
    // }
    // else if(URGstatus_[0] < 0 || URGstatus_[1] < 0 || IMU_FIX_Odom_Status_[0] < 0 ||
    //         IMU_FIX_Odom_Status_[1] < -1 ||IMU_FIX_Odom_Status_[2] < 0 || Joypad_Status_ < 0 ){
    //     //FixはError==-10
    //     visText.fg_color.r = 155/255.0;
    //     visText.fg_color.g = 165.0/255.0;
    //     visText.fg_color.b = 0.0/255.0;
    //     visText.fg_color.a = 1.0;
    // }
    // else{
    //     visText.fg_color.r = 103.0/255.0;
    //     visText.fg_color.g = 180.0/255.0;
    //     visText.fg_color.b = 206.0/255.0;
    //     visText.fg_color.a = 1.0;
    // }
    // visText.bg_color.r = 0.0;
    // visText.bg_color.g = 0.0;
    // visText.bg_color.b = 0.0;
    // visText.bg_color.a = 0.5;
    
    // pub_VisText_->publish(move(visText));    

    //count_cb_roadpoint ++;
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


// double Waypoint_planner::Identify_MovingObject(void){

//     static deque <time_val> dq_minDist;
//     double velrel = 9999;

//     if(init_nanosecond_ <= 0)init_nanosecond_ = this->get_clock()->now().nanoseconds();

//     time_val tv;
//     tv.ts = (this->get_clock()->now().nanoseconds() - init_nanosecond_)/1000000000; //[s]
//     tv.val = minFrontDist_;
//     printf("WP: tv.ts:%lf ",tv.ts);

//     dq_minDist.push_back(tv);
//     if(dq_minDist.size() > 5) dq_minDist.pop_front();

//     if(dq_minDist.size() >= 5){
//         Least_squares_method lsm;
//         velrel = lsm.slope(dq_minDist);
//         printf("WP: Identify moving object: %lf[m], %lf \n",minFrontDist_,velrel);
//     }


//     return velrel;
// }
