#include 	<iostream>

#include 	<cstdio>
#include 	<cstdlib>
#include 	<cstring>

#include 	<GL/glut.h>
#include 	<pthread.h>

#include	"motion_coordinator_node.hpp"

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
    //else if(flag_spin_ || min_dist_frontGL < GRIDMAP_RANGE  || minDist_Object_on_WPLineGL_ < WP_LINE_WIDTH ){
    else if(flag_spin_ || min_dist_frontGL < GRIDMAP_RANGE ){


        //一時停止+その場回転で回避
        if(min_dist_frontGL < DIST_DEFAULT_FRONT || flag_spin_){
         
          //printf("2 \n");
          if(flag_spin_ &&  NearestForwardObject.poseCount <= 0 && NearestForwardObject.spinCount <= 0){
             NearestForwardObject.poseCount      = 20;
             NearestForwardObject.reverseCount   = 0;
             NearestForwardObject.spinCount      = 30;
          }

          // Detect New Object
          if(Diff_dist_frontGL > 0.5 && min_dist_frontGL < DIST_DEFAULT_FRONT ){
            NearestForwardObject.poseCount     = 20;
            if(min_dist_frontGL < DIST_CRITICAL_FRONT){
              NearestForwardObject.reverseCount 	= 20;
              //printf("3 \n");
            }
            else{
              NearestForwardObject.reverseCount 	= 0;
              //printf("4 \n");
            }
              NearestForwardObject.spinCount     = 30;
          }

          //Step1: Slow down till pose
          if(NearestForwardObject.poseCount > 0 ){

            RefVel_ -= 5;

            if( (RefVel_ <= vel_extreamLow_ && fabs(Vel_) <= vel_extreamLow_)){
              //printf("5 \n");  
              RefVel_ = vel_extreamLow_;

              //printf("MC: min_dist_frontGL=%lf, RefVel_ <= vel_extreamLow_ && fabs(Vel) <= vel_extreamLow_: %d/%d ,Diff_dist_frontGL =%lf: %d\n",
              //       min_dist_frontGL,mNearestForwardObject.poseCount,mNearestForwardObject.spinCount, Diff_dist_frontGL,static_cast<int>(flag_spin));
              Set_PoseMode(-1.0); // m_control_mode = POSE_MODE;
            }
            //カウント
            NearestForwardObject.poseCount--;
            if(NearestForwardObject.poseCount < 0){
              NearestForwardObject.poseCount = 0;
              //printf("6 \n");
            }
          }

          //Step2 後退
          else if(min_dist_frontGL < DIST_AVOID_REVERSE && NearestForwardObject.reverseCount > 0){
            control_mode_ = AVOID_REVERSE_MODE;
            action_mode_ = ACTION_AVOID;
            //printf("7 \n");
            //カウント
            NearestForwardObject.reverseCount--;
            if(NearestForwardObject.reverseCount < 0){
              NearestForwardObject.reverseCount = 0;
              //printf("8 \n");
            }
          }

          //Step3 回転
          else if(NearestForwardObject.poseCount <= 0 && NearestForwardObject.spinCount > 0){
            control_mode_ = SPIN_MODE;
            if(flag_spin_){
              action_mode_  = ACTION_ROVING;
            }
            else{
              action_mode_  = ACTION_AVOID;
            }
            //printf("9 \n");
            //カウント
            NearestForwardObject.spinCount--;
            if(NearestForwardObject.spinCount < 0){
              NearestForwardObject.spinCount = 0;
            }
          }

          //Step4 回避
          else if(NearestForwardObject.poseCount <= 0 && NearestForwardObject.spinCount <= 0){

            RefVel_ = vel_low_;
            control_mode_   = NAVI_MODE;
            if(flag_spin_){
              action_mode_  = ACTION_ROVING;
            //printf("10 \n");
            }
            else{
              action_mode_  = ACTION_AVOID;
            //printf("11 \n");
            }

            if(flag_spin_ == true) flag_spin_ = false;
          }
        }

        //追従走行
        //else if(minDist_Object_on_WPLineGL_ >= 0.5 ){
        //else if(freespc_pt_.dist_follow >= 0.5 ){
        else if(freespc_pt_.dist_follow >= 0.5 ){
        
          action_mode_ = ACTION_FOLLOW;
          control_mode_ = NAVI_MODE;
          
          RefVel_ = AdaptiveSpeed(min_dist_frontGL);
          
          //printf("11.5 \n");
        }       
        else if(freespc_pt_.dist_follow >= 0.0 && freespc_pt_.dist_follow <= 0.5 ){
          
          double diff_qrad_wpt2free =0;
          
          diff_qrad_wpt2free = fabs(freespc_pt_.qradFS - cur_wpt_.qfs_rad);
          //printf("MC: diff_qrad_wpt2free=%lf\n",diff_qrad_wpt2free*M_RAD2DEG);

          RefVel_ = AdaptiveSpeed(min_dist_frontGL);
          if(diff_qrad_wpt2free < 10.0*M_DEG2RAD) action_mode_ = ACTION_ROVING;
          else action_mode_ = ACTION_FOLLOW;
          control_mode_ = NAVI_MODE;
          //printf("12 \n");
        }
        //すり抜け
        else {
          AvoidCounterReset();

          RefVel_ = AdaptiveSpeed(min_dist_frontGL);
          if(RefVel_ <= vel_low_) RefVel_ = vel_low_;

          action_mode_ = ACTION_ROVING ;
          //printf("13 \n");
          if(control_mode_ != STAY_MODE){
            control_mode_ = NAVI_MODE;
          //printf("14 \n");
          }
        }
        
      } //	<- 前方物体に対する行動＆速度決定
      
      else{
        AvoidCounterReset();
        action_mode_ = ACTION_ROVING;
        RefVel_ = AdaptiveSpeed(min_dist_frontGL);
        //printf("15 \n");
        
        if(control_mode_ != STAY_MODE){
          control_mode_ = NAVI_MODE;
          //printf("16 \n");
        }
      }

    return 0;
}

float Motion_coordinator::AdaptiveSpeed(double target_dist){

  float vel = 0;

  vel = target_dist*100/DISTTIME;

  if(cur_wpt_.type == WPT_CURV){
    if(vel > vel_mid_) vel = vel_mid_;
  }
  else{
    if(vel > vel_high_) vel = vel_high_;
  }

  if(vel < vel_veryLow_) vel = vel_veryLow_;

  return vel;
}

//対策2：カウンタ初期化
void Motion_coordinator:: AvoidCounterReset(){
    NearestForwardObject.poseCount 	= 0;
    NearestForwardObject.reverseCount 	= 0;
    NearestForwardObject.spinCount 	= 0;
}

int Motion_coordinator::SetSubWayPoint(void){

    if(cur_wpt_.type == 1){
        Set_CurrentWayPoint_to_SubWayPoint();
    }
    else{    
        if(action_mode_ == ACTION_AVOID){		//回避:Step1: 一時停止
            Set_SubWayPoint_in_FreeSpace();
        }

        else if(action_mode_ == ACTION_FOLLOW ){ 	//追従
            Set_CurrentWayPoint_to_SubWayPoint();
        }
        else{  // ACTION_ROVING           
            Set_SubWayPoint_in_FreeSpace();
        }
    }

    visDirection_.pose.position.x = xGL_;
    visDirection_.pose.position.y = yGL_;
    visDirection_.pose.position.z = Z_WPT*2;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, dst_motion_cod_.qgl_rad);

    visDirection_.scale.x = dst_motion_cod_.dist;
    visDirection_.scale.y = visDirection_.scale.z = 0.1;

    visDirection_.pose.orientation.x = q.x();
    visDirection_.pose.orientation.y = q.y();
    visDirection_.pose.orientation.z = q.z();
    visDirection_.pose.orientation.w = q.w();
    
    visDirection_.header.stamp = this->get_clock()->now();

    pub_VisDirection_->publish(move(visDirection_));
    
    char strc_text[256];
    char strc_control[32];
    char strc_action[32];
        
    if(control_mode_ == READY_MODE)     sprintf(strc_control,"READY");
    else if(control_mode_ == NAVI_MODE) sprintf(strc_control,"NAVI");
    else if(control_mode_ == POSE_MODE) sprintf(strc_control,"POSE");
    else if(control_mode_ == SPIN_MODE) sprintf(strc_control,"SPIN");
    else if(control_mode_ == AVOID_REVERSE_MODE)sprintf(strc_control,"REVERSE");
    else if(control_mode_ == FINISH_MODE)       sprintf(strc_control,"READY");
    else if(control_mode_ == EMAGENCYSTOP_MODE) sprintf(strc_control,"E-STOP");
    else if(control_mode_ == STAY_MODE)         sprintf(strc_control,"STAY");
    else  sprintf(strc_control,"control: ERROR");

    if(action_mode_ == ACTION_ROVING)       sprintf(strc_action,"ROVING");
    else if(action_mode_ == ACTION_POSE)    sprintf(strc_action,"POSE");
    else if(action_mode_ == ACTION_FOLLOW)  sprintf(strc_action,"FOLLOW");
    else if(action_mode_ == ACTION_AVOID)   sprintf(strc_action,"AVOID");

    sprintf(strc_text,"V:%2.1lf/%2.1lf \n control:%s,action:%s",Vel_,RefVel_,strc_control,strc_action);
    //printf("MC:Vel_=%lf\n",Vel_);

    visString_.pose.position.x = xGL_-1.5;
    visString_.pose.position.y = yGL_;
    visString_.pose.position.z = 2.0;//Z_WPT*2;

    visString_.header.stamp = this->get_clock()->now();
    visString_.text = strc_text;

    pub_VisString_->publish(move(visString_));

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

