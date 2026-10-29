#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "motion_coordinator_node.hpp"

int Motion_coordinator::controll_routine(void){
    
    static bool flag_first = true;
    static int count_wsum= 0;
    static bool l_manual_control = manual_contol_;

    //printf("MC: Control Thread ON \n");

    /////// 初期化  //////////////
    if(flag_first){
      CarrotCommand(CARROT_STOP_FREE,0,mcom_);  
      flag_first = false;  
    }

    if(manual_contol_ == true){
      if(l_manual_control == false) {
        Voice(WAV_MANUAL_ON);
      }
      //printf("MC: RefVel_manual_ = %lf current=%lf, count_wsum=%d\n",RefVel_manual_,current_,count_wsum);

      if(fabs(Vel_ - RefVel_manual_) > 5.0 || RefVel_manual_ < 0.1 || QradFS_manual_ < 0.1){
        if(current_ > 10 ) count_wsum++;
      }
    }
    else{
      if(l_manual_control == true) {
        if(!flag_first) 
          Voice(WAV_MANUAL_OFF);
          //Voice(WAV_START);
      }
      //printf("MC: RefVel_ = %lf current=%lf, count_wsum=%d\n",RefVel_,current_,count_wsum);     
      if(fabs(Vel_-RefVel_) > 5.0 || RefVel_ < 0.1){
        if(current_ > 10 ) count_wsum++;
      }
    }
      
    l_manual_control = manual_contol_;

    if(count_wsum > 40){
      w_sum_ = 0;
      count_wsum = 0;
    }
    if(carrot_id_ == -1 ){
      w_sum_ = 0; 
    }
      
    if(manual_contol_ == true){

      //printf("MC: Manual::carrot_id --- > %d \n",carrot_id);
      // 一時停止
      if(carrot_id_ == CARROT_STOP_Q) {
        //printf("MC: Manual::CARROT_STOP_Q --- > %d \n",carrot_id_);
        CarrotCommand(CARROT_STOP_Q,0,mcom_);
      }
      else if(carrot_id_ == CARROT_SPIN_GL){
        SpinControlFunction();
      }
      else if(carrot_id_ == CARROT_VW_GL){
        NaviControlFunction(true);
      }  
      else{
        NaviControlFunction(false);
      }
     
      return 1;
    }

    if(key_start_ <= 0) return 3;
    else if(key_start_==1){
      Voice(WAV_START);
      key_start_ = 2;
      key_pose_ = 0;
      flag_pose_ = false;

      InitStartFunction();
      w_sum_ = 0;
    }
   
    // 一時停止
    if(key_pose_ > 0) {
        //printf("MC: COMMAND_POSE ON \n");
        PosingFunction();
        w_sum_ = 0;
        return 4;
    }
    //再発進
    else if(key_pose_ <= 0 && flag_pose_ == true){
        //printf("MC: RESTART \n");
        w_sum_ = 0;
        flag_trafficLight_ = false;
        RestartFunction();
    }
    else {
      flag_pose_ = false;
    }

    //初回動作
    if(flag_first){
      InitStartFunction();
      flag_first = false;
      w_sum_ = 0;
    }

    //緊急停止モード
    if(control_mode_ == EMAGENCYSTOP_MODE){
      w_sum_ = 0;
      EmergencyStopFunction();
      l_manual_control = manual_contol_;
      flag_spin_init_ = true;
     
      return 5;
    }

    // Keep Staying
    if(key_pose_ > 1/*pcrtx->control_mode_ == STAY_MODE*/){
      PoseControlFunction();
      flag_spin_init_ = true;
      w_sum_ = 0;
    }

    // ナビゲーション走行
    else if(control_mode_ == NAVI_MODE ){
      //printf("MC:NaviControlFunction\n");
      NaviControlFunction(true);
      flag_spin_init_ = true;
    }

    // その場回転
    else if(control_mode_ == SPIN_MODE){
      SpinControlFunction();
    }

    // 停止
    else if(control_mode_ == POSE_MODE){//停止
      PoseControlFunction();
      flag_spin_init_ = true;
      w_sum_ = 0;
    }

    // 停止,後退
    else if(control_mode_ == AVOID_REVERSE_MODE){//過接近につき停止
      //ReverseFunction();
      Reverse_and_Spin();
      w_sum_ = 0;
    }

    return 100;
}

void *ControllerThread(void* lpvParam){

  Motion_coordinator* pmot = (Motion_coordinator*)lpvParam;
  printf("MC::ControllerThread: %d\n");
    
  while(pmot->key_ControllThread()){
    
    pmot->controll_routine();
    
    msleep(100);
  }
    
  return nullptr;
}
 