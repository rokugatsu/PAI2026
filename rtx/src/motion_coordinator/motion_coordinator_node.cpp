// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "motion_coordinator_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

Motion_coordinator* pmotcoord;
Motion_coordinator::Motion_coordinator(): Node("motion_coordinator_node")
{
    // タイマー実行されるイベントハンドラー関数
    auto controller =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {
        //controll_routine();
        //static int count = 0;
        //printf("MC::controller %d \n",count++);
        //msleep(1000);
      };

    auto motion_coordinator =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {

        SetSubWayPoint();
        SetBehavior();

        if(Joypad_error_count_++ > 100) Joypad_Status_ = -1;
        else Joypad_Status_ = 1;

        if(Vehicle_error_count_++ > 100) Vehicle_Status_ = -1;
        else Vehicle_Status_ = 1;

        if(Joypad_error_count_ > 200) Joypad_error_count_ = 200;
        if(Vehicle_error_count_ > 200) Vehicle_error_count_ = 200;

        //printf("Vehicle_error_count_=%d,Vehicle_Status_=%d\n",Vehicle_error_count_,Vehicle_Status_);
        Show_Status();
     
      };

    printf("MC::Start Motion_coordinator_node \n");

    declare_parameter("runMode",0);
    declare_parameter("vrefMax",VREF_MAX);
    declare_parameter("vel_veryHigh",VEL_VERYHIGH);
    declare_parameter("vel_high",VEL_HIGH);
    declare_parameter("vel_mid",VEL_MID);
    declare_parameter("vel_low",VEL_LOW);
    declare_parameter("vel_veryLow",VEL_VERY_LOW );
    declare_parameter("vel_lightLow",VEL_LIGHT_LOW);
    declare_parameter("vel_extreamLow",VEL_EXTREAM_LOW );
    declare_parameter("vel_hill",VEL_HILL);
    declare_parameter("vel_init",VEL_INIT);
    declare_parameter("vel_max",VEL_MAX );
    declare_parameter("vel_step",VEL_STEP);
    declare_parameter("stop_radius",STOP_RADIUS);
    declare_parameter("Dist_stopQueue",STOP_DIST_QUEUE);
    declare_parameter("DistTime",DISTTIME);
    declare_parameter("MaxCurrent_mA",8000);
    declare_parameter("Dist_Default_Front",DIST_DEFAULT_FRONT);
    declare_parameter("Dist_Critical_Front",DIST_CRITICAL_FRONT);
    declare_parameter("Dist_Avoid_Reverse",DIST_AVOID_REVERSE);
    declare_parameter("Reverse_Count_Default",REVERSE_COUNT_DEFAULT);
    declare_parameter("ForwardObject_PoseCount",20);
    declare_parameter("Pedestrian_StopDist",1.2);
    declare_parameter("SpinThreshold",15.0);

    runMode_  = this->get_parameter("runMode").as_int();
    vrefMax_ = this->get_parameter("vrefMax").as_double();
    vel_veryHigh_ = this->get_parameter("vel_veryHigh").as_double();
    vel_high_ = this->get_parameter("vel_high").as_double();
    vel_mid_ = this->get_parameter("vel_mid").as_double();
    vel_low_ = this->get_parameter("vel_low").as_double();
    vel_veryLow_ = this->get_parameter("vel_veryLow").as_double();
    vel_lightLow_ = this->get_parameter("vel_lightLow").as_double();
    vel_extreamLow_ = this->get_parameter("vel_extreamLow").as_double();
    vel_hill_ = this->get_parameter("vel_hill").as_double();
    vel_init_ = this->get_parameter("vel_init").as_double();
    vel_max_ = this->get_parameter("vel_max").as_double();
    vel_step_ = this->get_parameter("vel_step").as_double();
    
    stop_radius_ = this->get_parameter("stop_radius").as_double();
    Dist_stopQueue_ = this->get_parameter("Dist_stopQueue").as_double();
    DistTime_ = this->get_parameter("DistTime").as_double();
    maxCurrentmA_ = this->get_parameter("MaxCurrent_mA").as_int();
    Dist_Front_Default_ = this->get_parameter("Dist_Default_Front").as_double();
    Dist_Critical_Front_ = this->get_parameter("Dist_Critical_Front").as_double();
    Dist_Avoid_Reverse_ = this->get_parameter("Dist_Avoid_Reverse").as_double();
    Reverse_Count_Default_ = this->get_parameter("Reverse_Count_Default").as_int();
    ForwardObject_PoseCount_= this->get_parameter("ForwardObject_PoseCount").as_int();
    Pedestrian_StopDist_= this->get_parameter("Pedestrian_StopDist").as_double();

    SpinThreshold_ = this->get_parameter("SpinThreshold").as_double();

    printf("MC::runMode_: %lf \n",runMode_);  
    printf("MC::vrefMax_: %lf \n",vrefMax_);
    printf("MC::vel_veryHigh_: %lf \n",vel_veryHigh_);
    printf("MC::vel_high_: %lf \n",vel_high_);
    printf("MC::vel_mid_: %lf \n",vel_mid_);
    printf("MC::vel_low_: %lf \n",vel_low_);
    printf("MC::vel_lightLow_: %lf \n",vel_lightLow_);    
    printf("MC::vel_extreamLow_: %lf \n",vel_extreamLow_);
    printf("MC::vel_veryLow_: %lf \n",vel_veryLow_);
    printf("MC::vel_hill_: %lf \n",vel_hill_);
    printf("MC::vel_init_: %lf \n",vel_init_);
    printf("MC::vel_max_: %lf \n",vel_max_);
    printf("MC::vel_step_: %lf \n",vel_step_);
    printf("MC::stop_radius_: %lf \n",stop_radius_);
    printf("MC::Dist_stopQueue_: %lf \n",Dist_stopQueue_);   
    printf("MC::DistTime_: %lf \n",DistTime_);
    printf("MC::maxCurrentmA_: %lf \n",maxCurrentmA_);

    printf("MC::Dist_Default_Front_: %lf \n",Dist_Front_Default_);
    printf("MC::Dist_Critical_Front_: %lf \n",Dist_Critical_Front_);
    printf("MC::Dist_Avoid_Reverse_: %lf \n",Dist_Avoid_Reverse_);
    printf("MC::Reverse_Count_Default_: %d \n",Reverse_Count_Default_);
    printf("MC::ForwardObject_PoseCount_: %d \n",ForwardObject_PoseCount_);
    printf("MC::Pedestrian_StopDist_: %lf \n",Pedestrian_StopDist_);
    printf("MC::SpinThreshold_: %lf \n",SpinThreshold_); 
    
    Init_Status();

    visDirection_.id = 0;
    visDirection_.type = visualization_msgs::msg::Marker::ARROW;
    visDirection_.scale.x = visDirection_.scale.y = visDirection_.scale.z = 0.5;
    visDirection_.color.r = 0.25f;
    visDirection_.color.g = 0.5f;
    visDirection_.color.b = 1.0f;
    visDirection_.color.a = 1.0f;
    visDirection_.header.frame_id = "map";
    visDirection_.header.stamp = this->get_clock()->now();
    visDirection_.ns = "direction";
    visDirection_.action = visualization_msgs::msg::Marker::ADD;
    visDirection_.pose.orientation.w = 1.0;

    visString_.id = 1;
    visString_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    visString_.scale.x = visString_.scale.y = visString_.scale.z = 1.0;
    visString_.color.r = 1.0f;
    visString_.color.g = 1.0f;
    visString_.color.b = 1.0f;
    visString_.color.a = 1.0f;
    visString_.header.frame_id = "map";
    visString_.header.stamp = this->get_clock()->now();
    visString_.ns = "Text";
    visString_.action = visualization_msgs::msg::Marker::ADD;
    visString_.pose.orientation.w = 1.0;

    freespc_pt_.dist_front = 999;
    freespc_pt_.dist_follow = 999;
    freespc_pt_.relvel_front_object = 9999;
    freespc_pt_.count = -1;
    
    travelMileage_ = 0;
    flag_trafficLight_ = false;
    flag_ready_for_pose_ = false;

    trafficLight_green_ = 0;
    trafficLight_red_ = 0;
    
    Step_for_trafficLight_ = 0;
    prev_Step_for_trafficLight_ = 0;   

    NearestForwardObject_.poseCount=0;
    NearestForwardObject_.reverseCount=0;
    NearestForwardObject_.spinCount=0;

    Vel_ = 0;
    RefVel_ = 0.0;

    xGL_=0;
    yGL_=0;
    qdegGL_=0;

    xgl_pose_ = 0;
    ygl_pose_ = 0;
    qgl_deg_pose_ = 0;

    freespc_pt_.qradFS = 0;
    dst_motion_cod_.qfs_rad = 0;

    pub_carrot_ = create_publisher<robo_msgs::msg::CarrotCommand>("carrot_command", rclcpp::QoS(10).reliable().durability_volatile());
    pub_VisDirection_ = create_publisher<visualization_msgs::msg::Marker>("vis_direction", rclcpp::QoS(1).reliable().durability_volatile()); //!!!
    //pub_VisString_ = create_publisher<visualization_msgs::msg::Marker>("vis_text", rclcpp::QoS(1).reliable().durability_volatile()); //!!!
    pub_VisText_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>("text_overlay",rclcpp::QoS(1).reliable().durability_volatile());
   
    sub_Joypad_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_joypad, this, _1));
    sub_vehicle_ = this->create_subscription<robo_msgs::msg::Vehicle>("vehicle", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Motion_coordinator::cb_vehicle, this, _1));
    sub_localization_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_error_cov", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_localization, this, _1));
    sub_waypoint_ = this->create_subscription<robo_msgs::msg::Waypoint>("waypoint", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_waypoint, this, _1));
    sub_followpoint_ = this->create_subscription<robo_msgs::msg::Freespace>("followpoint", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_followpoint, this, _1));
    sub_initPose_ = this->create_subscription<geometry_msgs::msg::Pose>("initPose", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_initialPose, this, _1));
    sub_trafficLight_ = this->create_subscription<robo_msgs::msg::TrafficLight>("trafficlight", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_trafficlight, this, _1));
    sub_URGstatus_ = this->create_subscription<std_msgs::msg::Int16MultiArray>("/URG_Status", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_URGstatus, this, _1));
    sub_IMU_FIX_Odom_Status_ = this->create_subscription<std_msgs::msg::Int16MultiArray>("/IMU_FIX_Odom_Status", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Motion_coordinator::cb_IMU_FIX_Odom_status, this, _1));
    sub_scene_ = this->create_subscription<robo_msgs::msg::Scene>("scene", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Motion_coordinator::cb_scene, this, _1));


    timer_[0] = create_wall_timer(50ms, motion_coordinator);

    pthread_t tid;
    pmotcoord = this;

    if(runMode_ ==0){
        pthread_create(&tid, nullptr, ControllerThread,pmotcoord);
    }
    
};

int Motion_coordinator::Init_Status(void){

    key_pose_= 1;
    //flag_pose_ = true;
    flag_pose_ = false;
    
    key_start_ = 0;
    flag_goal_ = false;
    flag_asure_vehicle_ = false;
    manual_contol_ = false;
    flag_vehicleActive_ = false;
    flag_stay_key_ = false;
    flag_spin_ = false;
    flagLeavefromTarget_ = false;
    key_controllThread_ = true;

    return 0;
}

void Motion_coordinator::Show_Status(void){

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

        if(action_mode_ == ACTION_NEUTRAL)       sprintf(strc_action,"NEUTRAL");
        else if(action_mode_ == ACTION_POSE)    sprintf(strc_action,"POSE");
        else if(action_mode_ == ACTION_FOLLOW)  sprintf(strc_action,"FOLLOW");
        else if(action_mode_ == ACTION_NAVI)   sprintf(strc_action,"NAVI");

        //printf("V:%2.1lf/%2.1lf,minDist:%2.2lf\n control:%s,action:%s \n Milage:%4.1lf\n",
        //    Vel_,RefVel_,freespc_pt_.dist_front,strc_control,strc_action,travelMileage_);
        // //printf("MC:Vel_=%lf\n",Vel_);

        //sprintf(strc_text,"V:%2.1lf/%2.1lf,minDist:%2.2lf RelVel:%2.1lf\n control:%s,action:%s \n Milage:%4.1lf\n",
        //            Vel_,RefVel_,freespc_pt_.dist_front,freespc_pt_.relvel_front_object*100,strc_control,strc_action,travelMileage_);
        
        std::string str_mode;
        if(manual_contol_ == true){
            str_mode = "Manual";
        }
        else{
            str_mode = "Auto";
        }

        char strc_text[512];
        sprintf(strc_text,"%s\nWPT:%d,type:%d Scene:%d\nV[cm/s]:%2.1lf/%2.1lf\n\
            FrontDist:%2.1lf\n\
            Mileage[m]:%4.1lf\n\
            Control:%s, Action:%s \n\
            Count::Pose:%d / Rev:%d / Spin:%d\n\
            St::URG F:%d, R:%d, IMU:%d, Fix:%d, Odom:%d\n\
            St::JoyP:%d, Vcl:%d \n\
            Cur[mA]:%d/%d\n\
            key_pose:%d\n\
            key_start:%d",
            str_mode.c_str(),
            cur_wpt_.num,
            cur_wpt_.type,
            scene_,
            Vel_,
            RefVel_,
            freespc_pt_.dist_front,
            travelMileage_,
            strc_control,
            strc_action,
            NearestForwardObject_.poseCount,
            NearestForwardObject_.reverseCount,
            NearestForwardObject_.spinCount,
            URGstatus_[0],
            URGstatus_[1],
            IMU_FIX_Odom_Status_[0],
            IMU_FIX_Odom_Status_[1],
            IMU_FIX_Odom_Status_[2],
            Joypad_Status_,
            Vehicle_Status_,
            current_,
            maxCurrentmA_,
            key_pose_,
            key_start_);

        //---
        rviz_2d_overlay_msgs::msg::OverlayText visText;

        // visText.text = strc_text;
        // visText.width = 550;
        // visText.height = 320;
        // visText.text_size = 20;
        
        visText.text = strc_text;
        visText.width = 280;
        visText.height = 280;
        visText.text_size = 12;

        //デバイス異常
        if(current_ > maxCurrentmA_|| Vehicle_Status_ < 0){ //電流値
            visText.fg_color.r = 255/255.0;
            visText.fg_color.g = 0.0/255.0;
            visText.fg_color.b = 0.0/255.0;
            visText.fg_color.a = 1.0;
        }
        else if(URGstatus_[0] < 0 || URGstatus_[1] < 0 || IMU_FIX_Odom_Status_[0] < 0 ||
            IMU_FIX_Odom_Status_[1] < -1 ||IMU_FIX_Odom_Status_[2] < 0 /*|| Joypad_Status_ < 0*/ ){
            //FixはError==-10
            visText.fg_color.r = 155/255.0;
            visText.fg_color.g = 165.0/255.0;
            visText.fg_color.b = 0.0/255.0;
            visText.fg_color.a = 1.0;
        }
        else{
            visText.fg_color.r = 103.0/255.0;
            visText.fg_color.g = 180.0/255.0;
            visText.fg_color.b = 206.0/255.0;
            visText.fg_color.a = 1.0;
        }
        visText.bg_color.r = 0.0;
        visText.bg_color.g = 0.0;
        visText.bg_color.b = 0.0;
        visText.bg_color.a = 0.5;
    
        pub_VisText_->publish(move(visText));    

}

void Motion_coordinator::cb_scene(const robo_msgs::msg::Scene::SharedPtr msg) {
    
    static int last_scene= 99;
    static int count = 0;
    static int thanks_count = 0;
    static bool flag_thanks = false;
    scene_ = msg->scene;

    if((scene_ != last_scene) && count <=0 && key_start_ > 0){
        if(scene_ == 10){
            flag_thanks = true;
            //Voice(WAV_SCENE10);
        }
        //else if(scene_ == 20) Voice(WAV_SCENE20);
        //else if(scene_ == 21) Voice(WAV_SCENE21);
        //else if(scene_ == 30) Voice(WAV_SCENE30);
        //else if(scene_ == 40) Voice(WAV_SCENE40);
        //else Voice(WAV_SCENE99);
        count = 30;
    }

    if(count-- <=0) count = 0;

    if(scene_ > 20 && count <= 0 && flag_thanks == true){
        //if(key_start_ > 0) //Voice(WAV_THANKS );
        flag_thanks = false;
    }

    last_scene = scene_;
    //printf("cb_scene::scene=%d, count=%d\n",scene_,count);   
}

void Motion_coordinator::cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){

    tf2::Quaternion quat;
    double roll,pitch,yaw;
    double x_roboGL=0;
    double y_roboGL=0;
    double qrad_roboGL=0;
    double qdeg_roboGL=0;
    
    static double l_xGL=0;
    static double l_yGL=0;

    static double initTimeStamp_toSec = this->get_clock()->now().seconds();
    static int count_cb_vehicle = 0;

    static bool flag_Voice1 = true;
    static bool flag_Voice2 = true;

    passedTime_ = this->get_clock()->now().seconds() - initTimeStamp_toSec;

    l_xGL = xGL_;
    l_yGL = yGL_;

    xGL_ = x_roboGL = static_cast<double>(msg->pose.pose.position.x);
    yGL_ = y_roboGL = static_cast<double>(msg->pose.pose.position.y);
    zGL_ = y_roboGL = static_cast<double>(msg->pose.pose.position.z);

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    yawGL_ = yaw;
    qdegGL_ = yawGL_*M_RAD2DEG;

    cur_wpt_.qfs_rad  = (cur_wpt_.qgl_deg - qdegGL_)*M_DEG2RAD;

    double dist_from_point_to_line = Dist_from_point_to_line(xGL_,yGL_,xgl_pose_,ygl_pose_,qgl_deg_pose_+90);
    double dist_point_to_point =Dist_point_to_point(xGL_,yGL_,xgl_pose_,ygl_pose_);
    

    if(flag_ready_for_pose_ && 
        (dist_point_to_point < stop_radius_ || 
                dist_from_point_to_line < stop_radius_ ||
                    Judge_OverLine(xGL_,yGL_,xgl_pose_,ygl_pose_,qgl_deg_pose_))){

        printf("MC: xGL:%lf,yGL:%lf \n",xGL_,yGL_);
        printf("MC: xgl_pose_:%lf,ygl_pose_:%lf, qgl_deg_pose_:%lf\n",xgl_pose_,ygl_pose_,qgl_deg_pose_);
        
        printf("MC:dist_point_to_point = %4.2lf[m],dist_from_point_to_line = %4.2lf[m] \n",
                dist_point_to_point,
                dist_from_point_to_line);

        flag_ready_for_pose_ = false;
        ////Voice(WAV_ANZEN_KAKUNIN);
        ////Voice(WAV_ALIVE_VEHICLE);
    
        flag_pose_ = false;
        if(flag_trafficLight_){
            key_pose_ = 2;
            //Voice(WAV_STOP_TRAFFIC_SIGNAL);
            printf("MC: Stop for TrafficLight \n");            
        }
        else{
            key_pose_ = 1;
            ////Voice(WAV_ALIVE_VEHICLE);
            //Voice(WAV_ANZEN_KAKUNIN);
            printf("MC: Stop for Intersection \n"); 
        }
    }


    travelMileage_ += Dist_point_to_point(l_xGL,l_yGL,xGL_,yGL_);

    count_cb_vehicle++;
    if(count_cb_vehicle > 255) count_cb_vehicle = 0;
}

void Motion_coordinator::cb_waypoint(robo_msgs::msg::Waypoint::SharedPtr msg){

    if(cur_wpt_.num == msg->num){
        //printf("MC: cb_waypoint:: cur_wpt_.num=%lf\n",cur_wpt_.num);
        return;
    }

    l_cur_wpt_.num   = cur_wpt_.num;
    l_cur_wpt_.xgl   = cur_wpt_.xgl;
    l_cur_wpt_.ygl   = cur_wpt_.ygl;
    l_cur_wpt_.qgl_deg  = cur_wpt_.qgl_deg;
    l_cur_wpt_.road_width  = cur_wpt_.road_width;
    l_cur_wpt_.stop        = cur_wpt_.stop;

    cur_wpt_.num   = msg->num;
    cur_wpt_.xgl   = msg->xgl;
    cur_wpt_.ygl   = msg->ygl;
    cur_wpt_.qgl_deg  = msg->qgl_deg;
    cur_wpt_.road_width  = msg->road_width;

    printf("MC: cur_wpt_.num=%d\n",cur_wpt_.num);

    cur_wpt_.l_xgl     = msg->l_xgl;
    cur_wpt_.l_ygl     = msg->l_ygl;
    cur_wpt_.l_qgl_deg = msg->l_qgl_deg;

    cur_wpt_.dist      = msg->dist;
    cur_wpt_.type      = msg->type;
    cur_wpt_.stop      = msg->stop;

    //xgl_pose_ = cur_wpt_.xgl;
    //ygl_pose_ = cur_wpt_.ygl;
    //qgl_deg_pose_ = cur_wpt_.qgl_deg;

    if(cur_wpt_.stop > 0 && flag_ready_for_pose_ == false){
     
        //Voice(WAV_STOP_NEXT);
        
        flag_ready_for_pose_ = true;        
        
        xgl_pose_ = cur_wpt_.xgl;
        ygl_pose_ = cur_wpt_.ygl;
        // //qgl_deg_pose_ = l_cur_wpt_.qgl_deg;
        qgl_deg_pose_ = cur_wpt_.qgl_deg;
    
        if(cur_wpt_.type == WPT_TRAFFICLIGHT){
            flag_trafficLight_ = true;
            Step_for_trafficLight_ = 0;
            prev_Step_for_trafficLight_ = 0;    
        }
        else{ //241207
            flag_trafficLight_ = false;
            // Step_for_trafficLight_ = 0;
            // prev_Step_for_trafficLight_ = 0;   
        }
    }

 
    if(cur_wpt_.type == WPT_GOAL){
        flag_goal_ = true;
        //Voice(WAV_GOAL_IN);
    }
}

void Motion_coordinator::cb_followpoint(robo_msgs::msg::Freespace::SharedPtr msg){

    freespc_pt_.count    = msg->count;
    freespc_pt_.xGL      = msg->xgl;
    freespc_pt_.yGL      = msg->ygl;
    freespc_pt_.qradGL   = msg->qgl_rad;

    freespc_pt_.xFS      = msg->xfs;
    freespc_pt_.yFS      = msg->yfs;
    freespc_pt_.qradFS   = msg->qfs_rad;

    freespc_pt_.dist_follow      = msg->dist_follow;
    min_dist_frontArea_ = freespc_pt_.dist_front = msg->dist_front_object;
    freespc_pt_.relvel_front_object = msg->relvel_front_object;

  //printf("MC:cb_followpoint::(%lf,%lf,%lf) \n",freespc_pt_.xGL,freespc_pt_.yGL,freespc_pt_.qradGL );
}

#if 1
void Motion_coordinator::cb_joypad(const sensor_msgs::msg::Joy::SharedPtr msg){

    static bool key[10]={false};
    static float Vref=0,QradFS=0;
    static bool flag_manual = false;
    static int ii = 0;
    static int count= 0x00;

    static sensor_msgs::msg::Joy l_msg = *msg;

    count++;
    if(count > 0xffff) count = 0;
    
    Joypad_error_count_ = 0; 

    /*
    printf("IF ----------------------------------------------------------\n");
    printf("axes:%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf \n",
           msg->axes[0],msg->axes[1],msg->axes[2],msg->axes[3],msg->axes[4],msg->axes[5],msg->axes[6],msg->axes[7]);

    printf("button:%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \n",
           msg->buttons[0],msg->buttons[1],msg->buttons[2],msg->buttons[3],msg->buttons[4],msg->buttons[5],msg->buttons[6],msg->buttons[7],msg->buttons[8],msg->buttons[9],msg->buttons[10]);
    */

    //Carot Control Command publish
    if(msg->buttons[6] > 0 && l_msg.buttons[6] == 0){ //modeボタン
      flag_asure_vehicle_ = true;
      printf("MC: >> Asure\n") ;
    }

    if(msg->buttons[7] > 0 && l_msg.buttons[7] == 0){ //STARTボタン
      if(key_start_ == 0){

        manual_contol_ = false;

        RefVel_ = 0;
        key_start_ = 1;
        control_mode_ = NAVI_MODE;
        
        travelMileage_ = 0;

        printf("MC: >> Start\n") ;
        //msleep(50);
      }

      else if(key_pose_ <= 0){
        key_pose_  = 1;
        printf("MC: >> COMMAND_POSE ON\n");
        flag_spin_init_ = true;
        carrot_id_ = CARROT_STOP_Q;
      }

      else{
        key_pose_  = 0;
        RefVel_ = vel_low_;
        printf("MC: >> COMMAND_POSE OFF\n") ;
      }

      l_msg = *msg;
      return;
    }

    // else if(msg->buttons[2] > 0){ //Xボタン
    //     // Mark points
    //     flag_point_mark_ = true;
    //     //Voice(WAV_POINT_MARK);
    // }

    else if(msg->buttons[8] > 0 && l_msg.buttons[8] == 0){ //Logitechボタン

      if(manual_contol_ == false){
        manual_contol_ = true;
        ////Voice(WAV_MANUAL_ON);
        printf("MC: cb_interface:: MANUAL=ON\n ");
      }
      else{
        manual_contol_ = false;
        ////Voice(WAV_MANUAL_OFF);
        printf("MC: cb_interface:: MANUAL=OFF\n ");
      }

      l_msg = *msg;

      return;
    }

    else if((msg->buttons[9] > 0 && l_msg.buttons[9] == 0) ||
        (msg->buttons[10] > 0 && l_msg.buttons[10] == 0)){ //AL or ARボタン

        if(key_pose_ <= 0){
            key_pose_  = 1;

            printf("MC: >> COMMAND_POSE ON\n");

            carrot_id_ = CARROT_STOP_Q;
        }
        else{
            key_pose_  = 0;
            printf("MC: >> COMMAND_POSE ON\n");

        }
      flag_spin_init_ = true;  
      l_msg = *msg;

      return;
    }

    //AR: Carrot_VW
    else if(manual_contol_ == true){
        //if(msg->axes[3] != 0 || msg->axes[4] != 0){
        if(msg->axes[3] < -0.001 || msg->axes[3] > 0.001 || msg->axes[4] < -0.001|| msg->axes[4] > 0.001){
        
            ii = 3;

            QradFS  = QRADFS_MAX*msg->axes[ii];
            Vref = vrefMax_*sqrt(msg->axes[ii]*msg->axes[ii] + msg->axes[ii+1]*msg->axes[ii+1]);

            if(msg->axes[ii+1] < 0) Vref = -Vref;

            if(Vref > vrefMax_) Vref = vrefMax_;
            if(Vref < -vrefMax_) Vref = -vrefMax_;

            //0.000625
            if(Vref > -0.001 && Vref < 0.001 && QradFS > -0.001 && QradFS < 0.001) QradFS = 0;

            RefVel_manual_ = Vref;
            QradFS_manual_ = QradFS;

            carrot_id_ = CARROT_VW_GL;
            key_pose_ = 0;
            printf("MC:: ID=%d >> COMMAND_VW_GL(%lf,%lf)\n",carrot_id_,RefVel_manual_,QradFS_manual_) ;
        }

        else if(msg->axes[2] < -0.5){
            printf("MC:: ID=%d >> COMMAND_SPIN_GL +90deg\n") ;
            carrot_id_ = CARROT_SPIN_GL;
            QradFS_manual_ = 90.0*M_DEG2RAD;
            msg->axes[2] = 0;
        }

        else if (msg->axes[5] < -0.5){
            printf("MC:: ID=%d >> COMMAND_SPIN_GL -90deg\n") ;
            carrot_id_ = CARROT_SPIN_GL;
            QradFS_manual_ = -90.0*M_DEG2RAD;
            msg->axes[5] = 0;
        }
    }
    else{
        carrot_id_ = -1;
    }

  l_msg = *msg;
}

void Motion_coordinator::cb_initialPose(const geometry_msgs::msg::Pose::SharedPtr msg) {

        double x,y,z,roll, pitch, yaw,yawDeg;

        x=msg->position.x;
        y=msg->position.y;
        z=msg->position.z;

      tf2::Quaternion q(
          msg->orientation.x,
          msg->orientation.y,
          msg->orientation.z,
          msg->orientation.w);

      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
     /*
      printf("MC:: cb_initialPose: x=%lf, y=%lf, z=%lf, roll=%lf, pitch=%lf, yaw=%lf\n",
          x,
          y,
          z,
          roll,
          pitch,
          yaw);
      */
      yawDeg = modAngle(yaw)*180.0/M_PI;

      mcom_[0]=x;
      mcom_[1]=y;
      mcom_[2]=yawDeg;

      //Voice(WAV_SET_POSITION);

      printf("MC:SetPos:%lf, %lf, %lf \n",x,y,yawDeg);
      CarrotCommand(CARROT_SET_GL,3,mcom_);
}

void Motion_coordinator::cb_vehicle(const robo_msgs::msg::Vehicle::SharedPtr msg) {

  // if(flag_asure_vehicle_){
  //   //Voice(WAV_ALIVE_VEHICLE);
  //   flag_asure_vehicle_ = false;
  // }
    flag_vehicleActive_ = true;
    Vel_   = msg->v;
    Wrad_  = msg->dw;
    Qdeg_Odom_ = msg->q;
    travelMileage_ = msg->dist;
    GyroTemp_  = msg->gyro_temp;
    GyroRefV_  = msg->gyro_ref_v;
    current_   = msg->current;
    GyroStatus_ = msg->gyro_status;

   //printf("MC::current_=%d \n",current_);
    Vehicle_error_count_ = 0;
}

void Motion_coordinator::cb_trafficlight(robo_msgs::msg::TrafficLight::SharedPtr msg){
    
    trafficLight_red_ = msg->red;
    trafficLight_green_ = msg->green;

    //printf("MC:TrafficLight::Green=%d, Red=%d\n",trafficLight_green_,trafficLight_red_);
}

void Motion_coordinator::cb_URGstatus(std_msgs::msg::Int16MultiArray::SharedPtr msg){
    URGstatus_[0] = msg->data[0]; //Front URG
    URGstatus_[1] = msg->data[1]; //Rear URG
}
//cb_IMU_FIXstatus(std_msgs::msg::Int16MultiArray::SharedPtr msg);
void Motion_coordinator::cb_IMU_FIX_Odom_status(std_msgs::msg::Int16MultiArray::SharedPtr msg){
    IMU_FIX_Odom_Status_[0] = msg->data[0]; //IMU
    IMU_FIX_Odom_Status_[1] = msg->data[1]; //FIX
    IMU_FIX_Odom_Status_[2] = msg->data[2]; //Odom
}


int Motion_coordinator::CarrotCommand(int CommandID,int n,float* com){

    static short count = 0x00;
    robo_msgs::msg::CarrotCommand carrot;

    carrot.id = (short)CommandID;
    for(int k=0;k<n;k++){
        carrot.com[k]=com[k];
    }

    carrot.count = count;
    pub_carrot_->publish(std::move(carrot));

    count++;
    if(count>=0xff){
        count=0x00;
    }

    return CommandID;
}

#endif
void Motion_coordinator::test(void){
  printf("testdest\n");
}

bool Motion_coordinator::key_ControllThread(void){
  return  key_controllThread_;
}

int main(int argc, char * argv[])
{
  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Motion_coordinator>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
