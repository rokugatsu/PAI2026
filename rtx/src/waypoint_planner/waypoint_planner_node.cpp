#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "waypoint_planner_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


#define ARRAY_SIZE 100

Waypoint_planner::Waypoint_planner(): Node("waypoint_planner_node")
{

    // タイマー実行されるイベントハンドラー関数
    auto depict_waypoint =
      [this]() -> void  // ラムダ式による関数オブジェクトの定義
      {
        static int n=0;
        std::string str,str1,str2;
        char strc_wpt[256];
        char strc_wpt2[256];

        geometry_msgs::msg::Point p;
        geometry_msgs::msg::Point p1;

        current_waypoint_.points.clear();
        follow_waypoint_.points.clear();
        points_.points.clear();
        stops_.points.clear();
        queues_.points.clear();
        follows_.points.clear();
        trafficLight_.points.clear();
        markpoints_.points.clear();
        line_strip_.points.clear();
        line_strip2_.points.clear();
        line_strip3_.points.clear();
        line_strip4_.points.clear();
        turning_circle_.points.clear();

        wpt_string_.points.clear();
        
        cluster_arrows_.points.clear();

        Markers_.markers.clear();
        ClusterVectorMarkers_.markers.clear();

        int dq_wpt_size = 0;

        for(uint k=0; k < dq_Waypoint_.size();k++){

            dq_wpt_size = dq_Waypoint_.size();

            p.x = dq_Waypoint_[k].xgl;
            p.y = dq_Waypoint_[k].ygl;
            p.z = Z_WPT;

            if(k > 0 && k == cur_wpt_num_){

                p1 = p;
                p1.z *=2;

                current_waypoint_.pose.position.x = p1.x;
                current_waypoint_.pose.position.y = p1.y;
                current_waypoint_.pose.position.z = p1.z;

                current_waypoint_.points.push_back(p1);

                if(mode_== 2) {
                    Edit_Waypoint(k);
                }

                if(dq_Waypoint_[k].stop > 0){
                    if(dq_Waypoint_[k].type == WPT_TRAFFICLIGHT){
                        sprintf(strc_wpt,"Stop TrafficLight::WPT %d",k);
                    }
                    else{
                        sprintf(strc_wpt,"Stop::WPT %d",k);
                    } 
                }
                else{
                    //sprintf(strc_wpt,"WPT %d",k);
                    sprintf(strc_wpt,"WPT %d / %d / %d:\nV[cm/s]: %2.1lf\nF:%4.1lf\nL:%4.1lf\nR:%4.1lf\nLS:%d ",
                        k,wpt_modifyOrder_,Wpt_Counter_,Vel_cmps_,minFrontDist_,minLeftDist_,minRightDist_,lane_side_);

                }
                str1 = strc_wpt;
                
                if(dq_Waypoint_[k].type == WPT_FOLLOW){
                    //sprintf(strc_wpt2,"Follow_WPT");
                    str2 = " : Follow_WPT";
                }
                else if(dq_Waypoint_[k].type == WPT_QUEUE){
                    //sprintf(strc_wpt2,"Queue");
                    str2 = " : Queue";
                }
                
                str = str1 + str2;

                wpt_string_.pose.position.x = p.x+5;
                wpt_string_.pose.position.y = p.y+5;
                wpt_string_.pose.position.z = 0.0;
                wpt_string_.text = str.c_str();
                wpt_string_.points.push_back(p);

                current_wpt_.num = cur_wpt_num_;
                current_wpt_.xgl = dq_Waypoint_[k].xgl;
                current_wpt_.ygl = dq_Waypoint_[k].ygl;
                current_wpt_.x0gl = dq_Waypoint_[k].x0gl;
                current_wpt_.y0gl = dq_Waypoint_[k].y0gl;
                current_wpt_.qgl_deg = dq_Waypoint_[k].qgl_deg;
                //current_wpt_.qfs_rad = dq_Waypoint_[k].qfs_rad;
                
                trans_GL_to_FSf(current_wpt_.xgl, current_wpt_.ygl, &current_wpt_.xfs, &current_wpt_.yfs, &current_wpt_.qfs_rad);
                
                if(mode_== 0) {
                    //Prev wpt
                    if(k > 0){
                        prev_wpt_.num = cur_wpt_num_-1;
                        prev_wpt_.xgl = dq_Waypoint_[k-1].xgl;
                        prev_wpt_.ygl = dq_Waypoint_[k-1].ygl;

                        prev_wpt_.x0gl = dq_Waypoint_[k-1].x0gl;
                        prev_wpt_.y0gl = dq_Waypoint_[k-1].y0gl;

                        prev_wpt_.qgl_deg = dq_Waypoint_[k-1].qgl_deg;
                        prev_wpt_.qfs_rad = dq_Waypoint_[k-1].qfs_rad;
                    }

                    if(k == 0){
                        prev_wpt_.num = cur_wpt_num_;

                        prev_wpt_.xgl = prev_wpt_.l_xgl = current_wpt_.l_xgl = dq_Waypoint_[k].xgl;
                        prev_wpt_.ygl = prev_wpt_.l_ygl = current_wpt_.l_ygl = dq_Waypoint_[k].ygl;

                        prev_wpt_.x0gl = current_wpt_.x0gl = dq_Waypoint_[k].x0gl;
                        prev_wpt_.y0gl = current_wpt_.y0gl = dq_Waypoint_[k].y0gl;

                        prev_wpt_.qgl_deg = prev_wpt_.l_qgl_deg = current_wpt_.l_qgl_deg = dq_Waypoint_[k].qgl_deg;
                        prev_wpt_.qfs_rad = dq_Waypoint_[k].qfs_rad;
                    }
                    else{
                        prev_wpt_.xgl = current_wpt_.l_xgl = dq_Waypoint_[k-1].xgl;
                        prev_wpt_.ygl = current_wpt_.l_ygl = dq_Waypoint_[k-1].ygl;

                        prev_wpt_.x0gl = dq_Waypoint_[k-1].x0gl;
                        prev_wpt_.y0gl = dq_Waypoint_[k-1].y0gl;

                        prev_wpt_.qgl_deg = current_wpt_.l_qgl_deg = dq_Waypoint_[k-1].qgl_deg;
                        prev_wpt_.qfs_rad = dq_Waypoint_[k-1].qfs_rad;

                        if(k == 1){
                            prev_wpt_.l_xgl = dq_Waypoint_[k-1].xgl;
                            prev_wpt_.l_ygl = dq_Waypoint_[k-1].ygl;
                            prev_wpt_.l_qgl_deg = dq_Waypoint_[k-1].qgl_deg;
                        }
                        else{
                            prev_wpt_.l_xgl = dq_Waypoint_[k-2].xgl;
                            prev_wpt_.l_ygl = dq_Waypoint_[k-2].ygl;
                            prev_wpt_.l_qgl_deg = dq_Waypoint_[k-2].qgl_deg;
                        }
                    }

                    //next wpt
                    if(k < dq_Waypoint_.size()-1){

                        next_wpt_.num = cur_wpt_num_+1;
                        next_wpt_.xgl = dq_Waypoint_[k+1].xgl;
                        next_wpt_.ygl = dq_Waypoint_[k+1].ygl;
                        next_wpt_.x0gl = dq_Waypoint_[k+1].x0gl;
                        next_wpt_.y0gl = dq_Waypoint_[k+1].y0gl;
                        next_wpt_.qgl_deg = dq_Waypoint_[k+1].qgl_deg;
                        next_wpt_.qfs_rad = dq_Waypoint_[k+1].qfs_rad;
                    
                        next_wpt_.l_xgl = dq_Waypoint_[k].xgl;
                        next_wpt_.l_ygl= dq_Waypoint_[k].ygl;
                        next_wpt_.l_qgl_deg = dq_Waypoint_[k].qgl_deg; 
                        next_wpt_.type = dq_Waypoint_[k].type; 

                        if(k < dq_Waypoint_.size()-2){
                            next2_wpt_.num = cur_wpt_num_+2;
                            next2_wpt_.xgl = dq_Waypoint_[k+2].xgl;
                            next2_wpt_.ygl = dq_Waypoint_[k+2].ygl;
                            next2_wpt_.x0gl = dq_Waypoint_[k+2].x0gl;
                            next2_wpt_.y0gl = dq_Waypoint_[k+2].y0gl;
                            next2_wpt_.qgl_deg = dq_Waypoint_[k+2].qgl_deg;
                            next2_wpt_.qfs_rad = dq_Waypoint_[k+2].qfs_rad;
                    
                            next2_wpt_.l_xgl = dq_Waypoint_[k+1].xgl;
                            next2_wpt_.l_ygl= dq_Waypoint_[k+1].ygl;
                            next2_wpt_.l_qgl_deg = dq_Waypoint_[k+1].qgl_deg;                             
                        }

                    }
                    else{
                        next_wpt_ = next2_wpt_ = current_wpt_;
                        next_wpt_.num = cur_wpt_num_+1;
                        next2_wpt_.num = cur_wpt_num_+2;
                        
                    }
                }

                line_strip2_.points.push_back(lane_pp1_);
                line_strip2_.points.push_back(lane_pp2_);
                line_strip2_.points.push_back(lane_pp3_);
                line_strip2_.points.push_back(lane_pp4_);
                line_strip2_.points.push_back(lane_pp1_);

                line_strip3_.points.push_back(ego_pp1_);
                line_strip3_.points.push_back(ego_pp2_);
                line_strip3_.points.push_back(ego_pp3_);
                line_strip3_.points.push_back(ego_pp4_);
                line_strip3_.points.push_back(ego_pp1_);

                //line_strip4_
                line_strip4_.points.push_back(minFD_pp1_);
                line_strip4_.points.push_back(minFD_pp2_);

                current_wpt_.dist = dq_Waypoint_[k].dist;
                current_wpt_.refvel_cm = dq_Waypoint_[k].refvel_cm;
                current_wpt_.road_width = dq_Waypoint_[k].road_width;
                current_wpt_.stop = dq_Waypoint_[k].stop;
                current_wpt_.type = dq_Waypoint_[k].type;
                current_wpt_.etc = dq_Waypoint_[k].etc;

                if(k > 0){
                    prev_wpt_.dist = dq_Waypoint_[k-1].dist;
                    prev_wpt_.refvel_cm = dq_Waypoint_[k-1].refvel_cm;
                    prev_wpt_.road_width = dq_Waypoint_[k-1].road_width;
                    prev_wpt_.stop = dq_Waypoint_[k-1].stop;
                    prev_wpt_.type = dq_Waypoint_[k-1].type;
                    prev_wpt_.etc = dq_Waypoint_[k-1].etc;
                }
            }

            points_.points.push_back(p);
            if(dq_Waypoint_[k].stop > 0){
                if(dq_Waypoint_[k].type == WPT_TRAFFICLIGHT){
                    trafficLight_.points.push_back(p);
                }
                else{
                    stops_.points.push_back(p);
                }
            }

            if(dq_Waypoint_[k].type == WPT_QUEUE){
                queues_.points.push_back(p);
            }

            if(dq_Waypoint_[k].type == WPT_FOLLOW){
                follows_.points.push_back(p);
            }

            p.z = Z_WPT*2;
            line_strip_.points.push_back(p);
        }

        //Turning Circle
        
        if(turn_side_ != -1){
            
            double r = turn_radius_;
            int n_draw = 100;
            for (int i = 0; i <= n_draw; ++i) {
                double theta = 2.0 * M_PI * i / n_draw;
                geometry_msgs::msg::Point p_tc_fs;
                geometry_msgs::msg::Point p_tc_gl;

                p_tc_fs.x = r * std::cos(theta);
                
                if(turn_side_ == 0) p_tc_fs.y = r*(1.0 + std::sin(theta)); //Left
                else p_tc_fs.y = -r*(std::sin(theta)+1.0);// Right
                
                trans_FS_to_GL(p_tc_fs.x, p_tc_fs.y, &p_tc_gl.x, &p_tc_gl.y);
                
                p_tc_gl.z = Z_WPT*2;

                turning_circle_.points.push_back(p_tc_gl);
            }
            //printf("vis turning_circle \n");
        }

        /// depict  mark points
        if(mode_ == 0 || mode_ == 2){
            char strc_data[128];
            point3D tmp_mark;
            geometry_msgs::msg::Point p_mark;
            F_MARK_ = fopen(str_mark_file_.c_str(),"r");
            if(F_MARK_!=nullptr){
            while(fgets(strc_data,sizeof(strc_data),F_MARK_) != nullptr) {
                sscanf(strc_data,"%lf %lf %lf\n",
                    &tmp_mark.x,
                    &tmp_mark.y,
                    &tmp_mark.z);

                p_mark.x = tmp_mark.x;
                p_mark.y = tmp_mark.y;
                p_mark.z = tmp_mark.z;
             
                //printf("p_mark.x:%lf,p_mark.y:%lf,p_mark.z:%lf \n",p_mark.x,p_mark.y,p_mark.z);

                markpoints_.points.push_back(p_mark);
            }
            fclose(F_MARK_);
            }
        }
                
        ///
        
        if(mode_ == 0 || mode_ == 2){
            Markers_.markers.push_back(current_waypoint_);
            Markers_.markers.push_back(follow_waypoint_);
        }
        
        Markers_.markers.push_back(points_);
        Markers_.markers.push_back(stops_);
        Markers_.markers.push_back(queues_);
        Markers_.markers.push_back(follows_);
        Markers_.markers.push_back(trafficLight_);
        Markers_.markers.push_back(markpoints_);
        Markers_.markers.push_back(line_strip_);
        Markers_.markers.push_back(wpt_string_);
        Markers_.markers.push_back(line_strip2_);
        Markers_.markers.push_back(line_strip3_);
        Markers_.markers.push_back(line_strip4_);
        Markers_.markers.push_back(turning_circle_);

        pub_VisArray_->publish(Markers_);

        // 送信するメッセージ
        auto msg = std::make_unique<robo_msgs::msg::Waypoint>();

        msg->num = current_wpt_.num;
        msg->xgl = current_wpt_.xgl;
        msg->ygl = current_wpt_.ygl;
        msg->qgl_deg = current_wpt_.qgl_deg;
        msg->qfs_rad = current_wpt_.qfs_rad;
        msg->l_xgl = current_wpt_.l_xgl;
        msg->l_ygl = current_wpt_.l_ygl;
        msg->l_qgl_deg = current_wpt_.l_qgl_deg;
        msg->dist = current_wpt_.dist;
        msg->refvel_cm = current_wpt_.refvel_cm;
        msg->road_width = current_wpt_.road_width;
        msg->etc = current_wpt_.etc;
        msg->type = current_wpt_.type;
        msg->stop = current_wpt_.stop;

        pub_Waypoint_->publish(std::move(msg));

        double resilienceA = WptResilicence_;
        double resilienceB = 1.0-WptResilicence_;

        if(mode_== 0){
            //printf("WP:wpt_modifyOrder_ =%d, xAdjWpt_=%lf,yAdjWpt_=%lf\n",wpt_modifyOrder_,xAdjWpt_,yAdjWpt_ );
           #if 0 
            if(wpt_modifyOrder_ == 0){
                /*
                dq_Waypoint_[cur_wpt_num_].xgl = dq_Waypoint_[cur_wpt_num_].x0gl;
                dq_Waypoint_[cur_wpt_num_].ygl = dq_Waypoint_[cur_wpt_num_].y0gl;

                if(cur_wpt_num_ < dq_Waypoint_.size()-1){
                    dq_Waypoint_[cur_wpt_num_-1].xgl = dq_Waypoint_[cur_wpt_num_-1].x0gl;
                    dq_Waypoint_[cur_wpt_num_-1].ygl = dq_Waypoint_[cur_wpt_num_-1].y0gl;

                    dq_Waypoint_[cur_wpt_num_+1].xgl = dq_Waypoint_[cur_wpt_num_+1].x0gl;
                    dq_Waypoint_[cur_wpt_num_+1].ygl = dq_Waypoint_[cur_wpt_num_+1].y0gl;
                }    
                */
                dq_Waypoint_[cur_wpt_num_].xgl = resilienceB*dq_Waypoint_[cur_wpt_num_].xgl + resilienceA*dq_Waypoint_[cur_wpt_num_].x0gl;
                dq_Waypoint_[cur_wpt_num_].ygl = resilienceB*dq_Waypoint_[cur_wpt_num_].ygl + resilienceA*dq_Waypoint_[cur_wpt_num_].y0gl;

                if(cur_wpt_num_ > 0 && cur_wpt_num_ < dq_Waypoint_.size()-1){

                    dq_Waypoint_[cur_wpt_num_-1].xgl =  resilienceB*dq_Waypoint_[cur_wpt_num_-1].xgl + resilienceA*dq_Waypoint_[cur_wpt_num_-1].x0gl;
                    dq_Waypoint_[cur_wpt_num_-1].ygl =  resilienceB*dq_Waypoint_[cur_wpt_num_-1].ygl + resilienceA*dq_Waypoint_[cur_wpt_num_-1].y0gl;

                    dq_Waypoint_[cur_wpt_num_+1].xgl = resilienceB*dq_Waypoint_[cur_wpt_num_+1].xgl + resilienceA*dq_Waypoint_[cur_wpt_num_+1].x0gl;
                    dq_Waypoint_[cur_wpt_num_+1].ygl = resilienceB*dq_Waypoint_[cur_wpt_num_+1].ygl + resilienceA*dq_Waypoint_[cur_wpt_num_+1].y0gl;

                     //for(int n = 1;n < 3;n++){
                     //    dq_Waypoint_[cur_wpt_num_+n].xgl = resilienceB*dq_Waypoint_[cur_wpt_num_+n].xgl + resilienceA*dq_Waypoint_[cur_wpt_num_+n].x0gl;
                     //    dq_Waypoint_[cur_wpt_num_+n].ygl = resilienceB*dq_Waypoint_[cur_wpt_num_+n].ygl + resilienceA*dq_Waypoint_[cur_wpt_num_+n].y0gl;                    
                     //}
                }


            }
            else 
            #endif
            if(wpt_modifyOrder_ == 0 || wpt_modifyOrder_ == 1){

                dq_Waypoint_[cur_wpt_num_].xgl = resilienceA*dq_Waypoint_[cur_wpt_num_].xgl + resilienceB*dq_Waypoint_[cur_wpt_num_].x0gl;
                dq_Waypoint_[cur_wpt_num_].ygl = resilienceA*dq_Waypoint_[cur_wpt_num_].ygl + resilienceB*dq_Waypoint_[cur_wpt_num_].y0gl;

                if(cur_wpt_num_ > 0 && cur_wpt_num_ < dq_Waypoint_.size()-1){

                    dq_Waypoint_[cur_wpt_num_-1].xgl =  resilienceA*dq_Waypoint_[cur_wpt_num_-1].xgl + resilienceB*dq_Waypoint_[cur_wpt_num_-1].x0gl;
                    dq_Waypoint_[cur_wpt_num_-1].ygl =  resilienceA*dq_Waypoint_[cur_wpt_num_-1].ygl + resilienceB*dq_Waypoint_[cur_wpt_num_-1].y0gl;

                    dq_Waypoint_[cur_wpt_num_+1].xgl = resilienceA*dq_Waypoint_[cur_wpt_num_+1].xgl + resilienceB*dq_Waypoint_[cur_wpt_num_+1].x0gl;
                    dq_Waypoint_[cur_wpt_num_+1].ygl = resilienceA*dq_Waypoint_[cur_wpt_num_+1].ygl + resilienceB*dq_Waypoint_[cur_wpt_num_+1].y0gl;                
                }
            }
            else if(wpt_modifyOrder_ == 2){

                dq_Waypoint_[cur_wpt_num_].xgl += xAdjWpt_;
                dq_Waypoint_[cur_wpt_num_].ygl += yAdjWpt_;

                if(cur_wpt_num_ > 0 && cur_wpt_num_ < dq_Waypoint_.size()-1){
                    dq_Waypoint_[cur_wpt_num_-1].xgl += xAdjWpt_;
                    dq_Waypoint_[cur_wpt_num_-1].ygl += yAdjWpt_;

                    dq_Waypoint_[cur_wpt_num_+1].xgl += xAdjWpt_;
                    dq_Waypoint_[cur_wpt_num_+1].ygl += yAdjWpt_;                
                }
            }

            wpt_size_ =  dq_Waypoint_.size();
            if(wpt_modifyOrder_ > 0){
                wpt_modifyOrder_ = 0;
                xAdjWpt_ = 0;
                yAdjWpt_ = 0;
            }
        }
    };


    declare_parameter("mode",0);
    mode_ = this->get_parameter("mode").as_int();
    
    declare_parameter("Wpt_Change_Radius_Normal",0.5);
    declare_parameter("Wpt_Change_Radius_PosePoint",0.0);
    declare_parameter("Wpt_Change_Radius_Curve",0.1);
    
    Wpt_change_radius_normal_ = this->get_parameter("Wpt_Change_Radius_Normal").as_double();
    Wpt_change_radius_posePoint_ = this->get_parameter("Wpt_Change_Radius_PosePoint").as_double();
    Wpt_change_radius_curve_ = this->get_parameter("Wpt_Change_Radius_Curve").as_double();
     
    declare_parameter("FreeSpaceWidth",0.5);
    declare_parameter("AvoidanceDepthMergin",0.5);
    
    declare_parameter("AvoidanceMergin",0.4);
    declare_parameter("AvoidanceArea_Front",5.0);
    declare_parameter("AvoidanceArea_Back",2.0);
    declare_parameter("AvoidanceArea_Side",5.0);

    declare_parameter("Robo_Height",0.3);
    declare_parameter("Robo_HalfWidth",0.325);
    declare_parameter("Robo_ClosedFront",0.05);
    declare_parameter("Robo_ClosedRear",-0.5); 
    declare_parameter("Robo_FarmostFront",5.0);
    
    declare_parameter("EgoRegionLength",3.0);
    declare_parameter("MinHeight",Z_WPT);
    declare_parameter("MinHeightFar",-0.25);
    declare_parameter("MinHeightMid",-0.5);
    declare_parameter("MinHeightNear",-0.75);
    
    declare_parameter("ObjectMonitorWidth_cm",50);
    declare_parameter("Wpt_resilience",0.8);
    declare_parameter("AvoidMinDist",0.1);
    declare_parameter("Wpt_KeepTime",20);
    declare_parameter("Count_avoid_timer",30);

    declare_parameter("VisRoboWidth",0.65);
    declare_parameter("VisRoboLength",0.85);
    declare_parameter("VisRoboHeight",0.9);
    declare_parameter("VisRoboCenterLength",-0.25);
    declare_parameter("VisRoboCenterHeight",0.9);

    FreeSpaceWidth_ = this->get_parameter("FreeSpaceWidth").as_double();
    AvoidanceDepthMergin_ = this->get_parameter("AvoidanceDepthMergin").as_double();
    
    AvoidanceMergin_ = this->get_parameter("AvoidanceMergin").as_double();
    AvoidanceArea_Front_ = this->get_parameter("AvoidanceArea_Front").as_double();
    AvoidanceArea_Back_ = this->get_parameter("AvoidanceArea_Back").as_double();
    AvoidanceArea_Side_ = this->get_parameter("AvoidanceArea_Side").as_double();
    EgoRegionLength_ = this->get_parameter("EgoRegionLength").as_double();
    ObjectMonitorWidth_cm_ = this->get_parameter("ObjectMonitorWidth_cm").as_int();
    WptResilicence_ = this->get_parameter("Wpt_resilience").as_double();
    Wpt_KeepTime_ = this->get_parameter("Wpt_KeepTime").as_int();
    AvoidMinDist_= this->get_parameter("AvoidMinDist").as_double();
    Count_avoid_timer_ = this->get_parameter("Count_avoid_timer").as_int();

    Robo_HalfWidth_ = this->get_parameter("Robo_HalfWidth").as_double();
    Robo_ClosedFront_ = this->get_parameter("Robo_ClosedFront").as_double();
    Robo_ClosedRear_ = this->get_parameter("Robo_ClosedRear").as_double();
    Robo_FarmostFront_= this->get_parameter("Robo_FarmostFront").as_double();
    Robo_Height_ = this->get_parameter("Robo_Height").as_double();

    MinHeight_= this->get_parameter("MinHeight").as_double();
    MinHeightFar_= this->get_parameter("MinHeightFar").as_double();
    MinHeightMid_= this->get_parameter("MinHeightMid").as_double();
    MinHeightNear_= this->get_parameter("MinHeightNear").as_double();
   
    VisRoboWidth_ = this->get_parameter("VisRoboWidth").as_double();
    VisRoboLength_ = this->get_parameter("VisRoboLength").as_double();
    VisRoboHeight_ = this->get_parameter("VisRoboHeight").as_double();
    VisRoboCenterLength_ = this->get_parameter("VisRoboCenterLength").as_double();
    VisRoboCenterHeight_ = this->get_parameter("VisRoboCenterHeight").as_double();

    printf("WPS: mode=%d\n",mode_);
    printf("WPS: Wpt_change_radius_normal=%lf\n",Wpt_change_radius_normal_);
    printf("WPS: Wpt_change_radius_posePoint=%lf\n",Wpt_change_radius_normal_);

    printf("WPS: FreeSpaceWidth=%lf\n",FreeSpaceWidth_); 
    printf("WPS: AvoidanceMergin=%lf\n",AvoidanceMergin_);
    printf("WPS: AvoidanceArea_Front=%lf\n",AvoidanceArea_Front_);
    printf("WPS: AvoidanceArea_Back=%lf\n",AvoidanceArea_Back_);
    printf("WPS: AvoidanceArea_Side=%lf\n",AvoidanceArea_Side_);
    printf("WPS: AvoidanceDepthMergin=%lf\n",AvoidanceDepthMergin_);

    printf("WPS: EgoRegionLength=%lf\n",EgoRegionLength_);
    printf("WPS: MinHeight=%lf\n",MinHeight_);
    printf("WPS: MinHeightFar=%lf\n",MinHeightFar_);
    printf("WPS: MinHeightMid=%lf\n",MinHeightMid_);
    printf("WPS: MinHeightNear=%lf\n",MinHeightNear_);
    printf("WPS: ObjectMonitorWidth_cm=%d\n",ObjectMonitorWidth_cm_);
    printf("WPS: WptResilicence=%lf\n",WptResilicence_);
    printf("WPS: Wpt_KeepTime=%lf\n",Wpt_KeepTime_);
    printf("WPS: AvoidMinDist=%lf\n",AvoidMinDist_);
    printf("WPS: Robo_HalfWidth=%lf\n",Robo_HalfWidth_);
    printf("WPS: Robo_ClosedFront=%lf\n",Robo_ClosedFront_);
    printf("WPS: Robo_ClosedRear=%lf\n",Robo_ClosedRear_);
    printf("WPS: Robo_FarmostFront=%lf\n",Robo_FarmostFront_);
    printf("WPS: Robo_Height=%lf\n",Robo_Height_);

    Init_Visualization();
    Set_Feature_Waypoint();

    Wpt_change_radius_ = Wpt_change_radius_normal_;
    dist_range_for_front_ = AvoidanceArea_Front_;
    search_width_ = FreeSpaceWidth_;
    Wpt_Counter_ = 0;

    dq_Waypoint_.clear();

    if(mode_ == 1){ //Generate Waypoint

        declare_parameter("output_waypoint_file","waypoint.csv");
        str_output_waypoint_file_ = this->get_parameter("output_waypoint_file").as_string();
        printf("WPS: output_waypoint_file=%s\n",str_output_waypoint_file_.c_str());

        F_WAYPOINT_ = fopen(str_output_waypoint_file_.c_str(),"w");

    }
    else if(mode_ == 0 || mode_ == 2){ //Navigate mode or Edit Waypoint

        st_Waypoint tmp_wpt;
        char strc_data[256];

        declare_parameter("input_waypoint_file","waypoint.csv");
        str_input_waypoint_file_ = this->get_parameter("input_waypoint_file").as_string();

        F_WAYPOINT_ = fopen(str_input_waypoint_file_.c_str(),"r");
        while(fgets(strc_data,sizeof(strc_data),F_WAYPOINT_) != nullptr) {
            sscanf(strc_data,"%d,%f,%f,%f,%f,%f,%d\n",
                &tmp_wpt.num,
                &tmp_wpt.xgl,
                &tmp_wpt.ygl,
                &tmp_wpt.qgl_deg,
                &tmp_wpt.refvel_cm,
                &tmp_wpt.stop,
                &tmp_wpt.type);

                tmp_wpt.x0gl = tmp_wpt.xgl;
                tmp_wpt.y0gl = tmp_wpt.ygl;
                
                dq_Waypoint_.push_back(tmp_wpt);
        }
        if(mode_ == 0 || mode_ == 2){
            declare_parameter("mark_file","mark.csv");
            str_mark_file_ = this->get_parameter("mark_file").as_string();
        }

        //Goal
        if(dq_Waypoint_.size() > 0){
            dq_Waypoint_.back().type = WPT_GOAL;
        }
        
        mx_ = 0;
        my_ = 0;
        cmd_del_ = false;
        cmd_add_= false;
        cmd_stop_ = false;
        cmd_save_ = false;
        cur_wpt_num_ = 1;

        xAdjWpt_ = 0;
        yAdjWpt_ = 0;

        xGL_ = yGL_ = 0.0;
        flag_goal_ = false;

        wpt_size_ =  dq_Waypoint_.size();

    }

    init_nanosecond_ = -1000;
    Vel_mps_ = Vel_cmps_ = 0;
    dxGL_ = dyGL_ = dyawGL_ = 0;
    currentClock_ = 0;

    //printf("WP:init_nanosecond_=%lf \n",init_nanosecond_);

    // トピックの送信設定
    pub_Waypoint_ = create_publisher<robo_msgs::msg::Waypoint>("waypoint", rclcpp::QoS(1).reliable().durability_volatile());
    pub_VisArray_ = create_publisher<visualization_msgs::msg::MarkerArray>("vis_array", rclcpp::QoS(1).reliable().durability_volatile()); //!!!
    pub_clusters_ = create_publisher<visualization_msgs::msg::MarkerArray>("vis_cluster", rclcpp::QoS(1).reliable().durability_volatile());//!!!
    pub_Vectors_ = create_publisher<visualization_msgs::msg::MarkerArray>("vis_vector", rclcpp::QoS(1).reliable().durability_volatile());//!!!
    pub_InitPose_ = create_publisher<geometry_msgs::msg::Pose>("initPose", rclcpp::QoS(1).reliable().durability_volatile()); //!!!
    pub_movingObject_= create_publisher<visualization_msgs::msg::MarkerArray>("moving_objects_marker", rclcpp::QoS(1).reliable().durability_volatile());
    
    pub_RoboShape_ = create_publisher<vision_msgs::msg::BoundingBox3D>("/RoboShape", rclcpp::QoS(1).reliable().durability_volatile());
    pub_RoboPoseErrorCov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/RoboPose_error_cov", rclcpp::QoS(1).reliable().durability_volatile());
    
    pub_PointcloudTargetSelected_ = create_publisher<sensor_msgs::msg::PointCloud2>("/target_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());
    pub_ClusterPointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("cluster_pc", rclcpp::QoS(1).reliable().durability_volatile());
    
    pub_Followpoint_  = create_publisher<robo_msgs::msg::Freespace>("followpoint", rclcpp::QoS(1).reliable().durability_volatile());

    pub_PointcloudObject1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/object1_pointcloud", rclcpp::QoS(1).reliable().durability_volatile());
    
    // publish_messageのXミリ秒周期でのタイマー実行
    timer_ = create_wall_timer(100ms, depict_waypoint);

    sub_Joypad_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Waypoint_planner::cb_joypad, this, _1));
    sub_localization_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("pose_error_cov", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Waypoint_planner::cb_localization, this, _1));
    sub_target_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("object1_pointcloud", rclcpp::QoS(1).reliable().durability_volatile(), std::bind(&Waypoint_planner::cb_Target_Pointcloud, this, _1));//!!!
    
    sub_vehicle_ = this->create_subscription<robo_msgs::msg::Vehicle>("vehicle", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Waypoint_planner::cb_vehicle, this, _1));
    sub_scene_ = this->create_subscription<robo_msgs::msg::Scene>("scene", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Waypoint_planner::cb_scene, this, _1));    
    sub_area_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("area_map", rclcpp::QoS(1).transient_local(), std::bind(&Waypoint_planner::cb_area_map, this, _1)); //
    
    //sub_LowObjectCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lowobstacles/points", rclcpp::QoS(1).best_effort().durability_volatile(), std::bind(&Waypoint_planner::cb_LowObjectPointcloud, this, _1));

    flag_allow_update_ = true;
    key_overTake_ = 0;

    Set_ClusterColor();

    //初期位置設定
    if(mode_ == 0){ //Navigate mode

        current_wpt_ = dq_Waypoint_[0];
        cmd_initPosition_ = true;
        //Voice(WAV_SET_POSITION);

        geometry_msgs::msg::Pose pose;

        pose.position.x = current_wpt_.xgl;
        pose.position.y = current_wpt_.ygl;
        pose.position.z = Z_WPT;

        tf2::Quaternion Quat;
        Quat.setRPY( 0, 0, current_wpt_.qgl_deg*M_DEG2RAD);

        pose.orientation.w = Quat.getW();
        pose.orientation.x = Quat.getX();
        pose.orientation.y = Quat.getY();
        pose.orientation.z = Quat.getZ();

        pub_InitPose_->publish(std::move(pose));

        flag_allow_update_ = true;
    }
}

void Waypoint_planner::test(void){
    printf("testdest\n");
}

void Waypoint_planner::Set_ClusterColor(void){
    cluster_color_[0]=0xff0000; //red
    cluster_color_[1]=0x00ff00; //green
    cluster_color_[2]=0x0000ff; //blue
    cluster_color_[3]=0xff00ff; //magenta
    cluster_color_[4]=0x00ffff; //cyan
    cluster_color_[5]=0xffff00; //yellow
    cluster_color_[6]=0xffa500; //orange
    cluster_color_[7]=0x778899; //lightslategray
    cluster_color_[8]=0xffc0cb; //pink
    cluster_color_[9]=0x00ff7f; //springgreen
    cluster_color_[10]=0x4169e1;//royalblue
    cluster_color_[11]=0xee82ee;//violet
    cluster_color_[12]=0x191970;//midnightblue
    cluster_color_[13]=0xd2691e;//chocolate
    cluster_color_[14]=0xb0c4de;//lightsteelblue
    cluster_color_[15]=0x4b0082;//indigo
}
void Waypoint_planner::cb_joypad(const sensor_msgs::msg::Joy::SharedPtr msg){

    static bool key[10]={false};
    static float Vref=0,QradFS=0;
    static bool flag_manual = false;
    static int ii = 0;
    static int count= 0x00;
    static sensor_msgs::msg::Joy l_msg = *msg;

    count++;
    if(count > 0xff) count = 0;
#if 1
    if(msg->buttons[3] && l_msg.buttons[3] == 0){ //Y
        if(cur_wpt_num_ < dq_Waypoint_.size()-1) cur_wpt_num_ ++;
    }
    if(msg->buttons[0] && l_msg.buttons[0] == 0){//A
        if(cur_wpt_num_ >= 1) cur_wpt_num_ --;
    }
    
    if(msg->buttons[4] && l_msg.buttons[4]==0){ //LB
        if(cur_wpt_num_ > 10) cur_wpt_num_ -=10;
    }
    if(msg->buttons[5] && l_msg.buttons[5]==0){ //RB
        if(cur_wpt_num_ < dq_Waypoint_.size()-10) cur_wpt_num_ +=10;
    }

    if(mode_ == 2){

        mx_ = msg->axes[6]*0.05;
        my_ = -msg->axes[7]*0.05;

        if(msg->buttons[1] > 0 && l_msg.buttons[1]==0){ //Bボタン
            cmd_add_ = true;
        }
        if(msg->buttons[2] > 0 && l_msg.buttons[2]==0){ //Xボタン
            cmd_del_ = true;
        }
        if(msg->buttons[4] && l_msg.buttons[4]==0){ //LB
            if(cur_wpt_num_ > 10) cur_wpt_num_ -=10;
        }
        if(msg->buttons[5] && l_msg.buttons[5]==0){ //RB
            if(cur_wpt_num_ < dq_Waypoint_.size()-10) cur_wpt_num_ +=10;
        }
        if(msg->buttons[6] && l_msg.buttons[6]==0){ //mode
            cmd_stop_ = true;
        }
        if(msg->buttons[7] && l_msg.buttons[7]==0){ //Start
            cmd_trafficLight_ = true;
        }
        if(msg->buttons[8] && l_msg.buttons[8]==0){ //Logitech
            cmd_save_= true;
        }
        if(msg->buttons[9] && l_msg.buttons[9]==0){ //L-analog
            cmd_queue_= true;
        }
        if(msg->buttons[10] && l_msg.buttons[10]==0){ //R-analog
            cmd_follow_ = true;
        }
    }
    else{ //normal

        if(msg->buttons[1] > 0 && l_msg.buttons[1]==0){ //Bボタン
            cmd_mark_ = true;
            F_MARK_ = fopen(str_mark_file_.c_str(),"a+");       
            fprintf(F_MARK_,"%lf %lf %lf \n",xGL_,yGL_,zGL_);  

            //Voice(WAV_POINT_MARK);
            fclose(F_MARK_);
        }

        if(msg->buttons[6] && l_msg.buttons[6]==0){ //back
            
            cmd_initPosition_ = true;
            //Voice(WAV_SET_POSITION);

            geometry_msgs::msg::Pose pose;

            pose.position.x = current_wpt_.l_xgl;
            pose.position.y = current_wpt_.l_ygl;
            pose.position.z = Z_WPT;

            tf2::Quaternion Quat;
            Quat.setRPY( 0, 0, current_wpt_.l_qgl_deg*M_DEG2RAD);

            pose.orientation.w = Quat.getW();
            pose.orientation.x = Quat.getX();
            pose.orientation.y = Quat.getY();
            pose.orientation.z = Quat.getZ();

            pub_InitPose_->publish(std::move(pose));

            flag_allow_update_ = true;
        }
    }
#endif

    if(msg->buttons[1] > 0 && l_msg.buttons[1] == 0){ //Bボタン
        printf("B button, msg->buttons[1]=%d,l_msg.buttons[1]=%d\n",msg->buttons[1],l_msg.buttons[1]);
        //Voice(WAV_POINT_MARK);
    }

    l_msg = *msg;    
}

void Waypoint_planner::Edit_Waypoint(int k){

    st_Waypoint add_wpt;

    dq_Waypoint_[k].xgl += mx_;
    dq_Waypoint_[k].ygl += my_;

    if(cmd_del_ ){
        dq_Waypoint_.erase(dq_Waypoint_.begin()+k);
        cmd_del_ = false;
    }
    if(cmd_add_){
        add_wpt.num = static_cast<uint>(k+1);
        add_wpt.xgl = dq_Waypoint_[k].xgl + 1.0;
        add_wpt.ygl = dq_Waypoint_[k].ygl + 1.0;
        add_wpt.qgl_deg = dq_Waypoint_[k].qgl_deg;
        add_wpt.road_width = dq_Waypoint_[k].road_width;
        add_wpt.refvel_cm = 60.0;
        add_wpt.stop = 0;
        add_wpt.type = dq_Waypoint_[k].type;

        dq_Waypoint_.insert(dq_Waypoint_.begin()+k+1,add_wpt);

        if(k < dq_Waypoint_.size()-2)cur_wpt_num_ ++;

        cmd_add_ = false;
    }

    if(cmd_stop_){

        if(dq_Waypoint_[k].stop < 0.01){
            dq_Waypoint_[k].stop =1.0;
            printf("WP: Stop ON \n");
        }
        else{
            dq_Waypoint_[k].stop = 0.0;
             printf("WP: Stop OFF \n");
        }
        cmd_stop_ = false;
    }

    if(cmd_trafficLight_){

        if(dq_Waypoint_[k].stop < 0.01){
            dq_Waypoint_[k].stop =1.0;
            dq_Waypoint_[k].type = WPT_TRAFFICLIGHT;
            printf("WP: Stop TrafficLight ON \n");
        }
        else{
            dq_Waypoint_[k].stop = 0.0;
            dq_Waypoint_[k].type = 0;
            printf("WP: Stop TrafficLight OFF \n");
        }
        cmd_trafficLight_ = false;
    }

    if(cmd_follow_){
        
        if(dq_Waypoint_[k].type == 0) dq_Waypoint_[k].type = WPT_FOLLOW; //WPT追従
        else dq_Waypoint_[k].type = 0;
        
        cmd_follow_ = false;
    }

    if(cmd_queue_){
        
        if(dq_Waypoint_[k].type == 0) dq_Waypoint_[k].type = WPT_QUEUE; //行列
        else dq_Waypoint_[k].type = 0;
        
        cmd_queue_ = false;
    }


    if(cmd_save_){

        string str_newWaypoint;
        string str_date;
        char strc_date[64];
        char strc_system[256];

        std::time_t t = std::time(nullptr);
        std::tm* now = std::localtime(&t);

        sprintf(strc_date,"_%2.2d%2.2d%2.2d_%2.2d%2.2d.csv", (now->tm_year-100),(now->tm_mon+1),now->tm_mday,now->tm_hour,now->tm_min);
        str_date = strc_date;
        str_newWaypoint = str_input_waypoint_file_;
        str_newWaypoint.replace(str_newWaypoint.find(".csv"),4, strc_date);

        printf("WS:: Save Old as this File : %s \n", str_newWaypoint.c_str());
        printf("WS:: Save Waypoint File : %s \n", str_input_waypoint_file_.c_str());

        sprintf(strc_system,"cp %s %s \n",str_input_waypoint_file_.c_str(),str_newWaypoint.c_str() );
        system(strc_system);

        FILE* F_NEW_WPT = std::fopen(str_input_waypoint_file_.c_str(),"w");

        if(F_NEW_WPT !=nullptr){
            double qrad = 0;
            for(int k=0;k < dq_Waypoint_.size();k++){

                if(k==0)qrad = dq_Waypoint_[0].qgl_deg*M_PI/180.0;
                else {
                    qrad = atan2(dq_Waypoint_[k].ygl - dq_Waypoint_[k-1].ygl,dq_Waypoint_[k].xgl - dq_Waypoint_[k-1].xgl);
                }

                dq_Waypoint_[k].qgl_deg = qrad*180/M_PI;

                fprintf(F_NEW_WPT,"%d,%f,%f,%f,%f,%f,%d\n",
                        k,
                        dq_Waypoint_[k].xgl,
                        dq_Waypoint_[k].ygl,
                        dq_Waypoint_[k].qgl_deg,
                        dq_Waypoint_[k].refvel_cm,
                        dq_Waypoint_[k].stop,
                        dq_Waypoint_[k].type);
            }
            fclose(F_NEW_WPT);
            //Voice(WAV_MARK);
        }

        cmd_save_= false;
    }

}

void Waypoint_planner::cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
    
    static double lyawGL = yawGL_;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(rollGL_, pitchGL_, yawGL_);
    dyawGL_ = yawGL_ - lyawGL; 
    
    dxGL_   = msg->pose.pose.position.x - xGL_; 
    dyGL_   = msg->pose.pose.position.y - yGL_; 
    
    xGL_ = msg->pose.pose.position.x;
    yGL_ = msg->pose.pose.position.y;
    zGL_ = msg->pose.pose.position.z;
  
    //if(area_map_ready_){
        
        // int gx,gy;
        // if(worldToGrid(xGL_,yGL_,gx,gy)){

        //     int index = gy*area_map_.info.width + gx;
        //     int value = area_map_.data[index];

        //     printf("WP: index=%d, gx=%d, gy=%d, area_map_value: %d\n",index, gx,gy,value);
        // }
    
        int value = area_map_.valueWorld(xGL_,yGL_);
        //printf("WP: xGL=%lf, yGL=%lf, area_map_value: %d\n",xGL_,yGL_,value);
    //}

    //---- Robot Shape ---
    vision_msgs::msg::BoundingBox3D box;

    // 中心座標（位置）
    double xd = VisRoboCenterLength_*cos(yawGL_);
    double yd = VisRoboCenterLength_*sin(yawGL_);
    
    //VLP-16: 高さ（厚さ）: 約 72 mm（2.82 インチ）
    box.center.position.x = xGL_+xd;
    box.center.position.y = yGL_+yd;
    box.center.position.z = zGL_+ VisRoboCenterHeight_;0.8;//-0.818;//-0.8;//-0.9;//-0.828;

    // 向き（姿勢） — クォータニオン（単位姿勢）
    box.center.orientation.x = msg->pose.pose.orientation.x;
    box.center.orientation.y = msg->pose.pose.orientation.y;
    box.center.orientation.z = msg->pose.pose.orientation.z;
    box.center.orientation.w = msg->pose.pose.orientation.w;

    // サイズ [m]
    box.size.x = VisRoboLength_;   // 長さ
    box.size.y = VisRoboWidth_;   // 幅
    box.size.z = VisRoboHeight_;   // 高さ

    pub_RoboShape_->publish(std::move(box));
    
   //----- Robo Pose -----
   /*
    geometry_msgs::msg::PoseWithCovarianceStamped poseErrorCov;

    poseErrorCov.header.stamp = this->get_clock()->now();
    poseErrorCov.header.frame_id = "map";

    poseErrorCov.pose.pose.position.x = xGL_;
    poseErrorCov.pose.pose.position.y = yGL_;
    poseErrorCov.pose.pose.position.z = zGL_;

    poseErrorCov.pose.pose.orientation = msg->pose.pose.orientation;
    pub_RoboPoseErrorCov_->publish(std::move(poseErrorCov));
    */
    
    //----
    qgl_deg_ = yawGL_*M_RAD2DEG;
  
    static double Wpt_change_radius = Wpt_change_radius_;
    double distWpt = Dist_point_to_point(dq_Waypoint_[cur_wpt_num_].xgl,dq_Waypoint_[cur_wpt_num_].ygl,xGL_,yGL_);
    distWpt_ = distWpt;
    
    
    if(mode_ == 2) mode_ = 0; //Force set to Navigation mode

    if(mode_ == 0){ //Navigate mode cur_wpt_num_

        if(flag_allow_update_){
            //if(Judge_OverLine(xGL_,yGL_,dq_Waypoint_[cur_wpt_num_].xgl,dq_Waypoint_[cur_wpt_num_].ygl,dq_Waypoint_[cur_wpt_num_].qgl_deg) ||
            //    distWpt < Wpt_change_radius_){
            
            static int status_corner = -1;
            if(cur_wpt_num_ < dq_Waypoint_.size() - 2 && status_corner == -1){
                
                double dqrad = (dq_Waypoint_[cur_wpt_num_+1].qgl_deg - dq_Waypoint_[cur_wpt_num_].qgl_deg)*M_DEG2RAD;
                double diff_attitude_rad = fabs(modAngle(dqrad));                
                double threshold_dqrad = 15.0 * M_DEG2RAD;                
                
                if(dq_Waypoint_[cur_wpt_num_].stop > 0){
                    Wpt_change_radius = Wpt_change_radius_posePoint_;//[m]
                    status_corner = 0;
                }
                else if(cur_wpt_num_ > 1 && diff_attitude_rad > threshold_dqrad){
                    status_corner = 1;
                    Wpt_change_radius = Wpt_change_radius_curve_;//[m]
                    printf("WP:: Here corner!! Wpt_change_radius=%4.2lf\n",Wpt_change_radius);
                }
                else{
                    Wpt_change_radius = Wpt_change_radius_;
                    status_corner = 0;
                }
            }

            double dist_clossline = Dist_from_point_to_crossline(xGL_,yGL_,dq_Waypoint_[cur_wpt_num_-1].xgl,dq_Waypoint_[cur_wpt_num_-1].ygl,dq_Waypoint_[cur_wpt_num_].xgl,dq_Waypoint_[cur_wpt_num_].ygl);
            bool flag_Judge_OverLine = Judge_OverLine(xGL_,yGL_,dq_Waypoint_[cur_wpt_num_].xgl,dq_Waypoint_[cur_wpt_num_].ygl,dq_Waypoint_[cur_wpt_num_].qgl_deg) ;

            if( flag_Judge_OverLine || (( dist_clossline < Wpt_change_radius) && (distWpt < Wpt_change_radius))){

               printf("WP: flag_Judge_OverLine =%d, dist_clossline=%lf, distWpt=%lf\n",flag_Judge_OverLine,dist_clossline,distWpt);

                status_corner = -1;
                Wpt_change_radius = Wpt_change_radius_; //元に戻す
                
                if(cur_wpt_num_ >= dq_Waypoint_.size()-1){
                    flag_goal_ = true;
                }
                else{
                    cur_wpt_num_++;
                    printf(">>NextWPT=%d/%d::xGL=%lf,yGL=%lf \n",
                        cur_wpt_num_,
                        dq_Waypoint_.size(),
                        dq_Waypoint_[cur_wpt_num_].xgl,
                        dq_Waypoint_[cur_wpt_num_].ygl);
                }
            }
        }


        double l,r,w;
        if(area_map_.traversableWidth(dq_Waypoint_[cur_wpt_num_].x0gl, 
            dq_Waypoint_[cur_wpt_num_].y0gl, 
            dq_Waypoint_[cur_wpt_num_].qgl_deg*M_DEG2RAD, 5.0, l, r, w)){
            //printf("WP:: drivable_area_width = %lf [m]\n",w);
            drivable_area_width_ = w;
        }
        else{
            drivable_area_width_ = 5.0;
        }
    }
    else if(mode_ == 1){ //Waypoint Generate mode

        static st_Waypoint wpt;
        static double l_xgl = xGL_;
        static double l_ygl = yGL_;
        static double l_yaw = yawGL_;

        static bool flag_init_wpt =true;
        double dist_wpt = Dist_point_to_point(xGL_,yGL_,l_xgl,l_ygl);
        double cos_dyaw = fabs(cos(yawGL_-l_yaw));

        if(flag_init_wpt || dist_wpt > 3.0 || (dist_wpt > 0.25 && cos_dyaw < COS_30)){ // 30deg 以上

            wpt.num;
            wpt.xgl     = xGL_;
            wpt.ygl     = yGL_;
            wpt.qgl_deg = qgl_deg_;
            wpt.refvel_cm = 60;
            wpt.stop = 0;
            wpt.type = 0;
            wpt.etc = "-";

            dq_Waypoint_.push_back(wpt);

            static bool first=true;
            static double last_x;
            static double last_y;

            // 通過した領域を走行可能領域に設定する
            // area_map_.paintTrajectory(l_xgl,l_ygl,xGL_,yGL_,1.0);
            // area_map_.savePGM(
            //     "/home/june/data/test_pcd_all2/global_drivable_area_map_updated.pgm",
            //     "/home/june/data/test_pcd_all2/global_drivable_area_map_updated.yaml");

            l_xgl = xGL_;
            l_ygl = yGL_;
            l_yaw = yawGL_;

            if(flag_init_wpt) flag_init_wpt = false;

            fprintf(F_WAYPOINT_,"%d,%lf,%lf,%lf,%lf,%lf,%d\n",
                wpt.num,
                wpt.xgl,
                wpt.ygl,
                wpt.qgl_deg,
                wpt.refvel_cm,
                wpt.stop,
                wpt.type
            );

            wpt.num ++;

            printf("WP: %d::(%lf,%lf)\n",wpt.num,wpt.xgl,wpt.ygl);
        }

    }

    lyawGL = yawGL_;
}


//点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
double Waypoint_planner::Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2){

	double dist=0;
	double tmp1,tmp2;
	double q_rad;

    double dx = x2-x1;
    double dy = y2-y1;
    
    if(x2==x1){
        return 9999;
    }
    
    dist = fabs((dx*(y1-yp)+(x1-xp)*dy)/sqrt(dx*dx+dy*dy));

    return dist;
}

//点と点の距離
double Waypoint_planner::Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y){
  double dist =0;

  dist = (p1_x-p2_x)*(p1_x-p2_x) + (p1_y-p2_y)*(p1_y-p2_y);
  dist = sqrt(dist);

  return dist;
}

visualization_msgs::msg::Marker Waypoint_planner::makeMarker(
      const std::string &frame_id, const std::string &marker_ns,
      int marker_id,
      const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
      float r, float g, float b, float a)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();

    marker.ns = marker_ns;
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
    marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
    marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = max_pt.x() - min_pt.x();
    marker.scale.y = max_pt.y() - min_pt.y();
    marker.scale.z = max_pt.z() - min_pt.z();

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    //marker.lifetime = rclcpp::Duration(1, 1000000);

    return marker;
}

pcl::PointCloud<pcl::PointXYZI> Waypoint_planner::difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test)
{
    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

    //double resolution = 0.00001;//Octreeの解像度を指定
    double resolution = 0.1;//Octreeの解像度を指定

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);//Octreeを作成

    octree.setInputCloud (cloud_base);//元となる点群を入力
    octree.addPointsFromInputCloud ();

    octree.switchBuffers ();//バッファの切り替え

    octree.setInputCloud (cloud_test);//比較対象の点群を入力
    octree.addPointsFromInputCloud ();

    std::vector<int> newPointIdxVector;//

    octree.getPointIndicesFromNewVoxels (newPointIdxVector);//比較の結果差分と判断された点郡の情報を保管

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_diff (new pcl::PointCloud<pcl::PointXYZI> );//出力先


    //保管先のサイズの設定
    cloud_diff->width = cloud_base->points.size() + cloud_test->points.size();
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    //printf("WP:newPointIdxVector.size=%d\n",newPointIdxVector.size());

    int n = 0;//差分点群の数を保存する
    for(size_t i = 0; i < newPointIdxVector.size (); i++)
    {
        cloud_diff->points[i].x = cloud_test->points[newPointIdxVector[i]].x;
        cloud_diff->points[i].y = cloud_test->points[newPointIdxVector[i]].y;
        cloud_diff->points[i].z = cloud_test->points[newPointIdxVector[i]].z;
        cloud_diff->points[i].data[3]= (float)(newPointIdxVector[i]);
        n++;
    }
    //差分点群のサイズの再設定
    cloud_diff->width = n;
    cloud_diff->height = 1;
    cloud_diff->points.resize (cloud_diff->width * cloud_diff->height);

    return *cloud_diff;
}


int Waypoint_planner::ObjectClasifitaion(pcl::PointCloud<pcl::PointXYZI>::Ptr Pointcloud_basePcl, pcl::PointCloud<pcl::PointXYZI>::Ptr Pointcloud_targetPcl)
{
    //cloud_baseは元となる点群
    //cloud_testは比較対象の点群
    //cloud_diffは比較の結果差分とされた点群

    //1. Moving points
    double resolution = 0.1;//Octreeの解像度を指定

    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree (resolution);//Octreeを作成

    octree.setInputCloud (Pointcloud_basePcl);//元となる点群を入力
    octree.addPointsFromInputCloud ();

    octree.switchBuffers ();//バッファの切り替え

    octree.setInputCloud (Pointcloud_targetPcl);//比較対象の点群を入力
    octree.addPointsFromInputCloud ();

    std::vector<int> vec_MovingPointIndices;//

    octree.getPointIndicesFromNewVoxels(vec_MovingPointIndices);//比較の結果差分と判断された点郡の情報を保管

    for(size_t i = 0; i < vec_MovingPointIndices.size (); i++){
        Pointcloud_targetPcl->points[vec_MovingPointIndices[i]].data[0]=777;
        Pointcloud_targetPcl->points[vec_MovingPointIndices[i]].data[1]=777;
        Pointcloud_targetPcl->points[vec_MovingPointIndices[i]].data[2]=777;
        Pointcloud_targetPcl->points[vec_MovingPointIndices[i]].data[3]=777;
    }

    for(size_t i = 0; i < Pointcloud_targetPcl->points.size(); i++){
        printf("WP:%d data[0]=%lf \n",i,Pointcloud_targetPcl->points[i].data[0]);
        printf("WP:%d data[1]=%lf \n",i,Pointcloud_targetPcl->points[i].data[1]);
        printf("WP:%d data[2]=%lf \n",i,Pointcloud_targetPcl->points[i].data[2]);
        printf("WP:%d data[3]=%lf \n\n",i,Pointcloud_targetPcl->points[i].data[3]);
    }

    pcl::toROSMsg(*Pointcloud_targetPcl, PointcloudMovingObj_ros_);
    PointcloudMovingObj_ros_.header.frame_id = "cloud_matching";
    pub_MovingObjectPointcloud_ ->publish(std::move(PointcloudMovingObj_ros_));

    return 1;
}


int Waypoint_planner::Detect_MovingObject(
    const pcl::PointCloud<PointT>::Ptr& cloud_prev,
    const pcl::PointCloud<PointT>::Ptr& cloud_curr,
    std::vector<st_movingObject>& movingObject)
{
    // --- 1. 点群登録（ICPなどで姿勢整合） ---
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_curr);
    icp.setInputTarget(cloud_prev);
    pcl::PointCloud<PointT>::Ptr cloud_aligned(new pcl::PointCloud<PointT>);
    icp.align(*cloud_aligned);

    if (!icp.hasConverged()) return -1;

    // --- 2. 差分点群を抽出 ---
    pcl::PointCloud<PointT>::Ptr moving_points(new pcl::PointCloud<PointT>);
    const float distance_threshold = 0.3f;  // 動的判定閾値
    for (size_t i = 0; i < cloud_aligned->points.size(); ++i) {
        const auto& p = cloud_aligned->points[i];
        // 最近傍探索などを厳密に行っても良い（簡略化）
        if (i < cloud_prev->points.size()) {
            const auto& q = cloud_prev->points[i];
            float dist = std::sqrt((p.x - q.x)*(p.x - q.x) + 
                                   (p.y - q.y)*(p.y - q.y) + 
                                   (p.z - q.z)*(p.z - q.z));
            if (dist > distance_threshold)
                moving_points->points.push_back(p);
        }
    }

    // --- 3. クラスタリングで移動物体分離 ---
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(moving_points);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    //ec.setClusterTolerance(0.5); // クラスタ間距離閾値
    ec.setClusterTolerance(1.0); // クラスタ間距離閾値
    
    //ec.setMinClusterSize(20);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(moving_points);
    ec.extract(cluster_indices);

    // --- 4. クラスタごとの方向推定 ---
    st_movingObject mo;
    //directions_out.clear();
    movingObject.clear();
    double cosGL = cos(yawGL_);
    double sinGL = sin(yawGL_);
    
    for (const auto& indices : cluster_indices) {
        Eigen::Vector3f prev_center(0,0,0), curr_center(0,0,0);
        for (int idx : indices.indices) {
            //auto& p = cloud_aligned->points[idx];
            pcl::PointXYZI p;
            auto& pp = cloud_aligned->points[idx];
            p.x = (pp.x*cosGL - pp.y*sinGL) + xGL_;
            p.y = (pp.x*sinGL + pp.y*cosGL) + yGL_;
            p.z = pp.z;
            //p.x = pp.x*cosGL + xGL_; 
            //p.y = pp.y*sinGL + yGL_; 

            curr_center += Eigen::Vector3f(p.x, p.y, p.z);
        }
        curr_center /= indices.indices.size();
        // 対応クラスタの前フレーム重心を探す場合はトラッキングが必要
        // ここでは近似的に同座標を仮定
        prev_center = curr_center - Eigen::Vector3f(0, 0, 0.1); // 仮の移動方向
        
        mo.direction = curr_center - prev_center;
        mo.position = curr_center;

        movingObject.push_back(mo);
        //directions_out.push_back(curr_center - prev_center);
    }

    return 1;
}

void Waypoint_planner::Visualize_movingObject(void){

    visualization_msgs::msg::MarkerArray marker_array;
    const double arrow_length_scale = 10.0;

    int id = 0;
    for (size_t i = 0; i < movingObject_.size(); ++i){

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "moving_objects";
        marker.id = id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = movingObject_[i].position(0);
        p_start.y = movingObject_[i].position(1);
        p_start.z = 0;//movingObject_[i].position(2);

        Eigen::Vector3f end_point = movingObject_[i].position + arrow_length_scale * movingObject_[i].direction.normalized();
        p_end.x = end_point(0);
        p_end.y = end_point(1);
        p_end.z = 0;//end_point(2);

        printf("WP::(%lf,%lf),(%lf,%lf),(%lf,%lf)\n",p_start.x,p_end.x,p_start.y,p_end.y,p_start.z,p_end.z);

        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        marker.scale.x = 0.1;  // 矢印の軸の太さ
        marker.scale.y = 0.2;  // 矢印の頭の太さ
        marker.scale.z = 0.2;  // 矢印の頭の長さ

        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(marker);
    }

    pub_movingObject_->publish(std::move(marker_array));
}


void Waypoint_planner::cb_vehicle(const robo_msgs::msg::Vehicle::SharedPtr msg) {

    Vel_cmps_   = msg->v;
    Vel_mps_ = Vel_cmps_*0.01;
    //printf("WP::Vel_mps_=%4.2lf \n",Vel_mps_);
}

void Waypoint_planner::cb_scene(const robo_msgs::msg::Scene::SharedPtr msg) {   
    scene_ = msg->scene;
    //printf("Scene=%d\n",scene_);
}

int Waypoint_planner::cb_Aligned_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

}

int Waypoint_planner::cb_Target_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){

    int key_detectObject = 0;
    int key_overTake = 1;
    static double l_sec=0;
    
    //std::cout << "cb_Target_Pointcloud" << std::endl;
    double time_TargetObject  = rclcpp::Time(cloud_msg->header.stamp).seconds();

    ClusterVectorMarkers_.markers.clear();
    double ts = this->get_clock()->now().seconds();

    if((*cloud_msg).data.size() > 0){

        pcl::fromROSMsg(*cloud_msg, PointcloudTarget_pcl_);
                
        double sec = this->get_clock()->now().seconds();

        dtCloud_ = sec-l_sec;
        currentClock_ = sec;
        
        l_sec=sec;
        key_detectObject = Detect_Object();

    }

   
    //printf("WP: key_overTake=%d \n",key_overTake);
    //前方にEgoRobotより遅い移動対象
    if(current_waypoint_.type == WPT_FOLLOW){
        key_overTake = 0; //追い越さない
    }
    // else{
    //     key_overTake = 1;
    // }

    /// Set Lane, Left or Right ///
    if(key_detectObject > 0){
        if(key_overTake > 0 && current_wpt_.type < WPT_QUEUE && next_wpt_.type < WPT_QUEUE && current_wpt_.stop < 1.0){ //WPT_NORMAL or CURV
            
            //printf("WP:: Vel_cmps_= %lf [mps] \n",Vel_cmps_);
            if(Vel_cmps_ > 20.0) { //[cm/sec]
                Search_Lane(); //Lane Change
            }
        }
        else{//WPT_QUEUE,WPT_FOLLOW
            if(minLeftDist_ > minRightDist_){
                lane_side_ = 0; //Left
            }    
            else{
                lane_side_ = 1; //Right
            }
        }
    }

    /// Search Free Space to Set SubWP ///
    //if(key_overTake > 0){
        Set_FreespacePoint();
    //}

    Depict_EgoRegion();
    Publish_Freespace_FollowPoint();
    
    return 1;
}


int Waypoint_planner::Detect_Object(void){

    static int count = 0;
    pcl::_PointXYZI pct;
    count++;
    //visualization_msgs::msg::MarkerArray marker_array;

    if(PointcloudTarget_pcl_.makeShared()==nullptr) return 0;

    PointcloudTargetSelected_Prev_pcl_ = PointcloudTargetSelected_pcl_;

    PointcloudTarget_pcl_.header = PointcloudTarget_pcl_.header;
    PointcloudTargetSelected_pcl_.clear();

    // KD木を作っておく。近傍点探索とかが早くなる。
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZI>);
    if(PointcloudTarget_pcl_.size()<=0) return 0;

    tree->setInputCloud(PointcloudTarget_pcl_.makeShared());

    //近傍点探索に使うパラメータと結果が入る変数
    double radius = AvoidanceArea_Front_;  //半径r
    
    std::vector<int> k_indices;  //範囲内の点のインデックスが入る
    std::vector<float> k_sqr_distances;  //範囲内の点の距離が入る
    //unsigned int max_nn = 5000;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    //unsigned int max_nn = 10000;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない
    unsigned int max_nn = 20000;  //何点見つかったら探索を打ち切るか。0にすると打ち切らない

    pcl::PointXYZI p;  //中心座標
    p.x = 0;
    p.y = 0;
    p.z = 0;

    //半径r以内にある点を探索
    k_indices.clear();
    tree->radiusSearch(p, radius, k_indices, k_sqr_distances, max_nn);
    
    //printf("WP: k_indices.size()=%d \n", k_indices.size());

    double min_dist = 9999;
    double dist = 0;

    double minFrontDist = 9999;
    double minLeftDist = 9999;
    double minRightDist = 9999;
    
    double tempFrontDist = 0;
    double tempLeftDist = 0;
    double tempRightDist = 0;
        
    pcl::PointXYZI tempMinPt;
    pcl::PointXYZI tempMinLeftPt;
    pcl::PointXYZI tempMinRightPt;
    
    pcl::PointXYZI pt;
    pcl::PointXYZRGB pt_rgb;
    point3D ptObj;

    double qfs_rad=0;
    
    //waypointに設定すること
    qfs_rad = atan2(current_wpt_.ygl-yGL_,current_wpt_.xgl-xGL_);
    tmp_followPoint_.xgl = current_wpt_.xgl;
    tmp_followPoint_.ygl = current_wpt_.ygl;
    tmp_followPoint_.qgl_rad = modAngle(qfs_rad + yawGL_);
    
    tmp_followPoint_.xfs = current_wpt_.xgl - xGL_;
    tmp_followPoint_.yfs = current_wpt_.ygl - yGL_;
    tmp_followPoint_.qgl_rad = qfs_rad;
    
    tmp_followPoint_.dist_follow = Dist_point_to_point(0,0,tmp_followPoint_.xfs,tmp_followPoint_.yfs);

    if(k_indices.size() == 0){             
        tmp_followPoint_.dist_front_object = minFrontDist;
        return 0;
    }
    
    vec_Object_.clear();
    double heightThreshold = -0.75;
    double distTarget = 0;
    
    double AvoidanceArea_Back = AvoidanceArea_Back_;
    //251130
    if(current_wpt_.type == WPT_QUEUE || current_wpt_.type == WPT_TRAFFICLIGHT ||  prev_wpt_.type == WPT_TRAFFICLIGHT){
        AvoidanceArea_Back = 0;
    }
    
    for(uint k=0;k < k_indices.size();k++ ){

        pt = PointcloudTarget_pcl_.points[k_indices[k]];        
        distTarget = sqrt(pt.x*pt.x + pt.y*pt.y);
        //printf("distTarget=%lf\n",distTarget);
        
        if(distTarget <= 2.0) heightThreshold = MinHeightNear_;
        else if(distTarget > 2.0 && distTarget < 3.0) heightThreshold = MinHeightMid_;  
        else heightThreshold = MinHeightFar_;
    
        if(pt.x < -AvoidanceArea_Back || pt.x > AvoidanceArea_Front_ || pt.y < -AvoidanceArea_Side_ || pt.y > AvoidanceArea_Side_ || pt.z < heightThreshold) continue;
        
        PointcloudTargetSelected_pcl_.push_back(pt); 
    
        //前方   Robo_ClosedFront: -0.5 #fina1  #Range of minDist
        if(pt.x > 0 && pt.x > Robo_ClosedFront_ && pt.x < Robo_FarmostFront_ && pt.y > -Robo_HalfWidth_ && pt.y < Robo_HalfWidth_ && pt.z < Robo_Height_) {

            tempFrontDist = sqrt(pt.x*pt.x + pt.y*pt.y);
            if(minFrontDist > tempFrontDist){
                minFrontDist = tempFrontDist;
                tempMinPt = pt;
            }
        }
        //左方 
        if(pt.x > Robo_ClosedRear_ &&  pt.x < min(minFrontDist_,Robo_FarmostFront_) && pt.y > Robo_HalfWidth_ && pt.y < AvoidanceArea_Side_ && pt.z < Robo_Height_) {

            tempLeftDist = sqrt(pt.x*pt.x + pt.y*pt.y);
            if(minLeftDist > tempLeftDist){
                minLeftDist = tempLeftDist;
                tempMinLeftPt = pt;
            }
        }

        //右方
        if(pt.x > Robo_ClosedRear_ && pt.x < min(minFrontDist_,Robo_FarmostFront_) && pt.y < -Robo_HalfWidth_ && pt.y > -AvoidanceArea_Side_ && pt.z < Robo_Height_) {

            tempRightDist = sqrt(pt.x*pt.x + pt.y*pt.y);
            if(minRightDist > tempRightDist){
                minRightDist = tempRightDist;
                tempMinRightPt = pt;
            }
        }

        //fprintf(F_OBJ,"%lf %lf %lf\n",pt.x,pt.y,pt.z);
        
        ptObj.x = pt.x;
        ptObj.y = pt.y;
        ptObj.z = pt.z;

        vec_Object_.push_back(ptObj);
        //printf("point[%d] of (%lf, %lf) -> x: %lf, y:%lf, z:%lf \n", k, p.x, p.y, result.x, result.y, result.z);
    }

    pcl::toROSMsg(PointcloudTargetSelected_pcl_, PointcloudTargetSelected_ros_); 

    PointcloudTargetSelected_ros_.header.frame_id = "/cloud_matching";
    pub_PointcloudTargetSelected_->publish(std::move(PointcloudTargetSelected_ros_));

    minFrontPt_ = tempMinPt;
    
    minFrontDist_ = minFrontDist;
    minLeftDist_  = minLeftDist;
    minRightDist_ = minRightDist;

    tmp_followPoint_.dist_front_object = minFrontDist_;
    
    relVel_frontObject_= Identify_MovingObject();
    tmp_followPoint_.relvel_front_object = relVel_frontObject_;
 
    return 1;
}

double Waypoint_planner::Get_Clock(void){
    double ts = 0;
    
    if(init_nanosecond_ <= 0)init_nanosecond_ = this->get_clock()->now().nanoseconds();
    ts = (this->get_clock()->now().nanoseconds() - init_nanosecond_)/1000000000; //[s]
    

    return ts;
}


double Waypoint_planner::Identify_MovingObject(void){

    static deque <time_val> dq_minDist;
    double relVel = 999.0;
    time_val tv;

    //if(init_nanosecond_ <= 0)init_nanosecond_ = this->get_clock()->now().nanoseconds();
    //tv.ts = (this->get_clock()->now().nanoseconds() - init_nanosecond_)/1000000000; //[s]
    tv.ts = Get_Clock();
    tv.val = minFrontDist_;

    //printf("WP: tv.ts:%lf \n",tv.ts);

    dq_minDist.push_back(tv);
    if(dq_minDist.size() > 5) dq_minDist.pop_front();

    if(dq_minDist.size() > 1){
        
        Least_squares_method lsm;
        
        relVel = lsm.slope(dq_minDist);//mps
        if(relVel > 5.0) relVel = 5.0;
        else if(relVel < -5.0) relVel = -5.0;
         
        //printf("WP: Identify moving object: %lf[m], %lf \n",minFrontDist_,velrel);
    }


    return relVel;
}

int Waypoint_planner::Search_Lane(void){

    //printf("WP: Search_Lane():1, vec_Object_.size()=%d\n",vec_Object_.size());
    double yawWpt=0;

    double cos_gl=cos(yawGL_);
    double sin_gl=sin(yawGL_);

    double xwpt_fs=0, ywpt_fs=0, zwpt_fs=0;

    double x2wpt_fs=0, y2wpt_fs=0, z2wpt_fs=0;
    double x1wpt_fs=0, y1wpt_fs=0, z1wpt_fs=0;

    double x1org=0, y1org=0;
    double x2org=0, y2org=0;

    double x1=0, y1=0;
    double x2=0, y2=0;

    double x1org_fs=0, y1org_fs=0;
    double x2org_fs=0, y2org_fs=0;

    double x1fs=0, y1fs=0;
    double x2fs=0, y2fs=0;

    double dist_p2p=0;
    
    double extra_distWpt = 1.5;
    
    double distMeasure = min(Robo_FarmostFront_,minFrontDist_+1.5); //251110
    
    if(distMeasure < 1.5) distMeasure = 1.5; //261004
    
    if(cur_wpt_num_ > 0 && cur_wpt_num_ < wpt_size_-1){

        //printf("WP: Search_Lane():2\n");

        x1org = prev_wpt_.x0gl-xGL_;
        y1org = prev_wpt_.y0gl-yGL_;

        x1 = prev_wpt_.xgl-xGL_;
        y1 = prev_wpt_.ygl-yGL_;

        if(distWpt_ < 5.0){ 
            x2org = next_wpt_.x0gl-xGL_;
            y2org = next_wpt_.y0gl-yGL_;
            x2 = next_wpt_.xgl-xGL_;
            y2 = next_wpt_.ygl-yGL_;
        }
        else{ //無効化
            x2org = current_wpt_.x0gl-xGL_;
            y2org = current_wpt_.y0gl-yGL_;
            x2 = current_wpt_.xgl-xGL_;
            y2 = current_wpt_.ygl-yGL_; 
        }

        yawWpt = atan2(y2-y1,x2-x1);
        cos_gl = cos(yawGL_);
        sin_gl = sin(yawGL_);

        x1org_fs = cos_gl*x1org + sin_gl*y1org;
        y1org_fs = -sin_gl*x1org + cos_gl*y1org;

        x2org_fs = cos_gl*x2org + sin_gl*y2org;
        y2org_fs = -sin_gl*x2org + cos_gl*y2org;

        x1fs = cos_gl*x1 + sin_gl*y1;
        y1fs = -sin_gl*x1 + cos_gl*y1;

        x2fs = cos_gl*x2 + sin_gl*y2;
        y2fs = -sin_gl*x2 + cos_gl*y2;

    }

    int NumObjectPt = 0;
    int NumObjectPtL = 0;
    int NumObjectPtR = 0;
    int NumObjectPtOrg = 0;

    double tmp_xAdjWpt = xAdjWpt_;
    double tmp_yAdjWptL = 0;
    double tmp_yAdjWptR = 0;

    point3D ptObj;

    bool flag_Left=false;
    bool flag_Right=false;


    double distObj=0;

    double xAdjWptR = 0;
    double yAdjWptR = 0;
    double xAdjWptL = 0;
    double yAdjWptL = 0;

    double distL = 9999;
    double distR = 9999;
    
    double ObjectMonitorWidth_cm = ObjectMonitorWidth_cm_;
    
    if(drivable_area_width_  < 5.0){
        ObjectMonitorWidth_cm = drivable_area_width_*10; //[cm]
    }
    
    if(vec_Object_.size()>0){

        //Original path
        for(uint k=0;k < vec_Object_.size();k++ ){
            ptObj = vec_Object_[k];
            dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1org_fs,y1org_fs,x2org_fs,y2org_fs));
            distObj = Dist_point_to_point(0,0,ptObj.x,ptObj.y);
            
            if(ptObj.x > 0 && distObj > AvoidMinDist_&& distObj < distMeasure && dist_p2p < AvoidanceMergin_){
                NumObjectPtOrg++;
                //printf("WP: Search_Lane():2.5 dist_p2p = %lf, ptObj.x=%lf\n",dist_p2p,ptObj.x);                
                break;
            }
        }

        if(NumObjectPtOrg > 0){
        
            //Current path
            for(uint k=0;k < vec_Object_.size();k++ ){
                ptObj = vec_Object_[k];
                dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs,y1fs,x2fs,y2fs));
                distObj = Dist_point_to_point(0,0,ptObj.x,ptObj.y);
            
                if(ptObj.x > 0 && distObj > AvoidMinDist_&& distObj < distMeasure && dist_p2p < AvoidanceMergin_){
                    NumObjectPt++;
                    //printf("WP: Current Path Search_Lane():2.5.1 dist_p2p = %lf, ptObj.x=%lf\n",dist_p2p,ptObj.x);                
                    break;
                }
            }

            //printf("WP: Search_Lane():3 NumObjectPt = %d \n",NumObjectPt);

            if(NumObjectPt > 0){

                for(uint i=0;i < ObjectMonitorWidth_cm;i++ ){ 

                    // --- Right ----
                    tmp_yAdjWptR = -0.01*i;//[cm]->[m]
                    //tmp_yAdjWptR = -0.005*i;//[cm]->[m]
                    
                    NumObjectPtR = 0;
                    for(uint k=0;k < vec_Object_.size();k++ ){

                        ptObj = vec_Object_[k];
                    
                        dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs,y1fs+tmp_yAdjWptR,x2fs,y2fs+tmp_yAdjWptR));
                        distObj = Dist_point_to_point(0,0,ptObj.x,ptObj.y);

                        if(ptObj.x > 0 && distObj > AvoidMinDist_ && distObj < distMeasure && dist_p2p < AvoidanceMergin_ ){
                            NumObjectPtR ++;
                            //printf("WP: Right Search_Lane():3 %d NumObjectPtR = %d dist_p2p=%lf \n",i,NumObjectPtR,dist_p2p);
                            break;
                        }
                    }

                    //printf("WP: Search_Lane():3.1 NumObjectPtR = %d / %d \n",NumObjectPtR,i);

                    if(NumObjectPtR == 0){
                        //printf("WP:Right(%d)=%lf\n",i,dist_p2p);

                        double tmp2_xAdjWptR = cos_gl*tmp_xAdjWpt - sin_gl*tmp_yAdjWptR;
                        double tmp2_yAdjWptR = sin_gl*tmp_xAdjWpt + cos_gl*tmp_yAdjWptR;
                        //printf("WP: Right Search_Lane():3.1.1 tmp2_yAdjWptR = %lf -> %lf: ptObj.y=%lf \n",tmp2_yAdjWptR,y1fs+tmp2_yAdjWptR,ptObj.y);
                                           
                        if(flag_Right==false){
                            
                            if(area_map_.valueWorld(dq_Waypoint_[cur_wpt_num_].xgl+tmp2_xAdjWptR,dq_Waypoint_[cur_wpt_num_].ygl+tmp2_yAdjWptR) == 0){
                                xAdjWptR = tmp2_xAdjWptR;
                                yAdjWptR = tmp2_yAdjWptR;
                                distR = sqrt(xAdjWptR*xAdjWptR + yAdjWptR*yAdjWptR);
                                
                                flag_Right=true;
                                
                                break;
                            }
                            else{
                                //printf("WP: Right :: Out of Area \n");
                            }                            
                        } 
                    }
                }

                //-- Left --
                for(uint i=0;i < ObjectMonitorWidth_cm;i++ ){ 
                
                    tmp_yAdjWptL = 0.01*i;
                    
                    NumObjectPtL = 0;
                    for(uint k=0;k < vec_Object_.size();k++ ){

                        ptObj = vec_Object_[k];
                        dist_p2p = fabs(Dist_from_point_to_line2(ptObj.x,ptObj.y,x1fs,y1fs+tmp_yAdjWptL,x2fs,y2fs+tmp_yAdjWptL));
                        distObj = Dist_point_to_point(0,0,ptObj.x,ptObj.y);                    
                    
                        if(ptObj.x > 0 && distObj > AvoidMinDist_ && distObj < distMeasure && dist_p2p < AvoidanceMergin_ ){
                            NumObjectPtL ++;
                            //printf("WP: Left Search_Lane():3 %d NumObjectPt = %d dist_p2p=%lf \n",i,NumObjectPtL,dist_p2p);
                            break;
                        }
                    }

                    //printf("WP: Search_Lane():3.2 NumObjectPtL = %d / %d \n",NumObjectPtL,i);

                    if(NumObjectPtL == 0){
 
                        double tmp2_xAdjWptL = cos_gl*tmp_xAdjWpt - sin_gl*tmp_yAdjWptL;
                        double tmp2_yAdjWptL = sin_gl*tmp_xAdjWpt + cos_gl*tmp_yAdjWptL;
                        //printf("WP: Left Search_Lane():3.2.1 tmp2_yAdjWptL = %lf -> %lf: ptObj.y=%lf \n",tmp2_yAdjWptL,y1fs+tmp2_yAdjWptL,ptObj.y);

                        if(flag_Left==false ){
 
                            if(area_map_.valueWorld(dq_Waypoint_[cur_wpt_num_].xgl+tmp2_xAdjWptL,dq_Waypoint_[cur_wpt_num_].ygl+tmp2_yAdjWptL) == 0){
                                
                                xAdjWptL = tmp2_xAdjWptL;
                                yAdjWptL = tmp2_yAdjWptL;
                        
                                distL = sqrt(xAdjWptL*xAdjWptL + yAdjWptL*yAdjWptL);            
                                
                                flag_Left=true;
                                
                                break;
                            }
                            else{
                                //printf("WP: Left :: Out of Area \n");
                            }
                        } 
                    }
                }
            }
        }
    }

    if(NumObjectPtOrg > 0){

        if(flag_Left == true && flag_Right == false){ //左側
            xAdjWpt_ = xAdjWptL;
            yAdjWpt_ = yAdjWptL;
            lane_side_ = 0; //L //251130
            //printf("dL: dL=%lf,lane_side=%d,NumObjectPtL=%d,NumObjectPtR=%d, xAdjWpt_=%lf, yAdjWpt_=%lf \n",
            //    lane_side_,distL,NumObjectPtL,NumObjectPtR,xAdjWpt_,yAdjWpt_);
        }
        else if(flag_Left == false && flag_Right == true){//右側
            xAdjWpt_ = xAdjWptR;
            yAdjWpt_ = yAdjWptR;
            lane_side_ = 1; //R //251130
            //printf("dR: dR=%lf,lane_side=%d,NumObjectPtL=%d,NumObjectPtR=%d, xAdjWpt_=%lf, yAdjWpt_=%lf \n",
            //    lane_side_,distR,NumObjectPtL,NumObjectPtR,xAdjWpt_,yAdjWpt_);
        }
        else{
            if(distL < distR){
                xAdjWpt_ = xAdjWptL;
                yAdjWpt_ = yAdjWptL;
                lane_side_ = 0; //L //251130
                //printf("lane_side=%d,dL < dR: dL=%lf,dR=%lf,NL=%d,NR=%d \n",lane_side_,distL,distR,NumObjectPtL,NumObjectPtR);
            }
            else{
                xAdjWpt_ = xAdjWptR;
                yAdjWpt_ = yAdjWptR;
                //printf("lane_side=%d,dL >= dR: dL=%lf,dR=%lf,NL=%d,NR=%d  \n",lane_side_,distL,distR,NumObjectPtL,NumObjectPtR);
                lane_side_ = 1;//R
            }
        }

    }

    if(--Wpt_Counter_ < 0) Wpt_Counter_ = 0;

    if(scene_ <= 1){    
        wpt_modifyOrder_ = -1;
    }
    else if(cur_wpt_num_ < dq_Waypoint_.size() && 
                (dq_Waypoint_[cur_wpt_num_].stop < 1.0 || dq_Waypoint_[cur_wpt_num_+1].stop < 1.0)){
        if(NumObjectPtOrg == 0 && Wpt_Counter_ <= 0){
            wpt_modifyOrder_ = 0;
        }
        else if(NumObjectPt == 0){
            wpt_modifyOrder_ = 1;
        }
         else{   
            wpt_modifyOrder_ = 2;
            Wpt_Counter_ = Wpt_KeepTime_;
        }
    }
    //printf("wpt_modifyOrder_=%d, Wpt_Counter_=%d\n",wpt_modifyOrder_,Wpt_Counter_);

    distAoid_ = distMeasure;

    return 1;
}

int Waypoint_planner::Depict_EgoRegion(void){

    geometry_msgs::msg::Point p1;
    double x1,y1,x2,y2;
    double xp,yp,xp1,yp1,xp2,yp2;
    double xpp1,ypp1,xpp2,ypp2;
    double as,xs1,ys1,xs2,ys2;
    //crossing point;
    double xcp;
    double ycp;

    double d_region = dist_avoid_freespace_;//dist_range_for_front_; 
    double cos_gl = cos(followPoint_.qgl_rad);
    double sin_gl = sin(followPoint_.qgl_rad);

    x1 = xp = xGL_ + Robo_ClosedRear_*cos_gl;
    y1 = yp = yGL_ + Robo_ClosedRear_*sin_gl;

    x2 =  d_region*cos_gl + xGL_;
    y2 =  d_region*sin_gl + yGL_;

    //SearchWindow
    //点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
    Calc_CrossPoint4viz(x1,y1,x2,y2,xp,yp,&xcp,&ycp);
    Calc_rect4viz(search_width_,xcp,ycp,x2,y2,&xp1,&yp1,&xpp1,&ypp1);
    Calc_rect4viz(search_width_,x2,y2,x1,y1,&xp2,&yp2,&xpp2,&ypp2);

    cos_gl = cos(yawGL_);
    sin_gl = sin(yawGL_);

    double x2_mfd =  minFrontDist_*cos_gl + xGL_;
    double y2_mfd =  minFrontDist_*sin_gl + yGL_;

    double xp2_mfd,yp2_mfd,xpp2_mfd,ypp2_mfd;
    double measure_width_half = Robo_HalfWidth_;
    
    //Mose Closed Object Distance
    Calc_rect4viz(measure_width_half,x2_mfd,y2_mfd,x1,y1,&xp2_mfd,&yp2_mfd,&xpp2_mfd,&ypp2_mfd);

    ego_pp1_.x = xp2;
    ego_pp1_.y = yp2;
    ego_pp1_.z = Z_WPT;
    
    ego_pp2_.x = xp1;
    ego_pp2_.y = yp1;
    ego_pp2_.z = Z_WPT;

    ego_pp3_.x = xpp1;
    ego_pp3_.y = ypp1;
    ego_pp3_.z = Z_WPT;

    ego_pp4_.x = xpp2;
    ego_pp4_.y = ypp2;
    ego_pp4_.z = Z_WPT;

    minFD_pp1_.x = xp2_mfd;
    minFD_pp1_.y = yp2_mfd;
    minFD_pp1_.z = ego_pp1_.z;

    minFD_pp2_.x = xpp2_mfd;
    minFD_pp2_.y = ypp2_mfd;
    minFD_pp2_.z = ego_pp4_.z;

    return 1;
}

int Waypoint_planner::Set_LineFollowPoint(void){

    static int count = 0;
    geometry_msgs::msg::Point p1;
    double x1,y1,x2,y2;
    double xp,yp,xp1,yp1,xp2,yp2;
    double xpp1,ypp1,xpp2,ypp2;
    double as,xs1,ys1,xs2,ys2;

    double xcp;
    double ycp;

    double cos_gl = cos(yawGL_);
    double sin_gl = sin(yawGL_);

    x1 =  current_wpt_.l_xgl;
    y1 =  current_wpt_.l_ygl;

    x2 =  current_wpt_.xgl;
    y2 =  current_wpt_.ygl;

    xp = xGL_;
    yp = yGL_;

    //Lane
    //点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
    Calc_CrossPoint4viz(x1,y1,x2,y2,xp,yp,&xcp,&ycp);
    Calc_rect4viz(AvoidanceMergin_,xcp,ycp,x2,y2,&xp1,&yp1,&xpp1,&ypp1);
    Calc_rect4viz(AvoidanceMergin_,x2,y2,x1,y1,&xp2,&yp2,&xpp2,&ypp2);

    //FollowPoint
    if(fabs(x2-x1) > 0){

        double d = min(distAoid_-1.0,3.0);

        if(d < 0.1) d = 0.1;

        as = (y2-y1)/(x2-x1);
        xs1 = xcp + d/sqrt(as*as+1);
        xs2 = xcp - d/sqrt(as*as+1);
        ys1 = as*(xs1 - xcp) + ycp;
        ys2 = as*(xs2 - xcp) + ycp;

        double dist1 = Dist2_point_to_point(x2,y2,xs1,ys1);
        double dist2 = Dist2_point_to_point(x2,y2,xs2,ys2);

        if(dist1 < dist2) {
            xfollowWptGL_ = xs1;
            yfollowWptGL_ = ys1;
        }else{
            xfollowWptGL_ = xs2;
            yfollowWptGL_ = ys2;
        }

        p1.x = xfollowWptGL_;
        p1.y = yfollowWptGL_;
        p1.z = Z_WPT*2;


        follow_waypoint_.pose.position.x = p1.x;
        follow_waypoint_.pose.position.y = p1.y;
        follow_waypoint_.pose.position.z = p1.z;

        followPoint_.xgl = xfollowWptGL_;
        followPoint_.ygl = yfollowWptGL_;

        double xtmp = xfollowWptGL_-xGL_;
        double ytmp = yfollowWptGL_-yGL_;

        followPoint_.xfs = cos_gl*xtmp + sin_gl*ytmp;
        followPoint_.yfs = -sin_gl*xtmp + cos_gl*ytmp;
        followPoint_.qfs_rad = atan2(followPoint_.yfs,followPoint_.xfs);

        followPoint_.qgl_rad = modAngle(yawGL_ + followPoint_.qfs_rad);
        followPoint_.count = count;

        followPoint_.dist_front_object = minFrontDist_;
        followPoint_.dist_follow = Dist_point_to_point(0,0,followPoint_.xfs,followPoint_.yfs);

        count ++;
        if(count > 1000) count = 0;

        pub_Followpoint_->publish(std::move(followPoint_));
    }

    lane_pp1_.x = xp2;
    lane_pp1_.y = yp2;
    lane_pp1_.z = Z_WPT;

    lane_pp2_.x = xp1;
    lane_pp2_.y = yp1;
    lane_pp2_.z = Z_WPT;

    lane_pp3_.x = xpp1;
    lane_pp3_.y = ypp1;
    lane_pp3_.z = Z_WPT;

    lane_pp4_.x = xpp2;
    lane_pp4_.y = ypp2;
    lane_pp4_.z = Z_WPT;

    return 1;
}

int Waypoint_planner::Publish_Freespace_FollowPoint(void){

    followPoint_ = tmp_followPoint_;
    pub_Followpoint_->publish(std::move(followPoint_));
    
    return 1;
}

#define DIST_RANGE_FOR_FRONT 3.0

int Waypoint_planner::Set_FreespacePoint(void){

    PositionBase freespaceFS;

    if(current_wpt_.type == WPT_QUEUE || current_wpt_.type == WPT_TRAFFICLIGHT || prev_wpt_.type == WPT_TRAFFICLIGHT){ //251204 
        search_width_ = FreeSpaceWidth_*0.5;
    } 
    else if(current_wpt_.type == WPT_FOLLOW ){
        search_width_ = FreeSpaceWidth_*0.5;
    }
    else {
        search_width_ = FreeSpaceWidth_;
    }
       
    double qdeg_range = 40.0; 
    float dqdeg     = 1.0;//0.25;
    static int timer_avoid = 0;
    double free_width = search_width_;
    
    if(timer_avoid <= 0){
        if(fabs(modAngle(current_wpt_.qfs_rad - prev_wpt_.qfs_rad)) < 5.0 *M_DEG2RAD && 
            fabs(modAngle(current_wpt_.qfs_rad - next_wpt_.qfs_rad)) < 5.0 *M_DEG2RAD && current_wpt_.type == WPT_NORMAL  ){
            dist_avoid_freespace_ = min(minFrontDist_*1.4142,3.0);       
        }
        else{
            dist_avoid_freespace_ = min(minFrontDist_*1.4142,distWpt_);
        }
    }

    if(minFrontDist_ < 3.0 && timer_avoid <= 0){
        timer_avoid = Count_avoid_timer_;
    }
    else if(timer_avoid > 0){
        timer_avoid --;
        printf("timer_avoid=%d \n",timer_avoid);
    }

    if(Vel_cmps_ <= 0) timer_avoid = 0; //Recovery 動作対応

    Search_freespace2(dist_avoid_freespace_,80,80,free_width,dqdeg);
    
    return 1;
}


//点:(x1,y1),(x2,y2)を通り，点(x1,y1)から距離:w 離れた点(xp1,yp1),(xp2,yp2)
int Waypoint_planner::Calc_rect4viz(double w, double x1,double y1,double x2,double y2,double *xp1, double *yp1, double *xpp1, double *ypp1){

    double ap=0;
    if(fabs(x2-x1) < 0.000000001){
        ap = 99999999999999;
    }else{
        //ap = -(x2-x1)/(y2-y1);
        ap = -(y2-y1)/(x2-x1);
    }

    *yp1 = -w/sqrt(ap*ap+1)+y1;
    *ypp1 = w/sqrt(ap*ap+1)+y1;

    *xp1 = ap*(*yp1-y1)+x1;
    *xpp1 = ap*(*ypp1-y1)+x1;

    return 1;
}

//点:(x1,y1),(x2,y2)を通る直線:lと，点(xp,yp)を通り，直線lに直交する直線lpの交点(xpp,ypp)
int Waypoint_planner::Calc_CrossPoint4viz(double x1,double y1,double x2,double y2,double xp, double yp, double *xpp, double *ypp){

    double a=0;
    double ap=0;

    if(fabs(y2-y1) < 0.000000001){
        a = 99999999999999;
    }else{
        a = (x2-x1)/(y2-y1);
    }

    if(fabs(x2-x1) < 0.000000001){
        ap = 99999999999999;
    }else{
        ap = -(y2-y1)/(x2-x1);
    }

    *ypp = (a*y1-ap*yp-x1+xp)/(a-ap);
    *xpp = a*(*ypp-y1)+x1;

    return 1;
}



void Waypoint_planner::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr aligned_cloud_msg,const sensor_msgs::msg::PointCloud2::SharedPtr target_cloud_msg){
    static int n=0;
    //printf("WP: topic_callback:%d\n",n++);
}


void Waypoint_planner::Set_Feature_Waypoint(void){

   // Green
    printf("WP:Set color:mode=%d \n",mode_);

    if(mode_ > 0){
        points_.color.r = 0.0f;
        points_.color.g = 156.0f/255.0f;
        points_.color.b = 209.0f/255.0f;
        points_.color.a = 1.0f;
    }
    else{
        points_.color.r = 0.0f;
        points_.color.g = 1.0f;
        points_.color.b = 0.0f;
        points_.color.a = 1.0f;
    }
    
    // Magenta
    stops_.color.r = 1.0f;
    stops_.color.g = 0;
    stops_.color.b = 1.0f;
    stops_.color.a = 0.8f;

    // Yellow
    queues_.color.r = 1.0f;
    queues_.color.g = 1.0f;
    queues_.color.b = 0.0f;
    queues_.color.a = 1.0f;

    // Cyan
    follows_.color.r = 0.0f;
    follows_.color.g = 1.0f;
    follows_.color.b = 1.0f;
    follows_.color.a = 1.0f;

    trafficLight_.color.r = 1.0f;//233/255;
    trafficLight_.color.g = 0.0f;//163/255;
    trafficLight_.color.b = 0.0;//38/255;
    trafficLight_.color.a = 0.8f;

    // Passed points
    if(mode_ > 0){
        passed_points_.color.r = 0.8f;
        passed_points_.color.g = 0.8f;
        passed_points_.color.b = 0.0f;
        passed_points_.color.a = 1.0;
    }
    else{
        passed_points_.color.r = 0.6f;
        passed_points_.color.g = 0.6f;
        passed_points_.color.b = 0.6f;
        passed_points_.color.a = 1.0;
    }

    //Mark points
    if(mode_ > 0){
        markpoints_.color.r = 0.9f;
        markpoints_.color.g = 0.2f;
        markpoints_.color.b = 0.4f;
        markpoints_.color.a = 1.0;
    }
    else{
        markpoints_.color.r = 0.9f;
        markpoints_.color.g = 0.2f;
        markpoints_.color.b = 0.4f;
        markpoints_.color.a = 1.0;
    }
}

void Waypoint_planner::Init_Visualization(void){

    points_.points.clear();
    stops_.points.clear();
    queues_.points.clear();
    follows_.points.clear();
    trafficLight_.points.clear();

    passed_points_.points.clear();
    markpoints_.points.clear();
    line_strip_.points.clear();
    current_waypoint_.points.clear();
    follow_waypoint_.points.clear();
    line_strip2_.points.clear();
    line_strip3_.points.clear();
    line_strip4_.points.clear();

    turning_circle_.points.clear();


    points_.id              = 0;
    line_strip_.id          = 1;
    cylinder_.id            = 2;
    current_waypoint_.id    = 3;
    wpt_string_.id          = 4;
    passed_points_.id       = 5;
    markpoints_.id          = 6;
    line_strip2_.id         = 7;
    follow_waypoint_.id     = 8;
    line_strip3_.id         = 9;
    stops_.id               = 10;
    queues_.id              = 11;
    follows_.id             = 12;
    trafficLight_.id        = 13;
    line_strip4_.id         = 14;
    turning_circle_.id      = 15;

    points_.type =  passed_points_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    stops_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    queues_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    follows_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    trafficLight_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    
    line_strip_.type     = visualization_msgs::msg::Marker::LINE_STRIP;
    cylinder_.type       = visualization_msgs::msg::Marker::CYLINDER;
    current_waypoint_.type       = visualization_msgs::msg::Marker::SPHERE;
    follow_waypoint_.type       = visualization_msgs::msg::Marker::SPHERE;
    wpt_string_.type     = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    markpoints_.type     = visualization_msgs::msg::Marker::SPHERE_LIST;
    
    line_strip2_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip3_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip4_.type = visualization_msgs::msg::Marker::LINE_STRIP;

    turning_circle_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    

    turning_circle_.scale.x = 0.05;

    turning_circle_.color.r = 0.25;
    turning_circle_.color.g = 0.5;
    turning_circle_.color.b = 1.0;
    turning_circle_.color.a = 1.0;

    // POINTS markers use x and y scale for width/height respectively
    points_.scale.x = passed_points_.scale.y = 0.5;
    points_.scale.y = passed_points_.scale.y = 0.5;

    stops_.scale.x = 1.0;
    stops_.scale.y = 1.0;

    queues_.scale.x = 0.5;
    queues_.scale.y = 0.5;
    queues_.scale.z = 0.25;

    follows_.scale.x = 0.5;
    follows_.scale.y = 0.5;
    follows_.scale.z = 0.25;

    trafficLight_.scale.x = 0.75;
    trafficLight_.scale.y = 0.75;
    trafficLight_.scale.z = 0.01;

    //Waypoint scale
    current_waypoint_.scale.x = 0.4;
    current_waypoint_.scale.y = 0.4;
    current_waypoint_.scale.z = 0.4;

   //Follow Waypoint scale
    follow_waypoint_.scale.x = 0.5;
    follow_waypoint_.scale.y = 0.5;
    follow_waypoint_.scale.z = 0.5;
    
    // Waypoints:red
    current_waypoint_.color.r = 1.0f;
    current_waypoint_.color.g = 0.0f;
    current_waypoint_.color.b = 0.0f;
    current_waypoint_.color.a = 1.0;

    // Follow Waypoints: Blue
    follow_waypoint_.color.r = 0.25f;
    follow_waypoint_.color.g = 0.5f;
    follow_waypoint_.color.b = 1.0f;
    follow_waypoint_.color.a = 1.0;

    markpoints_.scale.x = 0.4;
    markpoints_.scale.y = 0.4;
    markpoints_.scale.z = 0.4;

    //cylinder
    cylinder_.scale.x = 1.0;
    cylinder_.scale.y = 1.0;
    cylinder_.scale.z = 0.01;

    cylinder_.color.r = 1.0;
    cylinder_.color.a = 0.4;

    //Waypoint string
    wpt_string_.scale.x = 1.2;
    wpt_string_.scale.y = 1.2;
    wpt_string_.scale.z = 1.2;
    wpt_string_.text = "-";

    //Waypoint color
    wpt_string_.color.r = 1.0f;
    wpt_string_.color.g = 1.0f;
    wpt_string_.color.b = 1.0f;
    wpt_string_.color.a = 1.0;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip_.scale.x = 0.04;
    line_strip2_.scale.x =  0.05;
    line_strip3_.scale.x =  0.05;
    line_strip4_.scale.x =  0.25;

    // Line strip is white
    line_strip_.color.r = 0.8;
    line_strip_.color.g = 0.8;
    line_strip_.color.b = 0.8;
    line_strip_.color.a = 1.0;

    // Line list is red
    line_strip2_.color.r = 1.0;
    line_strip2_.color.a = 1.0;

    // Line list is blue
    line_strip3_.color.r = 0.25;
    line_strip3_.color.g = 0.55;
    line_strip3_.color.b = 1.0;
    line_strip3_.color.a = 1.0;

    // Line list is Green　R:255 G:179 B:71
    line_strip4_.color.r = 255/255;
    line_strip4_.color.g = 179/255;
    line_strip4_.color.b = 71/255;
    line_strip4_.color.a = 1.0;

    //global座標系からみたrobot座標系の原点位置と方向の格納
    points_.header.frame_id = stops_.header.frame_id = queues_.header.frame_id = follows_.header.frame_id = \
    trafficLight_.header.frame_id = passed_points_.header.frame_id = line_strip_.header.frame_id = cylinder_.header.frame_id = \
    current_waypoint_.header.frame_id = wpt_string_.header.frame_id = markpoints_.header.frame_id = "map"; //
    
    follow_waypoint_.header.frame_id = "map";
    line_strip2_.header.frame_id = "map";
    line_strip3_.header.frame_id = "map";
    line_strip4_.header.frame_id = "map";
    turning_circle_.header.frame_id = "map";

    points_.header.stamp = stops_.header.stamp = trafficLight_.header.stamp = passed_points_.header.stamp = \
    line_strip_.header.stamp = cylinder_.header.stamp = current_waypoint_.header.stamp = wpt_string_.header.stamp = \
    markpoints_.header.stamp = line_strip2_.header.stamp = line_strip3_.header.stamp = line_strip4_.header.stamp = \
    follow_waypoint_.header.stamp = turning_circle_.header.stamp = this->get_clock()->now(); //
    
    points_.ns = stops_.ns = passed_points_.ns =line_strip_.ns =  cylinder_.ns = "waypoint_array";
    stops_.ns = "stop_points";
    queues_.ns = "queue_points";
    follows_.ns = "follow_points";
    trafficLight_.ns = "trafficlight_points";
    
    current_waypoint_.ns = wpt_string_.ns  = "waypoint";
    follow_waypoint_.ns = "follow_waypoint";
    markpoints_.ns = "markpoint";
    line_strip2_.ns = "freelane";
    line_strip3_.ns = "egoregion";
    line_strip4_.ns = "minFrontDist";

    turning_circle_.ns = "turning_circle";

    points_.action = stops_.action = queues_.action = trafficLight_.action = passed_points_.action = \
    line_strip_.action =  cylinder_.action =  current_waypoint_.action = wpt_string_.action = markpoints_.action =\
    line_strip2_.action = line_strip3_.action = line_strip4_.action =\
    follow_waypoint_.action = turning_circle_.action = visualization_msgs::msg::Marker::ADD; //
    
    cluster_arrows_.action = visualization_msgs::msg::Marker::ADD; //
    
    points_.pose.orientation.w = stops_.pose.orientation.w = trafficLight_.pose.orientation.w =queues_.pose.orientation.w = \
    follows_.pose.orientation.w = passed_points_.pose.orientation.w = line_strip_.pose.orientation.w = \
    cylinder_.pose.orientation.w =  current_waypoint_.pose.orientation.w = markpoints_.pose.orientation.w = \
    line_strip2_.pose.orientation.w = follow_waypoint_.pose.orientation.w = turning_circle_.pose.orientation.w = 1.0;//
    
    points_.pose.position.x = stops_.pose.position.x = queues_.pose.position.x = follows_.pose.position.x = \
    trafficLight_.pose.position.x = passed_points_.pose.position.x = markpoints_.pose.position.x = \
    current_waypoint_.pose.position.x = line_strip2_.pose.position.x = line_strip3_.pose.position.x = \
    line_strip4_.pose.position.x = follow_waypoint_.pose.position.x = turning_circle_.pose.position.x = 0;
    
    points_.pose.position.y = stops_.pose.position.y = queues_.pose.position.y = follows_.pose.position.y = \
    trafficLight_.pose.position.y = passed_points_.pose.position.y = markpoints_.pose.position.y = \
    current_waypoint_.pose.position.y = line_strip2_.pose.position.y = line_strip3_.pose.position.y = \
    line_strip4_.pose.position.y = follow_waypoint_.pose.position.y = turning_circle_.pose.position.y = 0;
    
    points_.pose.position.z = stops_.pose.position.z = queues_.pose.position.z = follows_.pose.position.z = \
    trafficLight_.pose.position.z = passed_points_.pose.position.z = markpoints_.pose.position.z = \
    current_waypoint_.pose.position.z = line_strip2_.pose.position.z = line_strip3_.pose.position.z = \
    line_strip4_.pose.position.z = follow_waypoint_.pose.position.z = turning_circle_.pose.position.z = Z_WPT;

    cluster_arrows_.type = visualization_msgs::msg::Marker::ARROW;
    cluster_arrows_.action = visualization_msgs::msg::Marker::ADD;
    cluster_arrows_.header.frame_id = "cloud_matching";
    cluster_arrows_.ns = 'cluster_vector';

}

void Waypoint_planner::cb_area_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
    area_map_.setMap(*msg);    
    //area_map_ready_=true;
}

void Waypoint_planner::ExtractObject1(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size){

    //pcl::PointCloud<pcl::PointXYZI> PointcloudObject1_pcl;
    sensor_msgs::msg::PointCloud2 PointcloudObject1_ros;

    //Extract Object
    static pcl::VoxelGrid<pcl::PointXYZI> voxelSamplerObject;
    voxelSamplerObject.setInputCloud(Pointcloud_input->makeShared());
    voxelSamplerObject.setLeafSize(leaf_size, leaf_size, leaf_size);

    voxelSamplerObject.setFilterFieldName("z");
  
    voxelSamplerObject.setFilterLimits(-0.75, 0.5);
    voxelSamplerObject.filter(PointcloudTarget_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudTarget_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("x");
    voxelSamplerObject.setFilterLimits(-2.0, 30.0); //251111
    voxelSamplerObject.filter(PointcloudTarget_pcl_);

    voxelSamplerObject.setInputCloud(PointcloudTarget_pcl_.makeShared());
    voxelSamplerObject.setFilterFieldName("y");
    voxelSamplerObject.setFilterLimits(-5.0, 5.0);
    voxelSamplerObject.filter(PointcloudTarget_pcl_);


    pcl::toROSMsg(PointcloudTarget_pcl_, PointcloudObject1_ros);
    
    PointcloudObject1_ros.header.stamp = this->get_clock()->now();
    PointcloudObject1_ros.header.frame_id = "/map";
    
    pub_PointcloudObject1_->publish(std::move(PointcloudObject1_ros));
}

int main(int argc, char * argv[])
{
  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // Waypiont_planner nodeの生成とSpin開始
  auto node = std::make_shared<Waypoint_planner>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
