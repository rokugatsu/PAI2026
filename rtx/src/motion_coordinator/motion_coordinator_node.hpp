#ifndef MOTION_COORDINATOR_NODE_HPP_
#define MOTION_COORDINATOR_NODE_HPP_

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>


#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include "robo_msgs/msg/waypoint.hpp"
#include "robo_msgs/msg/freespace.hpp"
#include "robo_msgs/msg/carrot_command.hpp"
#include "robo_msgs/msg/vehicle.hpp"
#include "robo_msgs/msg/motion_cod.hpp"
#include "robo_msgs/msg/traffic_light.hpp"
#include "robo_msgs/msg/scene.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/flann_search.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
/*
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>  // 固有値・固有ベクトル計算 etc. （外積）
*/

//#include <jsk_rviz_plugins/msg/overlay_text.hpp>
//#include <jsk_rviz_msgs/msg/overlay_text.hpp>

///opt/ros/humble/include/rviz_2d_overlay_msgs/rviz_2d_overlay_msgs/msg/overlay_text.hpp
//#include <rviz_2d_overlay_msgs/rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include </opt/ros/humble/include/rviz_2d_overlay_msgs/rviz_2d_overlay_msgs/msg/overlay_text.hpp>
//#include </opt/ros/humble/include/pie_chart_display.h>

#include <std_msgs/msg/color_rgba.hpp>
#include "../common/common_const.h"
#include "../common/common_function.h"
#include "../common/common_struct.h"
#include "../common/rtx_const.h"

#include "motion_coordinator_const.h"
#include "CarrotCommandID.h"
//#include "RTX_Sound.hpp"

#define COM_ARY_NUM 10

//#ifndef MOTION_COORDINATOR_NODE_HPP_
//#define MOTION_COORDINATOR_NODE_HPP_

class Motion_coordinator : public rclcpp::Node
{

  typedef struct FrontObject{
    float x;
    float y;
    bool    flagAlive;
    int     poseCount;
    int     reverseCount;
    int     spinCount;
  }FrontObject;

public:
    Motion_coordinator();
    void test(void);

    
    void cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void cb_waypoint(const robo_msgs::msg::Waypoint::SharedPtr msg);
    void cb_followpoint(const robo_msgs::msg::Freespace::SharedPtr msg);
    void cb_joypad(const sensor_msgs::msg::Joy::SharedPtr msg);
    void cb_initialPose(const geometry_msgs::msg::Pose::SharedPtr msg);
    void cb_vehicle(const robo_msgs::msg::Vehicle::SharedPtr msg);
    void cb_trafficlight(robo_msgs::msg::TrafficLight::SharedPtr msg);
    void cb_URGstatus(std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void cb_IMU_FIX_Odom_status(std_msgs::msg::Int16MultiArray::SharedPtr msg);
    void cb_scene(const robo_msgs::msg::Scene::SharedPtr msg);
 

    int CarrotCommand(int CommandID,int n,float* com);
    int controll_routine(void);
    int Init_Status(void);
    void Show_Status(void);
    int InitOdometry(void);
    int Vel2StopTime(void);
    int PosingFunction(void);
    int EmergencyStopFunction(void);
    int RestartFunction(void);
    int InitStartFunction(void);
    int WaitStarting(void);
    int KeepStayingFunction(void);
    int NaviControlFunction(bool flag_control);
    int FollowDirection(void);
    int SpinControlFunction(void);
    int SpinControlLoop(void);
    
    int PoseControlFunction(void);
    int ReverseFunction(void);
    int Stop_Until_Vel(double vel_stop,int limit,int time_msec_stop_mergin);
    int Stop_Mild_Until_Vel(double vel_stop,int limit);
    int Reverse(double dist_refGL,int limit);
    int Spin(double qdeg_refFS,double qdeg_range,int tmp_limit);
    int SpinLoop(double qdeg_refFS,double qdeg_range,int tmp_limit);
    
    int Reverse_and_Spin(void);

    int Set_PoseMode(double pose_time);
    int SetBehavior(void);

    int SetSubWayPoint(void);
    int Set_CurrentWayPoint_to_SubWayPoint(void);
    int Set_SubWayPoint_in_FreeSpace(void);
    
    double Identify_MovingObject(void);


    float AdaptiveSpeed(double target_dist,bool flag_allowStop);
    void AvoidCounterReset(void);

    bool key_ControllThread(void);
    
    //rviz_2d_overlay_msgs/rviz_2d_overlay_msgs/msg/overlay_text
    rclcpp::Publisher<robo_msgs::msg::CarrotCommand>::SharedPtr pub_carrot_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_VisDirection_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_VisString_;
    rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr pub_VisText_;
       
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_Joypad_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_localization_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_initPose_;
    rclcpp::Subscription<robo_msgs::msg::Freespace>::SharedPtr sub_followpoint_ ;
    rclcpp::Subscription<robo_msgs::msg::Waypoint>::SharedPtr sub_waypoint_ ;
    rclcpp::Subscription<robo_msgs::msg::Vehicle>::SharedPtr sub_vehicle_ ;
    rclcpp::Subscription<robo_msgs::msg::TrafficLight>::SharedPtr sub_trafficLight_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_URGstatus_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sub_IMU_FIX_Odom_Status_;
    rclcpp::Subscription<robo_msgs::msg::Scene>::SharedPtr sub_scene_ ;

    private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_[2];

    int runMode_;

    double passedTime_;    
    double xGL_;
    double yGL_;
    double zGL_;
    double yawGL_;
    double qdegGL_;

    st_Waypoint cur_wpt_;
    st_Waypoint l_cur_wpt_;
    
    st_followpoint freespc_pt_;
    robo_msgs::msg::MotionCod dst_motion_cod_;

    double qdegFS_motion_cod_;
    double min_dist_frontArea_;

    int key_pose_;
    bool flag_pose_;
    int key_start_;
    bool flag_goal_; 
    bool flag_asure_vehicle_;
    bool manual_contol_;
    bool flag_vehicleActive_;
    bool flag_stay_key_;
    bool flag_spin_;
    bool flagLeavefromTarget_;
    bool key_controllThread_;

    int mode_;
    int control_mode_;
    int action_mode_;

    float RefVel_;
    int carrot_id_;

    float RefVel_manual_;
    float QradFS_manual_;
    float QdegFS_manual_;
    float w_sum_;
    float count_wsum_;
    float qdegFS_;
    float wrefError_;
    double pose_time_;
    double pose_start_time_;
    int stay_count_;
    int count_arekana_;
    
    FrontObject NearestForwardObject_;
    float mcom_[COM_ARY_NUM];

    //vehicle
    double Vel_;
    double Wrad_;
    double travelMileage_;
    double GyroTemp_;
    double GyroRefV_;
    int current_=0;
    int GyroStatus_;
    double Qdeg_Odom_ = 0;

    //Vel
    double vrefMax_;     //VREF_MAX      60       //[cm/s]
    double vel_veryHigh_;   //VEL_VERYHIGH	85 	//[cm/sec]
    double vel_high_;       //VEL_HIGH      60 	//[cm/sec]
    double vel_mid_;        //VEL_MID       50 	//[cm/sec]
    double vel_low_;        //VEL_LOW       40 	//[cm/sec]
    double vel_lightLow_;   //VEL_LIGHT_LOW 30 	//[cm/sec]
    double vel_veryLow_;    //VEL_MID       50 	//[cm/sec]
    double vel_extreamLow_; //VEL_EXTREAM_LOW       10 	//[cm/sec]
    double vel_hill_;       //VEL_HILL       50 	//[cm/sec]
    double vel_init_;       //VEL_INIT       40 	//[cm/sec]
    double vel_max_;        //VEL_MAX       80 	//[cm/sec]

    double vel_step_;

    bool flag_ready_for_pose_;
    bool flag_trafficLight_;

    double xgl_pose_;
    double ygl_pose_;
    double qgl_deg_pose_;

    double stop_radius_;
    double Dist_stopQueue_;
    double DistTime_;

    visualization_msgs::msg::Marker visDirection_;
    visualization_msgs::msg::Marker visString_;

    int trafficLight_red_;
    int trafficLight_green_;

    int Step_for_trafficLight_;
    int prev_Step_for_trafficLight_; 
    
    short URGstatus_[2]={0,0};
    short IMU_FIX_Odom_Status_[3]={0,0,0};
    short Joypad_Status_ = 0;
    short Vehicle_Status_ = 0;

    int Joypad_error_count_ = 0;
    int Vehicle_error_count_ = 0;

    int maxCurrentmA_;

    int scene_ = 99;

    double Dist_Front_Default_ = DIST_DEFAULT_FRONT;
    double Dist_Critical_Front_ = DIST_CRITICAL_FRONT;
    double Dist_Avoid_Reverse_ = DIST_AVOID_REVERSE;
    int Reverse_Count_Default_ = REVERSE_COUNT_DEFAULT;
    int ForwardObject_PoseCount_ = 20;
    int Pedestrian_StopDist_= 1.2;
    
    double SpinThreshold_ = 15.0;

    bool flag_spin_init_ = true;
};

extern Motion_coordinator* pmotcoord;
extern void *ControllerThread(void* lpvParam);

#endif
