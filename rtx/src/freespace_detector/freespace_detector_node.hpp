#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <sensor_msgs/msg/imu.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "robo_msgs/msg/waypoint.hpp"
#include "robo_msgs/msg/freespace.hpp"

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
#include "common_const.h"
#include "common_function.h"
#include "common_struct.h"
#include "RTX_Sound.h"


#ifndef FREESPACE_DETECTOR_NODE_HPP_
#define FREESPACE_DETECTOR_NODE_HPP_

class Freespace_detector : public rclcpp::Node
{

typedef struct point3D {
  double x;
  double y;
  double z;
}point3D;

typedef struct cluster_size{
  Eigen::Vector4f min_pt;
  Eigen::Vector4f max_pt;
}cluster_size; 

public:

    Freespace_detector();
    void cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    int cb_Aligned_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    int cb_Target_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    void Set_Feature_Waypoint(void);
    void Init_Visualization(void);
    //点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
    double Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2);
    double Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y);
    int Calc_rect4viz(double w, double x1,double y1,double x2,double y2,double *xp1, double *yp1, double *xpp1, double *ypp1);
    int Calc_CrossPoint4viz(double x1,double y1,double x2,double y2,double xp, double yp, double *xpp1, double *ypp1);

    visualization_msgs::msg::Marker makeMarker(
      const std::string &frame_id, const std::string &marker_ns,
      int marker_id,
      const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
      float r, float g, float b, float a); 

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<robo_msgs::msg::Freespace>::SharedPtr pub_Followpoint_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_VisArray_;    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_target_pointcloud_;
            
    std::string str_output_waypoint_file_ ;
    std::string str_input_waypoint_file_ ;
    
    int mode_;
    double xgl_;
    double ygl_;
    double qgl_deg_;
    double roll_; 
    double pitch_;
    double yaw_;
    
    double AvoidanceMergin_;
    double AvoidanceArea_Front_;
    double AvoidanceArea_Back_;
    double AvoidanceArea_Side_;
    double EgoRegionLength_;
    double MinHeight_;

    int cur_wpt_num_;

    deque<st_Waypoint> dq_Waypoint_;
    st_Waypoint current_wpt_;
    st_Waypoint prev_wpt_;
    st_Waypoint next_wpt_;
    int wpt_size_;
    int editor_mode_;
    bool flag_allow_update_;
    int wpt_modifyOrder_;
    visualization_msgs::msg::Marker points_;
    visualization_msgs::msg::Marker dst_points_;
    visualization_msgs::msg::Marker line_strip_;
    visualization_msgs::msg::Marker line_strip2_;
    visualization_msgs::msg::Marker line_strip3_; 
    visualization_msgs::msg::Marker cylinder_;
    visualization_msgs::msg::Marker current_waypoint_;
    visualization_msgs::msg::Marker follow_waypoint_;
    visualization_msgs::msg::Marker wpt_string_;
    visualization_msgs::msg::Marker passed_points_;
    visualization_msgs::msg::Marker markpoints_;

    visualization_msgs::msg::MarkerArray Markers_;

    
    geometry_msgs::msg::Point lane_pp1_;
    geometry_msgs::msg::Point lane_pp2_;
    geometry_msgs::msg::Point lane_pp3_;
    geometry_msgs::msg::Point lane_pp4_;

    geometry_msgs::msg::Point ego_pp1_;
    geometry_msgs::msg::Point ego_pp2_;
    geometry_msgs::msg::Point ego_pp3_;
    geometry_msgs::msg::Point ego_pp4_;

    
    //way point;
    double xfollowWptFS_;
    double yfollowWptFS_;

    double xfollowWptGL_;
    double yfollowWptGL_;

    double xObjAvoid_;
    double yObjAvoid_;

    double distAoid_;
    double distObject_;
    pcl::PointXYZ avoidPt_;

    pcl::PointCloud<pcl::PointXYZ> PointcloudObject_pcl_;    
    pcl::PointCloud<pcl::PointXYZ> PointcloudDiff_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudAligned_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudTarget_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudObjectLast_pcl_;

    pcl::PointCloud<pcl::PointXYZ> PointcloudMovingObj_pcl_;
    sensor_msgs::msg::PointCloud2 PointcloudMovingObj_ros_;
    
    double minFrontDist_;
    pcl::PointXYZ minFrontPt_;

    double avoidDist_;
    double AvoidMinDist_;
    int ObjectMonitorWigth_cm_;
    double WptResilicence_;

    robo_msgs::msg::Freespace followPoint_;

};
