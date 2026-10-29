#include <iostream>

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
#include "robo_msgs/msg/vehicle.hpp"
#include "robo_msgs/msg/scene.hpp"

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vision_msgs/msg/bounding_box3_d.hpp>

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

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>

// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/registration/ndt.h>
// #include <pcl/registration/gicp.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree.h>

#include <pcl/common/impl/accumulators.hpp>
#include <pcl/common/impl/centroid.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

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

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "../common/common_const.h"
#include "../common/common_function.h"
#include "../common/common_struct.h"
#include "../common/rtx_const.h"

#include "RTX_Sound.hpp"

#include "waypoint_planner_function.hpp"
#include "waypoint_planner_AreaMap.h"


#ifndef WAYPOINT_PLANNER_NODE_HPP_
#define WAYPOINT_PLANNER_NODE_HPP_

#define MAX_NUM_CLUSTER 32
//#define MAX_NUM_CLUSTER 16


using PointT = pcl::PointXYZI;

class Waypoint_planner : public rclcpp::Node
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

typedef struct PositionBase{
    double dist;
    double x;
    double y;
    double z;
    double qrad;
}PositionBase;

typedef struct st_centroid{    
    Eigen::Vector4f FS;
    Eigen::Vector4f GL;
    double size;
}st_centroid;

typedef struct st_cluster{
    int id;
    
    st_centroid cur_centroid;
    st_centroid prev_centroid;

    double prev_clock;
    double cur_clock;

    double distance; 
    double vel_mps; 
    
}st_cluster;

typedef struct st_id{
    int id;
    bool is_used;
}st_id;
    
typedef struct st_movingObject{
    Eigen::Vector3f position;
    Eigen::Vector3f direction;
}st_movingObject;

inline double modAngle(double a) {
    while (a < -M_PI)
        a += 2.0 * M_PI;
    while (a > M_PI)
        a -= 2.0 * M_PI;

    return a;
}

public:

    Waypoint_planner();
    void cb_joypad(const sensor_msgs::msg::Joy::SharedPtr msg);
    void cb_localization(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void cb_vehicle(const robo_msgs::msg::Vehicle::SharedPtr msg);
    void cb_scene(const robo_msgs::msg::Scene::SharedPtr msg);

    //int cb_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    int cb_Aligned_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    int cb_Target_Pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr aligned_cloud_msg,const sensor_msgs::msg::PointCloud2::SharedPtr target_cloud_msg);
    pcl::PointCloud<pcl::PointXYZI> difference_extraction(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test);
    int ObjectClasifitaion(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_base, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_test);

    void cb_area_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    int cb_LowObjectPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    void ExtractObject1(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size);


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_MovingObjectPointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ClusterPointcloud_;

    void test(void);
    void Set_Feature_Waypoint(void);
    void Init_Visualization(void);
    void Edit_Waypoint(int k);
    //点(xp,yp)と,(x1,y1),(x2,y2)を通る直線との距離
    double Dist_from_point_to_line2(double xp,double yp,double x1,double y1,double x2,double y2);
    double Dist_point_to_point(double p1_x,double p1_y,double p2_x,double p2_y);
    int Calc_rect4viz(double w, double x1,double y1,double x2,double y2,double *xp1, double *yp1, double *xpp1, double *ypp1);
    int Calc_CrossPoint4viz(double x1,double y1,double x2,double y2,double xp, double yp, double *xpp1, double *ypp1);
    int Search_Lane(void);
    int Set_LineFollowPoint(void);
    int Set_FreespacePoint(void);
    int Search_freespace(double dist_range, double qdeg_range, double free_width,double dqdeg);
    int Search_freespace2(double dist_range, double qdeg_range_left,double qdeg_range_right, double free_width,double dqdeg);

    int Publish_Freespace_FollowPoint(void);

    int Detect_Object(void);

    int Detect_MovingObject(
        const pcl::PointCloud<PointT>::Ptr& cloud_prev,
        const pcl::PointCloud<PointT>::Ptr& cloud_curr,
        std::vector<st_movingObject>& movingObject);
    
    int Depict_EgoRegion(void);

    double Identify_MovingObject(void);
    int ObjectTracking(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);    
    void Set_ClusterColor(void);
    double Get_Clock(void);

    int trans_DistAngle_to_FS(double dist, double qrad_FS, double *xFS, double *yFS);
    int trans_FS_to_GL(double xFS, double yFS, double *xGL, double *yGL);
    int trans_FS_to_GL2(double dist_range, double qrad_FS, double *xGL, double *yGL);
    
    int trans_GL_to_FS(double xGL, double yGL, double *xFS, double *yFS,double *qrad_FS);
    int trans_GL_to_FSf(double xGL, double yGL, float *xFS, float *yFS,float *qrad_FS);

    visualization_msgs::msg::Marker makeMarker(
        const std::string &frame_id, const std::string &marker_ns,
        int marker_id,
        const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
        float r, float g, float b, float a); 

    void Visualize_movingObject(void);
    
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> approximate_policy;
    //message_filters::Synchronizer<approximate_policy> sync_;
    //message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_aligned_pointcloud_;
    //message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_target_pointcloud_;

    //GridMap grid_map_;
    AreaMapAccessor area_map_;   
    double drivable_area_width_ = 9999;
    double dist_avoid_freespace_ = 3.0; 
    int Count_avoid_timer_ = 30;   
    
private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<robo_msgs::msg::Waypoint>::SharedPtr pub_Waypoint_;
    rclcpp::Publisher<robo_msgs::msg::Freespace>::SharedPtr pub_Followpoint_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_InitPose_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_VisArray_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_clusters_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_Vectors_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_PointcloudTargetSelected_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_movingObject_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox3D>::SharedPtr pub_RoboShape_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_RoboPoseErrorCov_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_PointcloudObject1_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_Joypad_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_localization_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<robo_msgs::msg::Vehicle>::SharedPtr sub_vehicle_ ;
    rclcpp::Subscription<robo_msgs::msg::Scene>::SharedPtr sub_scene_ ;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_aligned_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_target_pointcloud_;
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_LowObjectCloud_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_area_map_;
    
    //message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_aligned_pointcloud_;
    //message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_target_pointcloud_;
    
    std::string str_output_waypoint_file_ ;
    std::string str_input_waypoint_file_ ;    
    std::string str_mark_file_ ;
    
    int mode_;
    double xGL_;
    double yGL_;
    double zGL_;
    double qgl_deg_;
    double rollGL_; 
    double pitchGL_;
    double yawGL_;

    double dxGL_;
    double dyGL_;
    double dyawGL_;

    double dtCloud_;
    double currentClock_;

    double FreeSpaceWidth_;
    double AvoidanceMergin_;
    double AvoidanceArea_Front_;
    double AvoidanceArea_Back_;
    double AvoidanceArea_Side_;
    double AvoidanceDepthMergin_;
    
    double Robo_HalfWidth_;
    double Robo_ClosedFront_;
    double Robo_ClosedRear_; 
    double Robo_FarmostFront_;
    double Robo_Height_;

    double EgoRegionLength_;
    double MinHeight_;
    double MinHeightFar_;
    double MinHeightMid_;
    double MinHeightNear_;
    
    double VisRoboWidth_;
    double VisRoboLength_;
    double VisRoboHeight_;
    double VisRoboCenterLength_;
    double VisRoboCenterHeight_;
    
    deque<st_Waypoint> dq_Waypoint_;
    st_Waypoint current_wpt_;
    st_Waypoint prev_wpt_;
    st_Waypoint next_wpt_;
    st_Waypoint next2_wpt_;
    
    int wpt_size_;
    int editor_mode_;
    bool flag_allow_update_;
    int wpt_modifyOrder_ = 0;
    visualization_msgs::msg::Marker points_;
    visualization_msgs::msg::Marker stops_;
    visualization_msgs::msg::Marker queues_;
    visualization_msgs::msg::Marker follows_;
    visualization_msgs::msg::Marker trafficLight_;
    visualization_msgs::msg::Marker dst_points_;
    visualization_msgs::msg::Marker line_strip_;
    visualization_msgs::msg::Marker line_strip2_;
    visualization_msgs::msg::Marker line_strip3_; 
    visualization_msgs::msg::Marker line_strip4_;
     
    visualization_msgs::msg::Marker cylinder_;
    visualization_msgs::msg::Marker current_waypoint_;
    visualization_msgs::msg::Marker follow_waypoint_;
    visualization_msgs::msg::Marker wpt_string_;
    visualization_msgs::msg::Marker passed_points_;
    visualization_msgs::msg::Marker markpoints_;
    visualization_msgs::msg::Marker cluster_arrows_;
    
    visualization_msgs::msg::Marker turning_circle_;
    
    visualization_msgs::msg::MarkerArray Markers_;
    visualization_msgs::msg::MarkerArray ClusterVectorMarkers_;
    
    FILE* F_WAYPOINT_;// = std::fopen(str_waypoint.c_str(),"r");
    FILE* F_MARK_;// = std::fopen(str_waypoint.c_str(),"r");
    
    pcl::PCDWriter writer_;

    double mx_;
    double my_;
    bool cmd_del_;
    bool cmd_add_;
    bool cmd_stop_;
    bool cmd_save_;
    bool cmd_mark_;
    bool cmd_initPosition_;
    bool cmd_follow_;
    bool cmd_queue_; 
    bool cmd_trafficLight_;  

    int cur_wpt_num_;
    double Wpt_change_radius_;
    double Wpt_change_radius_normal_;
    double Wpt_change_radius_posePoint_;
    double Wpt_change_radius_curve_;

    bool flag_goal_;
    
    double distWpt_;
    double minFrontDist_=9999;
    double minLeftDist_=9999;
    double minRightDist_=9999;
    
    double dist_range_for_front_;
    int wpt_type_;

    geometry_msgs::msg::Point lane_pp1_;
    geometry_msgs::msg::Point lane_pp2_;
    geometry_msgs::msg::Point lane_pp3_;
    geometry_msgs::msg::Point lane_pp4_;

    geometry_msgs::msg::Point ego_pp1_;
    geometry_msgs::msg::Point ego_pp2_;
    geometry_msgs::msg::Point ego_pp3_;
    geometry_msgs::msg::Point ego_pp4_;

    geometry_msgs::msg::Point minFD_pp1_;
    geometry_msgs::msg::Point minFD_pp2_;

    double xAdjWpt_;
    double yAdjWpt_;

    //crossing point;
    //double xcp_;
    //double ycp_;
    
    //way point;
    double xfollowWptFS_;
    double yfollowWptFS_;

    double xfollowWptGL_;
    double yfollowWptGL_;

    double xObjAvoid_;
    double yObjAvoid_;

    double distAoid_;
    //double distObject_;
    pcl::PointXYZI avoidPt_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudObject_pcl_;
    
    pcl::PointCloud<pcl::PointXYZI> PointcloudDiff_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudAligned_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudTarget_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudObjectLast_pcl_;
    
    //pcl::PointCloud<pcl::PointXYZI> PointcloudLowObject_pcl_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudMovingObj_pcl_;
    pcl::PointCloud<pcl::PointXYZRGB> PointcloudCluster_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudTargetSelected_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudTargetSelected_Prev_pcl_;

    sensor_msgs::msg::PointCloud2 PointcloudTargetSelected_ros_;                                  
    sensor_msgs::msg::PointCloud2 PointcloudMovingObj_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudCluster_ros_;
    
    sensor_msgs::msg::PointCloud2 PointcloudObject1_ros_;
        
    
    pcl::PointXYZI minFrontPt_;

    double avoidDist_;
    double AvoidMinDist_;
    int ObjectMonitorWidth_cm_;
    double WptResilicence_;
    int Wpt_KeepTime_;
    int Wpt_Counter_;  

    robo_msgs::msg::Freespace followPoint_;
    robo_msgs::msg::Freespace tmp_followPoint_;

    vector<point3D> vec_Object_;

    double init_nanosecond_;
    double relVel_frontObject_;
    double Vel_cmps_;
    double Vel_mps_;
    
    double search_width_;

    int key_overTake_;
    uint32_t cluster_color_[16];
    uint32_t id_cluster_;

    double followPoint_angle_deg_ = 0;
    
    int scene_ = 99;

    //std::vector<Eigen::Vector3f> directions_out_;
    std::vector<st_movingObject> movingObject_;

    int lane_side_ = 1; // R

    double time_LowObject_ = 0;
    double time_TargetObject_ = 0;
    
    double turn_radius_ = -1;
    int turn_side_ = -1;
    // AreaMap    
    // nav_msgs::msg::OccupancyGrid area_map_;
    // bool area_map_ready_=false;
    // bool worldToGrid(double wx,double wy,int &gx,int &gy) const;
    
    //AreaMapAccessor area_map_;   

};


#endif