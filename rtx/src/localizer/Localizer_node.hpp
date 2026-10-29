#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "std_msgs/msg/int16_multi_array.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/flann_search.h>
#include <pcl/io/pcd_io.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_msgs/msg/point_indices.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>  // 固有値・固有ベクトル計算 etc. （外積）

#include "geo_pos_conv.hpp"
#include "robo_msgs/msg/urg.hpp"

#include "/home/june/colcon_ws/src/pcl_lab/src/Limo/so3_math.h"

#include "Localizer_AreaMapAccessor.h"
#include "Localizer_function.hpp"

#ifndef LOCALIZER_NODE_HPP_
#define LOCALIZER_NODE_HPP_

//#define LOG
using namespace Eigen;
//using namespace small_gicp;

#define Z_LIDAR_POSITION 0.879474
#define URG_DATA_SIZE 1081

using namespace Eigen;
using V18D = Eigen::Matrix<double,18,1>;
using M18D = Eigen::Matrix<double,18,18>;
using M1x18D = Eigen::Matrix<double,1,18>;
using M3x18D = Eigen::Matrix<double,3,18>;
using M3x6D = Eigen::Matrix<double,3,6>;
using M12D = Eigen::Matrix<double,12,12>;
using M6D = Eigen::Matrix<double,6,6>;
using M3D = Eigen::Matrix<double,3,3>;
using V3D = Eigen::Vector3d;
using V6D = Eigen::Matrix<double,6,1>;
using V12D = Eigen::Matrix<double,12,1>;

typedef struct pointcloudSet {
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  std::string file_path;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
}pointcloudSet;

typedef struct point2D {
  double x;
  double y;
}point2D;

typedef struct point3D {
  double x;
  double y;
  double z;
}point3D;

// Pose構造体
struct Pose3D {
    M3D Rot;
    V3D X;
};

class Localizer : public rclcpp::Node
{
  public:

    inline double modAngle(double a) {
        while (a < -M_PI)
            a += 2.0 * M_PI;
        while (a > M_PI)
            a -= 2.0 * M_PI;

        return a;
    }

    Localizer();

    void load_pointcloud(const std::string &file_name);
    void create_pointcloud(void);
    void pointcloud_Base(const std::string &file_name);
    void pointcloud_Move(const std::string &file_name);
    void align_to_target(void);

    Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::msg::Quaternion q_msg);

    void convertTransformEigenToROS(void);
    void accept_MapMatching(void);

    bool Is_LoadPCDFile(void);

    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void initPose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void joypad_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void fix_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

    int LowObjectPointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

    void broadcastTF(void);
    void broadcastTF_Odom(void);

    void setPose(void);
    void ExtractObject1(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size);
    void ExtractObject2(const std::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>> &Pointcloud_input, double leaf_size);

    geometry_msgs::msg::Pose matrix4d_to_pose(const Eigen::Matrix4d & eigen_pose_matrix);

    Eigen::Affine3f Make_transform_pointcloud(Pose3D pose);

    //EKF
    void EKF_init(void);
    void EKF_init(double x_init, double y_init, double yaw_init);
    void EKF_predict(void);
    void EKF_update(void);
    int  EKF_evalSensing(void);

    // GNSS
    double yawAdj_;
    double Xfix_;
    double Yfix_;
    double Zfix_;
    double Zfix0_;
    double YAWfix_;
    int fixStatus_;
    geo_pos_conv* pcgpc_;

    double InitLatitude_;
    double InitLongitude_;
    //int Obtain_Z_from_PointCloudMap(void);

  private:
    rclcpp::TimerBase::SharedPtr timer_[2];
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_SourcePointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_TargetPointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_AlignPointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_JustAlignPointcloud_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlign_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlignSum_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapMatching_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudObject1_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudObject2_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_[3];

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_PredictPoseVis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_EstimatePoseVis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_MapPoseVis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_UrgVis_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_PoseErrorCov_;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_URGfront_;
    rclcpp::Publisher<pcl_msgs::msg::PointIndices>::SharedPtr publisher_Indices_Pointcloud_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_IMU_FIX_Odom_status_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_AreaMap_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberLidar_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberURGfrontCloud_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriberOdom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberImu_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscriberJoypad_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriberInitPose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriberInitialpose_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriberURGfront_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriberFix_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberLowObjectCloud_;

    size_t count_;

    std::string file_path_[2];

    pcl::PointCloud<pcl::PointXYZI> PointcloudMapRaw_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudMap_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudRaw_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudSource_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudTarget_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudSubtract_pcl_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudAlign_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudAlign2_pcl_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudAlignVoxel_pcl_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudMapAlign_pcl_;
    pcl::PointCloud<pcl::PointXYZI> PointcloudMapMatching_pcl_;


    pcl::PointCloud<pcl::PointXYZI> PointcloudMapAlignSum_pcl_;
    pcl::PointCloud<pcl::PointXYZI> URGfrontCloud_pcl_;
    pcl::PointCloud<pcl::PointXYZI> cloud_pcl_[3];

    sensor_msgs::msg::PointCloud2 PointcloudTarget_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudSource_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign1_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign2_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudJustAlign_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudSubtract_ros_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudGround_pcl_;
    sensor_msgs::msg::PointCloud2 cloud_segmented_ros_;
    sensor_msgs::msg::PointCloud2 cloud_without_segmented_ros_;

    sensor_msgs::msg::PointCloud2 PointcloudAlignVoxel_ros_;

    sensor_msgs::msg::PointCloud2 PointcloudMap_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapAlign_ros_;

    sensor_msgs::msg::PointCloud2 PointcloudMapAlignSum_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapMatching_ros_;

    sensor_msgs::msg::PointCloud2 URGfrontCloud_ros_;
    sensor_msgs::msg::LaserScan scand_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudObject1_pcl_;
    sensor_msgs::msg::PointCloud2 PointcloudObject1_ros_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudObject2_pcl_;
    sensor_msgs::msg::PointCloud2 PointcloudObject2_ros_;

    pcl::PointCloud<pcl::PointXYZI> PointcloudLowObject_pcl_;

    std_msgs::msg::Int16MultiArray IMU_FIX_Odom_Status_;

    std::string str_pointcloud_map_;
    std::string str_source_pointcloud_;
    std::string str_target_pointcloud_;
    std::string str_input_odom_;
    std::string str_input_imu_;
    std::string str_save_pcd_dir_;

    sensor_msgs::msg::PointCloud2 cloud_ros_[3];
    geometry_msgs::msg::Pose transform_ros_;
    geometry_msgs::msg::Point predict_point_geometry_;

    Eigen::Matrix4d transform_eigen_;

    geometry_msgs::msg::PoseWithCovarianceStamped poseErrorCov_;

    nav_msgs::msg::Odometry poseOnOdom_;
    nav_msgs::msg::Odometry prev_poseOnOdom_;
    nav_msgs::msg::Odometry poseOnAlign_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::TransformListener* tf2_listener_;

    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> ptr_gicp_;
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> ptr_gicp2_;
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ptr_ndt_;
    //boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ptr_ndt_omp_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr align(int n, boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud );

    
    std::FILE* f_pose_;
    std::FILE* f_odom_;
    std::FILE* f_ekf_;
    std::FILE* f_fix_;
    std::FILE* f_fix2_;

    bool is_loadPCDFile_;
    double leafSize_;
    double matching_leafSize_;

    double current_timeOdom_;
    double current_timeLidar_;

    double dd_;
    double dx_;
    double dy_;
    double dz_;

    double dyaw_;
    double dpitch_;
    double droll_;

    double dgap_;

    double ddMap_;
    double dxMap_;
    double dyMap_;
    double dzMap_;
    double yawMap_;

    double ddOdom_;
    double dxOdom_;
    double dyOdom_;
    double dzOdom_;

    double dyawMap_;
    double dpitchMap_;
    double drollMap_;

    double Xpose_;
    double Ypose_;
    double Zpose_;

    double Yawpose_;
    double Pitchpose_;
    double Rollpose_;

    double tmp_Xpose_;
    double tmp_Ypose_;
    double tmp_Zpose_;

    double tmp_Yawpose_;
    double tmp_Pitchpose_;
    double tmp_Rollpose_;

    double Xinit_;
    double Yinit_;
    double YawInit_;

    double Lidar_height_;
    
    //GICP
    double  GICP_TransformationEpsilon_;
    double  GICP_EuclideanFitnessEpsilon_;
    int     GICP_MaximumIterations_;

    //NDT_OMP
    double NDT_OMP_TransformationEpsilon_;//(0.1);
    double NDT_OMP_StepSize_;//(0.75); //<- これかも
    double NDT_OMP_Resolution_;//(5.0);
    int NDT_OMP_MaximumIterations_;//(30);
    double NDT_OMP_Max_Correspondence_Dist_;
    double NDT_OMP_EuclideanFitnessEpsilon_;

    double align_fitnessScore_;
    double threshold_score_align_with_map_ ;
    double threshold_adjust_pose_ ;

    // Lidar odom
    double threshold_align_fitness_score_;
    double threshold_dist_save_pcd_;
    double SubmapRange_;
    double SumAlignFitnessScore_;

    //EKF
    double threshold_EKF_align_fitness_score_;
    double threshold_mh_dist_;

    std::vector <pointcloudSet> vec_tmp_pc_set_;
    std::vector <std::vector<pointcloudSet>> vec_pc_set_;
    pointcloudSet tmp_pc_set_;

    std::vector <pointcloudSet> vec_submapA_;
    pointcloudSet tmp_submap_;

    int count_RosbagPlay_;

    visualization_msgs::msg::Marker PrePoseVis_;
    visualization_msgs::msg::Marker EstPoseVis_;
    visualization_msgs::msg::Marker MapPoseVis_;
    visualization_msgs::msg::Marker UrgVis_;


    //For EKF
    Eigen::Matrix3d Ft_;
    Eigen::Matrix<double, 2, 3> Ht_;

    Eigen::Matrix3d prePt_;
    Eigen::Matrix3d postPt_;

    Eigen::Matrix2d St_;
    Eigen::Matrix3d Sxyzt_;

    Eigen::Matrix2d Rt_;
    Eigen::Matrix3d Qt_;
    Eigen::Matrix<double, 3, 2> Kt_;
    Eigen::Matrix3d It_;

    Eigen::Vector3d wNoize_;
    Eigen::Vector3d vNoize_;

    Eigen::Vector3d x_pre_;
    Eigen::Vector3d x_est_;
    Eigen::Vector2d x_z_;

    double EKF_Cov_SystemNoise_SXX_;
    double EKF_Cov_SystemNoise_SYY_;
    double EKF_Cov_SystemNoise_SYawYaw_;

    double EKF_Cov_SensorNoise_X_;
    double EKF_Cov_SensorNoise_Y_;

    double EKF_Wnoise_X_;
    double EKF_Wnoise_Y_;
    double EKF_Wnoise_Yaw_;

    double EKF_Vnoise_X_;
    double EKF_Vnoise_Y_;
    double EKF_Vnoise_Yaw_;

    int n_EKF_eval_;

    double dtOdom_;
    double dtLidar_;

    double vOdom_;
    double aOdom_;
    double wOdom_;
    //double qOdom_;

    double xOdom_;
    double yOdom_;
    double zOdom_;
    double yawOdom_;
    //double yawAdjOdom_;

    double vLidar_;
    double wLidar_;

    //double yawAdjust_;

    double dxy_mh_;

    double Sx2_;
    double Sy2_;
    double Sz2_;

    double YawImu_;
    double PitchImu_;
    double RollImu_;

    double minZ_;
    bool flag_AlignedWithMap_;

    double radius_search_ground_;

    pcl_msgs::msg::PointIndices indices_ros_;
    pcl_msgs::msg::ModelCoefficients coefficients_ros_;

    robo_msgs::msg::Urg urg_scan_front_;
    double tbl_q_urg_rad_[URG_DATA_SIZE];
    geometry_msgs::msg::Point urg_point_;
    geometry_msgs::msg::Pose pose_;

    int IMU_error_count_ = 0;
    int FIX_error_count_ = 0;
    int Odom_error_count_ = 0;

    //PointCloudMap
    double x_map_offset_=0;
    double y_map_offset_=0;
    double z_map_offset_=0;
    double map_scale_=1.0;
    double trim_area_range_ = 80.0;

    bool flag_show_all_map_ = false;

    //AreaMap
    AreaMapAccessor area_map_;
    bool flag_aera_map_ready_ = false;
   
    //座標系統一
    //Eigen::Isometry3d pose_map_= Eigen::Isometry3d::Identity();
    // double x() const;
    // double y() const;
    // double yaw() const;

    double time_LowObject_;
    int mode_=0;
};

#endif
