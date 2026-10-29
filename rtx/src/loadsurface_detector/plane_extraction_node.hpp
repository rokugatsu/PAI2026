#ifndef POINTCLOUD_PLANE_EXTRACTION__PLANE_EXTRACTION_NODE_HPP_
#define POINTCLOUD_PLANE_EXTRACTION__PLANE_EXTRACTION_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pointcloud_plane_extraction
{

// センサ座標系の点群から「路面（走行可能領域）」とみなせる平面を RANSAC で抽出するノード。
// base_frame_id（既定 base_link, REP-105 の Z-up 前提）へ点群を変換したうえで、
// ロボット前方の ROI（min_x/max_x/min_y/max_y/min_z/max_z）に絞り込み、
// Z 軸にほぼ垂直な平面のみを検出することで、壁など路面以外の平面を誤検出しにくくしている。
class PlaneExtractionNode : public rclcpp::Node
{
public:
  explicit PlaneExtractionNode(const rclcpp::NodeOptions & options);

private:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr hull_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // TF（センサ座標系 -> base_frame_id への変換用）
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- パラメータ ----
  std::string input_topic_;
  std::string ground_topic_;
  std::string obstacle_topic_;
  std::string hull_topic_;
  bool publish_ground_;
  bool publish_obstacles_;
  bool publish_drivable_polygon_;

  std::string base_frame_id_;
  double tf_timeout_sec_;

  // ロボット前方の走行可能領域探索範囲（base_frame_id, REP-103 準拠: X=前方, Y=左, Z=上）
  bool use_roi_filter_;
  double min_x_;
  double max_x_;
  double min_y_;
  double max_y_;
  double min_z_;
  double max_z_;

  bool use_voxel_filter_;
  double voxel_leaf_size_;

  double distance_threshold_;
  int max_iterations_;
  bool optimize_coefficients_;
  int min_plane_points_;

  bool use_perpendicular_constraint_;
  double axis_x_;
  double axis_y_;
  double axis_z_;
  double eps_angle_deg_;

  // 障害物検知の対象範囲（base_frame_id, ground の ROI とは独立に設定可能）。
  // ロボット手前・近傍を狙う想定で、Z 方向は路面 ROI より広め（背の高い障害物も拾う）にしている。
  bool use_obstacle_roi_filter_;
  double obstacle_min_x_;
  double obstacle_max_x_;
  double obstacle_min_y_;
  double obstacle_max_y_;
  double obstacle_min_z_;
  double obstacle_max_z_;
  double obstacle_distance_threshold_;
};

}  // namespace pointcloud_plane_extraction

#endif  // POINTCLOUD_PLANE_EXTRACTION__PLANE_EXTRACTION_NODE_HPP_