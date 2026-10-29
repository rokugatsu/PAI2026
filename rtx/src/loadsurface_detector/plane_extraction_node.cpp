#include "plane_extraction_node.hpp"

#include <cmath>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace pointcloud_plane_extraction
{

PlaneExtractionNode::PlaneExtractionNode(const rclcpp::NodeOptions & options)
: Node("plane_extraction_node", options)
{
  // ---- パラメータ宣言 ----
  input_topic_ = declare_parameter<std::string>("input_topic", "points_in");
  ground_topic_ = declare_parameter<std::string>("ground_topic", "ground/points");
  obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "obstacles/points");
  hull_topic_ = declare_parameter<std::string>("hull_topic", "drivable_area/polygon");
  publish_ground_ = declare_parameter<bool>("publish_ground", true);
  publish_obstacles_ = declare_parameter<bool>("publish_obstacles", true);
  publish_drivable_polygon_ = declare_parameter<bool>("publish_drivable_polygon", true);

  // 路面（Z-up）を仮定する基準フレーム。センサ座標系がこれと異なる場合は TF で変換する。
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.2);

  // 路面候補をロボット前方の ROI（base_frame_id, X:前方 / Y:左 / Z:上, REP-103準拠）で絞り込む。
  // 既定はロボット直前〜前方5m、左右3mの範囲。壁・天井・遠方の誤検出対策と高速化を兼ねる。
  use_roi_filter_ = declare_parameter<bool>("use_roi_filter", true);
  min_x_ = declare_parameter<double>("min_x", 0.0);
  max_x_ = declare_parameter<double>("max_x", 5.0);
  min_y_ = declare_parameter<double>("min_y", -3.0);
  max_y_ = declare_parameter<double>("max_y", 3.0);
  min_z_ = declare_parameter<double>("min_z", -0.3);
  max_z_ = declare_parameter<double>("max_z", 0.3);

  use_voxel_filter_ = declare_parameter<bool>("use_voxel_filter", true);
  voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);

  distance_threshold_ = declare_parameter<double>("distance_threshold", 0.03);
  max_iterations_ = declare_parameter<int>("max_iterations", 200);
  optimize_coefficients_ = declare_parameter<bool>("optimize_coefficients", true);
  min_plane_points_ = declare_parameter<int>("min_plane_points", 100);

  // 路面はほぼ水平（base_frame_id の Z 軸に垂直）という前提を既定で利用する
  use_perpendicular_constraint_ = declare_parameter<bool>("use_perpendicular_constraint", true);
  axis_x_ = declare_parameter<double>("axis_x", 0.0);
  axis_y_ = declare_parameter<double>("axis_y", 0.0);
  axis_z_ = declare_parameter<double>("axis_z", 1.0);
  eps_angle_deg_ = declare_parameter<double>("eps_angle_deg", 15.0);

  // 障害物検知の対象範囲（base_frame_id, ground の ROI とは独立に設定可能）。
  // 既定はロボット手前〜前方2m、左右1.5m、高さ-0.3〜1.5m（路面 ROI より高さ方向を広く取り、
  // 背の高い障害物も検知できるようにしている）。ロボットの制動距離や車幅に合わせて調整する。
  use_obstacle_roi_filter_ = declare_parameter<bool>("use_obstacle_roi_filter", true);
  obstacle_min_x_ = declare_parameter<double>("obstacle_min_x", 0.0);
  obstacle_max_x_ = declare_parameter<double>("obstacle_max_x", 2.0);
  obstacle_min_y_ = declare_parameter<double>("obstacle_min_y", -1.5);
  obstacle_max_y_ = declare_parameter<double>("obstacle_max_y", 1.5);
  obstacle_min_z_ = declare_parameter<double>("obstacle_min_z", -0.3);
  obstacle_max_z_ = declare_parameter<double>("obstacle_max_z", 1.5);
  obstacle_distance_threshold_ = declare_parameter<double>("obstacle_distance_threshold", 0.05);

  // ---- 実行時パラメータ変更コールバック ----
  param_cb_handle_ = add_on_set_parameters_callback(
    std::bind(&PlaneExtractionNode::onParamChange, this, std::placeholders::_1));

  // ---- TF ----
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ---- Pub/Sub ----
  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PlaneExtractionNode::pointCloudCallback, this, std::placeholders::_1));

  if (publish_ground_) {
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::SensorDataQoS());
  }
  if (publish_obstacles_) {
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic_, rclcpp::SensorDataQoS());
  }
  if (publish_drivable_polygon_) {
    hull_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
      hull_topic_, rclcpp::QoS(10));
  }

  RCLCPP_INFO(
    get_logger(),
    "plane_extraction_node started: input='%s' (base_frame='%s') -> ground='%s'",
    input_topic_.c_str(), base_frame_id_.c_str(), ground_topic_.c_str());
}

rcl_interfaces::msg::SetParametersResult PlaneExtractionNode::onParamChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const auto & name = p.get_name();
    if (name == "distance_threshold") {
      distance_threshold_ = p.as_double();
    } else if (name == "max_iterations") {
      max_iterations_ = static_cast<int>(p.as_int());
    } else if (name == "optimize_coefficients") {
      optimize_coefficients_ = p.as_bool();
    } else if (name == "min_plane_points") {
      min_plane_points_ = static_cast<int>(p.as_int());
    } else if (name == "use_voxel_filter") {
      use_voxel_filter_ = p.as_bool();
    } else if (name == "voxel_leaf_size") {
      voxel_leaf_size_ = p.as_double();
    } else if (name == "use_roi_filter") {
      use_roi_filter_ = p.as_bool();
    } else if (name == "min_x") {
      min_x_ = p.as_double();
    } else if (name == "max_x") {
      max_x_ = p.as_double();
    } else if (name == "min_y") {
      min_y_ = p.as_double();
    } else if (name == "max_y") {
      max_y_ = p.as_double();
    } else if (name == "min_z") {
      min_z_ = p.as_double();
    } else if (name == "max_z") {
      max_z_ = p.as_double();
    } else if (name == "use_perpendicular_constraint") {
      use_perpendicular_constraint_ = p.as_bool();
    } else if (name == "axis_x") {
      axis_x_ = p.as_double();
    } else if (name == "axis_y") {
      axis_y_ = p.as_double();
    } else if (name == "axis_z") {
      axis_z_ = p.as_double();
    } else if (name == "eps_angle_deg") {
      eps_angle_deg_ = p.as_double();
    } else if (name == "tf_timeout_sec") {
      tf_timeout_sec_ = p.as_double();
    } else if (name == "use_obstacle_roi_filter") {
      use_obstacle_roi_filter_ = p.as_bool();
    } else if (name == "obstacle_min_x") {
      obstacle_min_x_ = p.as_double();
    } else if (name == "obstacle_max_x") {
      obstacle_max_x_ = p.as_double();
    } else if (name == "obstacle_min_y") {
      obstacle_min_y_ = p.as_double();
    } else if (name == "obstacle_max_y") {
      obstacle_max_y_ = p.as_double();
    } else if (name == "obstacle_min_z") {
      obstacle_min_z_ = p.as_double();
    } else if (name == "obstacle_max_z") {
      obstacle_max_z_ = p.as_double();
    } else if (name == "obstacle_distance_threshold") {
      obstacle_distance_threshold_ = p.as_double();
    }
  }
  return result;
}

void PlaneExtractionNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->width * msg->height == 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "受信した点群が空です。スキップします。");
    return;
  }

  // ---- 必要であれば base_frame_id（Z-up 前提のフレーム）へ変換 ----
  sensor_msgs::msg::PointCloud2 cloud_in_base;
  const bool need_transform =
    !base_frame_id_.empty() && msg->header.frame_id != base_frame_id_;

  if (need_transform) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      // 厳密なタイムスタンプ一致ではなく、最新の TF を使う
      // （静的な外部パラメータであることが多いセンサ-車体間変換を想定した現実的な選択）
      transform = tf_buffer_->lookupTransform(
        base_frame_id_, msg->header.frame_id, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_sec_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "TF変換 '%s' -> '%s' の取得に失敗しました: %s",
        msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      return;
    }
    tf2::doTransform(*msg, cloud_in_base, transform);
    cloud_in_base.header.frame_id = base_frame_id_;
    cloud_in_base.header.stamp = msg->header.stamp;
  } else {
    cloud_in_base = *msg;
  }

  // ROS2 -> PCL
  CloudT::Ptr cloud(new CloudT());
  pcl::fromROSMsg(cloud_in_base, *cloud);

  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "変換後の点群が空です。スキップします。");
    return;
  }

  // ロボット前方の ROI（base_frame_id: X=前方, Y=左, Z=上）で路面候補を絞り込む
  CloudT::Ptr roi_filtered(new CloudT());
  if (use_roi_filter_) {
    pcl::CropBox<PointT> crop;
    crop.setInputCloud(cloud);
    crop.setMin(Eigen::Vector4f(
      static_cast<float>(min_x_), static_cast<float>(min_y_), static_cast<float>(min_z_), 1.0f));
    crop.setMax(Eigen::Vector4f(
      static_cast<float>(max_x_), static_cast<float>(max_y_), static_cast<float>(max_z_), 1.0f));
    crop.filter(*roi_filtered);
  } else {
    roi_filtered = cloud;
  }

  if (roi_filtered->empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ROIフィルタ後の点群が空です（x:[%.2f, %.2f], y:[%.2f, %.2f], z:[%.2f, %.2f]）。",
      min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    return;
  }

  // 任意: ダウンサンプリング（処理負荷軽減）
  CloudT::Ptr filtered(new CloudT());
  if (use_voxel_filter_) {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(roi_filtered);
    voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_),
      static_cast<float>(voxel_leaf_size_));
    voxel.filter(*filtered);
  } else {
    filtered = roi_filtered;
  }

  if (filtered->empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "フィルタ後の点群が空です。スキップします。");
    return;
  }

  // RANSAC による路面平面の抽出
  pcl::SACSegmentation<PointT> seg;
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  seg.setOptimizeCoefficients(optimize_coefficients_);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(max_iterations_);
  seg.setDistanceThreshold(distance_threshold_);

  if (use_perpendicular_constraint_) {
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(
      static_cast<float>(axis_x_), static_cast<float>(axis_y_), static_cast<float>(axis_z_)));
    seg.setEpsAngle(eps_angle_deg_ * M_PI / 180.0);
  } else {
    seg.setModelType(pcl::SACMODEL_PLANE);
  }

  seg.setInputCloud(filtered);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty() ||
    static_cast<int>(inliers->indices.size()) < min_plane_points_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "路面平面が見つかりませんでした（inliers=%zu, 閾値=%d）",
      inliers->indices.size(), min_plane_points_);
    return;
  }

  // inlier（路面）点群を抽出
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(filtered);
  extract.setIndices(inliers);

  CloudT::Ptr ground_cloud(new CloudT());
  extract.setNegative(false);
  extract.filter(*ground_cloud);

  std_msgs::msg::Header out_header;
  out_header.frame_id = base_frame_id_.empty() ? msg->header.frame_id : base_frame_id_;
  out_header.stamp = msg->header.stamp;

  if (publish_ground_ && ground_pub_) {
    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*ground_cloud, ground_msg);
    ground_msg.header = out_header;
    ground_pub_->publish(ground_msg);
  }

  // 障害物候補の点群を抽出・publish。
  // 路面 ROI（min_x/max_x 等）とは独立に obstacle_min_x/obstacle_max_x 等で範囲を設定し、
  // その範囲内の点を「検出した路面平面からの距離」で判定する（路面 ROI 内で fit した平面を、
  // より広い／別の範囲に対して適用することで、路面 ROI より背の高い障害物も拾える）。
  if (publish_obstacles_ && obstacle_pub_) {
    CloudT::Ptr obstacle_roi(new CloudT());
    if (use_obstacle_roi_filter_) {
      pcl::CropBox<PointT> obstacle_crop;
      obstacle_crop.setInputCloud(cloud);
      obstacle_crop.setMin(Eigen::Vector4f(
        static_cast<float>(obstacle_min_x_), static_cast<float>(obstacle_min_y_),
        static_cast<float>(obstacle_min_z_), 1.0f));
      obstacle_crop.setMax(Eigen::Vector4f(
        static_cast<float>(obstacle_max_x_), static_cast<float>(obstacle_max_y_),
        static_cast<float>(obstacle_max_z_), 1.0f));
      obstacle_crop.filter(*obstacle_roi);
    } else {
      obstacle_roi = cloud;
    }

    const double a = coefficients->values[0];
    const double b = coefficients->values[1];
    const double c = coefficients->values[2];
    const double d = coefficients->values[3];
    const double normal_norm = std::sqrt(a * a + b * b + c * c);

    CloudT::Ptr obstacle_cloud(new CloudT());
    if (normal_norm > 1e-6) {
      obstacle_cloud->points.reserve(obstacle_roi->points.size());
      for (const auto & pt : obstacle_roi->points) {
        const double dist = std::fabs(a * pt.x + b * pt.y + c * pt.z + d) / normal_norm;
        if (dist > obstacle_distance_threshold_) {
          obstacle_cloud->points.push_back(pt);
        }
      }
      obstacle_cloud->width = static_cast<uint32_t>(obstacle_cloud->points.size());
      obstacle_cloud->height = 1;
      obstacle_cloud->is_dense = false;
    }

    sensor_msgs::msg::PointCloud2 obstacle_msg;
    pcl::toROSMsg(*obstacle_cloud, obstacle_msg);
    obstacle_msg.header = out_header;
    obstacle_pub_->publish(obstacle_msg);
  }

  // 走行可能領域（路面点群の凸包）を polygon として publish
  if (publish_drivable_polygon_ && hull_pub_ && ground_cloud->size() >= 3) {
    pcl::ConvexHull<PointT> hull;
    hull.setInputCloud(ground_cloud);
    hull.setDimension(2);
    CloudT::Ptr hull_cloud(new CloudT());
    hull.reconstruct(*hull_cloud);

    if (hull_cloud->size() >= 3) {
      geometry_msgs::msg::PolygonStamped polygon_msg;
      polygon_msg.header = out_header;
      polygon_msg.polygon.points.reserve(hull_cloud->size());
      for (const auto & pt : hull_cloud->points) {
        geometry_msgs::msg::Point32 p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        polygon_msg.polygon.points.push_back(p);
        
      }
      hull_pub_->publish(polygon_msg);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "走行可能領域の凸包生成に失敗しました。");
    }
  }

//   RCLCPP_INFO_THROTTLE(
//     get_logger(), *get_clock(), 2000,
//     "路面検出: inliers=%zu/%zu, coeff=[%.3f, %.3f, %.3f, %.3f]",
//     inliers->indices.size(), filtered->size(),
//     coefficients->values[0], coefficients->values[1],
//     coefficients->values[2], coefficients->values[3]);
}

}  // namespace pointcloud_plane_extraction
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<pointcloud_plane_extraction::PlaneExtractionNode>(
    rclcpp::NodeOptions());
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}