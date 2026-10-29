// pointcloud_map_transformer_node.cpp
//
// 目的:
//   rviz2内部のtf2_ros::MessageFilterが引き起こす既知のデッドロック
//   (https://github.com/ros2/geometry2/issues/992) を回避するため、
//   点群をあらかじめtarget_frame(デフォルト: map)へ変換してから配信する。
//
//   tf2_ros::MessageFilterは一切使わず、tf_buffer_->lookupTransform()による
//   同期ポーリングのみで変換するため、問題の発生源となる
//   addTransformableRequest / testTransformableRequests の経路を通らない。
//
// 入力トピック:  "input"  (camera_color_optical_frame など、任意のframe_id)
// 出力トピック:  "output" (target_frameパラメータで指定した座標系、デフォルトmap)

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

using namespace std::chrono_literals;

class PointcloudMapTransformer : public rclcpp::Node
{
public:
  PointcloudMapTransformer()
  : Node("pointcloud_map_transformer")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    lookup_timeout_sec_ = this->declare_parameter<double>("lookup_timeout_sec", 0.05);
    use_latest_fallback_ = this->declare_parameter<bool>("use_latest_fallback", true);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // realsenseなどセンサ系トピックと合わせ、Best Effort/Volatileで購読・配信する
    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", qos,
      std::bind(&PointcloudMapTransformer::pointcloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

    RCLCPP_INFO(
      this->get_logger(),
      "pointcloud_map_transformer started: target_frame='%s', lookup_timeout=%.3fs, "
      "use_latest_fallback=%s",
      target_frame_.c_str(), lookup_timeout_sec_, use_latest_fallback_ ? "true" : "false");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // 入力が既にtarget_frameなら変換不要でそのまま流す
    if (msg->header.frame_id == target_frame_) {
      pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    bool got_transform = false;

    // 1. まずメッセージのタイムスタンプで厳密な変換を試す(短いタイムアウト)
    try {
      transform = tf_buffer_->lookupTransform(
        target_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(lookup_timeout_sec_));
      got_transform = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Exact-time transform %s -> %s failed: %s",
        msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
    }

    // 2. 失敗した場合、許可されていれば直近の変換にフォールバック
    if (!got_transform && use_latest_fallback_) {
      try {
        transform = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
        got_transform = true;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Transform %s -> %s not available (even latest): %s",
          msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what());
      }
    }

    if (!got_transform) {
      // 変換できないフレームは静かにドロップする
      // (tf2_ros::MessageFilterのような「保留リクエスト」は一切作らない)
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_out;
    try {
      tf2::doTransform(*msg, cloud_out, transform);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "doTransform failed: %s", ex.what());
      return;
    }

    pub_->publish(cloud_out);
  }

  std::string target_frame_;
  double lookup_timeout_sec_;
  bool use_latest_fallback_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointcloudMapTransformer>());
  rclcpp::shutdown();
  return 0;
}