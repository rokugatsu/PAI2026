// polygon_map_transformer_node.cpp
//
// 目的:
//   drivable_area/polygon (geometry_msgs/PolygonStamped) を、
//   pointcloud_map_transformer_node と同じ考え方で target_frame(デフォルト: map)
//   へ同期変換してから配信する。rviz2内部のtf2_ros::MessageFilterによる
//   デッドロック(ros2/geometry2#992)を回避するための表示専用ノード。
//
//   PolygonStampedはtf2_geometry_msgsが変換関数を提供していないため、
//   tf2::Transformを直接ポイントに適用する。

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class PolygonMapTransformer : public rclcpp::Node
{
public:
  PolygonMapTransformer()
  : Node("polygon_map_transformer")
  {
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    lookup_timeout_sec_ = this->declare_parameter<double>("lookup_timeout_sec", 0.05);
    use_latest_fallback_ = this->declare_parameter<bool>("use_latest_fallback", true);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "input", rclcpp::QoS(10),
      std::bind(&PolygonMapTransformer::polygonCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("output", rclcpp::QoS(10));

    RCLCPP_INFO(
      this->get_logger(),
      "polygon_map_transformer started: target_frame='%s'", target_frame_.c_str());
  }

private:
  void polygonCallback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg)
  {
    if (msg->header.frame_id == target_frame_) {
      pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    bool got_transform = false;

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

    if (!got_transform && use_latest_fallback_) {
      try {
        transform = tf_buffer_->lookupTransform(
          target_frame_, msg->header.frame_id, tf2::TimePointZero);
        got_transform = true;
      } catch (const tf2::TransformException & ex2) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Transform %s -> %s not available (even latest): %s",
          msg->header.frame_id.c_str(), target_frame_.c_str(), ex2.what());
      }
    }

    if (!got_transform) {
      return;
    }

    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    geometry_msgs::msg::PolygonStamped out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = target_frame_;
    out.polygon.points.reserve(msg->polygon.points.size());

    for (const auto & p : msg->polygon.points) {
      tf2::Vector3 v(p.x, p.y, p.z);
      tf2::Vector3 v_out = tf2_transform * v;

      geometry_msgs::msg::Point32 p_out;
      p_out.x = static_cast<float>(v_out.x());
      p_out.y = static_cast<float>(v_out.y());
      p_out.z = static_cast<float>(v_out.z());
      out.polygon.points.push_back(p_out);
    }

    pub_->publish(out);
  }

  std::string target_frame_;
  double lookup_timeout_sec_;
  bool use_latest_fallback_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolygonMapTransformer>());
  rclcpp::shutdown();
  return 0;
}