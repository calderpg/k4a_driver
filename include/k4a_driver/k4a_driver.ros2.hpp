#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <k4a/k4a.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace k4a_driver
{
class K4ACamera
{
public:
  K4ACamera(
      rclcpp::Node::SharedPtr nh, const std::string& tf_topic,
      const std::string& camera_name, const int32_t device_number,
      const k4a_device_configuration_t& device_configuration);

  K4ACamera(
      rclcpp::Node::SharedPtr nh, const std::string& tf_topic,
      const std::string& camera_name, const std::string& serial_number,
      const k4a_device_configuration_t& device_configuration);

  ~K4ACamera();

  void Loop();

private:
  static std::string GetSerialNumber(const k4a_device_t device);

  void Initialize(const std::string& tf_topic);

  void LoadHardwareInformation();

  void LoadCameraIntrinsics();

  rclcpp::Node::SharedPtr node_;
  image_transport::ImageTransport it_;
  std::string camera_name_;
  k4a_device_configuration_t device_configuration_;
  k4a_device_t device_ = nullptr;
  k4a_calibration_t calibration_;
  k4a_transformation_t transformation_ = nullptr;
  geometry_msgs::msg::TransformStamped camera_to_color_transform_;
  geometry_msgs::msg::TransformStamped camera_to_depth_transform_;
  tf2_msgs::msg::TFMessage camera_transforms_;
  sensor_msgs::msg::CameraInfo color_camera_intrinsics_;
  sensor_msgs::msg::CameraInfo depth_camera_intrinsics_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  image_transport::CameraPublisher color_pub_;
  image_transport::CameraPublisher depth_pub_;
};
}  // namespace k4a_driver
