#pragma once

#include <cstdint>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <k4a/k4a.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>

namespace k4a_driver
{
class K4ACamera
{
public:
  K4ACamera(
      const ros::NodeHandle& nh, const std::string& tf_topic,
      const std::string& camera_name, const int32_t device_number,
      const k4a_device_configuration_t& device_configuration);

  K4ACamera(
      const ros::NodeHandle& nh, const std::string& tf_topic,
      const std::string& camera_name, const std::string& serial_number,
      const k4a_device_configuration_t& device_configuration);

  ~K4ACamera();

  void Loop();

private:
  static std::string GetSerialNumber(const k4a_device_t device);

  void Initialize(const std::string& tf_topic);

  void LoadHardwareInformation();

  void LoadCameraIntrinsics();

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::string camera_name_;
  k4a_device_configuration_t device_configuration_;
  k4a_device_t device_ = nullptr;
  k4a_calibration_t calibration_;
  k4a_transformation_t transformation_ = nullptr;
  geometry_msgs::TransformStamped camera_to_color_transform_;
  geometry_msgs::TransformStamped camera_to_depth_transform_;
  tf2_msgs::TFMessage camera_transforms_;
  sensor_msgs::CameraInfo color_camera_intrinsics_;
  sensor_msgs::CameraInfo depth_camera_intrinsics_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher tf_pub_;
  image_transport::CameraPublisher color_pub_;
  image_transport::CameraPublisher depth_pub_;
};
}  // namespace k4a_driver
