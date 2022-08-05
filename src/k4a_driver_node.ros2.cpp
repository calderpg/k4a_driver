#include <cstdint>
#include <stdexcept>
#include <string>

#include <k4a/k4a.h>
#include <k4a_driver/k4a_driver.ros2.hpp>
#include <rclcpp/rclcpp.hpp>

inline int32_t int64_to_int32(int64_t input)
{
  return static_cast<int32_t>(0xFFFFFFFF & input);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("k4a_driver_node");
  // Load parameters
  const std::string tf_topic =
      node->declare_parameter("~/tf_topic", "/tf");
  const std::string camera_name =
      node->declare_parameter("~/camera_name", "k4a");
  const int32_t device_number =
      int64_to_int32(node->declare_parameter("device_number", 0));
  // Note that roscpp does not parse numerical strings passed as arg params into
  // strings, and instead parses them into numbers. K4A serial numbers may start
  // with zeros, so loading as an integer and converting to string may not work.
  // See https://github.com/ros/ros_comm/issues/1339 for more.
  const std::string serial_number =
      node->declare_parameter("serial_number", "");
  const int32_t nominal_frame_rate =
      int64_to_int32(node->declare_parameter("nominal_frame_rate", 30));
  const std::string color_resolution =
      node->declare_parameter("color_resolution", "720p");
  const bool enable_wide_depth_fov =
      node->declare_parameter("enable_wide_depth_fov", false);
  const bool enable_depth_binning =
      node->declare_parameter("enable_depth_binning", false);
  const std::string sync_mode =
      node->declare_parameter("sync_mode", "standalone");
  // Per https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/436,
  // subordinate devices should be delayed by 160us to avoid depth interference.
  const int32_t subordinate_sync_delay_usec = int64_to_int32(
      node->declare_parameter("subordinate_sync_delay_usec", 160));
  // Set configuration options
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  if (color_resolution == "720p" || color_resolution == "720P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    RCLCPP_INFO(
        node->get_logger(), "Using color resolution K4A_COLOR_RESOLUTION_720P");
  }
  else if (color_resolution == "1080p" || color_resolution == "1080P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    RCLCPP_INFO(
        node->get_logger(),
        "Using color resolution K4A_COLOR_RESOLUTION_1080P");
  }
  else if (color_resolution == "1440p" || color_resolution == "1440P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    RCLCPP_INFO(
        node->get_logger(),
        "Using color resolution K4A_COLOR_RESOLUTION_1440P");
  }
  else if (color_resolution == "1536p" || color_resolution == "1536P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    RCLCPP_INFO(
        node->get_logger(),
        "Using color resolution K4A_COLOR_RESOLUTION_1536P");
  }
  else if (color_resolution == "2160p" || color_resolution == "2160P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    RCLCPP_INFO(
        node->get_logger(),
        "Using color resolution K4A_COLOR_RESOLUTION_2160P");
  }
  else if (color_resolution == "3072p" || color_resolution == "3072P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    RCLCPP_INFO(
        node->get_logger(),
        "Using color resolution K4A_COLOR_RESOLUTION_3072P");
  }
  else
  {
    throw std::invalid_argument(
        "Invalid color resolution option [" + color_resolution +
        "], valid options are 720p, 1080p, 1440p, 1536p, 2160p, or 3072p");
  }
  if (enable_wide_depth_fov)
  {
    if (enable_depth_binning)
    {
      config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
      RCLCPP_INFO(
          node->get_logger(), "Using depth mode K4A_DEPTH_MODE_WFOV_2X2BINNED");
    }
    else
    {
      config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
      RCLCPP_INFO(
          node->get_logger(), "Using depth mode K4A_DEPTH_MODE_WFOV_UNBINNED");
    }
  }
  else
  {
    if (enable_depth_binning)
    {
      config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
      RCLCPP_INFO(
          node->get_logger(), "Using depth mode K4A_DEPTH_MODE_NFOV_2X2BINNED");
    }
    else
    {
      config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
      RCLCPP_INFO(
          node->get_logger(), "Using depth mode K4A_DEPTH_MODE_NFOV_UNBINNED");
    }
  }
  if (nominal_frame_rate == 5)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    RCLCPP_INFO(node->get_logger(), "Using K4A_FRAMES_PER_SECOND_5");
  }
  else if (nominal_frame_rate == 15)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    RCLCPP_INFO(node->get_logger(), "Using K4A_FRAMES_PER_SECOND_15");
  }
  else if (nominal_frame_rate == 30)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    RCLCPP_INFO(node->get_logger(), "Using K4A_FRAMES_PER_SECOND_30");
  }
  else
  {
    throw std::runtime_error(
        "Invalid frame rate [" + std::to_string(nominal_frame_rate) +
        "], valid options are 5, 15, or 30");
  }
  if (sync_mode == "master")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
    RCLCPP_INFO(
        node->get_logger(), "Using sync mode K4A_WIRED_SYNC_MODE_MASTER");
  }
  else if (sync_mode == "subordinate")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    RCLCPP_INFO(
        node->get_logger(), "Using sync mode K4A_WIRED_SYNC_MODE_SUBORDINATE");
    if (subordinate_sync_delay_usec >= 0)
    {
      config.subordinate_delay_off_master_usec =
          static_cast<uint32_t>(subordinate_sync_delay_usec);
    }
    else
    {
      throw std::runtime_error("subordinate_sync_delay_usec must be >= 0");
    }
  }
  else if (sync_mode == "standalone")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
    RCLCPP_INFO(
        node->get_logger(), "Using sync mode K4A_WIRED_SYNC_MODE_STANDALONE");
  }
  else
  {
    throw std::runtime_error(
        "Invalid sync mode [" + sync_mode +
        "], valid options are standalone, master, or subordinate");
  }
  // We never want unsynchronized images
  config.synchronized_images_only = true;
  // Create the camera & capture
  RCLCPP_INFO(
      node->get_logger(), "Using device_number [%i]", device_number);
  RCLCPP_INFO(
      node->get_logger(), "Using serial_number [%s]", serial_number.c_str());
  if (serial_number != "")
  {
    k4a_driver::K4ACamera camera(
        node, tf_topic, camera_name, serial_number, config);
    camera.Loop();
  }
  else
  {
    k4a_driver::K4ACamera camera(
        node, tf_topic, camera_name, device_number, config);
    camera.Loop();
  }
  return 0;
}
