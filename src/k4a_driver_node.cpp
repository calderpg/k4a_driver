#include <cstdint>
#include <stdexcept>
#include <string>

#include <k4a/k4a.h>
#include <k4a_driver/k4a_driver.hpp>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "k4a_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Load parameters
  const std::string tf_topic =
      nhp.param(std::string("tf_topic"), std::string("/tf"));
  const std::string camera_name =
      nhp.param(std::string("camera_name"), std::string("k4a"));
  const int32_t device_number = nhp.param(std::string("device_number"), 0);
  // Note that roscpp does not parse numerical strings passed as arg params into
  // strings, and instead parses them into numbers. K4A serial numbers may start
  // with zeros, so loading as an integer and converting to string may not work.
  // See https://github.com/ros/ros_comm/issues/1339 for more.
  const std::string serial_number =
      nhp.param(std::string("serial_number"), std::string(""));
  const int32_t nominal_frame_rate =
      nhp.param(std::string("nominal_frame_rate"), 30);
  const std::string color_resolution =
      nhp.param(std::string("color_resolution"), std::string("720p"));
  const bool enable_wide_depth_fov =
      nhp.param(std::string("enable_wide_depth_fov"), false);
  const bool enable_depth_binning =
      nhp.param(std::string("enable_depth_binning"), false);
  const std::string sync_mode =
      nhp.param(std::string("sync_mode"), std::string("standalone"));
  // Per https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/436,
  // subordinate devices should be delayed by 160us to avoid depth interference.
  const int32_t subordinate_sync_delay_usec =
      nhp.param(std::string("subordinate_sync_delay_usec"), 160);
  // Set configuration options
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  if (color_resolution == "720p" || color_resolution == "720P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_720P");
  }
  else if (color_resolution == "1080p" || color_resolution == "1080P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_1080P");
  }
  else if (color_resolution == "1440p" || color_resolution == "1440P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_1440P");
  }
  else if (color_resolution == "1536p" || color_resolution == "1536P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_1536P");
  }
  else if (color_resolution == "2160p" || color_resolution == "2160P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_2160P");
  }
  else if (color_resolution == "3072p" || color_resolution == "3072P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
    ROS_INFO("Using color resolution K4A_COLOR_RESOLUTION_3072P");
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
      ROS_INFO("Using depth mode K4A_DEPTH_MODE_WFOV_2X2BINNED");
    }
    else
    {
      config.depth_mode = K4A_DEPTH_MODE_WFOV_UNBINNED;
      ROS_INFO("Using depth mode K4A_DEPTH_MODE_WFOV_UNBINNED");
    }
  }
  else
  {
    if (enable_depth_binning)
    {
      config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
      ROS_INFO("Using depth mode K4A_DEPTH_MODE_NFOV_2X2BINNED");
    }
    else
    {
      config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
      ROS_INFO("Using depth mode K4A_DEPTH_MODE_NFOV_UNBINNED");
    }
  }
  if (nominal_frame_rate == 5)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_5;
    ROS_INFO("Using K4A_FRAMES_PER_SECOND_5");
  }
  else if (nominal_frame_rate == 15)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    ROS_INFO("Using K4A_FRAMES_PER_SECOND_15");
  }
  else if (nominal_frame_rate == 30)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    ROS_INFO("Using K4A_FRAMES_PER_SECOND_30");
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
    ROS_INFO("Using sync mode K4A_WIRED_SYNC_MODE_MASTER");
  }
  else if (sync_mode == "subordinate")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
    ROS_INFO("Using sync mode K4A_WIRED_SYNC_MODE_SUBORDINATE");
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
    ROS_INFO("Using sync mode K4A_WIRED_SYNC_MODE_STANDALONE");
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
  ROS_INFO("Using device_number [%i]", device_number);
  ROS_INFO("Using serial_number [%s]", serial_number.c_str());
  if (serial_number != "")
  {
    k4a_driver::K4ACamera camera(
        nh, tf_topic, camera_name, serial_number, config);
    camera.Loop();
  }
  else
  {
    k4a_driver::K4ACamera camera(
        nh, tf_topic, camera_name, device_number, config);
    camera.Loop();
  }
  return 0;
}
