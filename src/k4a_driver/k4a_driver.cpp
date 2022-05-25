#include <k4a_driver/k4a_driver.hpp>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <k4a/k4a.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace k4a_driver
{
namespace
{
geometry_msgs::msg::TransformStamped ExtrinsicsToTransformStamped(
    const k4a_calibration_extrinsics_t& extrinsics,
    const std::string& source_frame_name, const std::string& target_frame_name)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.frame_id = source_frame_name;
  transform_msg.child_frame_id = target_frame_name;
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  // Rotation (row-major)
  transform.matrix()(0, 0) = extrinsics.rotation[0];
  transform.matrix()(0, 1) = extrinsics.rotation[1];
  transform.matrix()(0, 2) = extrinsics.rotation[2];
  transform.matrix()(1, 0) = extrinsics.rotation[3];
  transform.matrix()(1, 1) = extrinsics.rotation[4];
  transform.matrix()(1, 2) = extrinsics.rotation[5];
  transform.matrix()(2, 0) = extrinsics.rotation[6];
  transform.matrix()(2, 1) = extrinsics.rotation[7];
  transform.matrix()(2, 2) = extrinsics.rotation[8];
  // Translation (in millimeters)
  transform.matrix()(0, 3) = extrinsics.translation[0] * 0.001;
  transform.matrix()(1, 3) = extrinsics.translation[1] * 0.001;
  transform.matrix()(2, 3) = extrinsics.translation[2] * 0.001;
  // Convert to quaternion + translation
  const Eigen::Vector3d trans = transform.translation();
  const Eigen::Quaterniond quat(transform.rotation());
  transform_msg.transform.translation.x = trans.x();
  transform_msg.transform.translation.y = trans.y();
  transform_msg.transform.translation.z = trans.z();
  transform_msg.transform.rotation.w = quat.w();
  transform_msg.transform.rotation.x = quat.x();
  transform_msg.transform.rotation.y = quat.y();
  transform_msg.transform.rotation.z = quat.z();
  return transform_msg;
}

sensor_msgs::msg::CameraInfo IntrinsicsToCameraInfo(
    const k4a_calibration_intrinsics_t& intrinsics, const uint32_t width,
    const uint32_t height, const std::string& camera_optical_frame_name)
{
  sensor_msgs::msg::CameraInfo intrinsics_msg;
  intrinsics_msg.header.frame_id = camera_optical_frame_name;
  intrinsics_msg.width = width;
  intrinsics_msg.height = height;
  // Distortion model
  intrinsics_msg.distortion_model = "plumb_bob";
  intrinsics_msg.d =
      {intrinsics.parameters.param.k1,
       intrinsics.parameters.param.k2,
       intrinsics.parameters.param.p1,
       intrinsics.parameters.param.p2,
       intrinsics.parameters.param.k3};
  // Intrinsic matrix
  intrinsics_msg.k =
      {intrinsics.parameters.param.fx, 0.0, intrinsics.parameters.param.cx,
       0.0, intrinsics.parameters.param.fy, intrinsics.parameters.param.cy,
       0.0 , 0.0, 1.0};
  // Rectification matrix
  intrinsics_msg.r =
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0};
  // Projection matrix
  intrinsics_msg.p =
      {intrinsics.parameters.param.fx, 0.0, intrinsics.parameters.param.cx, 0.0,
       0.0, intrinsics.parameters.param.fy, intrinsics.parameters.param.cy, 0.0,
       0.0 , 0.0, 1.0, 0.0};
  return intrinsics_msg;
}

template<typename K4AHandle, typename K4ARelease>
class ScopedK4AHandle
{
public:
  ScopedK4AHandle() {}

  explicit ScopedK4AHandle(K4AHandle handle)
  {
    if (handle != nullptr)
    {
      handle_ = handle;
    }
    else
    {
      throw std::invalid_argument("handle == nullptr");
    }
  }

  ~ScopedK4AHandle()
  {
    Release();
  }

  void Release()
  {
    if (handle_ != nullptr)
    {
      K4ARelease::Release(handle_);
      handle_ = nullptr;
    }
  }

  void Reset(K4AHandle handle)
  {
    Release();
    if (handle != nullptr)
    {
      handle_ = handle;
    }
    else
    {
      throw std::invalid_argument("handle == nullptr");
    }
  }

  K4AHandle& Get() { return handle_; }

  const K4AHandle& Get() const { return handle_; }

  explicit operator bool() const { return handle_ != nullptr; }

private:
  K4AHandle handle_ = nullptr;
};

class K4ACaptureRelease
{
public:
  static void Release(k4a_capture_t capture)
  {
    k4a_capture_release(capture);
  }
};

class K4AImageRelease
{
public:
  static void Release(k4a_image_t image)
  {
    k4a_image_release(image);
  }
};

using ScopedK4ACapture = ScopedK4AHandle<k4a_capture_t, K4ACaptureRelease>;

using ScopedK4AImage = ScopedK4AHandle<k4a_image_t, K4AImageRelease>;
}  // namespace

K4ACamera::K4ACamera(
    rclcpp::Node::SharedPtr node, const std::string& tf_topic,
    const std::string& camera_name, const int32_t device_number,
    const k4a_device_configuration_t& device_configuration)
    : node_(node), it_(node_), camera_name_(camera_name),
      device_configuration_(device_configuration)
{
  // Open device
  const uint32_t device_count = k4a_device_get_installed_count();
  if (device_count == 0)
  {
    throw std::runtime_error("No k4a devices found");
  }
  if (device_number < 0)
  {
    throw std::runtime_error(
        "device_number [" + std::to_string(device_number) + "] < 0");
  }
  else if (static_cast<uint32_t>(device_number) >= device_count)
  {
    throw std::runtime_error(
        "device_number [" + std::to_string(device_number) + "] greater than "
        "device_count [" + std::to_string(device_count) + "]");
  }
  if (k4a_device_open(static_cast<uint32_t>(device_number), &device_)
      != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to open device");
  }
  RCLCPP_INFO(node_->get_logger(), "Opened device [%i]", device_number);
  // Initialize
  Initialize(tf_topic);
}

K4ACamera::K4ACamera(
    rclcpp::Node::SharedPtr node, const std::string& tf_topic,
    const std::string& camera_name, const std::string& serial_number,
    const k4a_device_configuration_t& device_configuration)
    : node_(node), it_(node_), camera_name_(camera_name),
      device_configuration_(device_configuration)
{
  // Open device
  const uint32_t device_count = k4a_device_get_installed_count();
  if (device_count == 0)
  {
    throw std::runtime_error("No k4a devices found");
  }
  RCLCPP_INFO(node_->get_logger(), "Looking for device with serial number [%s]", serial_number.c_str());
  for (uint32_t device_num = 0; device_num < device_count; device_num++)
  {
    k4a_device_t device = nullptr;
    if (k4a_device_open(device_num, &device) == K4A_RESULT_SUCCEEDED)
    {
      const std::string device_serial = GetSerialNumber(device);
      RCLCPP_INFO(node_->get_logger(), 
          "Opened device [%i] with serial number [%s]",
          device_num, device_serial.c_str());
      if (device_serial == serial_number)
      {
        device_ = device;
        RCLCPP_INFO(node_->get_logger(), "Found device with serial [%s]", device_serial.c_str());
        break;
      }
      else
      {
        k4a_device_close(device);
      }
    }
    else
    {
      RCLCPP_INFO(node_->get_logger(), "Failed to open device [%i], is it already in use?", device_num);
    }
  }
  if (device_ == nullptr)
  {
    throw std::runtime_error(
        "Unable to find k4a device with serial [" + serial_number + "]");
  }
  // Initialize
  Initialize(tf_topic);
}

K4ACamera::~K4ACamera()
{
  if (transformation_ != nullptr)
  {
    k4a_transformation_destroy(transformation_);
  }
  if (device_ != nullptr)
  {
    k4a_device_close(device_);
  }
}

void K4ACamera::Loop()
{
  if (k4a_device_start_cameras(device_, &device_configuration_)
      == K4A_RESULT_SUCCEEDED)
  {
    ScopedK4AImage transformed_color_image;
    ScopedK4AImage point_cloud_image;
    int previous_depth_width = -1;
    int previous_depth_height = -1;
    const int color_pixel_size = 4 * static_cast<int>(sizeof(uint8_t));
    const int depth_pixel_size = static_cast<int>(sizeof(int16_t));
    const int depth_point_size = 3 * static_cast<int>(sizeof(int16_t));
    while (rclcpp::ok())
    {
      ScopedK4ACapture capture;
      const auto result =
          k4a_device_get_capture(device_, &capture.Get(), 1000);
      if (result == K4A_WAIT_RESULT_SUCCEEDED)
      {
        const auto capture_time = node_->now();
        ScopedK4AImage depth_image(
            k4a_capture_get_depth_image(capture.Get()));
        ScopedK4AImage color_image(
            k4a_capture_get_color_image(capture.Get()));
        if (depth_image && color_image)
        {
          // Convert images to ROS image form
          sensor_msgs::msg::Image ros_color_image;
          ros_color_image.header.stamp = capture_time;
          ros_color_image.header.frame_id =
              color_camera_intrinsics_.header.frame_id;
          ros_color_image.width = color_camera_intrinsics_.width;
          ros_color_image.height = color_camera_intrinsics_.height;
          ros_color_image.encoding = "bgra8";
          ros_color_image.is_bigendian = false;
          ros_color_image.step =
              ros_color_image.width * static_cast<uint32_t>(color_pixel_size);
          ros_color_image.data.resize(
              ros_color_image.step * ros_color_image.height, 0x00);
          const uint8_t* color_image_buffer =
              k4a_image_get_buffer(color_image.Get());
          std::memcpy(ros_color_image.data.data(), color_image_buffer,
                      ros_color_image.data.size());
          sensor_msgs::msg::Image ros_depth_image;
          ros_depth_image.header.stamp = capture_time;
          ros_depth_image.header.frame_id =
              depth_camera_intrinsics_.header.frame_id;
          ros_depth_image.width = depth_camera_intrinsics_.width;
          ros_depth_image.height = depth_camera_intrinsics_.height;
          ros_depth_image.encoding = "mono16";
          ros_depth_image.is_bigendian = false;
          ros_depth_image.step =
              ros_depth_image.width * static_cast<uint32_t>(depth_pixel_size);
          ros_depth_image.data.resize(
              ros_depth_image.step * ros_depth_image.height, 0x00);
          const uint8_t* depth_image_buffer =
              k4a_image_get_buffer(depth_image.Get());
          std::memcpy(ros_depth_image.data.data(), depth_image_buffer,
                      ros_depth_image.data.size());
          // Check if the depth image is the same size as allocated
          const int depth_width =
              k4a_image_get_width_pixels(depth_image.Get());
          const int depth_height =
              k4a_image_get_height_pixels(depth_image.Get());
          if (depth_width != previous_depth_width
              || depth_height != previous_depth_height)
          {
            k4a_image_t new_transformed_color_image = nullptr;
            k4a_image_t new_point_cloud_image = nullptr;
            // Create a new image
            const auto transformed_color_image_create_result =
                k4a_image_create(
                    K4A_IMAGE_FORMAT_COLOR_BGRA32, depth_width, depth_height,
                    depth_width * color_pixel_size,
                    &new_transformed_color_image);
            // Create a new pointcloud
            const auto point_cloud_image_create_result =
                k4a_image_create(
                    K4A_IMAGE_FORMAT_CUSTOM, depth_width, depth_height,
                    depth_width * depth_point_size, &new_point_cloud_image);
            if (transformed_color_image_create_result == K4A_RESULT_SUCCEEDED
                && point_cloud_image_create_result == K4A_RESULT_SUCCEEDED)
            {
              previous_depth_width = depth_width;
              previous_depth_height = depth_height;
              transformed_color_image.Reset(new_transformed_color_image);
              point_cloud_image.Reset(new_point_cloud_image);
            }
            else
            {
              throw std::runtime_error(
                  "Failed to create transformed color image and pointcloud");
            }
          }
          // Transform the color image into the depth image
          if (k4a_transformation_color_image_to_depth_camera(
                  transformation_, depth_image.Get(), color_image.Get(),
                  transformed_color_image.Get()) != K4A_RESULT_SUCCEEDED)
          {
            throw std::runtime_error(
                "Failed to transform color image to depth camera");
          }
          // Transform the depth image into a pointcloud
          if (k4a_transformation_depth_image_to_point_cloud(
                  transformation_, depth_image.Get(),
                  K4A_CALIBRATION_TYPE_DEPTH, point_cloud_image.Get())
              != K4A_RESULT_SUCCEEDED)
          {
            throw std::runtime_error(
                "Failed to transform depth image to pointcloud");
          }
          // Get the color image buffer
          const uint8_t* transformed_color_image_buffer =
              k4a_image_get_buffer(transformed_color_image.Get());
          // Get the pointcloud image buffer
          const uint8_t* point_cloud_image_buffer =
              k4a_image_get_buffer(point_cloud_image.Get());
          // Create PointCloud2 message
          sensor_msgs::msg::PointCloud2 pointcloud_message;
          pointcloud_message.header.stamp = capture_time;
          pointcloud_message.header.frame_id =
              camera_to_depth_transform_.child_frame_id;
          pointcloud_message.height = depth_height;
          pointcloud_message.width = depth_width;
          pointcloud_message.is_bigendian = false;
          pointcloud_message.is_dense = true;
          pointcloud_message.point_step =
              3 * static_cast<uint32_t>(sizeof(float))
              + 4 * static_cast<uint32_t>(sizeof(uint8_t));
          pointcloud_message.row_step =
              pointcloud_message.width * pointcloud_message.point_step;
          // Populate the fields
          sensor_msgs::msg::PointField x_field;
          x_field.name = "x";
          x_field.offset = 0;
          x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
          x_field.count = 1;
          sensor_msgs::msg::PointField y_field;
          y_field.name = "y";
          y_field.offset = 4;
          y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
          y_field.count = 1;
          sensor_msgs::msg::PointField z_field;
          z_field.name = "z";
          z_field.offset = 8;
          z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
          z_field.count = 1;
          // Of course RViz + PCL need it this way
          sensor_msgs::msg::PointField rgb_field;
          rgb_field.name = "rgb";
          rgb_field.offset = 12;
          rgb_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
          rgb_field.count = 1;
          pointcloud_message.fields =
              {x_field, y_field, z_field, rgb_field};
          // Copy the data
          pointcloud_message.data.resize(
              pointcloud_message.row_step * pointcloud_message.height, 0x00);
          size_t color_image_position = 0;
          size_t point_cloud_position = 0;
          size_t pointcloud2_data_index = 0;
          int16_t point_buffer[3] = {0, 0, 0};
          uint8_t color_buffer[4] = {0x00, 0x00, 0x00, 0x00};
          for (int row = 0 ; row < depth_height ; row++)
          {
            for (int column = 0 ; column < depth_width ; column++)
            {
              // Grab the current point
              std::memcpy(
                  point_buffer,
                  point_cloud_image_buffer + point_cloud_position,
                  depth_point_size);
              // Convert to meters
              const float x = static_cast<float>(point_buffer[0]) * 0.001f;
              const float y = static_cast<float>(point_buffer[1]) * 0.001f;
              const float z = static_cast<float>(point_buffer[2]) * 0.001f;
              // Copy the point
              std::memcpy(
                  &pointcloud_message.data.at(
                      pointcloud2_data_index + x_field.offset),
                  &x, sizeof(float));
              std::memcpy(
                  &pointcloud_message.data.at(
                      pointcloud2_data_index + y_field.offset),
                  &y, sizeof(float));
              std::memcpy(
                  &pointcloud_message.data.at(
                      pointcloud2_data_index + z_field.offset),
                  &z, sizeof(float));
              // Grab the current color
              std::memcpy(
                  color_buffer,
                  transformed_color_image_buffer + color_image_position,
                  color_pixel_size);
              // Copy the color while converting to RGB
              pointcloud_message.data.at(
                  pointcloud2_data_index + rgb_field.offset + 0)
                      = color_buffer[0];
              pointcloud_message.data.at(
                  pointcloud2_data_index + rgb_field.offset + 1)
                      = color_buffer[1];
              pointcloud_message.data.at(
                  pointcloud2_data_index + rgb_field.offset + 2)
                      = color_buffer[2];
              // Step forward
              color_image_position += static_cast<size_t>(color_pixel_size);
              point_cloud_position += static_cast<size_t>(depth_point_size);
              pointcloud2_data_index +=
                  static_cast<size_t>(pointcloud_message.point_step);
            }
          }
          // Update the camera transform timestamps
          for (auto& camera_transform : camera_transforms_.transforms)
          {
            camera_transform.header.stamp = capture_time;
          }
          // Update the camera info timestamps
          color_camera_intrinsics_.header.stamp = capture_time;
          depth_camera_intrinsics_.header.stamp = capture_time;
          // Publish
          tf_pub_->publish(camera_transforms_);
          pointcloud_pub_->publish(pointcloud_message);
          color_pub_.publish(ros_color_image, color_camera_intrinsics_);
          depth_pub_.publish(ros_depth_image, depth_camera_intrinsics_);
        }
        else
        {
          if (!depth_image)
          {
            throw std::runtime_error(
                "Failed to get depth image from capture");
          }
          if (!color_image)
          {
            throw std::runtime_error(
                "Failed to get color image from capture");
          }
        }
      }
      else if (result == K4A_WAIT_RESULT_TIMEOUT)
      {
        throw std::runtime_error("Timeout waiting for capture");
      }
      else if (result == K4A_WAIT_RESULT_FAILED)
      {
        throw std::runtime_error("Failed to read capture");
      }
      rclcpp::spin_some(node_);
    }
  }
  else
  {
    throw std::runtime_error("Failed to start cameras");
  }
}

std::string K4ACamera::GetSerialNumber(const k4a_device_t device)
{
  // Get the serial number
  size_t serial_num_buffer_size = 0;
  // Call once to get the buffer size
  k4a_device_get_serialnum(device, nullptr, &serial_num_buffer_size);
  std::string serial_num(serial_num_buffer_size - 1, 0x00);
  if (k4a_device_get_serialnum(
          device, &serial_num.at(0), &serial_num_buffer_size)
      != K4A_BUFFER_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to get device serial number");
  }
  return serial_num;
}

void K4ACamera::Initialize(const std::string& tf_topic)
{
  LoadHardwareInformation();
  LoadCameraIntrinsics();
  pointcloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          camera_name_ + "/points", rclcpp::QoS(rclcpp::KeepLast(1)));
  tf_pub_ = node_->create_publisher<tf2_msgs::msg::TFMessage>(tf_topic, rclcpp::QoS(rclcpp::KeepLast(1)));
  color_pub_ = it_.advertiseCamera(camera_name_ + "/color/image", 1, false);
  depth_pub_ = it_.advertiseCamera(camera_name_ + "/depth/image", 1, false);
}

void K4ACamera::LoadHardwareInformation()
{
  // Get the serial number
  const std::string serial_num = GetSerialNumber(device_);
  RCLCPP_INFO(node_->get_logger(), "K4A Device serial [%s]", serial_num.c_str());
  // Get the firmware version
  k4a_hardware_version_t hardware_version;
  if (k4a_device_get_version(device_, &hardware_version)
      != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to get device hardware version");
  }
  RCLCPP_INFO(node_->get_logger(), 
      "RGB Camera firmware version %u.%u.%u",
      hardware_version.rgb.major, hardware_version.rgb.minor,
      hardware_version.rgb.iteration);
  RCLCPP_INFO(node_->get_logger(), 
      "Depth Camera firmware version %u.%u.%u",
      hardware_version.depth.major, hardware_version.depth.minor,
      hardware_version.depth.iteration);
  RCLCPP_INFO(node_->get_logger(), 
      "Audio firmware version %u.%u.%u",
      hardware_version.audio.major, hardware_version.audio.minor,
      hardware_version.audio.iteration);
  RCLCPP_INFO(node_->get_logger(), 
      "Depth Sensor firmware version %u.%u.%u",
      hardware_version.depth_sensor.major,
      hardware_version.depth_sensor.minor,
      hardware_version.depth_sensor.iteration);
  if (hardware_version.firmware_build == K4A_FIRMWARE_BUILD_RELEASE)
  {
    RCLCPP_INFO(node_->get_logger(), "Firmware type = K4A_FIRMWARE_BUILD_RELEASE");
  }
  else if (hardware_version.firmware_build == K4A_FIRMWARE_BUILD_DEBUG)
  {
    RCLCPP_INFO(node_->get_logger(), "Firmware type = K4A_FIRMWARE_BUILD_DEBUG");
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Unrecognized firmware build type");
  }
  if (hardware_version.firmware_signature == K4A_FIRMWARE_SIGNATURE_MSFT)
  {
    RCLCPP_INFO(node_->get_logger(), "Firmware signature = K4A_FIRMWARE_SIGNATURE_MSFT");
  }
  else if (hardware_version.firmware_signature == K4A_FIRMWARE_SIGNATURE_TEST)
  {
    RCLCPP_INFO(node_->get_logger(), "Firmware signature = K4A_FIRMWARE_SIGNATURE_TEST");
  }
  else if (hardware_version.firmware_signature
          == K4A_FIRMWARE_SIGNATURE_UNSIGNED)
  {
    RCLCPP_INFO(node_->get_logger(), "Firmware signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED");
  }
  else
  {
    RCLCPP_WARN(node_->get_logger(), "Unrecognized firmware signature type");
  }
}

void K4ACamera::LoadCameraIntrinsics()
{
  // Get the device camera->camera calibration
  if (k4a_device_get_calibration(
          device_, device_configuration_.depth_mode,
          device_configuration_.color_resolution, &calibration_)
      != K4A_RESULT_SUCCEEDED)
  {
    throw std::runtime_error("Failed to get device calibration");
  }
  // Make the image transformation
  transformation_ = k4a_transformation_create(&calibration_);
  // Get the in-camera transforms
  camera_to_color_transform_ =
      ExtrinsicsToTransformStamped(
          calibration_.color_camera_calibration.extrinsics,
          camera_name_ + "_frame", camera_name_ + "_color_optical_frame");
  camera_to_depth_transform_ =
      ExtrinsicsToTransformStamped(
          calibration_.depth_camera_calibration.extrinsics,
          camera_name_ + "_frame", camera_name_ + "_depth_optical_frame");
  camera_transforms_.transforms =
      {camera_to_color_transform_, camera_to_depth_transform_};
  // Get the camera intrinsics
  color_camera_intrinsics_ =
      IntrinsicsToCameraInfo(
          calibration_.color_camera_calibration.intrinsics,
          static_cast<uint32_t>(
              calibration_.color_camera_calibration.resolution_width),
          static_cast<uint32_t>(
              calibration_.color_camera_calibration.resolution_height),
          camera_to_color_transform_.child_frame_id);
  depth_camera_intrinsics_ =
      IntrinsicsToCameraInfo(
          calibration_.depth_camera_calibration.intrinsics,
          static_cast<uint32_t>(
              calibration_.depth_camera_calibration.resolution_width),
          static_cast<uint32_t>(
              calibration_.depth_camera_calibration.resolution_height),
          camera_to_depth_transform_.child_frame_id);
}
}  // namespace k4a_driver
