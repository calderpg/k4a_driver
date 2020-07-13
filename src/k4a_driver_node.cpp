#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <k4a/k4a.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_msgs/TFMessage.h>

geometry_msgs::TransformStamped ExtrinsicsToTransformStamped(
    const k4a_calibration_extrinsics_t& extrinsics,
    const std::string& source_frame_name, const std::string& target_frame_name)
{
  geometry_msgs::TransformStamped transform_msg;
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

sensor_msgs::CameraInfo IntrinsicsToCameraInfo(
    const k4a_calibration_intrinsics_t& intrinsics, const uint32_t width,
    const uint32_t height, const std::string& camera_optical_frame_name)
{
  sensor_msgs::CameraInfo intrinsics_msg;
  intrinsics_msg.header.frame_id = camera_optical_frame_name;
  intrinsics_msg.width = width;
  intrinsics_msg.height = height;
  // Distortion model
  intrinsics_msg.distortion_model = "plumb_bob";
  intrinsics_msg.D =
      {intrinsics.parameters.param.k1,
       intrinsics.parameters.param.k2,
       intrinsics.parameters.param.p1,
       intrinsics.parameters.param.p2,
       intrinsics.parameters.param.k3};
  // Intrinsic matrix
  intrinsics_msg.K =
      {intrinsics.parameters.param.fx, 0.0, intrinsics.parameters.param.cx,
       0.0, intrinsics.parameters.param.fy, intrinsics.parameters.param.cy,
       0.0 , 0.0, 1.0};
  // Rectification matrix
  intrinsics_msg.R =
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0};
  // Projection matrix
  intrinsics_msg.P =
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

class K4ACamera
{
public:
  K4ACamera(
      const ros::NodeHandle& nh, const std::string& tf_topic,
      const std::string& camera_name, const int32_t device_number,
      const k4a_device_configuration_t& device_configuration)
      : nh_(nh), it_(nh_), camera_name_(camera_name),
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
    ROS_INFO("Opened device [%i]", device_number);
    LoadHardwareInformation();
    LoadCameraIntrinsics();
    pointcloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>(
            camera_name_ + "/points", 1, false);
    tf_pub_ = nh_.advertise<tf2_msgs::TFMessage>(tf_topic, 1, false);
    color_pub_ = it_.advertiseCamera(camera_name_ + "/color/image", 1, false);
    depth_pub_ = it_.advertiseCamera(camera_name_ + "/depth/image", 1, false);
  }

  ~K4ACamera()
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

  void Loop()
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
      while (ros::ok())
      {
        ScopedK4ACapture capture;
        const auto result =
            k4a_device_get_capture(device_, &capture.Get(), 1000);
        if (result == K4A_WAIT_RESULT_SUCCEEDED)
        {
          const auto capture_time = ros::Time::now();
          ScopedK4AImage depth_image(
              k4a_capture_get_depth_image(capture.Get()));
          ScopedK4AImage color_image(
              k4a_capture_get_color_image(capture.Get()));
          if (depth_image && color_image)
          {
            // Convert images to ROS image form
            sensor_msgs::Image ros_color_image;
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
            sensor_msgs::Image ros_depth_image;
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
            sensor_msgs::PointCloud2 pointcloud_message;
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
            sensor_msgs::PointField x_field;
            x_field.name = "x";
            x_field.offset = 0;
            x_field.datatype = sensor_msgs::PointField::FLOAT32;
            x_field.count = 1;
            sensor_msgs::PointField y_field;
            y_field.name = "y";
            y_field.offset = 4;
            y_field.datatype = sensor_msgs::PointField::FLOAT32;
            y_field.count = 1;
            sensor_msgs::PointField z_field;
            z_field.name = "z";
            z_field.offset = 8;
            z_field.datatype = sensor_msgs::PointField::FLOAT32;
            z_field.count = 1;
            // Of course RViz + PCL need it this way
            sensor_msgs::PointField rgb_field;
            rgb_field.name = "rgb";
            rgb_field.offset = 12;
            rgb_field.datatype = sensor_msgs::PointField::FLOAT32;
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
            tf_pub_.publish(camera_transforms_);
            pointcloud_pub_.publish(pointcloud_message);
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
        ros::spinOnce();
      }
    }
    else
    {
      throw std::runtime_error("Failed to start cameras");
    }
  }

private:
  void LoadHardwareInformation()
  {
    // Get the serial number
    size_t serial_num_buffer_size = 0;
    // Call once to get the buffer size
    k4a_device_get_serialnum(device_, nullptr, &serial_num_buffer_size);
    std::string serial_num(serial_num_buffer_size, 0x00);
    if (k4a_device_get_serialnum(
            device_, &serial_num.at(0), &serial_num_buffer_size)
        != K4A_BUFFER_RESULT_SUCCEEDED)
    {
      throw std::runtime_error("Failed to get device serial number");
    }
    ROS_INFO("K4A Device serial [%s]", serial_num.c_str());
    // Get the firmware version
    k4a_hardware_version_t hardware_version;
    if (k4a_device_get_version(device_, &hardware_version)
        != K4A_RESULT_SUCCEEDED)
    {
      throw std::runtime_error("Failed to get device hardware version");
    }
    ROS_INFO(
        "RGB Camera firmware version %u.%u.%u",
        hardware_version.rgb.major, hardware_version.rgb.minor,
        hardware_version.rgb.iteration);
    ROS_INFO(
        "Depth Camera firmware version %u.%u.%u",
        hardware_version.depth.major, hardware_version.depth.minor,
        hardware_version.depth.iteration);
    ROS_INFO(
        "Audio firmware version %u.%u.%u",
        hardware_version.audio.major, hardware_version.audio.minor,
        hardware_version.audio.iteration);
    ROS_INFO(
        "Depth Sensor firmware version %u.%u.%u",
        hardware_version.depth_sensor.major,
        hardware_version.depth_sensor.minor,
        hardware_version.depth_sensor.iteration);
    if (hardware_version.firmware_build == K4A_FIRMWARE_BUILD_RELEASE)
    {
      ROS_INFO("Firmware type = K4A_FIRMWARE_BUILD_RELEASE");
    }
    else if (hardware_version.firmware_build == K4A_FIRMWARE_BUILD_DEBUG)
    {
      ROS_INFO("Firmware type = K4A_FIRMWARE_BUILD_DEBUG");
    }
    else
    {
      ROS_WARN("Unrecognized firmware build type");
    }
    if (hardware_version.firmware_signature == K4A_FIRMWARE_SIGNATURE_MSFT)
    {
      ROS_INFO("Firmware signature = K4A_FIRMWARE_SIGNATURE_MSFT");
    }
    else if (hardware_version.firmware_signature == K4A_FIRMWARE_SIGNATURE_TEST)
    {
      ROS_INFO("Firmware signature = K4A_FIRMWARE_SIGNATURE_TEST");
    }
    else if (hardware_version.firmware_signature
            == K4A_FIRMWARE_SIGNATURE_UNSIGNED)
    {
      ROS_INFO("Firmware signature = K4A_FIRMWARE_SIGNATURE_UNSIGNED");
    }
    else
    {
      ROS_WARN("Unrecognized firmware signature type");
    }
  }

  void LoadCameraIntrinsics()
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
  // Create the camera
  K4ACamera camera(nh, tf_topic, camera_name, device_number, config);
  // Capture
  camera.Loop();
  return 0;
}
