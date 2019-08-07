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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "k4a_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  image_transport::ImageTransport it(nh);
  // Set up publisher
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
  ros::Publisher pointcloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(camera_name + "/points", 1, false);
  ros::Publisher tf_pub = nh.advertise<tf2_msgs::TFMessage>(tf_topic, 1, false);
  image_transport::CameraPublisher color_pub =
      it.advertiseCamera(camera_name + "/color/image", 1, false);
  image_transport::CameraPublisher depth_pub =
      it.advertiseCamera(camera_name + "/depth/image", 1, false);
  // Set up device
  const uint32_t device_count = k4a_device_get_installed_count();
  if (device_count == 0)
  {
    ROS_FATAL("No k4a devices found");
  }
  if (device_number < 0)
  {
    ROS_FATAL("device_number [%i] less than zero", device_number);
  }
  else if (static_cast<uint32_t>(device_number) >= device_count)
  {
    ROS_FATAL("device_number [%i] greater than device_count [%i]",
              device_number, device_count);
  }
  // Configure the device
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  if (color_resolution == "720p" || color_resolution == "720P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  }
  else if (color_resolution == "1080p" || color_resolution == "1080P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
  }
  else if (color_resolution == "1440p" || color_resolution == "1440P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1440P;
  }
  else if (color_resolution == "1536p" || color_resolution == "1536P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
  }
  else if (color_resolution == "2160p" || color_resolution == "2160P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
  }
  else if (color_resolution == "3072p" || color_resolution == "3072P")
  {
    config.color_resolution = K4A_COLOR_RESOLUTION_3072P;
  }
  else
  {
    ROS_FATAL(
        "Invalid color resolution option [%s], valid options are 720p, 1080p, "
        "1440p, 1536p, 2160p, or 3072p", color_resolution.c_str());
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
  }
  else if (nominal_frame_rate == 15)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_15;
  }
  else if (nominal_frame_rate == 30)
  {
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  }
  else
  {
    ROS_FATAL("Invalid frame rate [%i], valid options are 5, 15, or 30",
              nominal_frame_rate);
  }
  if (sync_mode == "master")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
  }
  else if (sync_mode == "subordinate")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
  }
  else if (sync_mode == "standalone")
  {
    config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
  }
  else
  {
    ROS_FATAL("Invalid sync mode [%s], valid options are standalone, master, or"
              " subordinate", sync_mode.c_str());
  }
  if (subordinate_sync_delay_usec >= 0)
  {
    config.subordinate_delay_off_master_usec =
        static_cast<uint32_t>(subordinate_sync_delay_usec);
  }
  else
  {
    ROS_FATAL("subordinate_sync_delay_usec must be >= 0");
  }
  config.synchronized_images_only = true;
  // Get the default device
  k4a_device_t device = nullptr;
  if (k4a_device_open(static_cast<uint32_t>(device_number), &device)
      != K4A_RESULT_SUCCEEDED)
  {
    if (device != nullptr)
    {
      k4a_device_close(device);
    }
    ROS_FATAL("Failed to open device");
  }
  ROS_INFO("Opened device [%i]", device_number);
  // Get the device camera->camera calibration
  k4a_calibration_t calibration;
  if (k4a_device_get_calibration(
          device, config.depth_mode, config.color_resolution, &calibration)
      != K4A_RESULT_SUCCEEDED)
  {
    if (device != nullptr)
    {
      k4a_device_close(device);
    }
    ROS_FATAL("Failed to get device calibration");
  }
  // Make the image transformation
  k4a_transformation_t transformation = k4a_transformation_create(&calibration);
  // Get the in-camera transforms
  const geometry_msgs::TransformStamped camera_to_color_transform =
      ExtrinsicsToTransformStamped(
          calibration.color_camera_calibration.extrinsics,
          camera_name + "_frame", camera_name + "_color_optical_frame");
  const geometry_msgs::TransformStamped camera_to_depth_transform =
      ExtrinsicsToTransformStamped(
          calibration.depth_camera_calibration.extrinsics,
          camera_name + "_frame", camera_name + "_depth_optical_frame");
  tf2_msgs::TFMessage camera_transforms;
  camera_transforms.transforms =
      {camera_to_color_transform, camera_to_depth_transform};
  // Get the camera intrinsics
  sensor_msgs::CameraInfo color_camera_intrinsics =
      IntrinsicsToCameraInfo(
          calibration.color_camera_calibration.intrinsics,
          static_cast<uint32_t>(
              calibration.color_camera_calibration.resolution_width),
          static_cast<uint32_t>(
              calibration.color_camera_calibration.resolution_height),
          camera_to_color_transform.child_frame_id);
  sensor_msgs::CameraInfo depth_camera_intrinsics =
      IntrinsicsToCameraInfo(
          calibration.depth_camera_calibration.intrinsics,
          static_cast<uint32_t>(
              calibration.depth_camera_calibration.resolution_width),
          static_cast<uint32_t>(
              calibration.depth_camera_calibration.resolution_height),
          camera_to_depth_transform.child_frame_id);
  // Start capture
  if (k4a_device_start_cameras(device, &config) == K4A_RESULT_SUCCEEDED)
  {
    k4a_capture_t capture = nullptr;
    k4a_image_t color_image = nullptr;
    k4a_image_t depth_image = nullptr;
    k4a_image_t transformed_color_image = nullptr;
    k4a_image_t point_cloud_image = nullptr;
    int previous_depth_width = -1;
    int previous_depth_height = -1;
    const int color_pixel_size = 4 * static_cast<int>(sizeof(uint8_t));
    const int depth_pixel_size = static_cast<int>(sizeof(int16_t));
    const int depth_point_size = 3 * static_cast<int>(sizeof(int16_t));
    while (ros::ok())
    {
      const auto result = k4a_device_get_capture(device, &capture, 1000);
      if (result == K4A_WAIT_RESULT_SUCCEEDED)
      {
        const auto capture_time = ros::Time::now();
        depth_image = k4a_capture_get_depth_image(capture);
        color_image = k4a_capture_get_color_image(capture);
        if (depth_image != nullptr && color_image != nullptr)
        {
          // Convert images to ROS image form
          sensor_msgs::Image ros_color_image;
          ros_color_image.header.stamp = capture_time;
          ros_color_image.header.frame_id =
              color_camera_intrinsics.header.frame_id;
          ros_color_image.width = color_camera_intrinsics.width;
          ros_color_image.height = color_camera_intrinsics.height;
          ros_color_image.encoding = "bgra8";
          ros_color_image.is_bigendian = false;
          ros_color_image.step =
              ros_color_image.width * static_cast<uint32_t>(color_pixel_size);
          ros_color_image.data.resize(
              ros_color_image.step * ros_color_image.height, 0x00);
          const uint8_t* color_image_buffer = k4a_image_get_buffer(color_image);
          std::memcpy(ros_color_image.data.data(), color_image_buffer,
                      ros_color_image.data.size());
          sensor_msgs::Image ros_depth_image;
          ros_depth_image.header.stamp = capture_time;
          ros_depth_image.header.frame_id =
              depth_camera_intrinsics.header.frame_id;
          ros_depth_image.width = depth_camera_intrinsics.width;
          ros_depth_image.height = depth_camera_intrinsics.height;
          ros_depth_image.encoding = "mono16";
          ros_depth_image.is_bigendian = false;
          ros_depth_image.step =
              ros_depth_image.width * static_cast<uint32_t>(depth_pixel_size);
          ros_depth_image.data.resize(
              ros_depth_image.step * ros_depth_image.height, 0x00);
          const uint8_t* depth_image_buffer = k4a_image_get_buffer(depth_image);
          std::memcpy(ros_depth_image.data.data(), depth_image_buffer,
                      ros_depth_image.data.size());
          // Check if the depth image is the same size as allocated
          const int depth_width = k4a_image_get_width_pixels(depth_image);
          const int depth_height = k4a_image_get_height_pixels(depth_image);
          if (depth_width != previous_depth_width
              || depth_height != previous_depth_height)
          {
            // Release the old image
            if (transformed_color_image != nullptr)
            {
              k4a_image_release(transformed_color_image);
              transformed_color_image = nullptr;
            }
            // Release the old pointcloud
            if (point_cloud_image != nullptr)
            {
              k4a_image_release(point_cloud_image);
              point_cloud_image = nullptr;
            }
            // Create a new image
            const auto transformed_color_image_create_result =
                k4a_image_create(
                    K4A_IMAGE_FORMAT_COLOR_BGRA32, depth_width, depth_height,
                    depth_width * color_pixel_size, &transformed_color_image);
            // Create a new pointcloud
            const auto point_cloud_image_create_result =
                k4a_image_create(
                    K4A_IMAGE_FORMAT_CUSTOM, depth_width, depth_height,
                    depth_width * depth_point_size, &point_cloud_image);
            if (transformed_color_image_create_result == K4A_RESULT_SUCCEEDED
                && point_cloud_image_create_result == K4A_RESULT_SUCCEEDED)
            {
              previous_depth_width = depth_width;
              previous_depth_height = depth_height;
            }
            else
            {
              ROS_ERROR(
                  "Failed to create transformed color image and pointcloud");
              break;
            }
          }
          // Transform the color image into the depth image
          if (k4a_transformation_color_image_to_depth_camera(
                  transformation, depth_image, color_image,
                  transformed_color_image) != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR("Failed to transform color image to depth camera");
            break;
          }
          // Transform the depth image into a pointcloud
          if (k4a_transformation_depth_image_to_point_cloud(
                  transformation, depth_image, K4A_CALIBRATION_TYPE_DEPTH,
                  point_cloud_image) != K4A_RESULT_SUCCEEDED)
          {
            ROS_ERROR("Failed to transform depth image to pointcloud");
          }
          // Get the color image buffer
          const uint8_t* transformed_color_image_buffer =
              k4a_image_get_buffer(transformed_color_image);
          if (transformed_color_image_buffer == nullptr)
          {
            ROS_ERROR("Failed to get transformed_color_image buffer");
            break;
          }
          // Get the pointcloud image buffer
          const uint8_t* point_cloud_image_buffer =
              k4a_image_get_buffer(point_cloud_image);
          if (point_cloud_image_buffer == nullptr)
          {
            ROS_ERROR("Failed to get point_cloud_image buffer");
            break;
          }
          // Create PointCloud2 message
          sensor_msgs::PointCloud2 pointcloud_message;
          pointcloud_message.header.stamp = capture_time;
          pointcloud_message.header.frame_id =
              camera_to_depth_transform.child_frame_id;
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
                  point_buffer, point_cloud_image_buffer + point_cloud_position,
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
          for (auto& camera_transform : camera_transforms.transforms)
          {
            camera_transform.header.stamp = capture_time;
          }
          // Update the camera info timestamps
          color_camera_intrinsics.header.stamp = capture_time;
          depth_camera_intrinsics.header.stamp = capture_time;
          // Publish
          tf_pub.publish(camera_transforms);
          pointcloud_pub.publish(pointcloud_message);
          color_pub.publish(ros_color_image, color_camera_intrinsics);
          depth_pub.publish(ros_depth_image, depth_camera_intrinsics);
        }
        else
        {
          if (depth_image == nullptr)
          {
            ROS_ERROR("Failed to get depth image from capture");
          }
          if (color_image == nullptr)
          {
            ROS_ERROR("Failed to get color image from capture");
          }
          break;
        }
      }
      else if (result == K4A_WAIT_RESULT_TIMEOUT)
      {
        ROS_ERROR("Timeout waiting for capture");
        break;
      }
      else if (result == K4A_WAIT_RESULT_FAILED)
      {
        ROS_ERROR("Failed to read capture");
      }
      // Release the current capture + images
      if (depth_image != nullptr)
      {
        k4a_image_release(depth_image);
        depth_image = nullptr;
      }
      if (color_image != nullptr)
      {
        k4a_image_release(color_image);
        color_image = nullptr;
      }
      if (capture != nullptr)
      {
        k4a_capture_release(capture);
        capture = nullptr;
      }
      ros::spinOnce();
    }
    if (point_cloud_image != nullptr)
    {
      k4a_image_release(point_cloud_image);
    }
    if (transformed_color_image != nullptr)
    {
      k4a_image_release(transformed_color_image);
    }
    if (depth_image != nullptr)
    {
      k4a_image_release(depth_image);
    }
    if (color_image != nullptr)
    {
      k4a_image_release(color_image);
    }
    if (capture != nullptr)
    {
      k4a_capture_release(capture);
    }
  }
  else
  {
    ROS_ERROR("Failed to start cameras");
  }
  if (transformation != nullptr)
  {
    k4a_transformation_destroy(transformation);
  }
  if (device != nullptr)
  {
    k4a_device_close(device);
  }
  return 0;
}
