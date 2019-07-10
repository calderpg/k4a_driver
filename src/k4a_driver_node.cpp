#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <k4a/k4a.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "k4a_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  // Set up publisher
  const std::string pointcloud_topic =
      nhp.param(std::string("pointcloud_topic"), std::string("k4a/points"));
  const std::string pointcloud_frame =
      nhp.param(std::string("pointcloud_frame"),
                std::string("k4a_depth_optical_frame"));
  ros::Publisher pointcloud_pub =
      nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 1, false);
  // Set up device
  const uint32_t device_count = k4a_device_get_installed_count();
  if (device_count == 0)
  {
    ROS_FATAL("No k4a devices found");
  }
  // Get the default device
  const uint8_t device_id = K4A_DEVICE_DEFAULT;
  k4a_device_t device = nullptr;
  if (k4a_device_open(device_id, &device) != K4A_RESULT_SUCCEEDED)
  {
    if (device != nullptr)
    {
      k4a_device_close(device);
    }
    ROS_ERROR("Failed to open device");
    return -1;
  }
  // Configure the device
  k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
  config.color_resolution = K4A_COLOR_RESOLUTION_720P;
  config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
  config.camera_fps = K4A_FRAMES_PER_SECOND_30;
  config.synchronized_images_only = true;
  // Get the device calibration
  k4a_calibration_t calibration;
  if (k4a_device_get_calibration(
          device, config.depth_mode, config.color_resolution, &calibration)
      != K4A_RESULT_SUCCEEDED)
  {
    if (device != nullptr)
    {
      k4a_device_close(device);
    }
    ROS_ERROR("Failed to get device calibration");
    return -1;
  }
  // Make the image transformation
  k4a_transformation_t transformation = k4a_transformation_create(&calibration);
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
          pointcloud_message.header.frame_id = pointcloud_frame;
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
          // Publish
          pointcloud_pub.publish(pointcloud_message);
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
      }
      if (color_image != nullptr)
      {
        k4a_image_release(color_image);
      }
      if (capture != nullptr)
      {
        k4a_capture_release(capture);
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

