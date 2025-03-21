/**
* This file is part of R-VIO2.
*
* Copyright (C) 2022 Zheng Huai <zhuai@udel.edu> and Guoquan Huang <ghuang@udel.edu>
* For more information see <http://github.com/rpng/R-VIO2>
*
* R-VIO2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* R-VIO2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with R-VIO2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <string>
#include <memory>
#include <opencv2/core/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "rvio2/System.h"

// ROS2 rosbag2 includes
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/metadata_io.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if(argc != 3)
  {
      RCLCPP_ERROR(rclcpp::get_logger("rvio2_mono_eval"),
                   "Usage: ros2 run <package> rvio2_mono_eval [path_to_settings] [path_to_rosbag]");
      rclcpp::shutdown();
      return -1;
  }

  // Initialize your system with settings file (argv[1])
  RVIO2::System Sys(argv[1]);

  cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);
  std::string path_to_bag(argv[2]);

  // Create and open a rosbag2 reader.
  rosbag2_cpp::Reader reader;
  reader.open(path_to_bag);

  // Retrieve bag metadata to determine overall start and end times.
  auto metadata = reader.get_metadata();
  // The metadata times are given in nanoseconds.
  rclcpp::Time bag_start(metadata->starting_time, RCL_SYSTEM_TIME);
  rclcpp::Time bag_end(metadata->starting_time + metadata->duration, RCL_SYSTEM_TIME);

  // Use settings from the config file to determine playback timing.
  int bag_skip = fsSettings["INI.nTimeskip"];  // seconds to skip at beginning
  int bag_durr = -1;                           // duration; negative means play full bag

  // Compute the start and finish times for bag playback.
  rclcpp::Time time_init = bag_start + rclcpp::Duration(static_cast<int64_t>(bag_skip * 1e9));
  rclcpp::Time time_finish = (bag_durr < 0) ? bag_end : (time_init + rclcpp::Duration(static_cast<int64_t>(bag_durr * 1e9)));

  RCLCPP_INFO(rclcpp::get_logger("rvio2_mono_eval"), "time start = %.6f", time_init.seconds());
  RCLCPP_INFO(rclcpp::get_logger("rvio2_mono_eval"), "time end   = %.6f", time_finish.seconds());

  // Loop through the bag messages.
  while (reader.has_next() && rclcpp::ok())
  {
    // Read the next serialized message from the bag.
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message = reader.read_next();
    // Convert the bag message time (nanoseconds) to rclcpp::Time.
    rclcpp::Time msg_time(bag_message->time_stamp, RCL_SYSTEM_TIME);
    if (msg_time < time_init || msg_time > time_finish)
    {
      continue;
    }

    std::string topic = bag_message->topic_name;

    // Create an rclcpp::SerializedMessage wrapping the bag message data.
    rclcpp::SerializedMessage serialized_msg;
    // (Note: The following accesses the underlying serialized data.
    // In production code you may wish to copy the data to ensure its lifetime.)
    serialized_msg.get_rcl_serialized_message().buffer_length = bag_message->serialized_data.size();
    serialized_msg.get_rcl_serialized_message().buffer = bag_message->serialized_data.data();

    // ----- Handle IMU messages -----
    if(topic == "/imu0" || topic == "/imu")
    {
      rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
      sensor_msgs::msg::Imu imu_msg;
      imu_ser.deserialize_message(&serialized_msg, &imu_msg);

      // Note: The ROS2 header does not include a sequence number.
      Eigen::Vector3f angular_velocity(imu_msg.angular_velocity.x,
                                       imu_msg.angular_velocity.y,
                                       imu_msg.angular_velocity.z);
      Eigen::Vector3f linear_acceleration(imu_msg.linear_acceleration.x,
                                          imu_msg.linear_acceleration.y,
                                          imu_msg.linear_acceleration.z);

      double currtime = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9;

      RVIO2::ImuData* pData = new RVIO2::ImuData();
      pData->AngularVel = angular_velocity;
      pData->LinearAccel = linear_acceleration;
      pData->Timestamp = currtime;

      static double lasttime = -1;
      if(lasttime != -1)
          pData->TimeInterval = currtime - lasttime;
      else
          pData->TimeInterval = 0;
      lasttime = currtime;

      Sys.PushImuData(pData);
    }
    // ----- Handle image messages -----
    else if(topic == "/cam0/image_raw" || topic == "/camera/image_raw")
    {
      rclcpp::Serialization<sensor_msgs::msg::Image> image_ser;
      sensor_msgs::msg::Image img_msg;
      image_ser.deserialize_message(&serialized_msg, &img_msg);

      // Convert the ROS2 image message to an OpenCV image.
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        // Using toCvCopy; note that shared pointers to messages are expected.
        cv_ptr = cv_bridge::toCvCopy(std::make_shared<sensor_msgs::msg::Image>(img_msg),
                                     sensor_msgs::image_encodings::MONO8);
      }
      catch (cv_bridge::Exception &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("rvio2_mono_eval"), "cv_bridge exception: %s", e.what());
        continue;
      }

      RVIO2::ImageData* pData = new RVIO2::ImageData();
      pData->Image = cv_ptr->image.clone();
      pData->Timestamp = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9;

      Sys.PushImageData(pData);
      Sys.run();
    }
  }

  rclcpp::shutdown();
  return 0;
}
