#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/type_adapter.hpp>

template<>
struct rclcpp::TypeAdapter<cv::Mat, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = cv::Mat;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination = *(cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", source).toImageMsg());
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(source, "bgr8");  // "bgr8" は適切なエンコーディングに変更
    
    destination = cv_ptr->image;
  }
};
