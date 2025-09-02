#pragma once

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace sort_tracker
{

class SortTracker : public rclcpp::Node
{

public:
  /**
   * @brief Constructor of SortTracker node
   */
  SortTracker();
  ~SortTracker() = default;

private:
  /**
  * @brief Syunchronization callback for image and detection messages
  * @param image_msg The synchronized image message
  * @param detection_msg The synchronized detection message
  */
  void syncCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & detection_msg);

//void processImageAndDetections(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
//    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg)
//{
//  // TODO: Implement your SORT tracking algorithm here

//  // Example: Log detection information
//  RCLCPP_INFO(this->get_logger(), "Processing frame with %zu detections",
//      detection_msg->detections.size());

//  for (size_t i = 0; i < detection_msg->detections.size(); ++i) {
//    const auto& detection = detection_msg->detections[i];

//    // Extract bounding box center and dimensions
//    double center_x = detection.bbox.center.position.x;
//    double center_y = detection.bbox.center.position.y;
//    double width = detection.bbox.size_x;
//    double height = detection.bbox.size_y;

//    RCLCPP_DEBUG(this->get_logger(),
//        "Detection %zu: center(%.2f, %.2f), size(%.2f x %.2f)",
//        i, center_x, center_y, width, height);

//    // TODO: Feed detections to SORT tracker
//    // TODO: Update tracks
//    // TODO: Publish tracking results
//  }
//}

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> img_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;

  // Exact time synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>> sync_;
};

} // namespace sort_tracker
