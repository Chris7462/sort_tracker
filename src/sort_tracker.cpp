#include "sort_tracker/sort_tracker.hpp"


namespace sort_tracker
{

SortTracker::SortTracker()
: Node("sort_tracker_node")
{
  // Initialize message filter subscribers
  img_sub_.subscribe(this, "/kitti/camera/color/left/image_raw");
  det_sub_.subscribe(this, "/fcos_object_detection/detection_array");

  // Create exact time synchronizer
  sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>>(
    img_sub_, det_sub_, 10);

  // Register callback for synchronized messages
  sync_->registerCallback(std::bind(&SortTracker::syncCallback, this,
    std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "SortTracker node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to:");
  RCLCPP_INFO(this->get_logger(), "  - kitti/camera/color/left/image_raw");
  RCLCPP_INFO(this->get_logger(), "  - fcos_object_detection/detection_array");
}

void SortTracker::syncCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr& detection_msg)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "I'm here!!!");

  // Verify timestamps match (they should due to exact time sync)
  if (image_msg->header.stamp != detection_msg->header.stamp) {
    RCLCPP_WARN(this->get_logger(), "Timestamp mismatch detected!");
    return;
  }

  RCLCPP_DEBUG(this->get_logger(), "Received synchronized messages with timestamp: %d.%d",
      image_msg->header.stamp.sec, image_msg->header.stamp.nanosec);

  // Process the synchronized image and detections
  //processImageAndDetections(image_msg, detection_msg);
}

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

}  // namespace sort_tracker
