#pragma once

// C++ header
#include <atomic>
#include <memory>
#include <mutex>
#include <queue>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <std_msgs/msg/header.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// SORT backend headers
#include "sort_backend/sort_backend.hpp"


namespace sort_tracker
{

class SortTracker : public rclcpp::Node
{

public:
  /**
   * @brief Constructor for SortTracker node
   */
  SortTracker();

  /**
   * @brief Destructor for SortTracker node
   */
  ~SortTracker();

private:
  /**
   * @brief Initialize node parameters with validation
   * @return true if initialization successful, false otherwise
   */
  bool initialize_parameters();

  /**
   * @brief Initialize SORT tracking backend
   * @return true if initialization successful, false otherwise
   */
  bool initialize_tracker();

  /**
   * @brief Initialize ROS2 publishers, subscribers, and timers
   */
  void initialize_ros_components();

  /**
   * @brief Callback for incoming detections
   * @param detection_msg Incoming detection array message
   */
  void detection_callback(vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg);

  /**
   * @brief Timer callback for processing queued detections at regular intervals
   */
  void timer_callback();

  /**
   * @brief Convert ROS detection message to SORT input format
   * @param detection_msg ROS detection array message
   * @return Eigen matrix in format [x1, y1, x2, y2, score] for each detection
   */
  Eigen::MatrixXf convert_detections_to_sort_format(
    vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg);

  /**
   * @brief Convert SORT tracking output back into a ROS detection message
   * @param tracking_results SORT output matrix [x1, y1, x2, y2, track_id]
   * @param header Original message header for timestamp consistency
   * @return Detection2DArray with bbox from tracking_results and id set to the track id
   */
  vision_msgs::msg::Detection2DArray convert_sort_output_to_detections(
    const Eigen::MatrixXf & tracking_results,
    const std_msgs::msg::Header & header);

  /**
   * @brief Publish tracked detections
   * @param tracked_detections Detection2DArray with track ids populated
   */
  void publish_tracked_detections(const vision_msgs::msg::Detection2DArray & tracked_detections);

private:
  // ROS2 subscriber
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;

  // ROS2 publisher
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr tracker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group for parallel execution
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // SORT tracker backend
  std::shared_ptr<sort::Sort> tracker_backend_;

  // ROS2 parameters
  std::string detection_input_topic_;
  std::string tracking_output_topic_;
  int queue_size_;
  double processing_frequency_;
  int max_processing_queue_size_;

  // SORT backend parameters
  int max_age_;
  int min_hits_;
  float iou_threshold_;

  // Detection confidence threshold parameter
  float detection_confidence_threshold_;

  // Queued detection buffer
  std::queue<vision_msgs::msg::Detection2DArray::ConstSharedPtr> detection_buff_;
  std::mutex mtx_;
  std::atomic<bool> processing_in_progress_;
};

} // namespace sort_tracker
