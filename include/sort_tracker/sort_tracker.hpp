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
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// Message filters header
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

// OpenCV header
#include <opencv2/core.hpp>

// SORT backend headers
#include "sort_backend/sort.hpp"


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
   * @brief Initialize ROS2 publishers, subscribers, and timers with message filters
   */
  void initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming images and detections
   * @param image_msg Incoming image message
   * @param detection_msg Incoming detection array message
   */
  void synchronized_callback(
    const sensor_msgs::msg::Image::SharedPtr image_msg,
    const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg);

  /**
   * @brief Timer callback for processing synchronized data at regular intervals
   */
  void timer_callback();

  /**
   * @brief Convert ROS detection message to SORT input format
   * @param detection_msg ROS detection array message
   * @return Eigen matrix in format [x1, y1, x2, y2, score] for each detection
   */
  Eigen::MatrixXf convert_detections_to_sort_format(
    const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg);

  /**
   * @brief Draw tracking results on image with bounding boxes and track IDs
   * @param image OpenCV image to draw on
   * @param tracking_results SORT output matrix [x1, y1, x2, y2, track_id]
   */
  void draw_tracking_results(cv::Mat & image, const Eigen::MatrixXf & tracking_results);

  /**
   * @brief Generate consistent color for a track ID
   * @param track_id Track identifier
   * @return OpenCV Scalar color (BGR format)
   */
  cv::Scalar get_track_color(int track_id);

  /**
   * @brief Publish tracking result image with bounding boxes and track IDs
   * @param result_image Tracking result as OpenCV Mat
   * @param header Original message header for timestamp consistency
   */
  void publish_tracking_result_image(
    const cv::Mat & result_image,
    const std_msgs::msg::Header & header);

private:
  // Synchronized data structure
  struct SyncedData {
    sensor_msgs::msg::Image::SharedPtr image;
    vision_msgs::msg::Detection2DArray::SharedPtr detections;
    rclcpp::Time sync_timestamp;
  };

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> detection_sub_;

  // ExactTime synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image,
    vision_msgs::msg::Detection2DArray>> sync_;

  // ROS2 publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr tracker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback groups for parallel execution
  rclcpp::CallbackGroup::SharedPtr sync_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;

  // SORT tracker backend
  std::shared_ptr<sort::Sort> tracker_backend_;

  // ROS2 parameters
  std::string image_input_topic_;
  std::string detection_input_topic_;
  std::string tracking_output_topic_;
  int queue_size_;
  double processing_frequency_;
  int max_processing_queue_size_;
  int sync_queue_size_;

  // SORT tracker parameters
  int max_age_;
  int min_hits_;
  float iou_threshold_;

  // Synchronized data buffer
  std::queue<SyncedData> sync_buff_;
  std::mutex mtx_;
  std::atomic<bool> processing_in_progress_;
};

} // namespace sort_tracker
