#pragma once

// C++ header
#include <memory>
#include <string>
#include <unordered_map>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

// Message filters header
#include <message_filters/subscriber.hpp>
#include <message_filters/time_synchronizer.hpp>

// OpenCV header
#include <opencv2/core.hpp>


namespace sort_tracker
{

class SortTrackerViz : public rclcpp::Node
{

public:
  /**
   * @brief Constructor for SortTrackerViz node
   */
  SortTrackerViz();

  /**
   * @brief Destructor for SortTrackerViz node
   */
  ~SortTrackerViz();

private:
  /**
   * @brief Initialize node parameters with validation
   * @return true if initialization successful, false otherwise
   */
  bool initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, and message filter synchronizer
   */
  void initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming raw image and tracked detections.
   * Draws the tracked bounding boxes/ids on the image and republishes it.
   * @param image_msg Incoming raw image message
   * @param tracked_detections_msg Incoming tracked detections (Detection2DArray with id set to track id)
   */
  void synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & tracked_detections_msg);

  /**
   * @brief Draw tracking results on image with bounding boxes and track IDs
   * @param image OpenCV image to draw on
   * @param tracked_detections Detection2DArray with id set to the track id for each detection
   */
  void draw_tracking_results(
    cv::Mat & image,
    const vision_msgs::msg::Detection2DArray & tracked_detections);

  /**
   * @brief Generate consistent color for a track ID
   * @param track_id Track identifier
   * @return OpenCV Scalar color (BGR format)
   */
  cv::Scalar get_track_color(int track_id);

  /**
   * @brief Publish annotated image
   * @param result_image Annotated result as OpenCV Mat
   * @param header Original message header for timestamp consistency
   */
  void publish_annotated_image(
    const cv::Mat & result_image,
    const std_msgs::msg::Header & header);

private:
  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
  message_filters::Subscriber<vision_msgs::msg::Detection2DArray> tracked_detections_sub_;

  // ExactTime synchronizer
  std::shared_ptr<message_filters::TimeSynchronizer<
      sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>> sync_;

  // ROS2 publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_image_pub_;

  // ROS2 parameters
  std::string image_input_topic_;
  std::string tracked_detections_input_topic_;
  std::string annotated_image_output_topic_;
  int queue_size_;
  int sync_queue_size_;
};

} // namespace sort_tracker
