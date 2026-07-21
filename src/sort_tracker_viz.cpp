// C++ header
#include <string>
#include <exception>
#include <cmath>

// OpenCV header
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// ROS header
#include <cv_bridge/cv_bridge.hpp>

// local header
#include "sort_tracker/sort_tracker_viz.hpp"


namespace sort_tracker
{

SortTrackerViz::SortTrackerViz()
: Node("sort_tracker_viz_node")
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 components with message filters
  initialize_ros_components();

  RCLCPP_INFO(get_logger(), "SORT tracker visualization node initialized successfully");
}

SortTrackerViz::~SortTrackerViz()
{
  RCLCPP_INFO(get_logger(), "SORT tracker visualization node shutting down");
}

bool SortTrackerViz::initialize_parameters()
{
  try {
    // ROS2 topic parameters
    image_input_topic_ = declare_parameter("image_input_topic", std::string(""));
    if (image_input_topic_.empty()) {
      RCLCPP_ERROR(get_logger(),
        "image_input_topic is empty. This must be remapped by the launch file "
        "(e.g. image_input_topic:=/carla/hero/cam2/image) - refusing to start "
        "with an unspecified input source.");
      return false;
    }

    tracked_detections_input_topic_ = declare_parameter("tracked_detections_input_topic",
      std::string("sort_tracker/tracks"));
    annotated_image_output_topic_ = declare_parameter("annotated_image_output_topic",
      std::string("sort_tracker/image"));

    queue_size_ = declare_parameter<int>("queue_size", 10);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    RCLCPP_INFO(get_logger(), "Parameters initialized successfully");
    RCLCPP_INFO(get_logger(), "Input topics: %s, %s",
      image_input_topic_.c_str(), tracked_detections_input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic: %s", annotated_image_output_topic_.c_str());

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

void SortTrackerViz::initialize_ros_components()
{
  // Configure QoS profile for image/detection transport
  rclcpp::QoS sub_qos(queue_size_);
  sub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  sub_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  sub_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // Create message filter subscribers
  image_sub_.subscribe(this, image_input_topic_, sub_qos);
  tracked_detections_sub_.subscribe(this, tracked_detections_input_topic_, sub_qos);

  // Create ExactTime synchronizer (queue_size first, then subscribers)
  sync_ = std::make_shared<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image, vision_msgs::msg::Detection2DArray>>(
      sync_queue_size_, image_sub_, tracked_detections_sub_);

  // Register synchronized callback
  sync_->registerCallback(&SortTrackerViz::synchronized_callback, this);

  // Create publisher
  annotated_image_pub_ = create_publisher<sensor_msgs::msg::Image>(
    annotated_image_output_topic_, sub_qos);

  RCLCPP_INFO(get_logger(), "ROS components initialized with exact-time synchronizer");
}

void SortTrackerViz::synchronized_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
  const vision_msgs::msg::Detection2DArray::ConstSharedPtr & tracked_detections_msg)
{
  // Skip drawing/publishing work entirely if nobody is listening
  if (annotated_image_pub_->get_subscription_count() == 0) {
    return;
  }

  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::BGR8);

    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty or invalid image");
      return;
    }

    // Draw tracking results if any tracks exist
    if (!tracked_detections_msg->detections.empty()) {
      draw_tracking_results(cv_ptr->image, *tracked_detections_msg);
    }

    // Publish annotated image
    publish_annotated_image(cv_ptr->image, image_msg->header);

    RCLCPP_DEBUG(get_logger(), "Drew and published %ld tracks",
      tracked_detections_msg->detections.size());

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during visualization: %s", e.what());
  }
}

void SortTrackerViz::draw_tracking_results(
  cv::Mat & image,
  const vision_msgs::msg::Detection2DArray & tracked_detections)
{
  for (const auto & detection : tracked_detections.detections) {
    // Convert center+size bbox back to corner coordinates
    const auto & bbox = detection.bbox;
    int x1 = static_cast<int>(bbox.center.position.x - bbox.size_x / 2.0);
    int y1 = static_cast<int>(bbox.center.position.y - bbox.size_y / 2.0);
    int x2 = static_cast<int>(bbox.center.position.x + bbox.size_x / 2.0);
    int y2 = static_cast<int>(bbox.center.position.y + bbox.size_y / 2.0);

    // Track id was stored as a string in Detection2D.id by sort_tracker_node
    int track_id = 0;
    try {
      track_id = std::stoi(detection.id);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Failed to parse track id '%s': %s", detection.id.c_str(), e.what());
      continue;
    }

    // Get consistent color for this track
    cv::Scalar box_color = get_track_color(track_id);

    // Draw bounding box
    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), box_color, 2);

    // Prepare track ID text
    std::string track_text = "ID:" + std::to_string(track_id);

    // Calculate text size and position
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.55;
    int thickness = 1.8;
    int baseline = 0;

    cv::Size text_size = cv::getTextSize(track_text, font_face, font_scale, thickness, &baseline);

    // Draw background rectangle for text (same color as box)
    cv::rectangle(image,
      cv::Point(x1, y1 - text_size.height - 4),
      cv::Point(x1 + text_size.width, y1),
      box_color, -1);

    // Choose text color based on background brightness
    // Use white text on dark backgrounds, black text on light backgrounds
    cv::Vec3b color_bgr = cv::Vec3b(box_color[0], box_color[1], box_color[2]);
    int brightness = (color_bgr[2] + color_bgr[1] + color_bgr[0]) / 3; // R+G+B/3
    cv::Scalar text_color = brightness < 128 ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0);

    // Draw text
    cv::putText(image, track_text, cv::Point(x1, y1 - 4),
      font_face, font_scale, text_color, thickness);
  }
}

cv::Scalar SortTrackerViz::get_track_color(int track_id)
{
  // Generate consistent color based on track ID using HSV color space
  // Use golden ratio for better color distribution
  const float golden_ratio = 0.618033988749895f;
  float hue = std::fmod(track_id * golden_ratio, 1.0f) * 360.0f;

  // Convert HSV to BGR (OpenCV format)
  // Fixed saturation and value for bright, distinguishable colors
  cv::Mat hsv_color(1, 1, CV_8UC3, cv::Scalar(hue / 2.0f, 255, 255)); // OpenCV hue is 0-180
  cv::Mat bgr_color;
  cv::cvtColor(hsv_color, bgr_color, cv::COLOR_HSV2BGR);

  cv::Vec3b bgr_pixel = bgr_color.at<cv::Vec3b>(0, 0);
  return cv::Scalar(bgr_pixel[0], bgr_pixel[1], bgr_pixel[2]);
}

void SortTrackerViz::publish_annotated_image(
  const cv::Mat & result_image,
  const std_msgs::msg::Header & header)
{
  try {
    // Convert OpenCV image back to ROS message
    cv_bridge::CvImage cv_image;
    cv_image.header = header;
    cv_image.encoding = sensor_msgs::image_encodings::BGR8;
    cv_image.image = result_image;

    // Publish the result
    auto output_msg = cv_image.toImageMsg();
    annotated_image_pub_->publish(*output_msg);

    RCLCPP_DEBUG(get_logger(), "Published annotated image");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during annotated image publishing: %s", e.what());
  }
}

} // namespace sort_tracker
