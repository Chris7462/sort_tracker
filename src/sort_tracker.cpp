// C++ header
#include <string>
#include <chrono>
#include <functional>
#include <exception>
#include <cmath>

// OpenCV header
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// ROS header
#include <cv_bridge/cv_bridge.hpp>

// Eigen header
#include <Eigen/Dense>

// local header
#include "sort_tracker/sort_tracker.hpp"


namespace sort_tracker
{

SortTracker::SortTracker()
: Node("sort_tracker_node"),
  processing_in_progress_(false)
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize SORT tracking backend
  if (!initialize_tracker()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize SORT tracker");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 components with message filters
  initialize_ros_components();

  RCLCPP_INFO(get_logger(),
    "SORT tracker node initialized successfully with bounded queue (max: %d)",
    max_processing_queue_size_);
}

SortTracker::~SortTracker()
{
  RCLCPP_INFO(get_logger(), "SORT tracker node shutting down");
}

bool SortTracker::initialize_parameters()
{
  try {
    // ROS2 topic parameters
    image_input_topic_ = declare_parameter("image_input_topic",
      std::string("kitti/camera/color/left/image_raw"));
    detection_input_topic_ = declare_parameter("detection_input_topic",
      std::string("fcos_object_detection/detection_array"));
    tracking_output_topic_ = declare_parameter("tracking_output_topic",
      std::string("sort_tracker/image"));

    queue_size_ = declare_parameter<int>("queue_size", 10);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    processing_frequency_ = declare_parameter<double>("processing_frequency", 50.0);
    if (processing_frequency_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid processing frequency: %.2f Hz", processing_frequency_);
      return false;
    }

    // Processing queue parameter - small bounded queue for burst handling
    max_processing_queue_size_ = declare_parameter<int>("max_processing_queue_size", 5);
    if (max_processing_queue_size_ <= 0 || max_processing_queue_size_ > 15) {
      RCLCPP_ERROR(get_logger(), "Invalid max processing queue size: %d (should be 1-15)",
        max_processing_queue_size_);
      return false;
    }

    // SORT tracker parameters
    max_age_ = declare_parameter<int>("max_age", 30);
    min_hits_ = declare_parameter<int>("min_hits", 3);
    iou_threshold_ = declare_parameter<float>("iou_threshold", 0.3f);

    if (max_age_ <= 0 || min_hits_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid SORT parameters: max_age=%d, min_hits=%d",
        max_age_, min_hits_);
      return false;
    }

    if (iou_threshold_ <= 0.0f || iou_threshold_ >= 1.0f) {
      RCLCPP_ERROR(get_logger(), "Invalid IoU threshold: %.3f (should be 0.0 < iou < 1.0)",
        iou_threshold_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Parameters initialized successfully");
    RCLCPP_INFO(get_logger(), "Input topics: %s, %s",
      image_input_topic_.c_str(), detection_input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic: %s", tracking_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "SORT config: max_age=%d, min_hits=%d, iou_thresh=%.3f",
      max_age_, min_hits_, iou_threshold_);

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

bool SortTracker::initialize_tracker()
{
  try {
    // Initialize SORT tracking backend with parameters
    tracker_backend_ = std::make_shared<sort::Sort>(max_age_, min_hits_, iou_threshold_);

    if (!tracker_backend_) {
      RCLCPP_ERROR(get_logger(), "Failed to create SORT tracker backend instance");
      return false;
    }

    RCLCPP_INFO(get_logger(), "SORT tracker backend initialized successfully");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception creating SORT tracker backend: %s", e.what());
    return false;
  }
}

void SortTracker::initialize_ros_components()
{
  // Configure QoS profile for reliable image transport
  rclcpp::QoS image_qos(queue_size_);
  image_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  image_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  image_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // Create separate callback groups for parallel execution
  sync_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create message filter subscribers with dedicated callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sync_callback_group_;

  // Create message filter subscribers
  image_sub_.subscribe(this, image_input_topic_,
    image_qos.get_rmw_qos_profile(), sub_options);

  detection_sub_.subscribe(this, detection_input_topic_,
    image_qos.get_rmw_qos_profile(), sub_options);

  // Create ExactTime synchronizer
  sync_ = std::make_shared<message_filters::TimeSynchronizer<
    sensor_msgs::msg::Image,
    vision_msgs::msg::Detection2DArray>>(image_sub_, detection_sub_, sync_queue_size_);

  // Register synchronized callback
  sync_->registerCallback(&SortTracker::synchronized_callback, this);

  // Create publisher
  tracker_pub_ = create_publisher<sensor_msgs::msg::Image>(tracking_output_topic_, image_qos);

  // Create timer for processing at specified frequency
  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&SortTracker::timer_callback, this),
    timer_callback_group_
  );

  RCLCPP_INFO(get_logger(), "ROS components initialized with separate callback groups");
  RCLCPP_INFO(get_logger(), "Processing frequency: %.1f Hz, Sync queue size: %d",
    processing_frequency_, sync_queue_size_);
}

void SortTracker::synchronized_callback(
  const sensor_msgs::msg::Image::SharedPtr image_msg,
  const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg)
{
  try {
    // Thread-safe queue management
    std::lock_guard<std::mutex> lock(mtx_);

    // Check if queue is full
    if (sync_buff_.size() >= static_cast<size_t>(max_processing_queue_size_)) {
      // Remove oldest synchronized data to make room for new one
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Tracking processing queue full, dropping oldest frame (queue size: %ld)",
        sync_buff_.size());
      sync_buff_.pop();
    }

    // Add new synchronized data to queue
    SyncedData synced_data{image_msg, detection_msg, rclcpp::Time(image_msg->header.stamp)};
    sync_buff_.push(std::move(synced_data));

    RCLCPP_DEBUG(get_logger(), "Synchronized data added to queue. Queue size: %ld",
      sync_buff_.size());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in synchronized callback: %s", e.what());
  }
}

void SortTracker::timer_callback()
{
  // Skip if already processing
  if (processing_in_progress_.load()) {
    return;
  }

  // Check if there are subscribers
  if (tracker_pub_->get_subscription_count() == 0) {
    // Still consume queue to prevent buildup
    std::lock_guard<std::mutex> lock(mtx_);
    while (!sync_buff_.empty()) {
      sync_buff_.pop();
    }
    return;
  }

  // Get next synchronized data from queue
  SyncedData synced_data;
  bool has_data = false;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!sync_buff_.empty()) {
      synced_data = std::move(sync_buff_.front());
      sync_buff_.pop();
      has_data = true;
    }
  }

  if (!has_data) {
    return; // No data to process
  }

  // Set processing flag
  processing_in_progress_.store(true);

  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      synced_data.image, sensor_msgs::image_encodings::BGR8);

    if (!cv_ptr || cv_ptr->image.empty()) {
      RCLCPP_WARN(get_logger(), "Received empty or invalid image");
      processing_in_progress_.store(false);
      return;
    }

    // Convert detections to SORT format
    Eigen::MatrixXf detection_matrix = convert_detections_to_sort_format(synced_data.detections);

    // Process detections with SORT tracker
    Eigen::MatrixXf tracking_results = tracker_backend_->update(detection_matrix);

    // Create result image (copy original)
    cv::Mat tracking_result_image = cv_ptr->image.clone();

    // Draw tracking results if any tracks exist
    if (tracking_results.rows() > 0) {
      draw_tracking_results(tracking_result_image, tracking_results);
    }

    // Publish tracking result image
    publish_tracking_result_image(tracking_result_image, synced_data.image->header);

    RCLCPP_DEBUG(get_logger(), "Processed tracking frame with %ld detections -> %ld tracks",
      synced_data.detections->detections.size(), tracking_results.rows());

  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during tracking processing: %s", e.what());
  }

  // Clear processing flag
  processing_in_progress_.store(false);
}

Eigen::MatrixXf SortTracker::convert_detections_to_sort_format(
  const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg)
{
  const auto & detections = detection_msg->detections;

  if (detections.empty()) {
    // Return empty matrix for SORT (0 rows, 5 columns)
    return Eigen::MatrixXf::Zero(0, 5);
  }

  // Create matrix for SORT input: [x1, y1, x2, y2, score]
  Eigen::MatrixXf detection_matrix(detections.size(), 5);

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];
    const auto & bbox = detection.bbox;

    // Extract bounding box parameters
    float center_x = bbox.center.position.x;
    float center_y = bbox.center.position.y;
    float size_x = bbox.size_x;
    float size_y = bbox.size_y;

    // Convert from center+size to corner coordinates
    float x1 = center_x - size_x / 2.0f;
    float y1 = center_y - size_y / 2.0f;
    float x2 = center_x + size_x / 2.0f;
    float y2 = center_y + size_y / 2.0f;

    // Get confidence score (use first hypothesis if available)
    float score = 1.0f; // default score
    if (!detection.results.empty()) {
      score = static_cast<float>(detection.results[0].hypothesis.score);
    }

    // Fill matrix row
    detection_matrix(i, 0) = x1;
    detection_matrix(i, 1) = y1;
    detection_matrix(i, 2) = x2;
    detection_matrix(i, 3) = y2;
    detection_matrix(i, 4) = score;
  }

  return detection_matrix;
}

void SortTracker::draw_tracking_results(cv::Mat & image, const Eigen::MatrixXf & tracking_results)
{
  for (int i = 0; i < tracking_results.rows(); ++i) {
    // Extract tracking result: [x1, y1, x2, y2, track_id]
    int x1 = static_cast<int>(tracking_results(i, 0));
    int y1 = static_cast<int>(tracking_results(i, 1));
    int x2 = static_cast<int>(tracking_results(i, 2));
    int y2 = static_cast<int>(tracking_results(i, 3));
    int track_id = static_cast<int>(tracking_results(i, 4));

    // Get consistent color for this track
    cv::Scalar color = get_track_color(track_id);

    // Draw bounding box
    cv::rectangle(image, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);

    // Prepare track ID text
    std::string track_text = "ID:" + std::to_string(track_id);

    // Calculate text size and position
    int font_face = cv::FONT_HERSHEY_SIMPLEX;
    double font_scale = 0.6;
    int thickness = 2;
    int baseline = 0;

    cv::Size text_size = cv::getTextSize(track_text, font_face, font_scale, thickness, &baseline);

    // Position text above the bounding box
    int text_x = x1;
    int text_y = y1 - 10;

    // Make sure text stays within image bounds
    if (text_y < text_size.height) {
      text_y = y1 + text_size.height + 10; // Move inside bbox if too close to top
    }

    // Draw text background rectangle
    cv::Point text_bg_tl(text_x - 2, text_y - text_size.height - 2);
    cv::Point text_bg_br(text_x + text_size.width + 2, text_y + baseline + 2);
    cv::rectangle(image, text_bg_tl, text_bg_br, color, -1);

    // Draw track ID text
    cv::putText(image, track_text, cv::Point(text_x, text_y),
                font_face, font_scale, cv::Scalar(255, 255, 255), thickness);
  }
}

cv::Scalar SortTracker::get_track_color(int track_id)
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

void SortTracker::publish_tracking_result_image(
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
    tracker_pub_->publish(*output_msg);

    RCLCPP_DEBUG(get_logger(), "Published tracking result image");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during result publishing: %s", e.what());
  }
}

} // namespace sort_tracker
