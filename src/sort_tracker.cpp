// C++ header
#include <string>
#include <chrono>
#include <functional>
#include <exception>

// local header
#include "sort_tracker/sort_tracker.hpp"


namespace sort_tracker
{

SortTracker::SortTracker()
: Node("sort_tracker_node"), processing_in_progress_(false)
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

  // Initialize ROS2 components
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
    detection_input_topic_ = declare_parameter("detection_input_topic",
      std::string("fcos_object_detection/detection_array"));
    tracking_output_topic_ = declare_parameter("tracking_output_topic",
      std::string("sort_tracker/tracks"));

    queue_size_ = declare_parameter<int>("queue_size", 10);

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

    // Detection confidence threshold parameter
    detection_confidence_threshold_ =
      declare_parameter<float>("detection_confidence_threshold", 0.5f);
    if (detection_confidence_threshold_ < 0.0f || detection_confidence_threshold_ > 1.0f) {
      RCLCPP_ERROR(get_logger(),
        "Invalid detection confidence threshold: %.3f (should be 0.0 <= threshold <= 1.0)",
        detection_confidence_threshold_);
      return false;
    }

    // SORT tracker parameters
    max_age_ = declare_parameter<int>("max_age", 30);
    min_hits_ = declare_parameter<int>("min_hits", 3);
    if (max_age_ <= 0 || min_hits_ <= 0) {
      RCLCPP_ERROR(get_logger(), "Invalid SORT parameters: max_age=%d, min_hits=%d",
        max_age_, min_hits_);
      return false;
    }

    iou_threshold_ = declare_parameter<float>("iou_threshold", 0.3f);
    if (iou_threshold_ <= 0.0f || iou_threshold_ >= 1.0f) {
      RCLCPP_ERROR(get_logger(), "Invalid IoU threshold: %.3f (should be 0.0 < iou < 1.0)",
        iou_threshold_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Parameters initialized successfully");
    RCLCPP_INFO(get_logger(), "Input topic: %s", detection_input_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output topic: %s", tracking_output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Detection confidence threshold: %.3f",
      detection_confidence_threshold_);
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
  // Configure QoS profile for detection transport
  rclcpp::QoS detection_qos(queue_size_);
  detection_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  detection_qos.durability(rclcpp::DurabilityPolicy::Volatile);
  detection_qos.history(rclcpp::HistoryPolicy::KeepLast);

  // Create a single REENTRANT callback group for all callbacks.
  // detection_callback only pushes onto a mutex-protected queue (cheap), and
  // timer_callback does the actual tracking work while guarded by an atomic
  // processing flag - both are independently thread-safe, so they're free to
  // run concurrently with each other.
  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Create subscription options with dedicated callback group
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  // Create detection subscriber
  detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    detection_input_topic_, detection_qos,
    std::bind(&SortTracker::detection_callback, this, std::placeholders::_1),
    sub_options
  );

  // Create publisher
  tracker_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>(
    tracking_output_topic_, detection_qos);

  // Create timer for processing at specified frequency
  auto timer_period = std::chrono::duration<double>(1.0 / processing_frequency_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
    std::bind(&SortTracker::timer_callback, this), callback_group_
  );

  RCLCPP_INFO(get_logger(), "ROS components initialized with a shared reentrant callback group");
  RCLCPP_INFO(get_logger(), "Processing frequency: %.1f Hz", processing_frequency_);
}

void SortTracker::detection_callback(
  vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg)
{
  try {
    // Thread-safe queue management
    std::lock_guard<std::mutex> lock(mtx_);

    // Check if queue is full
    if (detection_buff_.size() >= static_cast<size_t>(max_processing_queue_size_)) {
      // Remove oldest detection to make room for new one
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Tracking processing queue full, dropping oldest detection (queue size: %ld)",
        detection_buff_.size());
      detection_buff_.pop();
    }

    // Add new detection to queue
    detection_buff_.push(detection_msg);

    RCLCPP_DEBUG(get_logger(), "Detection added to queue. Queue size: %ld",
      detection_buff_.size());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception in detection callback: %s", e.what());
  }
}

void SortTracker::timer_callback()
{
  // Atomically claim the "processing" slot
  bool expected = false;
  if (!processing_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  // Get next detection from queue
  vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg;

  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (detection_buff_.empty()) {
      processing_in_progress_.store(false);
      return;
    }
    detection_msg = detection_buff_.front();
    detection_buff_.pop();
  }

  try {
    // Convert detections to SORT format
    Eigen::MatrixXf detection_matrix = convert_detections_to_sort_format(detection_msg);

    // Process detections with SORT tracker
    Eigen::MatrixXf tracking_results = tracker_backend_->update(detection_matrix);

    // Convert tracking results back into a ROS detection message
    if (tracker_pub_->get_subscription_count() > 0) {
      vision_msgs::msg::Detection2DArray tracked_detections =
        convert_sort_output_to_detections(tracking_results, detection_msg->header);

      // Publish tracked detections
      publish_tracked_detections(tracked_detections);
    }

    RCLCPP_DEBUG(get_logger(), "Processed tracking frame with %ld detections -> %ld tracks",
      detection_msg->detections.size(), tracking_results.rows());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during tracking processing: %s", e.what());
  }

  // Clear processing flag
  processing_in_progress_.store(false);
}

Eigen::MatrixXf SortTracker::convert_detections_to_sort_format(
  vision_msgs::msg::Detection2DArray::ConstSharedPtr detection_msg)
{
  const auto & detections = detection_msg->detections;

  if (detections.empty()) {
    // Return empty matrix for SORT (0 rows, 5 columns)
    return Eigen::MatrixXf::Zero(0, 5);
  }

  // First pass: count detections that meet confidence threshold
  size_t valid_detection_count = 0;

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];

    // Get confidence score (iterate through all hypotheses and pick the highest)
    float score = 0.0f; // default: no confidence if no data available
    if (!detection.results.empty()) {
      for (const auto & result : detection.results) {
        score = std::max(score, static_cast<float>(result.hypothesis.score));
      }
    }

    // Check if this detection meets the confidence threshold
    if (score >= detection_confidence_threshold_) {
      valid_detection_count++;
    }
  }

  // Log filtering results
  RCLCPP_DEBUG(get_logger(),
    "Filtered detections: %ld/%ld passed confidence threshold (>= %.3f)",
    valid_detection_count, detections.size(), detection_confidence_threshold_);

  if (valid_detection_count == 0) {
    // Return empty matrix for SORT (0 rows, 5 columns)
    return Eigen::MatrixXf::Zero(0, 5);
  }

  // Create matrix for SORT input: [x1, y1, x2, y2, score]
  Eigen::MatrixXf detection_matrix(valid_detection_count, 5);

  // Second pass: populate matrix only with qualifying detections
  size_t matrix_row = 0;
  for (size_t i = 0; i < detections.size(); ++i) {
    const auto & detection = detections[i];

    // Get confidence score (iterate through all hypotheses and pick the highest)
    float score = 0.0f; // default: no confidence if no data available
    if (!detection.results.empty()) {
      for (const auto & result : detection.results) {
        score = std::max(score, static_cast<float>(result.hypothesis.score));
      }
    }

    // Only process detections that meet the confidence threshold
    if (score >= detection_confidence_threshold_) {
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

      // Fill matrix row
      detection_matrix(matrix_row, 0) = x1;
      detection_matrix(matrix_row, 1) = y1;
      detection_matrix(matrix_row, 2) = x2;
      detection_matrix(matrix_row, 3) = y2;
      detection_matrix(matrix_row, 4) = score;

      matrix_row++;
    }
  }

  return detection_matrix;
}

vision_msgs::msg::Detection2DArray SortTracker::convert_sort_output_to_detections(
  const Eigen::MatrixXf & tracking_results,
  const std_msgs::msg::Header & header)
{
  vision_msgs::msg::Detection2DArray tracked_detections;
  tracked_detections.header = header;
  tracked_detections.detections.reserve(static_cast<size_t>(tracking_results.rows()));

  for (int i = 0; i < tracking_results.rows(); ++i) {
    // Extract tracking result: [x1, y1, x2, y2, track_id]
    float x1 = tracking_results(i, 0);
    float y1 = tracking_results(i, 1);
    float x2 = tracking_results(i, 2);
    float y2 = tracking_results(i, 3);
    int track_id = static_cast<int>(tracking_results(i, 4));

    vision_msgs::msg::Detection2D detection;
    detection.header = header;

    // ID used for consistency across multiple detection messages, per the
    // field's documented purpose in vision_msgs - exactly what a track id is.
    detection.id = std::to_string(track_id);

    // Convert corner coordinates back to center+size format
    detection.bbox.center.position.x = (x1 + x2) / 2.0;
    detection.bbox.center.position.y = (y1 + y2) / 2.0;
    detection.bbox.size_x = x2 - x1;
    detection.bbox.size_y = y2 - y1;

    tracked_detections.detections.push_back(std::move(detection));
  }

  return tracked_detections;
}

void SortTracker::publish_tracked_detections(
  const vision_msgs::msg::Detection2DArray & tracked_detections)
{
  try {
    tracker_pub_->publish(tracked_detections);

    RCLCPP_DEBUG(get_logger(), "Published %ld tracked detections",
      tracked_detections.detections.size());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during tracked detections publishing: %s", e.what());
  }
}

} // namespace sort_tracker
