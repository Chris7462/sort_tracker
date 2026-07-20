// C++ header
#include <memory>

// ROS header
#include <rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp>

// local header
#include "sort_tracker/sort_tracker.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<sort_tracker::SortTracker>();

  // If parameter or tracker initialization failed inside the constructor,
  // it calls rclcpp::shutdown() internally. Detect that here and exit with
  // a non-zero status instead of proceeding to spin a half-constructed node.
  if (!rclcpp::ok()) {
    return 1;
  }

  // EventsCBGExecutor: uses 10-15% less CPU than MultiThreadedExecutor,
  // supports multiple ROS time sources, and manages threading internally.
  rclcpp::executors::EventsCBGExecutor executor;

  // Add node to executor
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "Starting SORT Tracker with EventsCBGExecutor");

  // Spin with multiple threads
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
