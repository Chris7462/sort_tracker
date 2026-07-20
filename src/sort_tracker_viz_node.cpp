// C++ header
#include <memory>

// ROS header
#include <rclcpp/rclcpp.hpp>

// local header
#include "sort_tracker/sort_tracker_viz.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Create the node
  auto node = std::make_shared<sort_tracker::SortTrackerViz>();

  // If parameter initialization failed inside the constructor, it calls
  // rclcpp::shutdown() internally. Detect that here and exit with a
  // non-zero status instead of proceeding to spin a half-constructed node.
  if (!rclcpp::ok()) {
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Starting SORT Tracker Visualization node");

  // This node has no timer and no bounded processing queue - all work
  // happens directly inside the message_filters synchronized callback, so a
  // single-threaded spin is sufficient (no benefit from a multi-threaded or
  // events-based executor here).
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
