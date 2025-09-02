// local header
#include "sort_tracker/sort_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sort_tracker::SortTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
