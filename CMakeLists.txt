cmake_minimum_required(VERSION 3.16)
project(sort_tracker
  VERSION 1.0.0
  DESCRIPTION "Modern C++ implementation of Simple Online Realtime Tracking (SORT) using Hungarian Algorithm and Kalman Filter"
  LANGUAGES CXX)

  # Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(sort_backend REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(${PROJECT_NAME}_node
  src/sort_tracker_node.cpp
  src/sort_tracker.cpp)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(${PROJECT_NAME}_node
  PUBLIC
    rclcpp
    std_msgs
    sensor_msgs
    vision_msgs
    cv_bridge
    message_filters
    sort_backend)

target_link_libraries(${PROJECT_NAME}_node
  PUBLIC
    Eigen3::Eigen)

target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)  # Require C++17

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
