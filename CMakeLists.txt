cmake_minimum_required(VERSION 3.8)
project(sync_recorded_data)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(sync_data src/sync_data.cpp)
target_link_libraries(sync_data ${PCL_LIBRARIES})
ament_target_dependencies(sync_data rclcpp std_msgs sensor_msgs message_filters PCL cv_bridge OpenCV)

add_executable(lidar_projection src/lidar_camera_fusion.cpp)
target_link_libraries(lidar_projection ${PCL_LIBRARIES} yaml-cpp)
ament_target_dependencies(lidar_projection rclcpp std_msgs sensor_msgs message_filters PCL cv_bridge OpenCV yaml-cpp)

install(TARGETS
  sync_data
  lidar_projection
  DESTINATION lib/${PROJECT_NAME})

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
