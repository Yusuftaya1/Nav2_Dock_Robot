cmake_minimum_required(VERSION 3.8)
project(auto_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)


install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
