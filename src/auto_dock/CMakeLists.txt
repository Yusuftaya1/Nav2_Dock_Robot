cmake_minimum_required(VERSION 3.8)
project(auto_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Bağımlılıkları bul
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bt_navigator REQUIRED)

add_executable(dock_test src/dock_test.cpp)
ament_target_dependencies(dock_test
  rclcpp
  sensor_msgs
  nav2_msgs
  geometry_msgs
  nav2_bt_navigator
)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  src/test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  dock_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
