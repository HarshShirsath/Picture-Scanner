cmake_minimum_required(VERSION 3.8)
project(odom_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)


add_executable(odom_updater src/odom_updater.cpp src/main.cpp)
ament_target_dependencies(odom_updater rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS 
odom_updater
DESTINATION lib/${PROJECT_NAME})

ament_package()
