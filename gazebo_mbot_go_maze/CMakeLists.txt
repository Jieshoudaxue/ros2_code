cmake_minimum_required(VERSION 3.8)
project(gazebo_mbot_go_maze)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tool/mbot_teletop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
