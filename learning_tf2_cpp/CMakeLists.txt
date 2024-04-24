cmake_minimum_required(VERSION 3.8)
project(learning_tf2_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(turtlesim REQUIRED)

include_directories(include)

add_library(tf_broadcaster SHARED src/turtle_tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp rclcpp_components tf2 tf2_ros turtlesim)
rclcpp_components_register_nodes(tf_broadcaster "learning_tf2_cpp::TurtleTfBroadcaster")

add_library(tf_listener SHARED src/turtle_tf_listener.cpp)
ament_target_dependencies(tf_listener rclcpp rclcpp_components tf2 tf2_ros turtlesim)
rclcpp_components_register_nodes(tf_listener "learning_tf2_cpp::TurtleTfListener")

install(TARGETS 
  tf_broadcaster
  tf_listener
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
