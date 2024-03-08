cmake_minimum_required(VERSION 3.8)
project(action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(diy_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(action_client_diy src/fibonacci_client.cpp)
ament_target_dependencies(action_client_diy rclcpp rclcpp_action diy_interface)

add_executable(action_server_diy src/fibonacci_server.cpp)
ament_target_dependencies(action_server_diy rclcpp rclcpp_action diy_interface)

install(TARGETS 
  action_client_diy
  action_server_diy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
