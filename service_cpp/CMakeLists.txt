cmake_minimum_required(VERSION 3.8)
project(service_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(diy_interface REQUIRED)

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp example_interfaces)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp example_interfaces)

add_executable(server_diy src/server_diy.cpp)
ament_target_dependencies(server_diy rclcpp diy_interface)

add_executable(client_diy src/client_diy.cpp)
ament_target_dependencies(client_diy rclcpp diy_interface)

install(TARGETS
  server
  client
  server_diy
  client_diy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
