cmake_minimum_required(VERSION 3.8)
project(component_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(sub_component SHARED src/sub_component.cpp)
ament_target_dependencies(sub_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(sub_component "component_demo::SubComponent")

add_library(pub_component SHARED src/pub_component.cpp)
ament_target_dependencies(pub_component rclcpp rclcpp_components std_msgs)
rclcpp_components_register_nodes(pub_component "component_demo::PubComponent")

install(TARGETS 
  sub_component
  pub_component
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
