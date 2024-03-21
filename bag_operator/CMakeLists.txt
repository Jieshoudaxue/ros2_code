cmake_minimum_required(VERSION 3.8)
project(bag_operator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(bag_record src/bag_record.cpp)
target_include_directories(bag_record PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(bag_record rclcpp rosbag2_cpp)

install(TARGETS 
  bag_record
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/bag_echo.py
  scripts/bag_filter.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
