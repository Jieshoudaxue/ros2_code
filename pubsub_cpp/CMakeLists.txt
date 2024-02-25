cmake_minimum_required(VERSION 3.8)
project(pubsub_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diy_interface REQUIRED)

add_executable(talker src/publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(talker_qos src/pub_hello_qos.cpp)
ament_target_dependencies(talker_qos rclcpp std_msgs)

add_executable(listener_qos src/sub_hello_qos.cpp)
ament_target_dependencies(listener_qos rclcpp std_msgs)

add_executable(talker_diy src/pub_diy_msg.cpp)
ament_target_dependencies(talker_diy rclcpp diy_interface)

add_executable(listener_diy src/sub_diy_msg.cpp)
ament_target_dependencies(listener_diy rclcpp diy_interface)


install(TARGETS
  talker
  listener
  talker_qos
  listener_qos
  talker_diy
  listener_diy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
