cmake_minimum_required(VERSION 3.8)
project(pubsub_mix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(publish_address_book src/publish_address_book.cpp)
ament_target_dependencies(publish_address_book rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(publish_address_book "${cpp_typesupport_target}")

install(TARGETS
  publish_address_book
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/subscribe_address_book.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
