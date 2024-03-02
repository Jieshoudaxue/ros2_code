cmake_minimum_required(VERSION 3.8)
project(polygon_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(area_node src/area_node.cpp)
target_include_directories(area_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(area_node pluginlib)

install(TARGETS 
  area_node
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 必须把接口类的头文件安装出来，不然插件库编译的时候找不到头文件
install(DIRECTORY 
  include/
  DESTINATION include
)

# 接口类头文件必须声明出来，不然插件库编译的时候也找不到头文件
ament_export_include_directories(
  include
)

ament_package()
