cmake_minimum_required(VERSION 3.8)
project(polygon_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(polygon_base REQUIRED)
find_package(pluginlib REQUIRED)

# 必须调用，不然polygon_base的area_node样例运行时找不到这个插件，无法实例化
pluginlib_export_plugin_description_file(polygon_base plugins.xml)

add_library(polygon_plugins src/polygon_plugins.cpp)
ament_target_dependencies(polygon_plugins polygon_base pluginlib)

install(TARGETS 
  polygon_plugins
  LIBRARY DESTINATION lib
)

ament_package()
