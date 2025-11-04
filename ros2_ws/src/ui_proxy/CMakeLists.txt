cmake_minimum_required(VERSION 3.8)
project(ui_proxy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(demo_interfaces REQUIRED)

add_executable(ui_proxy_node src/ui_proxy_node.cpp)
ament_target_dependencies(ui_proxy_node rclcpp demo_interfaces)

install(TARGETS ui_proxy_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

