cmake_minimum_required(VERSION 3.8)
project(control_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(demo_interfaces REQUIRED)

add_executable(control_publisher src/control_publisher.cpp)
ament_target_dependencies(control_publisher rclcpp demo_interfaces)

install(TARGETS control_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()

