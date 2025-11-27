cmake_minimum_required(VERSION 3.8)
project(xviz_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for Boost Beast
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(demo_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(xviz_converter_node src/xviz_converter_node.cpp)

ament_target_dependencies(xviz_converter_node 
  rclcpp 
  demo_interfaces
)

target_link_libraries(xviz_converter_node
  Boost::system
  pthread
)

install(TARGETS xviz_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

