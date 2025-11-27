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
find_package(Protobuf REQUIRED)

# Create xviz/v2 directory structure
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2)
file(GLOB PROTO_SRC_FILES "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
foreach(PROTO_FILE ${PROTO_SRC_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME)
  configure_file(${PROTO_FILE} 
                 ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/${PROTO_NAME} COPYONLY)
endforeach()

# List proto files in dependency order
set(PROTO_FILES
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/options.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/style.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/primitives.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/annotation.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/uiprimitives.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/core.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/envelope.proto
  ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/session.proto
)

# Generate C++ sources using custom command with correct proto_path
set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(PROTO_FILE ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
  list(APPEND PROTO_SRCS ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/${PROTO_NAME}.pb.cc)
  list(APPEND PROTO_HDRS ${CMAKE_CURRENT_BINARY_DIR}/xviz/v2/${PROTO_NAME}.pb.h)
endforeach()

add_custom_command(
  OUTPUT ${PROTO_SRCS} ${PROTO_HDRS}
  COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
    --proto_path=${CMAKE_CURRENT_BINARY_DIR}
    --cpp_out=${CMAKE_CURRENT_BINARY_DIR}
    ${PROTO_FILES}
  DEPENDS ${PROTO_FILES}
  COMMENT "Generating XVIZ protobuf C++ files"
  VERBATIM
)

add_custom_target(generate_proto ALL DEPENDS ${PROTO_SRCS} ${PROTO_HDRS})

add_executable(xviz_converter_node 
  src/xviz_converter_node.cpp
  ${PROTO_SRCS}
)

target_include_directories(xviz_converter_node PUBLIC
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
)

ament_target_dependencies(xviz_converter_node 
  rclcpp 
  demo_interfaces
)

target_link_libraries(xviz_converter_node
  Boost::system
  ${PROTOBUF_LIBRARIES}
  pthread
)

install(TARGETS xviz_converter_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

