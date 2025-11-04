cmake_minimum_required(VERSION 3.8)
project(live_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(live_server src/live_server.cpp)

install(TARGETS live_server
  DESTINATION bin)

