cmake_minimum_required(VERSION 3.20)

add_library(taskflow INTERFACE)

target_include_directories(taskflow INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/taskflow)
