cmake_minimum_required(VERSION 3.20)

add_library(glm INTERFACE)

target_include_directories(glm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/glm)
