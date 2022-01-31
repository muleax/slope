cmake_minimum_required(VERSION 3.20)

# TODO: make portable
find_package(OpenGL 4.6 REQUIRED)

add_library(glad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/glad_impl.cpp)

target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
