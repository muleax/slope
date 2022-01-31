cmake_minimum_required(VERSION 3.20)

add_library(stb STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/stb_impl.cpp)

target_include_directories(stb PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
