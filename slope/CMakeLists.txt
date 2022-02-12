cmake_minimum_required(VERSION 3.20)

slope_library(slope)

target_link_libraries(slope
        PUBLIC fmt taskflow tracy)
