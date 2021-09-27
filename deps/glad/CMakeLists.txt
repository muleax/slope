cmake_minimum_required(VERSION 3.20)

add_library(glad STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/glad_impl.cpp)
target_include_directories(glad PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

function(_target_add_glad target privacy path)
    find_package(OpenGL 4.6 REQUIRED)
    target_link_libraries(${target} ${privacy} glad)
endfunction()