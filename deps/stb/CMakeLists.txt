cmake_minimum_required(VERSION 3.20)

add_library(stb STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/stb_impl.cpp)
target_include_directories(stb PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

function(_target_add_stb target privacy path)
    target_link_libraries(${target} ${privacy} stb)
endfunction()
