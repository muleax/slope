cmake_minimum_required(VERSION 3.20)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(glfw)

function(_target_add_glfw target privacy path)
    target_link_libraries(${target} ${privacy} glfw)
endfunction()
