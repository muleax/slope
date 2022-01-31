cmake_minimum_required(VERSION 3.20)

file(GLOB LIB_HEADERS "imgui/imgui*.h")
file(GLOB LIB_SOURCES "imgui/imgui*.cpp" "imgui/backends/imgui_impl_opengl3.cpp" "imgui/backends/imgui_impl_glfw.cpp")

add_library(imgui STATIC ${LIB_HEADERS} ${LIB_SOURCES})

target_include_directories(imgui
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/imgui)
target_link_libraries(imgui PRIVATE glfw)
