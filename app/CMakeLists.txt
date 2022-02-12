cmake_minimum_required(VERSION 3.20)

slope_library(app)

target_link_libraries(app
        PUBLIC slope imgui
        PRIVATE glad glfw stb)
