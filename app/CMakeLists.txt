cmake_minimum_required(VERSION 3.20)

slope_library(app)

slope_internal_deps(app PUBLIC slope)
slope_external_deps(app PUBLIC imgui)
slope_external_deps(app PRIVATE stb glfw glad glm)
