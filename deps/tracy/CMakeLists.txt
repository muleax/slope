cmake_minimum_required(VERSION 3.20)

file(GLOB LIB_SOURCES "tracy/TracyClient.cpp")
file(GLOB LIB_HEADERS)

add_library(tracy STATIC ${LIB_HEADERS} ${LIB_SOURCES})

target_include_directories(tracy
        INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tracy)
