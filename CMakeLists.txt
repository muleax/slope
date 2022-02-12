cmake_minimum_required(VERSION 3.20)

option(TracyEnable "TracyEnable" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

set(SLOPE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if (TracyEnable)
    add_compile_definitions(TRACY_ENABLE)
    add_compile_definitions(SL_TRACY_ENABLE)
endif()

function(slope_library target)
    file(GLOB_RECURSE LIB_HEADERS "*.hpp")
    file(GLOB_RECURSE LIB_SOURCES "*.cpp")
    add_library(${target} STATIC ${LIB_HEADERS} ${LIB_SOURCES})

    target_include_directories(${target} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

function(slope_executable target output_dir)
    file(GLOB_RECURSE EXE_HEADERS "*.hpp")
    file(GLOB_RECURSE EXE_SOURCES "*.cpp")
    add_executable(${target} ${EXE_HEADERS} ${EXE_SOURCES})

    target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    set_target_properties(${target}
    	PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${output_dir}/bin/${target}
        VS_DEBUGGER_WORKING_DIRECTORY $<TARGET_FILE_DIR:${target}>
    )
endfunction()

project(SLOPE)

add_subdirectory("deps")
add_subdirectory("slope")
add_subdirectory("app")
add_subdirectory("samples")
add_subdirectory("tools")
