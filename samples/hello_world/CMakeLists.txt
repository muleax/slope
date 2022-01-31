cmake_minimum_required(VERSION 3.20)

slope_executable(hello_world ${SLOPE_SAMPLES_OUTPUT_DIR})

target_link_libraries(hello_world
        PRIVATE app glad)
