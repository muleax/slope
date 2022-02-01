cmake_minimum_required(VERSION 3.20)

slope_executable(playground ${SLOPE_SAMPLES_OUTPUT_DIR})

target_link_libraries(playground
        PRIVATE app glad)
