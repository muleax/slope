cmake_minimum_required(VERSION 3.20)

slope_executable(minimal_example ${SLOPE_SAMPLES_OUTPUT_DIR})

target_link_libraries(minimal_example
        PRIVATE slope)
