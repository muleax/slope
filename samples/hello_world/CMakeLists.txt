cmake_minimum_required(VERSION 3.20)

slope_executable(hello_world ${SLOPE_SAMPLES_OUTPUT_DIR})

slope_internal_deps(hello_world PRIVATE app)

slope_external_deps(hello_world PRIVATE glad)

#add_custom_command(
#        TARGET hello_world POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_directory
#                ${SLOPE_SAMPLES_RESOURCE_DIR}
#                $<TARGET_FILE_DIR:hello_world>/resources)
