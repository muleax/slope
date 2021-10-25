cmake_minimum_required(VERSION 3.20)

add_subdirectory(fmt)

function(_target_add_fmt target privacy path)
    target_link_libraries(${target} ${privacy} fmt::fmt)
endfunction()