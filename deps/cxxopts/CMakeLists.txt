cmake_minimum_required(VERSION 3.20)

function(_target_add_cxxopts target privacy path)
    target_include_directories(${target} ${privacy} ${path})
endfunction()
