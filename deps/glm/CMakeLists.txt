cmake_minimum_required(VERSION 3.20)

function(_target_add_glm target privacy path)
    target_include_directories(${target} ${privacy} ${path}/glm)
endfunction()