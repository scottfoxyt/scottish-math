cmake_minimum_required(VERSION 3.31)

project(
    ScottishMath
    VERSION 0.0.1
    LANGUAGES C CXX
)

add_library(ScottishMath INTERFACE
    include/scottish_math/scottish_math.h
)

target_include_directories(ScottishMath INTERFACE
    include
)
