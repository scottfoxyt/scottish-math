project(
    ScottishMath
    VERSION 0.0.1.0
    LANGUAGES C CXX
)

add_library(ScottishMath INTERFACE
    include/scottish_math/scottish_math.h
)
target_include_directories(ScottishMath
    include
)