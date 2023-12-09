cmake_minimum_required (VERSION 2.8.11)
project (motion-planning)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(src)
add_library(motion-planning src/ik.cpp src/rrt.cpp)

add_executable(test_ik test_ik.cpp)
add_executable(test_rrt test_rrt.cpp)
target_link_libraries(test_ik motion-planning)
target_link_libraries(test_rrt motion-planning)


