#Open3D Wrapper
cmake_minimum_required(VERSION 3.28)
project(triplclust_open3d)

set(CMAKE_CXX_STANDARD 20)

find_package(Open3D REQUIRED)

add_library(triplclust_open3d
    triplclust_wrapper.cpp
    triplclust_wrapper.h
)
target_link_libraries(${PROJECT_NAME}
    triplclust_lib
    Open3D::Open3D
)
target_include_directories(triplclust_open3d PUBLIC src)
