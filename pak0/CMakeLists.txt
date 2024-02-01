# CMake 最低版本要求
cmake_minimum_required (VERSION 2.8.11)

# 项目信息
project (Transformation)

find_package(Eigen3 REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

# 指定生成目标
add_executable (Transformation main.cpp)
