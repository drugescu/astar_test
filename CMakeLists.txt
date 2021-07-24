cmake_minimum_required(VERSION 3.0.0)
project(astar_test VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(astar_test astar.cpp)
include_directories(Eigen)
target_include_directories (astar_test PUBLIC Eigen)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
