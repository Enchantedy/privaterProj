cmake_minimum_required(VERSION 2.8)

project(bill)

add_compile_options(-std=c++11)

SET(CMAKE_SYSTEM_NAME Linux)

include_directories(
/home/c/umouse/codetest/myProject
)

add_executable(proj
/home/c/umouse/codetest/myProject/src/main.cpp
/home/c/umouse/codetest/myProject/src/skeletonService.cpp
/home/c/umouse/codetest/myProject/src/manager.cpp
/home/c/umouse/codetest/myProject/src/singlyList.cpp
/home/c/umouse/codetest/myProject/src/alongWall.cpp
)

target_link_libraries(proj pthread)