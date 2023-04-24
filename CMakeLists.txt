cmake_minimum_required(VERSION 3.24)
project(Bonnie_QPtest)

set(CMAKE_CXX_STANDARD 14)

add_subdirectory(quill)
add_subdirectory(qpOASES)

include_directories("/usr/include/eigen3")


add_executable(Bonnie_QPtest main.cpp WBC_srb.cpp WBC_srb.h FileOperator.cpp FileOperator.h)
target_link_libraries(Bonnie_QPtest quill::quill qpOASES)
