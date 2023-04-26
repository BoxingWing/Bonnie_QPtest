cmake_minimum_required(VERSION 3.24)
project(Bonnie_QPtest)

set(CMAKE_CXX_STANDARD 14)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots")
find_package(pinocchio REQUIRED)

add_subdirectory(quill)
add_subdirectory(qpOASES)

include_directories("/usr/include/eigen3")
include_directories("qpOASES/include")


add_executable(Bonnie_QPtest main.cpp WBC_srb.cpp WBC_srb.h FileOperator.cpp FileOperator.h Pinocchio_Utilities.h Pinocchio_Utilities.cpp)
target_link_libraries(Bonnie_QPtest quill::quill qpOASES pinocchio::pinocchio)
