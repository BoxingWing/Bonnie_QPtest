cmake_minimum_required(VERSION 3.24)
project(Bonnie_QPtest)

set(CMAKE_CXX_STANDARD 17)

add_executable(Bonnie_QPtest main.cpp)
target_link_libraries(Bonnie_QPtest libqpOASES.a)
