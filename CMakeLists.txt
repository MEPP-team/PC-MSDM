cmake_minimum_required(VERSION 2.6)
project(PC-MSDM)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fopenmp")
endif()

if(OPENMP_FOUND)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
endif()

add_executable(PC-MSDM main.cpp)

include_directories(eigen3/)

install(TARGETS PC-MSDM RUNTIME DESTINATION bin)
