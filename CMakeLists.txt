cmake_minimum_required(VERSION 3.12)
project(ModernRoboticsCpp)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR
        "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
set(warnings "-Wall -Wextra -Werror")
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
 set(warnings "/W4 /WX /EHsc")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_STANDARD 17)


SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )

message(STATUS "Checking Eigen availability...")
if (NOT EXISTS ${EIGEN3_INCLUDE_DIR})
    message(WARNING "Unable to find EIGEN3_INCLUDE_DIR...")
    message(FATAL_ERROR  "Please set the environment variable like this: export EIGEN3_INCLUDE_DIR=/path/to/eigen3/")
endif()


find_package (Eigen3 3.3 REQUIRED NO_MODULE)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

add_library(ModernRoboticsCpp SHARED src/modern_robotics.cpp include/modern_robotics.h)

install (TARGETS ModernRoboticsCpp
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
