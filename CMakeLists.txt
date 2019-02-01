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


if (LINUX)
    #TODO installation
endif()


if (APPLE)
    # default library path
    message(STATUS "Checking Eigen availability...")
    if (NOT EXISTS ${EIGEN3_INCLUDE_DIR})
        message(WARNING "Unable to find EIGEN3_INCLUDE_DIR...")
        message(STATUS "Installing Eigen...")
        # now we need to download the Eigen library
        execute_process(COMMAND brew install eigen)
    endif()
endif()

find_package (Eigen3 REQUIRED)

INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.googletest googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()



add_library(ModernRoboticsCpp SHARED src/modern_robotics.cpp include/modern_robotics.h)

SET(LIB_TEST_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lib_test.cpp
)

add_executable(lib_test ${LIB_TEST_SOURCES})
target_link_libraries( lib_test ModernRoboticsCpp gtest_main)



install (TARGETS ModernRoboticsCpp
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
