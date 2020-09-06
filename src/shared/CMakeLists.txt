cmake_minimum_required(VERSION 2.8.11)

PROJECT(amrl-shared-lib)
SET(CMAKE_CXX_STANDARD 11)

SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(libs glog pthread)

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wno-stringop-truncation")

ADD_LIBRARY(amrl-shared-lib
            util/helpers.cc
            util/pthread_utils.cc
            util/timer.cc
            util/random.cc
            util/serialization.cc
            util/terminal_colors.cc ros/ros_helper_xtensor.h)
TARGET_LINK_LIBRARIES(amrl-shared-lib ${libs})


#ADD_EXECUTABLE(unit_tests
#               tests/math/line2d_tests.cc
#               tests/math/math_tests.cc)
#TARGET_LINK_LIBRARIES(unit_tests amrl-shared-lib gtest gtest_main ${libs})
