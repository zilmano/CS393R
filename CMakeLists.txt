PROJECT(reference_car)
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

MESSAGE(STATUS "Compiling Reference Car Localization + Navigation")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Og -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ADD_LIBRARY(shared_library
            src/visualization/visualization.cc
            src/vector_map/vector_map.cc)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src)

add_subdirectory(thirdparty/xtl)
add_subdirectory(thirdparty/xtensor)
add_subdirectory(thirdparty/xtensor-blas)
add_subdirectory(thirdparty/xsimd)

find_package(OpenCV REQUIRED)

add_definitions(-DHAVE_CBLAS=1)

if (WIN32)
  find_package(OpenBLAS REQUIRED)
  set(BLAS_LIBRARIES ${CMAKE_INSTALL_PREFIX}${OpenBLAS_LIBRARIES})
else()
  find_package(BLAS REQUIRED)
  find_package(LAPACK REQUIRED)
endif()

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

SET(libs roslib roscpp rosbag glog gflags amrl-shared-lib boost_system lua5.1
        pthread xtensor xtensor-blas xsimd ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES}  ${OpenCV_LIBS})

ROSBUILD_ADD_EXECUTABLE(slam
                        src/slam/rasterizer.cpp
                        src/slam/slam_main.cc
                        src/slam/slam.cc)
TARGET_LINK_LIBRARIES(slam shared_library ${libs})


ROSBUILD_ADD_EXECUTABLE(particle_filter
                        src/particle_filter/particle_filter_main.cc
                        src/particle_filter/particle_filter.cc
                        src/particle_filter/observation_model.cpp
        )
TARGET_LINK_LIBRARIES(particle_filter shared_library ${libs})

ROSBUILD_ADD_EXECUTABLE(navigation
                        src/navigation/navigation_main.cc
                        src/navigation/navigation.cc
                        src/navigation/plotpublisher.cpp
                        src/navigation/clock.cpp
                        src/navigation/pointcloud.cpp
                        src/navigation/collisionplanner.cpp
                        src/navigation/state_estimator.cc
			src/navigation/global_planner.cc
        )
TARGET_LINK_LIBRARIES(navigation shared_library ${libs} )

target_include_directories(navigation PUBLIC
          ${xtensor_INCLUDE_DIRS}/xtensor ${xsimd_INCLUDE_DIRS}/xsimd ${xtensor-blas_INCLUDE_DIRS}/xtensor-blas)

target_include_directories(particle_filter PUBLIC
        ${xtensor_INCLUDE_DIRS}/xtensor ${xsimd_INCLUDE_DIRS}/xsimd ${xtensor-blas_INCLUDE_DIRS}/xtensor-blas)

target_include_directories(slam PUBLIC
        ${xtensor_INCLUDE_DIRS}/xtensor ${xsimd_INCLUDE_DIRS}/xsimd ${xtensor-blas_INCLUDE_DIRS}/xtensor-blas  ${OpenCV_INCLUDE_DIRS})


ADD_EXECUTABLE(eigen_tutorial
               src/eigen_tutorial.cc)
