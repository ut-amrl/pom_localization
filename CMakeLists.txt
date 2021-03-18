PROJECT(autodiff_gp)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Autodiff GP Example")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

#SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
SET(CMAKE_CXX_FLAGS "-std=c++17 -lstdc++fs -Wall -Werror -O2")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(PCL 1.3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
find_package(Ceres REQUIRED)

option(TESTING "Build tests" ON)

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib lua5.1 ceres
    pthread)

INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src)


ROSBUILD_ADD_EXECUTABLE(autodiff_gp
                        src/autodiff_gp_main.cc)
TARGET_LINK_LIBRARIES(autodiff_gp ${libs})
#
#ROSBUILD_ADD_EXECUTABLE(momo_pose_optimization src/pose_optimization/pose_optimization_main.cc)
#TARGET_LINK_LIBRARIES(momo_pose_optimization ${libs})

ROSBUILD_ADD_EXECUTABLE(momo_pose_optimization_test src/synthetic_problem/movable_pose_optimization_test_main.cc)
TARGET_LINK_LIBRARIES(momo_pose_optimization_test ${libs})

ROSBUILD_ADD_EXECUTABLE(read_h3d_point_cloud src/h3d_dataset/read_h3d_point_cloud.cc)
TARGET_LINK_LIBRARIES(read_h3d_point_cloud ${libs} ${PCL_LIBRARIES} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(run_on_h3d_detections src/h3d_dataset/run_on_h3d_detections.cc)
TARGET_LINK_LIBRARIES(run_on_h3d_detections ${libs} ${PCL_LIBRARIES} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(output_first_gps_entries src/h3d_dataset/output_first_gps_entries.cc)
TARGET_LINK_LIBRARIES(output_first_gps_entries ${libs} stdc++fs)


#IF(TESTING)
#  enable_testing()
##  add_subdirectory(test)
#
#  ADD_EXECUTABLE(autodiff_gp_test
#          test/pose_optimization/test_movable_observation_gp_cost_functor.cpp)
#  TARGET_LINK_LIBRARIES(autodiff_gp_test
#
#          gtest
#          gtest_main
#          libs)
#ENDIF()