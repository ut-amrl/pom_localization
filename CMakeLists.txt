PROJECT(autodiff_gp)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

MESSAGE(STATUS "Compiling Autodiff GP Example")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

#SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall")
SET(CMAKE_CXX_FLAGS "-std=c++17 -lstdc++fs -Wall -Wno-unused-function -Wno-unused-variable -Werror -O2")

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
find_package(rosbag REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN REQUIRED eigen3)

include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_LIBRARY_DIRS})
find_package(Ceres REQUIRED)

option(TESTING "Build tests" ON)

SET(libs roslib roscpp boost_system rosbag glog gflags amrl_shared_lib lua5.1 ceres
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
#
#ROSBUILD_ADD_EXECUTABLE(momo_pose_optimization_test src/synthetic_problem/movable_pose_optimization_test_main.cc)
#TARGET_LINK_LIBRARIES(momo_pose_optimization_test ${libs})

#ROSBUILD_ADD_EXECUTABLE(read_h3d_point_cloud src/h3d_dataset/read_h3d_point_cloud.cc)
#TARGET_LINK_LIBRARIES(read_h3d_point_cloud ${libs} ${PCL_LIBRARIES} stdc++fs)
#
#ROSBUILD_ADD_EXECUTABLE(run_on_h3d_detections src/h3d_dataset/run_on_h3d_detections.cc)
#TARGET_LINK_LIBRARIES(run_on_h3d_detections ${libs} ${PCL_LIBRARIES} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(evaluation_main src/evaluation/evaluation_main.cc)
TARGET_LINK_LIBRARIES(evaluation_main ${libs} stdc++fs)

#ROSBUILD_ADD_EXECUTABLE(output_first_gps_entries src/h3d_dataset/output_first_gps_entries.cc)
#TARGET_LINK_LIBRARIES(output_first_gps_entries ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(parking_spot_plotter src/evaluation/parking_spot_3d_plotter.cc)
TARGET_LINK_LIBRARIES(parking_spot_plotter ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(wheel_odom_extractor src/evaluation/wheel_odom_rosbag_extraction.cc)
TARGET_LINK_LIBRARIES(wheel_odom_extractor ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(kitti_to_trajectory_2d src/evaluation/kitti_to_trajectory_2d.cc)
TARGET_LINK_LIBRARIES(kitti_to_trajectory_2d ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(lidarodom_to_trajectory_2d src/evaluation/lidar_odom_to_trajectory_2d.cc)
TARGET_LINK_LIBRARIES(lidarodom_to_trajectory_2d ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(create_distribution_from_global_obj_poses src/evaluation/create_distribution_from_global_obj_poses.cc)
TARGET_LINK_LIBRARIES(create_distribution_from_global_obj_poses ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(distribution_gen_config_writer src/evaluation/distribution_generation_config_writer.cc)
TARGET_LINK_LIBRARIES(distribution_gen_config_writer ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(local_detections_creator src/evaluation/create_local_detections_from_global_detections.cc)
TARGET_LINK_LIBRARIES(local_detections_creator ${libs} stdc++fs)

ROSBUILD_ADD_EXECUTABLE(convert_object_detections_from_yaml src/evaluation/convert_object_detections_from_yaml.cc)
TARGET_LINK_LIBRARIES(convert_object_detections_from_yaml ${libs} stdc++fs yaml-cpp)

ROSBUILD_ADD_EXECUTABLE(plot_results src/evaluation/plot_results.cc)
TARGET_LINK_LIBRARIES(plot_results ${libs} stdc++fs)

#ROSBUILD_ADD_EXECUTABLE(uncertainty_aware_localization_test src/synthetic_problem/uncertainty_aware/uncertainty_aware_synthetic_problem_localization_test_main.cc)
#TARGET_LINK_LIBRARIES(uncertainty_aware_localization_test ${libs})


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