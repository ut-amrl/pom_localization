#!/bin/bash

base_dir=$HOME/datasets/kitti_odometry/bags/

cfg_file_name=$1
make

for sequence_num in 00 01 02 03 04 05 06 07 08 09 10
do
        echo $sequence_num
        rosparam set /distribution_gen_config_file ${cfg_file_name}
        rosparam set /distribution_samples_output_file_prefix /home/amanda/datasets/kitti_odometry/bags/samples_${sequence_num}_
        rosparam set /trajectory_2d_file /home/amanda/datasets/kitti_odometry/bags/planar_approx_poses_${sequence_num}.txt
        rosparam set /parking_spots_file /home/amanda/datasets/kitti_odometry/bags/global_bounding_box_2d_centroid_${sequence_num}.csv
        ./bin/create_distribution_from_global_obj_poses_multiple_config
done
