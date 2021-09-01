#!/bin/bash

input_dir=$HOME/datasets/kitti_odometry/bags/
output_dir=$HOME/datasets/kitti_odometry/bags/output/

seq_num=$1

rosparam set /odom_traj_est_file ${input_dir}lego_loam_2d_traj_est_${seq_num}.txt
rosparam set /aligned_est_traj_file ${output_dir}tuned_and_sped_up_single_run_aligned_kitti_det_local_from_global_est_trajectory_${seq_num}.txt
rosparam set /misaligned_est_traj_file ${output_dir}tuned_and_sped_up_single_run_misaligned_kitti_det_local_from_global_est_trajectory_${seq_num}.txt
#rosparam set /somewhat_aligned_est_traj_file ${input_dir}tuned_and_sped_up_single_run_slightly_misaligned_kitti_det_local_from_global_est_trajectory_${seq_num}_local_kitti_runtime_params_cfg_commit_52d288c.csv
rosparam set /somewhat_aligned_est_traj_file ${output_dir}tuned_and_sped_up_single_run_slightly_misaligned_kitti_det_local_from_global_est_trajectory_${seq_num}_kitti_runtime_params_cfg_commit_52d288c.csv
rosparam set /gt_trajectory_file ${input_dir}planar_approx_poses_${seq_num}.txt

make && ./bin/compute_kitti_results
