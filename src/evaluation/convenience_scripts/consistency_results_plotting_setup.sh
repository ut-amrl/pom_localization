#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
output_dir=${base_dir}output/backup_drive/
runtime_cfgs_dir=${base_dir}runtime_configs/
generated_dir=${base_dir}generated/
lego_loam_no_imu_out_dir=${base_dir}lego_loam_no_imu/output/
enml_out_dir=${base_dir}enml/
wheel_odom_out_dir=${base_dir}wheel_odom/

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-16-08-36
seventh_traj_date=2021-08-04-15-44-03
eighth_traj_date=2021-08-04-11-27-59

runtime_cfg_base_name=$1

consistency_evaluator_namespace=consistency_${runtime_cfg_base_name}

# Switching to 7 because 8th traj was giving problems
rosparam set /${consistency_evaluator_namespace}/waypoint_consistency_num_trajectories 8
#rosparam set /${consistency_evaluator_namespace}/waypoint_consistency_num_trajectories 7
rosparam set /${consistency_evaluator_namespace}/trajectory_0/trajectory_output_file "${output_dir}momo_first_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_0/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/trajectory_output_file "${output_dir}momo_second_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/trajectory_output_file "${output_dir}momo_third_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_3/trajectory_output_file "${output_dir}momo_fourth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_3/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_4/trajectory_output_file "${output_dir}momo_fifth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_4/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_5/trajectory_output_file "${output_dir}momo_sixth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_5/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_6/trajectory_output_file "${output_dir}momo_seventh_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_6/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_7/trajectory_output_file "${output_dir}momo_eighth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_7/waypoint_to_node_id_file "${generated_dir}waypoints_by_node_${eighth_traj_date}.csv"

rosparam set /${consistency_evaluator_namespace}/num_comparison_approaches 3
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/approach_label "LeGO-LOAM (no IMU)"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_0/trajectory_output_file "${lego_loam_no_imu_out_dir}${first_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_0/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_1/trajectory_output_file "${lego_loam_no_imu_out_dir}${second_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_1/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_2/trajectory_output_file "${lego_loam_no_imu_out_dir}${third_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_2/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_3/trajectory_output_file "${lego_loam_no_imu_out_dir}${fourth_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_3/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_4/trajectory_output_file "${lego_loam_no_imu_out_dir}${fifth_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_4/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_5/trajectory_output_file "${lego_loam_no_imu_out_dir}${sixth_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_5/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_6/trajectory_output_file "${lego_loam_no_imu_out_dir}${seventh_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_6/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_7/trajectory_output_file "${lego_loam_no_imu_out_dir}${eighth_traj_date}_traj_est.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/trajectory_7/waypoint_to_node_id_file "${lego_loam_no_imu_out_dir}waypoints_by_node_${eighth_traj_date}.csv"

rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/approach_label "Wheel Odometry"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_0/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_0/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_1/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_1/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_2/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_2/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_3/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_3/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_4/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_4/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_5/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_5/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_6/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_6/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_7/trajectory_output_file "${wheel_odom_out_dir}trajectory_est_${eighth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_1/trajectory_7/waypoint_to_node_id_file "${wheel_odom_out_dir}waypoints_by_node_${eighth_traj_date}.csv"

rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/approach_label "EnML"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_0/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_0/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_1/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_1/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_2/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_2/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_3/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_3/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_4/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_4/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_5/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_5/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_6/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_6/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_7/trajectory_output_file "${enml_out_dir}trajectory_est_with_waypoints_${eighth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/trajectory_7/waypoint_to_node_id_file "${enml_out_dir}waypoints_by_node_${eighth_traj_date}.csv"

echo Running consistency evaluator
#python src/evaluation/plot_waypoint_consistency_results.py --param_prefix ${consistency_evaluator_namespace}

rosparam set /${consistency_evaluator_namespace}/results_file "${output_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}

rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/waypoint_consistency_num_trajectories 8
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/results_file "${lego_loam_no_imu_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_0

rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/waypoint_consistency_num_trajectories 8
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/results_file "${enml_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_2

