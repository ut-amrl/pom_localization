#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
#output_dir=${base_dir}output/backup_drive/
output_dir=${base_dir}output/
runtime_cfgs_dir=${base_dir}runtime_configs/
generated_dir=${base_dir}generated/
lego_loam_no_imu_out_dir=${base_dir}lego_loam_no_imu/output/
enml_out_dir=${base_dir}enml/
wheel_odom_out_dir=${base_dir}wheel_odom/
carto_out_dir=${base_dir}cartographer/

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
segmentation_consistency_evaluator_namespace=consistency_samples_50_obj_40_x_0_475_k5_15_k6_17_quad_freq_cluster_config_22_02_23_19_40_fewer_sample_per_point_server

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

rosparam set /${consistency_evaluator_namespace}/num_comparison_approaches 4
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


rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/approach_label "POM-Localization (sem seg)"
seg_traj_0_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_0/trajectory_output_file"
segmentation_traj_0=`eval ${seg_traj_0_cmd}`
seg_wp_0_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_0/waypoint_to_node_id_file"
segmentation_wp_0=`eval ${seg_wp_0_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_0/trajectory_output_file "${segmentation_traj_0}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_0/waypoint_to_node_id_file "${segmentation_wp_0}"

seg_traj_1_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_1/trajectory_output_file"
segmentation_traj_1=`eval ${seg_traj_1_cmd}`
seg_wp_1_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_1/waypoint_to_node_id_file"
segmentation_wp_1=`eval ${seg_wp_1_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_1/trajectory_output_file "${segmentation_traj_1}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_1/waypoint_to_node_id_file "${segmentation_wp_1}"

seg_traj_2_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_2/trajectory_output_file"
segmentation_traj_2=`eval ${seg_traj_2_cmd}`
seg_wp_2_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_2/waypoint_to_node_id_file"
segmentation_wp_2=`eval ${seg_wp_2_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_2/trajectory_output_file "${segmentation_traj_2}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_2/waypoint_to_node_id_file "${segmentation_wp_2}"

seg_traj_3_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_4/trajectory_output_file"
segmentation_traj_3=`eval ${seg_traj_3_cmd}`
seg_wp_3_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_4/waypoint_to_node_id_file"
segmentation_wp_3=`eval ${seg_wp_3_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_3/trajectory_output_file "${segmentation_traj_3}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_3/waypoint_to_node_id_file "${segmentation_wp_3}"

seg_traj_4_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_4/trajectory_output_file"
segmentation_traj_4=`eval ${seg_traj_4_cmd}`
seg_wp_4_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_4/waypoint_to_node_id_file"
segmentation_wp_4=`eval ${seg_wp_4_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_4/trajectory_output_file "${segmentation_traj_4}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_4/waypoint_to_node_id_file "${segmentation_wp_4}"

seg_traj_5_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_5/trajectory_output_file"
segmentation_traj_5=`eval ${seg_traj_5_cmd}`
seg_wp_5_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_5/waypoint_to_node_id_file"
segmentation_wp_5=`eval ${seg_wp_5_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_5/trajectory_output_file "${segmentation_traj_5}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_5/waypoint_to_node_id_file "${segmentation_wp_5}"

seg_traj_6_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_6/trajectory_output_file"
segmentation_traj_6=`eval ${seg_traj_6_cmd}`
seg_wp_6_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_6/waypoint_to_node_id_file"
segmentation_wp_6=`eval ${seg_wp_6_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_6/trajectory_output_file "${segmentation_traj_6}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_6/waypoint_to_node_id_file "${segmentation_wp_6}"

seg_traj_7_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_7/trajectory_output_file"
segmentation_traj_7=`eval ${seg_traj_7_cmd}`
seg_wp_7_cmd="rosparam get /${segmentation_consistency_evaluator_namespace}/trajectory_7/waypoint_to_node_id_file"
segmentation_wp_7=`eval ${seg_wp_7_cmd}`
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_7/trajectory_output_file "${segmentation_traj_7}"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/trajectory_7/waypoint_to_node_id_file "${segmentation_wp_7}"

rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/approach_label "Cartographer"
#echo "Setting comparison approach 4 traj 0 to ${carto_out_dir}2dscan-${first_traj_date}.csv"
#rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_0/trajectory_output_file "${carto_out_dir}2dscan-${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_0/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_0/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_1/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_1/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_2/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_2/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_3/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_3/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_4/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_4/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_5/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_5/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_6/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_6/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${seventh_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_7/trajectory_output_file "${carto_out_dir}trajectory_est_with_waypoints_${eighth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/trajectory_7/waypoint_to_node_id_file "${carto_out_dir}waypoints_by_node_${eighth_traj_date}.csv"



echo Running consistency evaluator
python src/evaluation/plot_waypoint_consistency_results.py --param_prefix ${consistency_evaluator_namespace}

rosparam set /${consistency_evaluator_namespace}/results_file "${output_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}

rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/waypoint_consistency_num_trajectories 8
rosparam set /${consistency_evaluator_namespace}/comparison_approach_0/results_file "${lego_loam_no_imu_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_0

rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/waypoint_consistency_num_trajectories 8
rosparam set /${consistency_evaluator_namespace}/comparison_approach_2/results_file "${enml_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_2

rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/waypoint_consistency_num_trajectories 8
rosparam set /${consistency_evaluator_namespace}/comparison_approach_3/results_file "${enml_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_3

#rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/waypoint_consistency_num_trajectories 1
#rosparam set /${consistency_evaluator_namespace}/comparison_approach_4/results_file "${carto_out_dir}consistency_${runtime_cfg_base_name}_8.csv"
#make && ./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}/comparison_approach_4
#
