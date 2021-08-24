#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
output_dir=${base_dir}output/
runtime_cfgs_dir=${base_dir}runtime_configs/
generated_dir=${base_dir}generated/

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-11-27-59
seventh_traj_date=2021-08-04-15-44-03

runtime_cfg_base_name=$1

first_traj_eval_namespace=evaluation_first_traj_${runtime_cfg_base_name}
second_traj_eval_namespace=evaluation_second_traj_${runtime_cfg_base_name}
third_traj_eval_namespace=evaluation_third_traj_${runtime_cfg_base_name}
fourth_traj_eval_namespace=evaluation_fourth_traj_${runtime_cfg_base_name}
fifth_traj_eval_namespace=evaluation_fifth_traj_${runtime_cfg_base_name}
sixth_traj_eval_namespace=evaluation_sixth_traj_${runtime_cfg_base_name}
seventh_traj_eval_namespace=evaluation_seventh_traj_${runtime_cfg_base_name}

consistency_evaluator_namespace=consistency_${runtime_cfg_base_name}

# Set ROS params
rosparam set /${first_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${first_traj_date}.csv"
rosparam set /${first_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${first_traj_date}.csv"
rosparam set /${first_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${first_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${first_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_first_traj_est_"

rosparam set /${second_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${second_traj_date}.csv"
rosparam set /${second_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${second_traj_date}.csv"
rosparam set /${second_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${second_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${second_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_second_traj_est_"

rosparam set /${third_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${third_traj_date}.csv"
rosparam set /${third_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${third_traj_date}.csv"
rosparam set /${third_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${third_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${third_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_third_traj_est_"

rosparam set /${fourth_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${fourth_traj_date}.csv"
rosparam set /${fourth_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${fourth_traj_date}.csv"
rosparam set /${fourth_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${fourth_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${fourth_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_fourth_traj_est_"

rosparam set /${fifth_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${fifth_traj_date}.csv"
rosparam set /${fifth_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${fifth_traj_date}.csv"
rosparam set /${fifth_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${fifth_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${fifth_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_fifth_traj_est_"

rosparam set /${sixth_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${sixth_traj_date}.csv"
rosparam set /${sixth_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${sixth_traj_date}.csv"
rosparam set /${sixth_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${sixth_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${sixth_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_sixth_traj_est_"

rosparam set /${seventh_traj_eval_namespace}/obj_det_curr_traj_file "${generated_dir}obj_detections_by_node_${seventh_traj_date}.csv"
rosparam set /${seventh_traj_eval_namespace}/odom_traj_est_file "${generated_dir}odom_est_${seventh_traj_date}.csv"
rosparam set /${seventh_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${seventh_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${seventh_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_seventh_traj_est_"

rosparam set /${consistency_evaluator_namespace}/waypoint_consistency_num_trajectories 3
rosparam set /${consistency_evaluator_namespace}/trajectory_0/trajectory_output_file "${output_dir}momo_first_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_0/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/trajectory_output_file "${output_dir}momo_second_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/trajectory_output_file "${output_dir}momo_third_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_3/trajectory_output_file "${output_dir}momo_fourth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_3/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${fourth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_4/trajectory_output_file "${output_dir}momo_fifth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_4/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${fifth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_5/trajectory_output_file "${output_dir}momo_sixth_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_5/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${sixth_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_6/trajectory_output_file "${output_dir}momo_seventh_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_6/waypoint_to_node_id_file "${generated_dir}}waypoints_by_node_${seventh_traj_date}.csv"

rosparam set /${consistency_evaluator_namespace}/results_file "${output_dir}/consistency_results_${runtime_cfg_base_name}.csv"

make 
echo Estimating trajectories
./bin/evaluation_main --param_prefix ${first_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${second_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${third_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${fourth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${fifth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${sixth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${seventh_traj_eval_namespace} &
wait

echo Running consistency evaluator
./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}

