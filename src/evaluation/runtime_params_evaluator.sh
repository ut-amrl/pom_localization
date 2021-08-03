#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
output_dir=${base_dir}output/
runtime_cfgs_dir=${base_dir}runtime_configs/

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45

runtime_cfg_base_name=$1

first_traj_eval_namespace=evaluation_first_traj_${runtime_cfg_base_name}
second_traj_eval_namespace=evaluation_second_traj_${runtime_cfg_base_name}
third_traj_eval_namespace=evaluation_third_traj_${runtime_cfg_base_name}

consistency_evaluator_namespace=consistency_${runtime_cfg_base_name}

# Set ROS params
rosparam set /${first_traj_eval_namespace}/obj_det_curr_traj_file "${base_dir}obj_detections_by_node_${first_traj_date}.csv"
rosparam set /${first_traj_eval_namespace}/odom_traj_est_file "${base_dir}odom_est_${first_traj_date}.csv"
rosparam set /${first_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${first_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${first_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_first_traj_est_"

rosparam set /${second_traj_eval_namespace}/obj_det_curr_traj_file "${base_dir}obj_detections_by_node_${second_traj_date}.csv"
rosparam set /${second_traj_eval_namespace}/odom_traj_est_file "${base_dir}odom_est_${second_traj_date}.csv"
rosparam set /${second_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${second_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${second_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_second_traj_est_"

rosparam set /${third_traj_eval_namespace}/obj_det_curr_traj_file "${base_dir}obj_detections_by_node_${third_traj_date}.csv"
rosparam set /${third_traj_eval_namespace}/odom_traj_est_file "${base_dir}odom_est_${third_traj_date}.csv"
rosparam set /${third_traj_eval_namespace}/past_samples_files "['${base_dir}samples_distribution_config_narrower_yet_5_traj.txt']"
rosparam set /${third_traj_eval_namespace}/runtime_params_config_file "${runtime_cfgs_dir}${runtime_cfg_base_name}.csv"
rosparam set /${third_traj_eval_namespace}/traj_est_output_file_prefix "${output_dir}momo_third_traj_est_"

rosparam set /${consistency_evaluator_namespace}/waypoint_consistency_num_trajectories 3
rosparam set /${consistency_evaluator_namespace}/trajectory_0/trajectory_output_file "${output_dir}momo_first_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_0/waypoint_to_node_id_file "${base_dir}waypoints_by_node_${first_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/trajectory_output_file "${output_dir}momo_second_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_1/waypoint_to_node_id_file "${base_dir}waypoints_by_node_${second_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/trajectory_output_file "${output_dir}momo_third_traj_est_${runtime_cfg_base_name}.csv"
rosparam set /${consistency_evaluator_namespace}/trajectory_2/waypoint_to_node_id_file "${base_dir}waypoints_by_node_${third_traj_date}.csv"
rosparam set /${consistency_evaluator_namespace}/results_file "${output_dir}/consistency_results_${runtime_cfg_base_name}.csv"

command() {
    echo $1 start
    sleep $(( $1 & 03 ))      # keep the seconds value within 0-3
    echo $1 complete
}

make 
echo Estimating trajectories
./bin/evaluation_main --param_prefix ${first_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${second_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${third_traj_eval_namespace} &
wait

echo Running consistency evaluator
./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}

