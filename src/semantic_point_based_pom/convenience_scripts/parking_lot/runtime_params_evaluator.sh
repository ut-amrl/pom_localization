#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
output_dir=${base_dir}semantic_point_output/
runtime_cfgs_dir=${base_dir}runtime_configs/
generated_dir=${base_dir}generated/
semantic_point_generated_dir=${base_dir}semantic_point_generated/
semantic_point_config_dir=${base_dir}semantic_point_config/

# Get input
clustering_config_file_base=$1
runtime_cfg_base_name=$2
rectangle_sampler_config_file_base=$3

# Constants
first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-16-08-36
seventh_traj_date=2021-08-04-15-44-03
eighth_traj_date=2021-08-04-11-27-59

odom_out_prefix=odom_est_
waypoints_by_node_prefix=waypoints_by_node_
semantic_points_by_node_prefix=semantic_points_by_node_
csv_suffix=.csv
consistency_results_base_prefix=consistency_results_
traj_est_base_prefix=traj_est_

shape_dim_base_name=shape_dim_by_sematic_class_car_camry${csv_suffix}
detection_sensor_rel_baselink_file_base=detection_sensor_rel_baselink${csv_suffix}
semantic_index_to_string_file_base=semantic_index_to_string${csv_suffix}


# Intermediate variables
first_traj_eval_namespace=evaluation_first_traj_${runtime_cfg_base_name}
second_traj_eval_namespace=evaluation_second_traj_${runtime_cfg_base_name}
third_traj_eval_namespace=evaluation_third_traj_${runtime_cfg_base_name}
fourth_traj_eval_namespace=evaluation_fourth_traj_${runtime_cfg_base_name}
fifth_traj_eval_namespace=evaluation_fifth_traj_${runtime_cfg_base_name}
sixth_traj_eval_namespace=evaluation_sixth_traj_${runtime_cfg_base_name}
seventh_traj_eval_namespace=evaluation_seventh_traj_${runtime_cfg_base_name}
eighth_traj_eval_namespace=evaluation_eighth_traj_${runtime_cfg_base_name}

consistency_evaluator_namespace=consistency_${runtime_cfg_base_name}


clustering_config_base_prefix=config_${clustering_config_file_base}_

# Set variables
consistency_results_file_name=${output_dir}${consistency_results_base_prefix}${clustering_config_base_prefix}${rectangle_sampler_config_file_base}_${runtime_cfg_base_name}${csv_suffix}
# TODO consider making the config for the distribution a param
past_samples_file=${base_dir}samples_distribution_config_narrower_yet_5_traj.txt
runtime_params_config_file_name=${runtime_cfgs_dir}${runtime_cfg_base_name}${csv_suffix}
detection_sensor_rel_baselink_file_name=${semantic_point_config_dir}${detection_sensor_rel_baselink_file_base}
semantic_index_to_string_file_name=${semantic_point_config_dir}${semantic_index_to_string_file_base}
shape_dimensions_by_semantic_class_file_name=${semantic_point_generated_dir}${shape_dim_base_name}
semantic_point_object_sampler_config_file_name=${semantic_point_generated_dir}${rectangle_sampler_config_file_base}${csv_suffix}

traj_out_common_prefix=${output_dir}${traj_est_base_prefix}

# These should line up with preprocess_parking_lot_data_for_seq
odom_traj_est_common_prefix=${semantic_point_generated_dir}${odom_out_prefix}
waypoints_by_node_common_prefix=${semantic_point_generated_dir}${waypoints_by_node_prefix}
semantic_points_by_node_common_prefix=${semantic_point_generated_dir}${semantic_points_by_node_prefix}


namespaces=(${first_traj_eval_namespace} ${second_traj_eval_namespace} ${third_traj_eval_namespace} ${fourth_traj_eval_namespace} ${fifth_traj_eval_namespace} ${sixth_traj_eval_namespace} ${seventh_traj_eval_namespace} ${eighth_traj_eval_namespace})
bag_time_strings=(${first_traj_date} ${second_traj_date} ${third_traj_date} ${fourth_traj_date} ${fifth_traj_date} ${sixth_traj_date} ${seventh_traj_date} ${eighth_traj_date})

# Set ROS params
for i in ${!namespaces[@]}; do
  curr_namespace=${namespaces[$i]}
  curr_bag_time_string=${bag_time_strings[$i]}
  echo "Iter ${i}: Setting params for namespace ${curr_namespace} and bag ${curr_bag_time_string}"

  cluster_cfg_base_name_and_bag_combo=${clustering_config_base_prefix}${curr_bag_time_string}

  # Set trajectory specific variables
  traj_est_output_file_for_run=${traj_out_common_prefix}${curr_bag_time_string}${csv_suffix}

  # These should line up with preprocess_parking_lot_data_for_seq
  semantic_point_det_curr_traj_file_name=${semantic_points_by_node_common_prefix}${cluster_cfg_base_name_and_bag_combo}${csv_suffix}
  odom_traj_est_file_name=${odom_traj_est_common_prefix}${cluster_cfg_base_name_and_bag_combo}${csv_suffix}
  waypoint_to_node_id_file_name=${waypoints_by_node_common_prefix}${cluster_cfg_base_name_and_bag_combo}${csv_suffix}

  # Common to all
  rosparam set /${curr_namespace}/past_samples_files "['${past_samples_file}']"
  rosparam set /${curr_namespace}/runtime_params_config_file "${runtime_params_config_file_name}"
  rosparam set /${curr_namespace}/detection_sensor_rel_baselink_file "${detection_sensor_rel_baselink_file_name}"
  rosparam set /${curr_namespace}/semantic_index_to_string_file "${semantic_index_to_string_file_name}"
  rosparam set /${curr_namespace}/shape_dimensions_by_semantic_class_file "${shape_dimensions_by_semantic_class_file_name}"
  rosparam set /${curr_namespace}/semantic_point_object_sampler_config_file "${semantic_point_object_sampler_config_file_name}"

  # Trajectory dependent
  rosparam set /${curr_namespace}/odom_traj_est_file "${odom_traj_est_file_name}"
  rosparam set /${curr_namespace}/semantic_point_det_curr_traj_file "${semantic_point_det_curr_traj_file_name}"
  rosparam set /${curr_namespace}/traj_est_output_file "${traj_est_output_file_for_run}"

  # For consistency evaluator
  rosparam set /${consistency_evaluator_namespace}/trajectory_${i}/trajectory_output_file "${traj_est_output_file_for_run}"
  rosparam set /${consistency_evaluator_namespace}/trajectory_${i}/waypoint_to_node_id_file "${waypoint_to_node_id_file_name}"
done

# Set the rest of the consistency evaluator params
rosparam set /${consistency_evaluator_namespace}/results_file "${consistency_results_file_name}"
rosparam set /${consistency_evaluator_namespace}/waypoint_consistency_num_trajectories 8

make
echo Estimating trajectories
./bin/evaluation_main --param_prefix ${first_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${second_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${third_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${fourth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${fifth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${sixth_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${seventh_traj_eval_namespace} &
./bin/evaluation_main --param_prefix ${eighth_traj_eval_namespace} &
wait

echo Running consistency evaluator
./bin/compute_waypoint_consistency_results --param_prefix ${consistency_evaluator_namespace}

