#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
output_dir=${base_dir}server_semantic_point_output/
runtime_cfgs_dir=${base_dir}runtime_configs/
semantic_point_generated_dir=${base_dir}semantic_point_generated/
semantic_point_config_dir=${base_dir}semantic_point_config/

# Get input
clustering_config_file_base=$1
runtime_cfg_base_name=$2
rectangle_sampler_config_file_base=$3
traj_indicator=$4

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
semantic_points_by_node_prefix=semantic_points_by_node_
csv_suffix=.csv
traj_est_base_prefix=traj_est_

shape_dim_base_name=shape_dim_by_semantic_class_car_camry${csv_suffix}
detection_sensor_rel_baselink_file_base=detection_sensor_rel_baselink${csv_suffix}
semantic_index_to_string_file_base=semantic_index_to_string${csv_suffix}

# Intermediate variables
first_traj_eval_namespace=evaluation_first_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
second_traj_eval_namespace=evaluation_second_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
third_traj_eval_namespace=evaluation_third_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
fourth_traj_eval_namespace=evaluation_fourth_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
fifth_traj_eval_namespace=evaluation_fifth_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
sixth_traj_eval_namespace=evaluation_sixth_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
seventh_traj_eval_namespace=evaluation_seventh_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}
eighth_traj_eval_namespace=evaluation_eighth_traj_${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}

clustering_config_base_prefix=config_${clustering_config_file_base}_

# Set variables
# TODO consider making the config for the distribution a param
detection_sensor_rel_baselink_file_name=${semantic_point_config_dir}${detection_sensor_rel_baselink_file_base}
semantic_index_to_string_file_name=${semantic_point_config_dir}${semantic_index_to_string_file_base}

traj_out_common_prefix=${output_dir}${traj_est_base_prefix}${runtime_cfg_base_name}_${clustering_config_file_base}_${rectangle_sampler_config_file_base}_

# These should line up with preprocess_parking_lot_data_for_seq
odom_traj_est_common_prefix=${semantic_point_generated_dir}${odom_out_prefix}
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
  rosparam set /${curr_namespace}/detection_sensor_rel_baselink_file "${detection_sensor_rel_baselink_file_name}"
  rosparam set /${curr_namespace}/semantic_index_to_string_file "${semantic_index_to_string_file_name}"

  # Trajectory dependent
  rosparam set /${curr_namespace}/odom_traj_est_file "${odom_traj_est_file_name}"
  rosparam set /${curr_namespace}/semantic_point_det_curr_traj_file "${semantic_point_det_curr_traj_file_name}"
  rosparam set /${curr_namespace}/traj_est_output_file "${traj_est_output_file_for_run}"
done

plot_points_namespace=""
make
echo Estimating trajectories
if [ ${traj_indicator} = "first" ] ; then
  plot_points_namespace=${first_traj_eval_namespace}
elif [ ${traj_indicator} = "second" ] ; then
  plot_points_namespace=${second_traj_eval_namespace}
elif [ ${traj_indicator} = "third" ] ; then
  plot_points_namespace=${third_traj_eval_namespace}
elif [ ${traj_indicator} = "fourth" ] ; then
  plot_points_namespace=${fourth_traj_eval_namespace}
elif [ ${traj_indicator} = "fifth" ] ; then
  plot_points_namespace=${fifth_traj_eval_namespace}
elif [ ${traj_indicator} = "sixth" ] ; then
  plot_points_namespace=${sixth_traj_eval_namespace}
elif [ ${traj_indicator} = "seventh" ] ; then
  plot_points_namespace=${seventh_traj_eval_namespace}
elif [ ${traj_indicator} = "eighth" ] ; then
  plot_points_namespace=${eighth_traj_eval_namespace}
else
  exit
fi

echo Running point plotter
./bin/plot_single_traj_odom_est_and_semantic_points --param_prefix ${plot_points_namespace}

