#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
generated_dir=${base_dir}generated/
orig_data_dir=${base_dir}original_data/
runtime_cfgs_dir=${base_dir}runtime_configs/
clustering_config_dir=${base_dir}semantic_point_config/

clustering_config_file_base=$1
bag_time_string=$2
bag_traj_identifer=$3

namespace=setup_lot53_${bag_traj_identifer}_traj

nodes_and_timestamps_file_prefix=nodes_and_timestamps_
odom_out_prefix=odom_est_
waypoints_by_node_prefix=waypoints_by_node_
waypoints_by_timestamp_prefix=waypoints_by_timestamp_
csv_suffix=.csv
bag_suffix=.bag

rosbag_file=${orig_data_dir}${bag_time_string}${bag_suffix}
odom_out_file_name=${generated_dir}${odom_out_prefix}${bag_time_string}${csv_suffix}
clustering_config_base_prefix=${clustering_config_file_base}_

clustering_config_file_name=${clustering_config_dir}${clustering_config_file_base}.csv

semantic_points_base_file_prefix=semantic_points_config_${clustering_config_base_prefix}${bag_time_string}_
semantic_points_by_timestamp_file=${generated_dir}${semantic_points_base_file_prefix}by_timestamp${csv_suffix}
semantic_points_by_node_file=${generated_dir}${semantic_points_base_file_prefix}by_node${csv_suffix}
node_id_and_timestamp_file_name=${generated_dir}${nodes_and_timestamps_file_prefix}${clustering_config_base_prefix}${bag_time_string}${csv_suffix}
waypoints_by_timestamps_file_name=${orig_data_dir}${waypoints_by_timestamp_prefix}${bag_time_string}${csv_suffix}
waypoints_by_node_file_name=${generated_dir}${waypoints_by_node_prefix}${clustering_config_base_prefix}${bag_time_string}${csv_suffix}

rosparam set /${namespace}/rosbag_file_name "${rosbag_file}"
rosparam set /${namespace}/odom_out_file_name "${odom_out_file_name}"
rosparam set /${namespace}/semantic_point_detections_by_timestamp_file "${semantic_points_by_timestamp_file}"
rosparam set /${namespace}/semantic_point_detections_by_node_file "${semantic_points_by_node_file}"
rosparam set /${namespace}/node_id_and_timestamp_file "${node_id_and_timestamp_file_name}"
rosparam set /${namespace}/waypoints_by_timestamps_file "${waypoints_by_timestamps_file_name}"
rosparam set /${namespace}/waypoints_by_node_file "${waypoints_by_node_file_name}"

make
./bin/point_cloud_labeler_from_cluster_heuristics --param_prefix ${namespace} --bag_file_name ${rosbag_file} --semantic_points_file ${semantic_points_by_timestamp_file} --cluster_config_file ${clustering_config_file_name}
./bin/wheel_odom_rosbag_extraction_semantic_points --param_prefix ${namespace}
./bin/find_node_ids_for_semantic_points_and_waypoints_with_timestamps --param_prefix ${namespace}
