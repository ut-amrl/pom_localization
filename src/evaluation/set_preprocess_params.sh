#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
generated_dir=${base_dir}generated/
orig_data_dir=${base_dir}original_data/
runtime_cfgs_dir=${base_dir}runtime_configs/

bag_time_string=$1
bag_traj_identifer=$2

namespace=setup_lot53_${bag_traj_identifer}_traj

nodes_and_timestamps_file_prefix=nodes_and_timestamps_
obj_det_timestamp_prefix=obj_detections_
obj_det_node_prefix=obj_detections_by_node_
obj_yaml_prefix=labeling_
odom_out_prefix=odom_est_
waypoints_by_node_prefix=waypoints_by_node_
waypoints_by_timestamp_prefix=waypoints_by_timestamp_
csv_suffix=.csv
yaml_suffix=.yaml
bag_suffix=.bag

rosparam set /${namespace}/node_id_and_timestamp_file "${generated_dir}${nodes_and_timestamps_file_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/node_id_and_timestamp_out_file "${generated_dir}${nodes_and_timestamps_file_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/obj_output_file_name "${generated_dir}${obj_det_timestamp_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/object_detection_file "${generated_dir}${obj_det_timestamp_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/object_detections_by_node_file "${generated_dir}${obj_det_node_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/object_detections_by_timestamp_file "${generated_dir}${obj_det_timestamp_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/object_file_yaml "${orig_data_dir}${obj_yaml_prefix}${bag_time_string}${yaml_suffix}"
rosparam set /${namespace}/odom_out_file_name "${generated_dir}${odom_out_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/rosbag_file_name "${orig_data_dir}${bag_time_string}${bag_suffix}"
rosparam set /${namespace}/waypoints_by_node_file "${generated_dir}${waypoints_by_node_prefix}${bag_time_string}${csv_suffix}"
rosparam set /${namespace}/waypoints_by_timestamps_file "${orig_data_dir}${waypoints_by_timestamp_prefix}${bag_time_string}${csv_suffix}"

make && ./bin/convert_object_detections_from_yaml --param_prefix ${namespace}
./bin/wheel_odom_extractor --param_prefix ${namespace}
./bin/find_node_ids_for_objs_with_timestamps --param_prefix ${namespace}
