#!/bin/bash

# Stop the script if a command fails
set -e

# For running on robodata
#input_dir=/home/aaadkins/data/momo_kitti/input_data/
#output_dir=/home/aaadkins/data/momo_kitti/output_data/
#semantic_points_dir=${input_dir}
# cd ~/workspaces/autodiff_gp

# For running locally
input_dir=$HOME/datasets/kitti_odometry/bags/
output_dir=$HOME/datasets/kitti_odometry/bags/output/
semantic_points_dir=$HOME/datasets/kitti_odometry/semantic_points/
cd ~/workspaces/momo-slam/audodiff_gp/autodiff_gp/

make


runtime_params_config_file_base=$1
rectangle_sampler_config_file_base=$2
sequence_num=$3

runtime_params_config_file_base_name=${runtime_params_config_file_base}.txt
rectangle_sampler_config_file_base_name=${rectangle_sampler_config_file_base}.csv

# Common to all
out_file_suffix=${runtime_params_config_file_base}_${rectangle_sampler_config_file_base}_${sequence_num}.csv
odom_traj_est_file_name=${input_dir}lego_loam_2d_traj_est_${sequence_num}.txt
gt_trajectory_file_name=${input_dir}planar_approx_poses_${sequence_num}.txt
runtime_params_config_file_name=${input_dir}${runtime_params_config_file_base_name}
detection_sensor_rel_baselink_file_name=${input_dir}detection_sensor_rel_baselink.csv
semantic_point_object_sampler_config_file_name=${input_dir}${rectangle_sampler_config_file_base_name}
shape_dimensions_by_semantic_class_file_name=${input_dir}shape_dim_by_semantic_class_car_camry.csv
semantic_index_to_string_file_name=${input_dir}semantic_index_to_string.csv
semantic_point_det_curr_traj_file_name=${semantic_points_dir}semantic_points_cars_seq_${sequence_num}.csv

str_0_100=0_100_
str_20_80=20_80_
str_50_50=50_50_
str_80_20=80_20_
str_100plus_0=100plus_0_

out_file_base_0_100=traj_est_out_${str_0_100}${out_file_suffix}
out_file_base_20_80=traj_est_out_${str_20_80}${out_file_suffix}
out_file_base_50_50=traj_est_out_${str_50_50}${out_file_suffix}
out_file_base_80_20=traj_est_out_${str_80_20}${out_file_suffix}
out_file_base_100plus_0=traj_est_out_${str_100plus_0}${out_file_suffix}

ros_prefix_0_100=kitti_eval_${str_0_100}prefix_${sequence_num}
ros_prefix_20_80=kitti_eval_${str_20_80}prefix_${sequence_num}
ros_prefix_50_50=kitti_eval_${str_50_50}prefix_${sequence_num}
ros_prefix_80_20=kitti_eval_${str_80_20}prefix_${sequence_num}
ros_prefix_100plus_0=kitti_eval_${str_100plus_0}prefix_${sequence_num}


#ros_prefix_0_100=kitti_eval_${str_0_100}prefix_${sequence_num}
#ros_prefix_20_80=kitti_eval_${str_20_80}prefix_${sequence_num}
#ros_prefix_50_50=kitti_eval_${str_50_50}prefix_${sequence_num}
#ros_prefix_80_20=kitti_eval_${str_80_20}prefix_${sequence_num}
#ros_prefix_100plus_0=kitti_eval_${str_100plus_0}prefix_${sequence_num}

past_sample_file_0_100=${input_dir}samples_off_gt_${sequence_num}_kitti_off_gt_objs_cfg.txt
past_sample_file_20_80=${input_dir}samples_${sequence_num}_kitti_sample_gen_cfg_20_80_c0e2c80.txt
past_sample_file_50_50=${input_dir}samples_${sequence_num}_kitti_sample_gen_cfg_50_50_480cea2d.txt
past_sample_file_80_20=${input_dir}samples_${sequence_num}_kitti_some_slightly_off_some_random_cfg.txt
past_sample_file_100plus_0=${input_dir}samples_${sequence_num}_kitti_gt_objs_cfg.txt

namespaces=(${ros_prefix_0_100} ${ros_prefix_20_80} ${ros_prefix_50_50} ${ros_prefix_80_20} ${ros_prefix_100plus_0})
past_samples_file_base_names=(${past_sample_file_0_100} ${past_sample_file_20_80} ${past_sample_file_50_50} ${past_sample_file_80_20} ${past_sample_file_100plus_0})
traj_out_file_bases=(${out_file_base_0_100} ${out_file_base_20_80} ${out_file_base_50_50} ${out_file_base_80_20} ${out_file_base_100plus_0})

# Set ROS params
for i in ${!namespaces[@]}; do
  curr_namespace=${namespaces[$i]}
  past_samples_file=${past_samples_file_base_names[$i]}
  traj_out_file_base=${traj_out_file_bases[$i]}
  echo "Setting params for namespace ${curr_namespace}: past_samples_file is ${past_samples_file}; traj out is ${traj_out_file_base}"

  rosparam set /${curr_namespace}/past_samples_files "['${past_samples_file}']"
  rosparam set /${curr_namespace}/odom_traj_est_file "${odom_traj_est_file_name}"
  rosparam set /${curr_namespace}/semantic_point_det_curr_traj_file "${semantic_point_det_curr_traj_file_name}"
  rosparam set /${curr_namespace}/traj_est_output_file_prefix "${output_dir}${traj_out_file_base}"
  rosparam set /${curr_namespace}/gt_trajectory_file "${gt_trajectory_file_name}"
  rosparam set /${curr_namespace}/runtime_params_config_file "${runtime_params_config_file_name}"
  rosparam set /${curr_namespace}/detection_sensor_rel_baselink_file "${detection_sensor_rel_baselink_file_name}"
  rosparam set /${curr_namespace}/semantic_index_to_string_file "${semantic_index_to_string_file_name}"
  rosparam set /${curr_namespace}/shape_dimensions_by_semantic_class_file "${shape_dimensions_by_semantic_class_file_name}"
  rosparam set /${curr_namespace}/semantic_point_object_sampler_config_file "${semantic_point_object_sampler_config_file_name}"
done


echo Estimating trajectories
#./bin/semantic_point_evaluation_main --param_prefix ${ros_prefix_0_100} & # TODO pipe output to file
#./bin/semantic_point_evaluation_main --param_prefix ${ros_prefix_20_80} & # TODO pipe output to file
#./bin/semantic_point_evaluation_main --param_prefix ${ros_prefix_50_50} & # TODO pipe output to file
#./bin/semantic_point_evaluation_main --param_prefix ${ros_prefix_80_20} & # TODO pipe output to file
./bin/semantic_point_evaluation_main --param_prefix ${ros_prefix_100plus_0} & # TODO pipe output to file
wait

results_computation_namespace=compute_results_${sequence_num}

# Compute results
rosparam set /${results_computation_namespace}/odom_traj_est_file "${odom_traj_est_file_name}"
rosparam set /gt_trajectory_file "${gt_trajectory_file_name}"

# Results computed by this approach under different samples
rosparam set /${results_computation_namespace}/misaligned_est_traj_file "${output_dir}${out_file_base_0_100}"
rosparam set /${results_computation_namespace}/eighty_percent_misaligned_est_traj_file "${output_dir}${out_file_base_20_80}"
rosparam set /${results_computation_namespace}/fifty_percent_misaligned_est_traj_file "${output_dir}${out_file_base_50_50}"
rosparam set /${results_computation_namespace}/twenty_percent_misaligned_est_traj_file "${output_dir}${out_file_base_80_20}"
rosparam set /${results_computation_namespace}/aligned_est_traj_file "${output_dir}${out_file_base_100plus_0}"

#make && ./bin/compute_kitti_results --run_viz --param_prefix ${results_computation_namespace}

