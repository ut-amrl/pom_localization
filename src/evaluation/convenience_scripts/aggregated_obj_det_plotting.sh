#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
#output_dir=${base_dir}output/backup_drive/
output_dir=${base_dir}output/
generated_dir=${base_dir}generated/

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-16-08-36
seventh_traj_date=2021-08-04-15-44-03
eighth_traj_date=2021-08-04-11-27-59

bag_time_strings=(${first_traj_date} ${second_traj_date} ${third_traj_date} ${fourth_traj_date} ${fifth_traj_date} ${sixth_traj_date} ${seventh_traj_date} ${eighth_traj_date})
bag_traj_identifers=("first" "second" "third" "fourth" "fifth" "sixth" "seventh" "eighth")
runtime_cfg_base_name=$1

plot_obj_det_namespace=obj_det_${runtime_cfg_base_name}

namespaces=(${ros_prefix_0_100} ${ros_prefix_20_80} ${ros_prefix_50_50} ${ros_prefix_80_20} ${ros_prefix_100plus_0})
past_samples_file_base_names=(${past_sample_file_0_100} ${past_sample_file_20_80} ${past_sample_file_50_50} ${past_sample_file_80_20} ${past_sample_file_100plus_0})
traj_out_file_bases=(${out_file_base_0_100} ${out_file_base_20_80} ${out_file_base_50_50} ${out_file_base_80_20} ${out_file_base_100plus_0})

max_bags=0
for i in ${!bag_time_strings[@]}; do
  echo "i: ${i}"
  bag_time_string=${bag_time_strings[$i]}
  bag_traj_identifer=${bag_traj_identifers[$i]}
  echo "Bag time string"
  echo ${bag_time_string}
  echo "Bag traj identifier"
  echo ${bag_traj_identifer}
  max_bags=${i}

  rosparam set /${plot_obj_det_namespace}/trajectory_${i}/trajectory_output_file "${output_dir}momo_${bag_traj_identifer}_traj_est_${runtime_cfg_base_name}.csv"
  rosparam set /${plot_obj_det_namespace}/trajectory_${i}/object_detections "${generated_dir}obj_detections_by_node_${bag_time_string}.csv"
done

let max_bags=${max_bags}+1
echo "Max bags ${max_bags}"
rosparam set /${plot_obj_det_namespace}/num_trajectories ${max_bags}

make && ./bin/plot_optimized_with_detections --param_prefix ${plot_obj_det_namespace}


