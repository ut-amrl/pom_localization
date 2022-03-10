#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
original_data_dir=${base_dir}original_data/
carto_dir=${base_dir}cartographer/

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-16-08-36
seventh_traj_date=2021-08-04-15-44-03
eighth_traj_date=2021-08-04-11-27-59
ninth_traj_date=2021-08-03-12-26-27

process_carto_traj() {
  date_str=$1
  rosparam set /trajectory_estimate_2d ${carto_dir}2dscan-${date_str}.csv
  rosparam set /odom_out_file_name ${carto_dir}trajectory_est_with_waypoints_${date_str}.csv
  rosparam set /node_id_and_timestamp_file ${carto_dir}node_id_and_timestamps_${date_str}.csv
  rosparam set /waypoints_by_timestamps_file ${original_data_dir}waypoints_by_timestamp_${date_str}.csv
  rosparam set /bag_file_name ${original_data_dir}${date_str}.bag
  rosparam set /waypoints_by_node_file ${carto_dir}waypoints_by_node_${date_str}.csv
  make && ./bin/interpolate_waypoints
  make && ./bin/find_node_ids_for_objs_with_timestamps
}

process_carto_traj ${first_traj_date}
process_carto_traj ${second_traj_date}
process_carto_traj ${third_traj_date}
process_carto_traj ${fourth_traj_date}
process_carto_traj ${fifth_traj_date}
process_carto_traj ${sixth_traj_date}
process_carto_traj ${seventh_traj_date}
process_carto_traj ${eighth_traj_date}
#process_carto_traj ${ninth_traj_date}