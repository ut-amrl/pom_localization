#!/bin/bash

clustering_config_file_base=$1

first_traj_date=2021-05-29-15-28-32
second_traj_date=2021-06-01-14-59-33
third_traj_date=2021-06-01-17-31-45
fourth_traj_date=2021-08-03-16-44-20
fifth_traj_date=2021-08-04-08-11-08
sixth_traj_date=2021-08-04-16-08-36
seventh_traj_date=2021-08-04-15-44-03
eighth_traj_date=2021-08-04-11-27-59

#bag_time_strings=(${first_traj_date} ${second_traj_date} ${third_traj_date} ${fourth_traj_date} ${fifth_traj_date} ${sixth_traj_date} ${seventh_traj_date} ${eighth_traj_date})
#bag_traj_identifers=("first" "second" "third" "fourth" "fifth" "sixth" "seventh" "eighth")

bag_time_strings=(${second_traj_date} ${third_traj_date} ${fourth_traj_date} ${fifth_traj_date} ${sixth_traj_date} ${seventh_traj_date} ${eighth_traj_date})
bag_traj_identifers=("second" "third" "fourth" "fifth" "sixth" "seventh" "eighth")


for i in ${!bag_time_strings[@]}; do
  bag_time_string=${bag_time_strings[$i]}
  bag_traj_identifer=${bag_traj_identifers[$i]}
  echo "Bag time string"
  echo ${bag_time_string}
  echo "Bag traj identifier"
  echo ${bag_traj_identifer}
  ./src/semantic_point_based_pom/convenience_scripts/parking_lot/preprocess_parking_lot_data_for_seq.sh ${clustering_config_file_base} ${bag_time_string} ${bag_traj_identifer} &
done

wait
echo Done with all trajectories
