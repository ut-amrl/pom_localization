#!/bin/bash

base_dir=$HOME/rosbags/momo_eval/lot53_bags/
runtime_cfgs_dir=${base_dir}runtime_configs/

run_gpc_viz_arg="--norun_gpc_viz"

namespace=past_sample_plotter
csv_suffix=.csv

past_samples_file=${base_dir}samples_distribution_config_narrower_yet_5_traj.txt

if [ $# -eq 1 ];
then
  echo "Running GPC viz"
  run_gpc_viz_arg="--run_gpc_viz"
  runtime_cfg_base_name=$1
  namespace=${namespace}_${runtime_cfg_base_name}

  runtime_params_config_file_name=${runtime_cfgs_dir}${runtime_cfg_base_name}${csv_suffix}
  rosparam set /${namespace}/runtime_params_config_file "${runtime_params_config_file_name}"
fi

rosparam set /${namespace}/past_samples_files "['${past_samples_file}']"

make
./bin/past_sample_publisher --param_prefix ${namespace} ${run_gpc_viz_arg}

