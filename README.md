
Published/presented at IROS 2022. 

Links: [[paper](https://arxiv.org/abs/2110.00128)] [[video](https://youtu.be/P4VFh5kD_CE)] [[github](https://github.com/ut-amrl/pom_localization/tree/semanticPoints)]

Authors: Amanda Adkins, Taijing Chen, [Joydeep Biswas](https://www.joydeepb.com)

Can we learn and leverage the patterns in where movable objects occur to improve localization robustness?

Robots deployed in settings such as warehouses and parking lots must cope with frequent and substantial changes when localizing in their environments. While many previous localization and mapping algorithms have explored methods of identifying and focusing on long-term features to handle change in such environments, we propose a different approach – can a robot understand the *distribution* of movable objects and relate it to observations of such objects to reason about global localization? In this paper, we present probabilistic object maps (POMs), which represent the distributions of movable objects using pose-likelihood sample pairs derived from prior trajectories through the environment and use a Gaussian process classifier to generate the likelihood of an object at a query pose. We also introduce POM-Localization, which uses an observation model based on POMs to perform inference on a factor graph for globally consistent long-term localization. We present empirical results showing that POM-Localization is indeed effective at producing globally consistent localization estimates in challenging real-world environments, and that POM-Localization improves trajectory estimates even when the POM is formed from partially incorrect data.

Checkout the [arXiv entry](https://arxiv.org/abs/2110.00128) and our (shorter) [results](https://youtu.be/0HoudDB_9UM) and (longer) [explainer](https://youtu.be/P4VFh5kD_CE) videos.



## Offline Evaluation

To run POM-Localization offline, 

1. Create a runtime config. You can use the runtime_params_config_writer. 
2. Set the ROS parameters. 

`rosparam set /<optional prefix>/obj_det_curr_traj_file "<Name of the file containing the object detections by the node id>"`  
`rosparam set /<optional_prefix>/odom_traj_est_file "<Name for the estimated trajectory coming from odometry. Should include a first node at the origin>"`  
`rosparam set /<optional_prefix>/past_samples_files "['<Name for the past samples file, multiple can be listed if multiple exist>']"`  
`rosparam set /<optional_prefix>/runtime_params_config_file <Name for the runtime params config file>`  
`rosparam set /<optional_prefix>/traj_est_output_file_prefix <Path plus file prefix for the output trajectory, actual output will have the runtime config file name appended to the end>`  

Also optionally add the ground truth trajectory if it is available:  
`rosparam set /<optional_prefix>/gt_trajectory_file "<Name for the ground truth trajectory. Should include a first node at the origin>"`  

The formats for each of these files are specified in the following reader/writer files:
- obj_det_curr_traj_file: object_positions_by_pose_io.h. Currently, the evaluation script hard-codes all observations to have semantic class "car" since the file does not specify the semantic class
- odom_traj_est_file: trajectory_2d_io.h
- past_samples_files: past_sample_io.h
- runtime_params_config_file: runtime_params_config_io.h
- traj_est_output_file_prefix: The output file will have the same format as the odom and gt trajectory files (specified by trajectory_2d_io.h). 
- gt_trajectory_file: trajectory_2d_io.h

3. Run the executable:
`./bin/evaluation_main `    
Optional arguments:  
--param_prefix \<prefix for the node\> should match the prefix used for the ROS params  
--run_gpc_viz: Use this flag to run the GPC visualization  
--skip_optimization: Use this flag to just run the initial visualization  

4. Optionally visualize using RViz
Relevant topics are (will be preceded with prefix if provided)
- \<semantic_class\>_past_sample_markers - Displays samples making up the POM for the given semantic class. High valued samples are red and low valued samples are blue and values in between integrate corresponding amounts of blue and red.
- est_pos_marker - Displays the trajectory as estimated by POM-Localization in red
- odom_pos_marker - Displays the trajectory estimate from odometry only in blue
- gt_visualization_marker - If the gt_trajectory_file is specified, displays the ground truth trajectory estimate in green
- est_\<semantic_class\>_obs_marker - Displays the given object observations for the specified semantic class from the estimated trajectory. This are shown in orange.
- odom_\<semantic_class\>_obs_marker - Displays the given object observations for the specified semantic class from the odometry-only trajectory. These are shown in purple
- gt_\<semantic_class\>_obs_marker - Displays the given object observations for the specified semantic class from the ground truth trajectory (if the ground truth trajectory is provided). These are shown in teal.
- classifier_max_val_for_pos - Displays the POM output evaluated on a discrete grid. Each grid cell displays the highest value for the 12 orientations evaluated. This and the other POM output visualizations are currently hard-coded to display for a specific range and only for car classes. Additional modification is needed for different semantic classes and different areas. This is only generated if the run_gpc_viz is included
- regressor_max_val_for_pos - Displays the POM output from the regressor only (doesn't take into account uncertainty) evaluated on a discrete grid. Each grid cell displays the highest value for the 12 orientations evaluated. This and the other POM output visualizations are currently hard-coded to display for a specific range and only for car classes. Additional modification is needed for different semantic classes and different areas. This is only generated if the run_gpc_viz is included
- variance_max_val_for_pos - Displays the variance output by the POM. The variance displayed is the variance for the highest classifier output on the 12 orientations evaluated.  This and the other POM output visualizations are currently hard-coded to display for a specific range and only for car classes. Additional modification is needed for different semantic classes and different areas. This is only generated if the run_gpc_viz is included


