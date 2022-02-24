#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <ctime>

#include <pose_optimization/offline/offline_problem_runner.h>
#include <pose_optimization/utils/pose_graph_creation_utils.h>
#include <pose_optimization/offline/ceres_visualization_callback_semantic_points_2d.h>
#include <semantic_point_based_pom/rectangle_sampler.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/past_sample_io.h>
#include <file_io/runtime_params_config_io.h>
#include <file_io/semantic_point_with_node_id_io.h>
#include <file_io/pose_3d_io.h>
#include <file_io/semantic_index_to_string_map_io.h>
#include <file_io/semantic_point_object_sampler_config_io.h>
#include <file_io/shape_dimensions_2d_by_semantic_class_io.h>
#include <unordered_map>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_bool(run_gpc_viz, false, "Run GPC viz");
DEFINE_bool(skip_optimization, false, "Skip optimization");
DEFINE_bool(debug_samples, false, "Debug samples");

const std::string kPastSamplesFilesParamName = "past_samples_files";

const std::string kOdomTrajectoryEstimatesFileParamName = "odom_traj_est_file";

const std::string kSemanticPointDetectionsCurrTrajectoryFileParamName = "semantic_point_det_curr_traj_file";

const std::string kTrajectoryOutputFileParamName = "traj_est_output_file";

const std::string kGtTrajectoryFile = "gt_trajectory_file";

const std::string kRuntimeParamsConfigParamName = "runtime_params_config_file";

const std::string kDetectionSensorRelBaseLinkFileParamName = "detection_sensor_rel_baselink_file";

const std::string kSemanticIndexToStringClassFileParamName = "semantic_index_to_string_file";

const std::string kShapeDimensionsBySemanticClassFileParamName = "shape_dimensions_by_semantic_class_file";

const std::string kSemanticPointObjectSamplerConfigFileParamName = "semantic_point_object_sampler_config_file";

//const double kMinOdomVar = pow(std::numeric_limits<double>::min(), 0.0003);
const double kMinOdomVar = pow(std::numeric_limits<double>::min(), 0.3);

pose_optimization::PoseOptimizationParameters setupPoseOptimizationParams(const file_io::RuntimeParamsConfig &config) {
    pose_optimization::PoseOptimizationParameters pose_opt_params;
    pose_optimization::CostFunctionParameters cost_function_params;

    cost_function_params.full_optimization_interval_ = config.full_optimization_interval_;
    cost_function_params.num_nodes_in_optimization_window_ = config.num_nodes_in_optimization_window_;
    cost_function_params.max_gpc_samples_ = config.max_gpc_samples_;


    cost_function_params.mean_position_kernel_len_ = config.mean_position_kernel_len_;
    cost_function_params.mean_orientation_kernel_len_ = config.mean_orientation_kernel_len_;

    cost_function_params.mean_position_kernel_var_ = config.mean_position_kernel_var_;
    cost_function_params.mean_orientation_kernel_var_ = config.mean_orientation_kernel_var_;

    cost_function_params.default_obj_probability_input_variance_for_mean_ =
            config.default_obj_probability_input_variance_for_mean_;

    cost_function_params.var_position_kernel_len_ = config.var_position_kernel_len_;
    cost_function_params.var_orientation_kernel_len_ = config.var_orientation_kernel_len_;

    cost_function_params.var_position_kernel_var_ = config.var_position_kernel_var_;
    cost_function_params.var_orientation_kernel_var_ = config.var_orientation_kernel_var_;

    cost_function_params.default_obj_probability_input_variance_for_var_ =
            config.default_obj_probability_input_variance_for_var_;

    pose_opt_params.cost_function_params_ = cost_function_params;
    return pose_opt_params;
}

std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> createGpc(
        const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                pose_graph::MovableObservationSemanticPoints2d>> &pose_graph,
        const std::string &semantic_class,
        const uint64_t &max_samples,
        const double &radius,
        const Eigen::Vector2d &search_point) {
    return pose_graph->getMovableObjGpcWithinRadius(semantic_class, radius, search_point, max_samples);
}

void runOptimizationVisualization(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                  const pose_graph::NodeId &max_node_id,
                                  const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3,
                                          2, double, 3, pose_graph::MovableObservationSemanticPoints2d>> &pose_graph,
                                  const offline_optimization::VisualizationTypeEnum &vis_stage,
                                  const std::vector<pose::Pose2d> &ground_truth_trajectory,
                                  const std::vector<pose::Pose2d> &unoptimized_trajectory,
                                  const std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>>
                                  &noisy_obj_observations_by_class,
                                  const std::unordered_map<std::string, std::unordered_map<pose_graph::NodeId, std::unordered_map<size_t, std::vector<pose::Pose2d>>>> &rectangle_samples,
                                  const std::unordered_map<std::string, Eigen::Vector2d> &shape_dimensions_by_class) {
    // TODO make this more generic (not specifically car class)
    switch (vis_stage) {
        case offline_optimization::VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION:


            for (const auto &obs_with_class : noisy_obj_observations_by_class) {
                std::string semantic_class = obs_with_class.first;
                std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> noisy_obj_observations = obs_with_class.second;
                if (!ground_truth_trajectory.empty()) {
                    vis_manager->displayTrueTrajectory(ground_truth_trajectory);
                    vis_manager->displaySemanticPointObsFromGtTrajectory(ground_truth_trajectory,
                                                                         noisy_obj_observations,
                                                                         semantic_class);
                }
                {


                    if (FLAGS_run_gpc_viz) {
                        std::vector<Eigen::Vector2d> poses_global_frame;
                        for (size_t node = 0; node < unoptimized_trajectory.size(); node++) {
                            pose::Pose2d robot_pose = unoptimized_trajectory[node];
                            for (const auto &semantic_detection_points : noisy_obj_observations[node]) {
                                for (const Eigen::Vector2d &semantic_point : semantic_detection_points.second) {
                                    poses_global_frame.emplace_back(pose::transformPoint(robot_pose, semantic_point));
                                }
                            }
                        }
                        std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
                                visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(
                                        poses_global_frame);
                        std::function<std::shared_ptr<gp_regression::GaussianProcessClassifier<3,
                                gp_kernel::Pose2dKernel>>(const Eigen::Vector2d &)> gpc_creator =
                                std::bind(createGpc, pose_graph, "car",
                                          500,
                                          8.5,
                                          std::placeholders::_1);
                        vis_manager->displayMaxGpRegressorOutput(
                                gpc_creator,
                                1,
                                min_max_points_to_display.first.x(),
                                min_max_points_to_display.second.x(),
                                min_max_points_to_display.first.y(),
                                min_max_points_to_display.second.y()
                        );
                    }
                }

                if (FLAGS_skip_optimization) {
                    exit(0);
                }
            }

            // Display true trajectory
            // Display true object poses
            // Display noisy car poses from GT
            // Display initial trajectory
            // Display initial car poses from initial trajectory
            break;
        case offline_optimization::VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION: {
            std::vector<pose::Pose2d> limited_odom_poses_list_extended;
            for (pose_graph::NodeId node_id = 0; node_id <= max_node_id; node_id++) {
                limited_odom_poses_list_extended.emplace_back(unoptimized_trajectory[node_id]);
            }

            vis_manager->displayOdomTrajectory(limited_odom_poses_list_extended);
            for (const auto &obs_with_class : noisy_obj_observations_by_class) {
                std::string semantic_class = obs_with_class.first;
                std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> noisy_obj_observations = obs_with_class.second;
                vis_manager->displaySemanticPointObsFromOdomTrajectory(limited_odom_poses_list_extended,
                                                                       noisy_obj_observations, semantic_class);
            }
        }
            break;
        case offline_optimization::VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION: {
            std::vector<pose::Pose2d> node_poses_list;
            std::unordered_map<pose_graph::NodeId, pose::Pose2d> node_poses;
            pose_graph->getNodePoses(node_poses);
            std::vector<pose::Pose2d> limited_odom_poses_list;
            for (pose_graph::NodeId node_id = 0; node_id <= max_node_id; node_id++) {
                node_poses_list.emplace_back(node_poses[node_id]);
                limited_odom_poses_list.emplace_back(unoptimized_trajectory[node_id]);
            }
            vis_manager->displayOdomTrajectory(limited_odom_poses_list);
            vis_manager->displayEstTrajectory(node_poses_list);
            vis_manager->displayTrueTrajectory(ground_truth_trajectory);

            for (const auto &obs_with_class : noisy_obj_observations_by_class) {
                std::string semantic_class = obs_with_class.first;
                std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> noisy_obj_observations = obs_with_class.second;
                std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> rectangle_samples_for_class;
                if (rectangle_samples.find(semantic_class) != rectangle_samples.end()) {
                    rectangle_samples_for_class = rectangle_samples.at(semantic_class);
                }
                vis_manager->displaySemanticPointObsFromEstTrajectory(node_poses_list, noisy_obj_observations,
                                                                      semantic_class,
                                                                      rectangle_samples_for_class,
                                                                      shape_dimensions_by_class.at(semantic_class));
                if (!ground_truth_trajectory.empty()) {
                    vis_manager->displaySemanticPointObsFromGtTrajectory(ground_truth_trajectory,
                                                                         noisy_obj_observations,
                                                                         semantic_class);

                }
                vis_manager->displaySemanticPointObsFromOdomTrajectory(limited_odom_poses_list,
                                                                       noisy_obj_observations, semantic_class);
            }
        }
            break;
        case offline_optimization::VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION:
            // Optionally display distribution intensity map (either over robot poses or over object poses)
            break;
        default:
            break;
    }
}

std::vector<pose_graph::MovableObservationSemanticPointsFactor2d> getMovableObservationFactorsFromDetectionsFile(
        const std::string &semantic_point_detections_file_name,
        const int &pose_sample_ratio,
        const std::string &semantic_index_to_string_map_file) {

    std::unordered_map<unsigned short, std::string> semantic_index_to_string_map;
    file_io::readSemanticIndexToStringMapFromFile(semantic_index_to_string_map_file, semantic_index_to_string_map);

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo> raw_obj_detections;
    LOG(INFO) << "Reading from file " << semantic_point_detections_file_name;
    semantic_segmentation::readSemanticallyLabeledPointWithNodeIdInfoFromFile(semantic_point_detections_file_name,
                                                                              raw_obj_detections);

    std::vector<pose_graph::MovableObservationSemanticPointsFactor2d> observation_factors;
    // Node id is outer key, cluster id is inner key
    std::unordered_map<uint64_t, std::unordered_map<uint32_t,
            std::vector<semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo>>> points_by_node_and_cluster;
    for (const semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo &semantic_points : raw_obj_detections) {
        if ((semantic_points.node_id % pose_sample_ratio) == 0) {
            points_by_node_and_cluster[semantic_points.node_id][semantic_points.cluster_label].emplace_back(
                    semantic_points);
        }
    }

    for (const auto &points_for_node : points_by_node_and_cluster) {
        uint64_t node_id = points_for_node.first;
        for (const auto &points_for_cluster : points_for_node.second) {
            unsigned short semantic_index = points_for_cluster.second.front().semantic_label;
            if (semantic_index_to_string_map.find(semantic_index) == semantic_index_to_string_map.end()) {
                LOG(WARNING) << "Skipping observations with cluster id " << points_for_cluster.first << " and node id "
                             << node_id << " because the semantic index was not found in the index to class string map";
                continue;
            }
            std::string semantic_string = semantic_index_to_string_map.at(semantic_index);
            pose_graph::MovableObservationSemanticPoints2d observation;
            observation.semantic_class_ = semantic_string;
            observation.cluster_num_ = points_for_cluster.first;
            for (const semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo &semantic_point : points_for_cluster.second) {
                observation.object_points_.emplace_back(
                        Eigen::Vector2d(semantic_point.point_x, semantic_point.point_y));
            }
            observation_factors.emplace_back(
                    pose_graph::MovableObservationSemanticPointsFactor2d(node_id, observation));
        }
    }

    return observation_factors;
}

std::vector<pose::Pose2d> readTrajFromFile(const std::string &file_name) {
    std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
    file_io::readRawTrajectory2dFromFile(file_name, trajectory_nodes);

    // Assuming trajectory nodes are in order
    std::vector<pose::Pose2d> init_traj_poses;
    for (const file_io::TrajectoryNode2d traj_node : trajectory_nodes) {
        init_traj_poses.emplace_back(pose::createPose2d(traj_node.transl_x_, traj_node.transl_y_, traj_node.theta_));
    }
    return init_traj_poses;
}

std::vector<pose_graph::GaussianBinaryFactor2d>
createOdomFactorsFromInitOdomEst(const std::vector<pose::Pose2d> &init_traj_est,
                                 const double &k1,
                                 const double &k2,
                                 const double &k3,
                                 const double &k4,
                                 const double &k5,
                                 const double &k6) {

    std::vector<pose_graph::GaussianBinaryFactor2d> odom_factors;
    pose::Pose2d prev_pose = init_traj_est[0];
    for (size_t i = 1; i < init_traj_est.size(); i++) {
        pose::Pose2d curr_pose = init_traj_est[i];
        pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose);

        pose_graph::GaussianBinaryFactor2d odom_factor;
        odom_factor.from_node_ = i - 1;
        odom_factor.to_node_ = i;
        odom_factor.translation_change_ = rel_pose.first;
        odom_factor.orientation_change_ = rel_pose.second;
//        LOG(INFO) << "Orientation change " << odom_factor.orientation_change_;
        Eigen::Matrix<double, 3, 3> odom_cov_mat = Eigen::Matrix<double, 3, 3>::Zero();

        double norm = rel_pose.first.norm();

        odom_cov_mat(0, 0) = std::max(pow(k1 * norm + k2 * abs(rel_pose.second), 2), kMinOdomVar);
        odom_cov_mat(1, 1) = std::max(pow(k3 * norm + k4 * abs(rel_pose.second), 2), kMinOdomVar);
        odom_cov_mat(2, 2) = std::max(pow(k5 * norm + k6 * abs(rel_pose.second), 2), kMinOdomVar);


        Eigen::Matrix<double, 3, 3> odom_sqrt_information_mat = odom_cov_mat.inverse().sqrt();
        odom_factor.sqrt_information_ = odom_sqrt_information_mat;
        odom_factors.emplace_back(odom_factor);

        prev_pose = curr_pose;
    }
    return odom_factors;
}

std::unordered_map<pose_graph::NodeId, pose::Pose2d>
getTrajectoryEstimate(const std::vector<pose::Pose2d> &odom_est_trajectory,
                      const std::vector<pose::Pose2d> &gt_trajectory,
                      const std::shared_ptr<visualization::VisualizationManager> &manager,
                      const std::vector<pose_graph::GaussianBinaryFactor2d> &odom_factors,
                      const std::vector<pose_graph::MapObjectObservation2d> &samples,
                      const std::vector<pose_graph::MovableObservationSemanticPointsFactor2d> &movable_object_observations,
                      const file_io::RuntimeParamsConfig &runtime_params_config,
                      const pose::Pose2d &detections_sensor_rel_baselink,
                      const std::string &shape_dimensions_by_semantic_class_file,
                      const std::string &semantic_point_object_sampler_config_file,
                      util_random::Random &random_generator) {

    offline_optimization::OfflinePoseOptimizer<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
            pose_graph::MovableObservationSemanticPoints<2>> offline_optimizer;

    std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>> noisy_observations_by_class;
    for (const pose_graph::MovableObservationSemanticPointsFactor2d &movable_factor : movable_object_observations) {
        pose_graph::NodeId observed_at_node = movable_factor.observed_at_node_;
        std::string semantic_class = movable_factor.observation_.semantic_class_;

        std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> observations_for_class;
        if (noisy_observations_by_class.find(semantic_class) == noisy_observations_by_class.end()) {
            for (size_t node_num = 0; node_num < odom_est_trajectory.size(); node_num++) {
                observations_for_class.push_back({});
            }
        } else {
            observations_for_class = noisy_observations_by_class[semantic_class];
        }
        std::vector<Eigen::Vector2d> baselink_detection_points;
        for (const Eigen::Vector2d &sensor_detection_point : movable_factor.observation_.object_points_) {
            baselink_detection_points.emplace_back(
                    pose::transformPoint(detections_sensor_rel_baselink, sensor_detection_point));
        }
        observations_for_class[observed_at_node][movable_factor.observation_.cluster_num_] = baselink_detection_points;
        noisy_observations_by_class[semantic_class] = observations_for_class;
    }

    std::vector<pose_graph::Node<2, double>> initial_node_positions;
    for (pose_graph::NodeId i = 0; i < odom_est_trajectory.size(); i++) {

        pose_graph::Node<2, double> new_node;
        new_node.id_ = i;
        pose::Pose2d est_pose = odom_est_trajectory[i];
        new_node.est_position_ = std::make_shared<Eigen::Vector2d>(est_pose.first);
        new_node.est_orientation_ = std::make_shared<double>(est_pose.second);

        initial_node_positions.emplace_back(new_node);
    }

    offline_optimization::OfflineProblemData<2, double, 3, 2, double,
            pose_graph::MovableObservationSemanticPoints2d> offline_problem_data;

    offline_problem_data.odometry_factors_ = odom_factors;
    offline_problem_data.map_object_observations_ = samples;
    offline_problem_data.movable_observation_factors_ = movable_object_observations;
    offline_problem_data.initial_node_positions_ = initial_node_positions;

    std::unordered_map<std::string, std::unordered_map<pose_graph::NodeId, std::unordered_map<size_t, std::vector<pose::Pose2d>>>> relative_object_samples_for_cluster;
    std::unordered_map<std::string, Eigen::Vector2d> shape_dimensions_by_semantic_class;
    file_io::readShapeDimensions2dBySemanticClassMapFromFile(shape_dimensions_by_semantic_class_file,
                                                             shape_dimensions_by_semantic_class);


    std::function<std::shared_ptr<ceres::IterationCallback>(
            const pose_graph::NodeId &,
            const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                    pose_graph::MovableObservationSemanticPoints2d>> &)> callback_creator =
            [manager, noisy_observations_by_class, &relative_object_samples_for_cluster, shape_dimensions_by_semantic_class](
                    const pose_graph::NodeId &node_id,
                    const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                            pose_graph::MovableObservationSemanticPoints2d>> &pose_graph) {
                return std::make_shared<offline_optimization::CeresVisualizationCallbackSemanticPoints2d>(
                        pose_graph, manager, node_id, noisy_observations_by_class, relative_object_samples_for_cluster,
                        shape_dimensions_by_semantic_class);
            };

    std::function<void(const pose_graph::NodeId &,
                       const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                               pose_graph::MovableObservationSemanticPoints<2>>> &,
                       const offline_optimization::VisualizationTypeEnum &)> visualization_callback =
            [manager, gt_trajectory, odom_est_trajectory, noisy_observations_by_class,
                    &relative_object_samples_for_cluster, shape_dimensions_by_semantic_class](
                    const pose_graph::NodeId &max_node_id,
                    const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double,
                            3, 2, double, 3, pose_graph::MovableObservationSemanticPoints<2>>> &pose_graph,
                    const offline_optimization::VisualizationTypeEnum &vis_stage) {
                return runOptimizationVisualization(manager, max_node_id, pose_graph, vis_stage, gt_trajectory,
                                                    odom_est_trajectory, noisy_observations_by_class,
                                                    relative_object_samples_for_cluster,
                                                    shape_dimensions_by_semantic_class);
            };

    pose_optimization::PoseOptimizationParameters pose_opt_params = setupPoseOptimizationParams(runtime_params_config);

    file_io::SemanticPointObjectSamplerConfig semantic_point_object_sampler_config;
    file_io::readSemanticPointObjectSamplerConfigFromFile(semantic_point_object_sampler_config_file,
                                                          semantic_point_object_sampler_config);
    std::function<std::vector<pose::Pose2d>(const pose_graph::MovableObservationSemanticPointsFactor2d &,
                                            const pose::Pose2d)> sample_object_pose_generator =
            [semantic_point_object_sampler_config, pose_opt_params, shape_dimensions_by_semantic_class, &random_generator, &relative_object_samples_for_cluster](
                    const pose_graph::MovableObservationSemanticPointsFactor2d &factor,
                    const pose::Pose2d &object_detection_sensor_pose_rel_baselink) {

                if (relative_object_samples_for_cluster.find(factor.observation_.semantic_class_) !=
                    relative_object_samples_for_cluster.end()) {
                    if (relative_object_samples_for_cluster[factor.observation_.semantic_class_].find(
                            factor.observed_at_node_) !=
                        relative_object_samples_for_cluster[factor.observation_.semantic_class_].end()) {
                        if (relative_object_samples_for_cluster[factor.observation_.semantic_class_][factor.observed_at_node_].find(
                                factor.observation_.cluster_num_) !=
                            relative_object_samples_for_cluster[factor.observation_.semantic_class_][factor.observed_at_node_].end()) {
                            return relative_object_samples_for_cluster[factor.observation_.semantic_class_][factor.observed_at_node_][factor.observation_.cluster_num_];
                        }
                    }
                }
                double shape_dimensions_x = shape_dimensions_by_semantic_class.at(
                        factor.observation_.semantic_class_).x();
                double shape_dimensions_y = shape_dimensions_by_semantic_class.at(
                        factor.observation_.semantic_class_).y();

                std::vector<pose::Pose2d> poses = semantic_point_pom::generateConsistentSamples(
                        factor.observation_.object_points_,
                        pose_opt_params.cost_function_params_.num_samples_per_movable_obj_observation_,
                        semantic_point_object_sampler_config.samples_per_point,
                        semantic_point_object_sampler_config.position_bin_size,
                        semantic_point_object_sampler_config.orientation_bin_size,
                        shape_dimensions_x,
                        shape_dimensions_y,
                        semantic_point_object_sampler_config.point_std_dev,
                        semantic_point_object_sampler_config.min_points_repped_in_bin,
                        semantic_point_object_sampler_config.minimum_bins,
                        random_generator);

                std::vector<pose::Pose2d> transformed_into_baselink;
                for (const pose::Pose2d &sample_pose : poses) {
                    transformed_into_baselink.emplace_back(
                            pose::combinePoses(object_detection_sensor_pose_rel_baselink, sample_pose));
                }

                relative_object_samples_for_cluster[factor.observation_.semantic_class_][factor.observed_at_node_][factor.observation_.cluster_num_] = transformed_into_baselink;
                return transformed_into_baselink;
            };


    std::function<std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
            pose_graph::MovableObservationSemanticPoints<2>>>(
            const pose_optimization::CostFunctionParameters &)> pose_graph_creator =
            std::bind(pose_graph::utils::createFully2dPoseGraphSemanticPointDetectionsFromParams,
                      std::placeholders::_1, detections_sensor_rel_baselink, sample_object_pose_generator);

    LOG(INFO) << "Running optimization";
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = offline_optimizer.runOfflineOptimization(
            offline_problem_data, pose_opt_params,
            pose_graph_creator,
            callback_creator,
            visualization_callback);

    return optimization_results;
}

pose::Pose2d getDetectionsSensorRelBaselinkPoseFromFile(const std::string &detection_sensor_rel_baselink_file) {
    std::vector<pose::Pose3d> sensor_poses_vec;
    file_io::readPose3dsFromFile(detection_sensor_rel_baselink_file, sensor_poses_vec);
    if (sensor_poses_vec.size() == 0) {
        LOG(ERROR) << "No poses specifying the detection sensor's pose relative to the baselink";
        exit(1);
    }
    if (sensor_poses_vec.size() != 1) {
        LOG(WARNING) << "Multiple poses in file specifying the detection sensor's pose relative to the base link. Using"
                        " the first entry";
    }
    pose::Pose3d sensor_pose_3d = sensor_poses_vec.front();
    pose::Pose2d sensor_pose_2d = pose::toPose2d(sensor_pose_3d);
    return sensor_pose_2d;
}

void plotRectangleSamplesForCluster(const pose_graph::MovableObservationSemanticPointsFactor2d &factor,
                                    const pose::Pose2d &detections_sensor_rel_baselink,
                                    const std::unordered_map<std::string, Eigen::Vector2d> &shape_dimensions_by_semantic_class,
                                    const file_io::SemanticPointObjectSamplerConfig &semantic_point_object_sampler_config,
                                    std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                    const size_t &num_samples_to_generate,
                                    util_random::Random &random_generator) {

    double shape_dimensions_x = shape_dimensions_by_semantic_class.at(
            factor.observation_.semantic_class_).x();
    double shape_dimensions_y = shape_dimensions_by_semantic_class.at(
            factor.observation_.semantic_class_).y();

    std::vector<pose::Pose2d> poses = semantic_point_pom::generateConsistentSamples(
            factor.observation_.object_points_,
            num_samples_to_generate,
            semantic_point_object_sampler_config.samples_per_point,
            semantic_point_object_sampler_config.position_bin_size,
            semantic_point_object_sampler_config.orientation_bin_size,
            shape_dimensions_x,
            shape_dimensions_y,
            semantic_point_object_sampler_config.point_std_dev,
            semantic_point_object_sampler_config.min_points_repped_in_bin,
            semantic_point_object_sampler_config.minimum_bins,
            random_generator);

    std::vector<pose::Pose2d> poses_rel_baselink;
    for (const pose::Pose2d &pose_rel_sensor : poses) {
        poses_rel_baselink.emplace_back(pose::combinePoses(detections_sensor_rel_baselink, pose_rel_sensor));
    }

    std::vector<Eigen::Vector2d> points_rel_baselink;
    for (const Eigen::Vector2d &point_rel_sensor : factor.observation_.object_points_) {
        points_rel_baselink.emplace_back(transformPoint(detections_sensor_rel_baselink, point_rel_sensor));
        break;
    }
    vis_manager->displaySemanticPointObsFromEstTrajectory({pose::createPose2d(0, 0, 0)},
                                                          {{{factor.observation_.cluster_num_, factor.observation_.object_points_}}},
                                                          factor.observation_.semantic_class_,
                                                          {{0, {{factor.observation_.cluster_num_, poses_rel_baselink}}}},
                                                          Eigen::Vector2d(shape_dimensions_x, shape_dimensions_y));

}

void debugSamples(const std::vector<pose_graph::MovableObservationSemanticPointsFactor2d> &factors,
                  const pose::Pose2d &detections_sensor_rel_baselink,
                  const std::string &shape_dimensions_by_semantic_class_file,
                  const std::string &semantic_point_object_sampler_config_file,
                  std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                  const size_t &num_samples_to_generate,
                  util_random::Random &random_generator) {


    std::unordered_map<std::string, Eigen::Vector2d> shape_dimensions_by_semantic_class;
    file_io::readShapeDimensions2dBySemanticClassMapFromFile(shape_dimensions_by_semantic_class_file,
                                                             shape_dimensions_by_semantic_class);

    file_io::SemanticPointObjectSamplerConfig semantic_point_object_sampler_config;
    file_io::readSemanticPointObjectSamplerConfigFromFile(semantic_point_object_sampler_config_file,
                                                          semantic_point_object_sampler_config);

    for (const pose_graph::MovableObservationSemanticPointsFactor2d &factor : factors) {
        plotRectangleSamplesForCluster(factor, detections_sensor_rel_baselink, shape_dimensions_by_semantic_class,
                                       semantic_point_object_sampler_config, vis_manager, num_samples_to_generate,
                                       random_generator);
        system("pause");
    }
}

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;


    util_random::Random random_generator;
//
//    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::system_clock::now().time_since_epoch()).count());

    std::string param_prefix = FLAGS_param_prefix;
    std::string node_prefix = FLAGS_param_prefix;
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "movable_obj_trajectory_estimation_semantic_points");
    ros::NodeHandle n;

    // Known config params
    std::string car_semantic_class = "car";

    std::string runtime_params_config_file_name;

    if (!n.getParam(param_prefix + kRuntimeParamsConfigParamName, runtime_params_config_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kRuntimeParamsConfigParamName;
        exit(1);
    }

    LOG(INFO) << "Reading runtime params config from file " << runtime_params_config_file_name;
    file_io::RuntimeParamsConfig runtime_params_config;
    file_io::readRuntimeParamsConfigFromFile(runtime_params_config_file_name, runtime_params_config);

    double odom_k1 = runtime_params_config.odom_k1_;
    double odom_k2 = runtime_params_config.odom_k2_;
    double odom_k3 = runtime_params_config.odom_k3_;
    double odom_k4 = runtime_params_config.odom_k4_;
    double odom_k5 = runtime_params_config.odom_k5_;
    double odom_k6 = runtime_params_config.odom_k6_;

    uint64_t pose_sample_ratio = runtime_params_config.pose_sample_ratio_;

    std::vector<std::string> past_sample_files;
    std::string odom_estimates_file_name;
    std::string semantic_point_detections_file_name;
    std::string traj_est_output_file;
    std::string gt_traj_file;
    std::string detection_sensor_rel_baselink_file;
    std::string semantic_index_to_string_file;
    std::string shape_dimensions_by_semantic_class_file;
    std::string semantic_point_object_sampler_config_file;

    if (!n.getParam(param_prefix + kPastSamplesFilesParamName, past_sample_files)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kPastSamplesFilesParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kOdomTrajectoryEstimatesFileParamName, odom_estimates_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kOdomTrajectoryEstimatesFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kSemanticPointDetectionsCurrTrajectoryFileParamName,
                    semantic_point_detections_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kSemanticPointDetectionsCurrTrajectoryFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kTrajectoryOutputFileParamName, traj_est_output_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kTrajectoryOutputFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kDetectionSensorRelBaseLinkFileParamName, detection_sensor_rel_baselink_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kDetectionSensorRelBaseLinkFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kSemanticIndexToStringClassFileParamName, semantic_index_to_string_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kSemanticIndexToStringClassFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kShapeDimensionsBySemanticClassFileParamName,
                    shape_dimensions_by_semantic_class_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kShapeDimensionsBySemanticClassFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kSemanticPointObjectSamplerConfigFileParamName,
                    semantic_point_object_sampler_config_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kSemanticPointObjectSamplerConfigFileParamName;
        exit(1);
    }

    std::string config_file_base_name = runtime_params_config_file_name.substr(
            runtime_params_config_file_name.find_last_of("/\\") + 1);
    size_t lastindex = config_file_base_name.find_last_of(".");
    config_file_base_name = config_file_base_name.substr(0, lastindex);

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n, param_prefix);

    pose::Pose2d detections_sensor_rel_baselink = getDetectionsSensorRelBaselinkPoseFromFile(
            detection_sensor_rel_baselink_file);

    std::vector<pose::Pose2d> gt_trajectory;
    if (n.getParam(param_prefix + kGtTrajectoryFile, gt_traj_file)) {
        gt_trajectory = readTrajFromFile(gt_traj_file);
    }

    // Read initial 2d initial estimates
    std::vector<pose::Pose2d> initial_trajectory_estimates = readTrajFromFile(odom_estimates_file_name);

    // Convert to odom factors
    std::vector<pose_graph::GaussianBinaryFactor2d> odom_factors = createOdomFactorsFromInitOdomEst(
            initial_trajectory_estimates, odom_k1, odom_k2, odom_k3, odom_k4, odom_k5, odom_k6);


    // Read samples
    std::vector<pose_graph::MapObjectObservation2d> samples;
    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories;
    for (const std::string &past_sample_file : past_sample_files) {
        std::vector<file_io::PastSample2d> past_samples;
        file_io::readPastSample2dsFromFile(past_sample_file, past_samples);
        for (const file_io::PastSample2d &past_sample : past_samples) {
            pose_graph::MapObjectObservation<2, double> map_observation;
            map_observation.semantic_class_ = past_sample.semantic_class_;
            map_observation.transl_ = Eigen::Vector2d(past_sample.transl_x_, past_sample.transl_y_);
            map_observation.orientation_ = past_sample.theta_;
            map_observation.obs_value_ = past_sample.value_;
            samples_for_prev_trajectories[past_sample.semantic_class_].emplace_back(
                    std::make_pair(std::make_pair(map_observation.transl_, map_observation.orientation_),
                                   map_observation.obs_value_));
            samples.emplace_back(map_observation);
        }
    }

    for (const auto &samples_for_prev_trajectories_with_class : samples_for_prev_trajectories) {
        manager->displayPastSampleValues(samples_for_prev_trajectories_with_class.first,
                                         samples_for_prev_trajectories_with_class.second);
    }

    // Read detections
    std::vector<pose_graph::MovableObservationSemanticPointsFactor2d> movable_observation_factors =
            getMovableObservationFactorsFromDetectionsFile(semantic_point_detections_file_name, pose_sample_ratio,
                                                           semantic_index_to_string_file);

//    pose_graph::MovableObservationSemanticPointsFactor2d factor_to_plot_samples_for = movable_observation_factors[0];
//    plotRectangleSamplesForCluster(factor_to_plot_samples_for,
//                                   detections_sensor_rel_baselink,
//                                   shape_dimensions_by_semantic_class_file,
//                                   semantic_point_object_sampler_config_file,
//                                   manager,
//                                   random_generator);

    if (FLAGS_debug_samples) {
        debugSamples(movable_observation_factors,
                     detections_sensor_rel_baselink,
                     shape_dimensions_by_semantic_class_file,
                     semantic_point_object_sampler_config_file,
                     manager,
                     50, // TODO read this from cfg
                     random_generator);
        exit(1);
    }

    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = getTrajectoryEstimate(
            initial_trajectory_estimates, gt_trajectory, manager, odom_factors, samples,
            movable_observation_factors, runtime_params_config, detections_sensor_rel_baselink,
            shape_dimensions_by_semantic_class_file, semantic_point_object_sampler_config_file, random_generator);

    std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
    for (pose_graph::NodeId node_id = 0; node_id < optimization_results.size(); node_id++) {
        if (optimization_results.find(node_id) == optimization_results.end()) {
            LOG(ERROR) << "Could not find pose for node " << node_id;
            exit(1);
        }
        pose::Pose2d pose_for_node = optimization_results[node_id];
        file_io::TrajectoryNode2d traj_node;
        traj_node.node_id_ = node_id;
        traj_node.transl_x_ = pose_for_node.first.x();
        traj_node.transl_y_ = pose_for_node.first.y();
        traj_node.theta_ = pose_for_node.second;
        trajectory_nodes.emplace_back(traj_node);
    }

    file_io::writeTrajectory2dToFile(traj_est_output_file, trajectory_nodes);

    return 0;
}