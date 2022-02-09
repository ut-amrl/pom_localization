#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <ctime>

#include <file_io/past_sample_io.h>
#include <pose_optimization/offline/offline_problem_runner.h>
#include <pose_optimization/utils/pose_graph_creation_utils.h>
#include <pose_optimization/offline/ceres_visualization_callback_2d.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/runtime_params_config_io.h>

#include <unordered_map>
#include <file_io/object_positions_by_pose_io.h>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_bool(run_gpc_viz, false, "Run GPC viz");
DEFINE_bool(skip_optimization, false, "Skip optimization");

const std::string kPastSamplesFilesParamName = "past_samples_files";

const std::string kOdomTrajectoryEstimatesFileParamName = "odom_traj_est_file";

const std::string kObjDetectionsCurrTrajectoryFileParamName = "obj_det_curr_traj_file";

const std::string kTrajectoryOutputFilePrefixParamName = "traj_est_output_file_prefix";

const std::string kGtTrajectoryFile = "gt_trajectory_file";

const std::string kRuntimeParamsConfigParamName = "runtime_params_config_file";

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

    // TODO configure

////    cost_function_params.mean_position_kernel_len_ = 0.8;
//    cost_function_params.mean_position_kernel_len_ = 0.3;
//    cost_function_params.mean_orientation_kernel_len_ = 0.1; // TODO fix
////    cost_function_params.mean_orientation_kernel_len_ = 10000;
////    cost_function_params.mean_position_kernel_var_ = 30;
////    cost_function_params.mean_orientation_kernel_var_ = 30;
//
////    cost_function_params.mean_position_kernel_var_ = 9;
//
//    cost_function_params.mean_position_kernel_var_ = 0.045;
////    cost_function_params.mean_orientation_kernel_var_ = 1;
//    cost_function_params.mean_orientation_kernel_var_ = 1000;
//
//
//    cost_function_params.default_obj_probability_input_variance_for_mean_ = 10;
//
////    cost_function_params.var_position_kernel_len_ = 1.8;
////    cost_function_params.var_position_kernel_len_ = 25;
//    cost_function_params.var_position_kernel_len_ = 7.5;
//    cost_function_params.var_orientation_kernel_len_ = 20;
//
//    cost_function_params.var_position_kernel_var_ = 30;
//    cost_function_params.var_orientation_kernel_var_ = 0.3;
//
//    cost_function_params.default_obj_probability_input_variance_for_var_ = 10;
//
//    cost_function_params.max_gpc_samples_ = 500;
//
////    cost_function_params.num_nodes_in_optimization_window_ = 20;
//    cost_function_params.num_nodes_in_optimization_window_ = 30;
//    cost_function_params.full_optimization_interval_ = 80;



    pose_opt_params.cost_function_params_ = cost_function_params;
    return pose_opt_params;

}

std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> createGpc(
        const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                pose_graph::MovableObservationObjectPose2d>> &pose_graph,
        const std::string &semantic_class,
        const uint64_t &max_samples,
        const double &radius,
        const Eigen::Vector2d &search_point) {
    return pose_graph->getMovableObjGpcWithinRadius(semantic_class, radius, search_point, max_samples);
}

void runOptimizationVisualization(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                  const pose_graph::NodeId &max_node_id,
                                  const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3,
                                          2, double, 3, pose_graph::MovableObservationObjectPose2d>> &pose_graph,
                                  const offline_optimization::VisualizationTypeEnum &vis_stage,
                                  const std::vector<pose::Pose2d> &ground_truth_trajectory,
                                  const std::vector<pose::Pose2d> &unoptimized_trajectory,
                                  const std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>>
                                  &noisy_obj_observations_by_class) {

    // TODO make this more generic (not specifically car class)
    switch (vis_stage) {
        case offline_optimization::VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION:


            for (const auto &obs_with_class : noisy_obj_observations_by_class) {
                std::string semantic_class = obs_with_class.first;
                std::vector<std::vector<pose::Pose2d>> noisy_obj_observations = obs_with_class.second;
                if (!ground_truth_trajectory.empty()) {
                    vis_manager->displayTrueTrajectory(ground_truth_trajectory);
                    vis_manager->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obj_observations,
                                                                        semantic_class);
                }
//                vis_manager->displayOdomTrajectory(unoptimized_trajectory);
//                vis_manager->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
//                                                                      noisy_obj_observations, semantic_class);
                {
//                    if (!ground_truth_trajectory.empty()) {

                    std::vector<pose::Pose2d> poses_global_frame;
                    for (size_t node = 0; node < unoptimized_trajectory.size(); node++) {
                        pose::Pose2d robot_pose = unoptimized_trajectory[node];
                        for (const pose::Pose2d &obj_pose : noisy_obj_observations[node]) {
                            poses_global_frame.emplace_back(pose::combinePoses(robot_pose, obj_pose));
                        }
                    }

                    if (FLAGS_run_gpc_viz) {

//                        std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
//                                visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(
//                                        poses_global_frame);
                        std::function<std::shared_ptr<gp_regression::GaussianProcessClassifier<3,
                                gp_kernel::Pose2dKernel>>(const Eigen::Vector2d &)> gpc_creator =
                                std::bind(createGpc, pose_graph, "car",
//                                              5000,
                                          500,
//                              7.5,
                                          8.5,
                                          std::placeholders::_1);
                        vis_manager->displayMaxGpRegressorOutput(
//                                pose_graph->getMovableObjGpc("car", 0.5),
                                gpc_creator,
//                                                                 0.3, // TODO revert
//                                                                 0.65,
//                                                                0.5,
                                .5,
//                                -55, 30, -5, 30
                                -65, 40, -10, 95
//                                                                 min_max_points_to_display.first.x(),
//                                                                 min_max_points_to_display.second.x(),
//                                                                 min_max_points_to_display.first.y(),
//                                                                 min_max_points_to_display.second.y()
                        );

//                    }
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
                std::vector<std::vector<pose::Pose2d>> noisy_obj_observations = obs_with_class.second;

                vis_manager->displayObjObservationsFromOdomTrajectory(limited_odom_poses_list_extended,
                                                                      noisy_obj_observations, semantic_class);
            }
//                vis_manager->displayOdomTrajectory(unoptimized_trajectory);
//                vis_manager->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
//                                                                      noisy_obj_observations, semantic_class);
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
                std::vector<std::vector<pose::Pose2d>> noisy_obj_observations = obs_with_class.second;
                vis_manager->displayObjObservationsFromEstTrajectory(node_poses_list, noisy_obj_observations,
                                                                     semantic_class);
                if (!ground_truth_trajectory.empty()) {
                    vis_manager->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obj_observations,
                                                                        semantic_class);
                }

                vis_manager->displayObjObservationsFromOdomTrajectory(limited_odom_poses_list,
                                                                      noisy_obj_observations, semantic_class);
//                vis_manager->displayOdomTrajectory(unoptimized_trajectory);
//                vis_manager->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
//                                                                      noisy_obj_observations, semantic_class);
            }
//                        ros::Duration(2).sleep();
        }
            break;
        case offline_optimization::VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION:
            // Optionally display distribution intensity map (either over robot poses or over object poses)
        {
//                std::vector<pose::Pose2d> poses_global_frame;
//                for (size_t node = 0; node < ground_truth_trajectory.size(); node++) {
//                    pose::Pose2d robot_pose = ground_truth_trajectory[node];
//                    for (const pose::Pose2d &obj_pose : noisy_obj_observations[node]) {
//                        poses_global_frame.emplace_back(pose::combinePoses(robot_pose, obj_pose));
//                    }
//                }
//
//                std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
//                        visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(poses_global_frame);
//                vis_manager->displayMaxGpRegressorOutput(pose_graph->getMovableObjGpc(car_class), 0.6, // TODO revert
//                                                         min_max_points_to_display.first.x(),
//                                                         min_max_points_to_display.second.x(),
//                                                         min_max_points_to_display.first.y(),
//                                                         min_max_points_to_display.second.y());
        }
            break;
        default:
            break;
    }
}

std::vector<pose_graph::MovableObservationObjectPoseFactor2d> getMovableObservationFactorsFromDetectionsFile(
        const std::string &semantic_class,
        const double &detection_variance_transl_x,
        const double &detection_variance_transl_y,
        const double &detection_variance_theta,
        const std::string &object_detections_file_name,
        const int &pose_sample_ratio) {

    std::vector<file_io::ObjectPositionByPose> raw_obj_detections;
    LOG(INFO) << "Reading from file " << object_detections_file_name;
    file_io::readObjectPositionsByPoseFromFile(object_detections_file_name, raw_obj_detections);

    std::vector<pose_graph::MovableObservationObjectPoseFactor2d> observation_factors;
    for (const file_io::ObjectPositionByPose &raw_detection : raw_obj_detections) {
        if ((raw_detection.pose_number_ % pose_sample_ratio) == 0) {
            pose_graph::MovableObservationObjectPose2d observation;
            observation.semantic_class_ = semantic_class;
            observation.observation_transl_ = Eigen::Vector2d(raw_detection.transl_x_, raw_detection.transl_y_);
            observation.observation_orientation_ = raw_detection.theta_;

            Eigen::Matrix3d observation_cov = Eigen::Matrix3d::Zero();
            observation_cov(0, 0) = detection_variance_transl_x;
            observation_cov(1, 1) = detection_variance_transl_y;
            observation_cov(2, 2) = detection_variance_theta;

            observation.observation_covariance_ = observation_cov;

            observation_factors.emplace_back(
                    pose_graph::MovableObservationObjectPoseFactor2d(raw_detection.pose_number_, observation));
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
        if ((i == 124) || (i == 125)) {
            LOG(INFO) << "i " << i;
            LOG(INFO) << "Norm " << norm << ", angle diff " << rel_pose.second;
            LOG(INFO) << "Odom cov " << odom_cov_mat.diagonal();
            LOG(INFO) << "Sqrt information " << odom_sqrt_information_mat;
        }
//        LOG(INFO) << odom_sqrt_information_mat;
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
                      const std::vector<pose_graph::MovableObservationObjectPoseFactor2d> &movable_object_observations,
                      const file_io::RuntimeParamsConfig &runtime_params_config) {

    offline_optimization::OfflinePoseOptimizer<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
            pose_graph::MovableObservationObjectPose2d> offline_optimizer;

    std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> noisy_observations_by_class;
    for (const pose_graph::MovableObservationObjectPoseFactor2d &movable_factor : movable_object_observations) {
        pose_graph::NodeId observed_at_node = movable_factor.observed_at_node_;
        std::string semantic_class = movable_factor.observation_.semantic_class_;

        std::vector<std::vector<pose::Pose2d>> observations_for_class;
        if (noisy_observations_by_class.find(semantic_class) == noisy_observations_by_class.end()) {
            for (size_t node_num = 0; node_num < odom_est_trajectory.size(); node_num++) {
                observations_for_class.push_back({});
            }
        } else {
            observations_for_class = noisy_observations_by_class[semantic_class];
        }
        observations_for_class[observed_at_node].emplace_back(
                std::make_pair(movable_factor.observation_.observation_transl_,
                               movable_factor.observation_.observation_orientation_));
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
            pose_graph::MovableObservationObjectPose2d> offline_problem_data;

    offline_problem_data.odometry_factors_ = odom_factors;
    offline_problem_data.map_object_observations_ = samples;
    offline_problem_data.movable_observation_factors_ = movable_object_observations;
    offline_problem_data.initial_node_positions_ = initial_node_positions;

    std::function<std::shared_ptr<ceres::IterationCallback>(
            const pose_graph::NodeId &,
            const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                    pose_graph::MovableObservationObjectPose2d>> &)> callback_creator =
            [manager, noisy_observations_by_class](
                    const pose_graph::NodeId &node_id,
                    const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                            pose_graph::MovableObservationObjectPose2d>> &pose_graph) {
                return std::make_shared<offline_optimization::CeresVisualizationCallback2d>(
                        pose_graph, manager, node_id, noisy_observations_by_class);
            };

    std::function<void(const pose_graph::NodeId &,
                       const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                               pose_graph::MovableObservationObjectPose2d>> &,
                       const offline_optimization::VisualizationTypeEnum &)> visualization_callback =
            std::bind(runOptimizationVisualization, manager,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                      gt_trajectory, odom_est_trajectory,
                      noisy_observations_by_class);

    LOG(INFO) << "Running optimization";
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = offline_optimizer.runOfflineOptimization(
            offline_problem_data, setupPoseOptimizationParams(runtime_params_config),
            pose_graph::utils::createFully2dPoseGraphObjectPoseDetectionsFromParams,
            callback_creator,
            visualization_callback);

    return optimization_results;
}


int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string param_prefix = FLAGS_param_prefix;
    std::string node_prefix = FLAGS_param_prefix;
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "movable_obj_trajectory_estimation");
    ros::NodeHandle n;

    // Known config params
    std::string car_semantic_class = "car";

    std::string runtime_params_config_file_name;

    if (!n.getParam(param_prefix + kRuntimeParamsConfigParamName, runtime_params_config_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kRuntimeParamsConfigParamName;
        exit(1);
    }

    file_io::RuntimeParamsConfig runtime_params_config;
    file_io::readRuntimeParamsConfigFromFile(runtime_params_config_file_name, runtime_params_config);

    // Configuration params -- need to play around with these

    double detection_variance_transl_x = runtime_params_config.detection_variance_transl_x_;
    double detection_variance_transl_y = runtime_params_config.detection_variance_transl_y_;
    double detection_variance_theta = runtime_params_config.detection_variance_theta_;

    double odom_k1 = runtime_params_config.odom_k1_;
    double odom_k2 = runtime_params_config.odom_k2_;
    double odom_k3 = runtime_params_config.odom_k3_;
    double odom_k4 = runtime_params_config.odom_k4_;
    double odom_k5 = runtime_params_config.odom_k5_;
    double odom_k6 = runtime_params_config.odom_k6_;

//    double detection_variance_transl_x = 0.01;
//    double detection_variance_transl_y = 0.01;
//    double detection_variance_theta = 0.02;
////    double odom_std_dev_transl_x = 0.1;
////    double odom_std_dev_transl_y = 0.1;
////    double odom_std_dev_theta = 0.15;
////        double odom_std_dev_transl_x = 1e-3;
////    double odom_std_dev_transl_y = 1e-3;
////    double odom_std_dev_theta = 1e-5;
//
////    double odom_k1 = 0.75;
////    double odom_k2 = 1;
////    double odom_k3 = 0.75;
////    double odom_k4 = 1;
////    double odom_k5 = 1;
////    double odom_k6 = 1;
//
//    double odom_k1 = 0.75;
//    double odom_k2 = 2;
//    double odom_k3 = 0.75;
//    double odom_k4 = 2;
//    double odom_k5 = 1;
//    double odom_k6 = 2;
//
////    double odom_k1 = 100;
////    double odom_k2 = 100;
////    double odom_k3 = 100;
////    double odom_k4 = 100;
////    double odom_k5 = 100;
////    double odom_k6 = 100;
////
////    double odom_k1 = 0.1;
////    double odom_k2 = 0.001;
////    double odom_k3 = 0.01;
////    double odom_k4 = 0.0001;
////    double odom_k5 = 0.001;
////    double odom_k6 = 0.01;
////    double odom_std_dev_transl_x = 1;
////    double odom_std_dev_transl_y = 1;
////    double odom_std_dev_theta = 1;
//    int pose_sample_ratio = 20;
    uint64_t pose_sample_ratio = runtime_params_config.pose_sample_ratio_;

    std::vector<std::string> past_sample_files;
    std::string odom_estimates_file_name;
    std::string object_detections_file_name;
    std::string traj_est_output_file_prefix;
    std::string gt_traj_file;

    if (!n.getParam(param_prefix + kPastSamplesFilesParamName, past_sample_files)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kPastSamplesFilesParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kOdomTrajectoryEstimatesFileParamName, odom_estimates_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kOdomTrajectoryEstimatesFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kObjDetectionsCurrTrajectoryFileParamName, object_detections_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kObjDetectionsCurrTrajectoryFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kTrajectoryOutputFilePrefixParamName, traj_est_output_file_prefix)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kTrajectoryOutputFilePrefixParamName;
        exit(1);
    }

    std::string config_file_base_name = runtime_params_config_file_name.substr(
            runtime_params_config_file_name.find_last_of("/\\") + 1);
    size_t lastindex = config_file_base_name.find_last_of(".");
    config_file_base_name = config_file_base_name.substr(0, lastindex);
    std::string traj_est_output_file = traj_est_output_file_prefix + config_file_base_name + ".csv";

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n, param_prefix);

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
    std::vector<pose_graph::MovableObservationObjectPoseFactor2d> movable_observation_factors =
            getMovableObservationFactorsFromDetectionsFile(car_semantic_class, detection_variance_transl_x,
                                                           detection_variance_transl_y, detection_variance_theta,
                                                           object_detections_file_name, pose_sample_ratio);

    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = getTrajectoryEstimate(
            initial_trajectory_estimates, gt_trajectory, manager, odom_factors, samples,
            movable_observation_factors, runtime_params_config);

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