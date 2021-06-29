//
// Created by amanda on 1/28/21.
//

#ifndef AUTODIFF_GP_SYNTHETIC_PROBLEM_RUNNER_2D_H
#define AUTODIFF_GP_SYNTHETIC_PROBLEM_RUNNER_2D_H

#include <synthetic_problem/synthetic_problem_config_2d.h>
#include <base_lib/pose_reps.h>
#include <pose_optimization/offline/offline_problem_data.h>
#include <pose_optimization/offline/offline_problem_runner.h>
#include <visualization/ros_visualization.h>
#include <pose_optimization/offline/ceres_visualization_callback_2d.h>

#include <pose_optimization/utils/pose_graph_creation_utils.h>
#include <pose_optimization/uncertainty_aware/uncertainty_aware_pose_opt_utils.h>

namespace synthetic_problem {

    class SyntheticProblemRunner2d {
    public:


        typedef offline_optimization::OfflineProblemData<2, double, 3, 2, double> OfflineProblemDataType;

        SyntheticProblemRunner2d(const std::shared_ptr<visualization::VisualizationManager> vis_manager,
                                 bool run_visualization) : vis_manager_(vis_manager), run_visualization_(run_visualization) {}

        void runOptimizationVisualization(
                const pose_graph::NodeId &max_node_id,
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
                const offline_optimization::VisualizationTypeEnum &vis_stage,
                const std::vector<pose::Pose2d> &ground_truth_trajectory,
                const std::vector<pose::Pose2d> &unoptimized_trajectory,
                const std::vector<pose::Pose2d> &ground_truth_obj_poses,
                const std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> &past_movable_object_poses,
                const std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> &noisy_obj_observations_by_type) {
            if (run_visualization_) {

                switch (vis_stage) {
                    case offline_optimization::VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION:
                        // Optionally display distribution intensity map (either over robot poses or over object poses)

                        for (const auto &samples_by_type : past_movable_object_poses) {
                            vis_manager_->displayPastSampleValues(samples_by_type.first, samples_by_type.second);
                        }



                        vis_manager_->displayTrueTrajectory(ground_truth_trajectory);
                        vis_manager_->displayOdomTrajectory(unoptimized_trajectory);

                        // TODO make this more generic (not specific to car class)
                        vis_manager_->displayTrueObjPoses(ground_truth_obj_poses, kCarClass);

                        for (const auto &noisy_obs_with_type : noisy_obj_observations_by_type) {
                            vis_manager_->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obs_with_type.second,
                                                                     noisy_obs_with_type.first);
                            vis_manager_->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
                                                                                 noisy_obs_with_type.second,
                                                                                 noisy_obs_with_type.first);
                        }
                        {
                            // TODO make this not specific to car class
                            std::vector<pose::Pose2d> poses_global_frame;
                            for (size_t node = 0; node < ground_truth_trajectory.size(); node++) {
                                pose::Pose2d robot_pose = ground_truth_trajectory[node];
                                for (const pose::Pose2d &obj_pose : noisy_obj_observations_by_type.at(kCarClass)[node]) {
                                    poses_global_frame.emplace_back(pose::combinePoses(robot_pose, obj_pose));
                                }
                            }

                            std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
                                    visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(poses_global_frame);

                            vis_manager_->displayMaxGpRegressorOutput(pose_graph->getMovableObjGpc(kCarClass),0.3,
                                                                      min_max_points_to_display.first.x(),
                                                                      min_max_points_to_display.second.x(),
                                                                      min_max_points_to_display.first.y(),
                                                                      min_max_points_to_display.second.y());

                        }

                        // Display true trajectory
                        // Display true object poses
                        // Display noisy car poses from GT
                        // Display initial trajectory
                        // Display initial car poses from initial trajectory
                        break;
                    case offline_optimization::VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION:
                        break;
                    case offline_optimization::VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION: {
                        std::vector<pose::Pose2d> node_poses_list;
                        std::unordered_map<pose_graph::NodeId, pose::Pose2d> node_poses;
                        pose_graph->getNodePoses(node_poses);
                        for (pose_graph::NodeId node_id = 0; node_id <= max_node_id; node_id++) {
                            node_poses_list.emplace_back(node_poses[node_id]);
                        }
                        vis_manager_->displayEstTrajectory(node_poses_list);

                        for (const auto &noisy_obs_with_type : noisy_obj_observations_by_type) {
                            vis_manager_->displayObjObservationsFromEstTrajectory(node_poses_list, noisy_obs_with_type.second,
                                                                                noisy_obs_with_type.first);
                        }

                        vis_manager_->displayTrueTrajectory(ground_truth_trajectory);
                        vis_manager_->displayOdomTrajectory(unoptimized_trajectory);

                        // TODO make this more generic (not specific to car class)
                        vis_manager_->displayTrueObjPoses(ground_truth_obj_poses, kCarClass);

                        for (const auto &noisy_obs_with_type : noisy_obj_observations_by_type) {
                            vis_manager_->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obs_with_type.second,
                                                                                 noisy_obs_with_type.first);
                            vis_manager_->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
                                                                                   noisy_obs_with_type.second,
                                                                                   noisy_obs_with_type.first);
                        }

//                        ros::Duration(2).sleep();
                    }
                        break;
                    case offline_optimization::VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION:
                        break;
                    default:
                        break;
                }
            }
        }

        std::shared_ptr<ceres::IterationCallback> createCeresIterationCallback(
                const pose_graph::NodeId &node_id,
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
                const std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> noisy_observations_by_class) {
            if (run_visualization_) {
                return std::make_shared<offline_optimization::CeresVisualizationCallback2d>(
                        pose_graph, vis_manager_, node_id, noisy_observations_by_class);
            } else {
                return nullptr;
            }
        }

        std::unordered_map<pose_graph::NodeId, pose::Pose2d> runSyntheticProblem(const std::vector<pose::Pose2d> &ground_truth_trajectory,
                const std::vector<pose::Pose2d> &noisy_odometry, // TODO remove?
                const std::unordered_map<std::string, std::vector<pose::Pose2d>> &object_gt_poses,
                const std::unordered_map<std::string, std::vector<std::vector<pose_optimization::ObjectDetectionRelRobot<pose::Pose2d , 3>>>> &movable_object_detections,
                const std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> &past_movable_object_poses,
                const SyntheticProblemNoiseConfig2d &noise_config,
                const pose_optimization::PoseOptimizationParameters &pose_optimization_params,
                util_random::Random &random_generator) {

            OfflineProblemDataType offline_problem_data;

//            // Compute true odometry and add noise to generate constraints ----------------------------------
//            std::vector<pose::Pose2d> true_odometry;
//            for (size_t i = 1; i < ground_truth_trajectory.size(); i++) {
//                true_odometry.emplace_back(pose::getPoseOfObj1RelToObj2(ground_truth_trajectory[i],
//                                                                        ground_truth_trajectory[i-1]));
//            }
//
////            util_random::Random random_generator;
//            std::vector<pose::Pose2d> noisy_odometry;
//            for (const pose::Pose2d &true_odom : true_odometry) {
//                noisy_odometry.emplace_back(pose::addRelativeGaussianNoise(true_odom, noise_config.odometry_x_std_dev_,
//                                                             noise_config.odometry_y_std_dev_,
//                                                             noise_config.odometry_yaw_std_dev_,
//                                                             random_generator));
//            }

            Eigen::Matrix<double, 3, 3> odom_cov_mat = Eigen::Matrix<double, 3, 3>::Zero();
            odom_cov_mat(0, 0) = pow(noise_config.odometry_x_std_dev_, 2);
            odom_cov_mat(1, 1) = pow(noise_config.odometry_y_std_dev_, 2);
            odom_cov_mat(2, 2) = pow(noise_config.odometry_yaw_std_dev_, 2);

            Eigen::Matrix<double, 3, 3> odom_sqrt_information_mat = odom_cov_mat.inverse().sqrt();

            for (uint64_t i = 0; i < noisy_odometry.size(); i++) {
                pose_graph::NodeId prev_node = i;
                pose_graph::NodeId to_node = i + 1;

                pose_graph::GaussianBinaryFactor2d factor;
                factor.to_node_ = to_node;
                factor.from_node_ = prev_node;
                factor.translation_change_ = noisy_odometry[i].first;
                factor.orientation_change_ = noisy_odometry[i].second;
                factor.sqrt_information_ = odom_sqrt_information_mat;
                offline_problem_data.odometry_factors_.emplace_back(factor);
            }

            std::vector<pose::Pose2d> initial_node_positions;
            pose::Pose2d prev_pose = ground_truth_trajectory[0];
//            pose::Pose2d prev_pose = pose::createPose2d(0, 0, 0);
            initial_node_positions.emplace_back(prev_pose);
            for (const pose::Pose2d &odom_to_next_pos : noisy_odometry) {
                pose::Pose2d odom_for_initial_node_pose = odom_to_next_pos;
                if (noise_config.add_additional_initial_noise_) {
                    odom_for_initial_node_pose = pose::addRelativeGaussianNoise(odom_to_next_pos,
                                                                  noise_config.init_pose_gaussian_noise_x_,
                                                                  noise_config.init_pose_gaussian_noise_y_,
                                                                  noise_config.init_pose_gaussian_noise_yaw_,
                                                                  random_generator);
                }
                pose::Pose2d new_pose = pose::combinePoses(prev_pose, odom_for_initial_node_pose);
                initial_node_positions.emplace_back(new_pose);
                prev_pose = new_pose;
            }

            for (pose_graph::NodeId node_num = 0; node_num < initial_node_positions.size(); node_num++) {
                pose_graph::Node<2, double> node;
                node.id_ = node_num;
                node.est_position_ = std::make_shared<Eigen::Vector2d>(initial_node_positions[node_num].first);
                node.est_orientation_ = std::make_shared<double>(initial_node_positions[node_num].second);
                offline_problem_data.initial_node_positions_.emplace_back(node);
            }

            std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> noisy_observations;
            for (const auto &observations_for_type : movable_object_detections) {
                noisy_observations[observations_for_type.first] = {};
                for (pose_graph::NodeId node_num = 0; node_num < observations_for_type.second.size(); node_num++) {
                    std::vector<pose::Pose2d> detections_at_node = {};
                    for (const pose_optimization::ObjectDetectionRelRobot<pose::Pose2d , 3> &observation : observations_for_type.second.at(node_num)) {
                        pose_graph::MovableObservation<2, double, 3> pg_observation;
                        detections_at_node.emplace_back(observation.pose_);
                        pg_observation.semantic_class_ = observations_for_type.first;
                        pg_observation.observation_transl_ = observation.pose_.first;
                        pg_observation.observation_orientation_ = observation.pose_.second;
                        Eigen::Matrix3d movable_obs_cov_mat;
                        movable_obs_cov_mat(0, 0) = observation.object_pose_variance_(0);
                        movable_obs_cov_mat(1, 1) = observation.object_pose_variance_(1);
                        movable_obs_cov_mat(2, 2) = observation.object_pose_variance_(2);
                        pg_observation.observation_covariance_ = movable_obs_cov_mat;

                        pose_graph::MovableObservationFactor<2, double, 3> factor(node_num, pg_observation);
                        offline_problem_data.movable_observation_factors_.emplace_back(factor);
                    }
                    noisy_observations[observations_for_type.first].emplace_back(detections_at_node);
                }
            }

            // Add map observations to offline problem data -------------------------------------------------
            for (const auto &map_observations_by_type : past_movable_object_poses) {
                for (const std::pair<pose::Pose2d, double> &sample_pose_and_value : map_observations_by_type.second) {
                    pose_graph::MapObjectObservation<2, double> observation;
                    observation.semantic_class_ = map_observations_by_type.first;
                    observation.transl_ = sample_pose_and_value.first.first;
                    observation.orientation_ = sample_pose_and_value.first.second;
                    observation.obs_value_ = sample_pose_and_value.second;
                    offline_problem_data.map_object_observations_.emplace_back(observation);
                }
            }

            offline_optimization::OfflinePoseOptimizer<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3> offline_optimizer;
            return offline_optimizer.runOfflineOptimization(
                    offline_problem_data, pose_optimization_params,
                    pose_graph::utils::createFully2dPoseGraphFromParams,
                    std::bind(&SyntheticProblemRunner2d::createCeresIterationCallback, this, std::placeholders::_1,
                              std::placeholders::_2, noisy_observations),
                    std::bind(&SyntheticProblemRunner2d::runOptimizationVisualization, this,
                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                              ground_truth_trajectory, initial_node_positions, object_gt_poses.at(kCarClass),
                              past_movable_object_poses, noisy_observations));
        }

    private:


        const std::string kCarClass = "car";

        std::shared_ptr<visualization::VisualizationManager> vis_manager_;

        bool run_visualization_;
    };
}

#endif //AUTODIFF_GP_SYNTHETIC_PROBLEM_RUNNER_2D_H
