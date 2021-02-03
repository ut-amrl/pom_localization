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

namespace synthetic_problem {

    class SyntheticProblemRunner2d {
    public:


        typedef offline_optimization::OfflineProblemData<2, double, 3, 2, double> OfflineProblemDataType;

        SyntheticProblemRunner2d(const std::shared_ptr<visualization::VisualizationManager> vis_manager,
                                 bool run_visualization) : vis_manager_(vis_manager), run_visualization_(run_visualization) {}

        static std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> createPoseGraph(const pose_optimization::CostFunctionParameters &cost_function_params) {

            gp_kernel::GaussianKernel<2> position_kernel(cost_function_params.position_kernel_len_,
                                                         cost_function_params.position_kernel_var_);
            gp_kernel::PeriodicGaussianKernel<1> orientation_kernel(M_PI * 2, cost_function_params.orientation_kernel_var_,
                                                                    cost_function_params.orientation_kernel_len_);
            gp_kernel::Pose2dKernel pose_2d_kernel(position_kernel, orientation_kernel);
            return std::make_shared<pose_graph::PoseGraph2dMovObjDistribution2d>(pose_2d_kernel);
        }

        void runOptimizationVisualization(
                const pose_graph::NodeId &max_node_id,
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
                const offline_optimization::VisualizationTypeEnum &vis_stage,
                const std::vector<pose::Pose2d> &ground_truth_trajectory,
                const std::vector<pose::Pose2d> &unoptimized_trajectory,
                const std::vector<pose::Pose2d> &ground_truth_obj_poses,
                const std::vector<std::vector<pose::Pose2d>> &noisy_obj_observations) {
            if (run_visualization_) {
                switch (vis_stage) {
                    case offline_optimization::VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION:
                        vis_manager_->displayTrueTrajectory(ground_truth_trajectory);
                        vis_manager_->displayTrueCarPoses(ground_truth_obj_poses);
                        vis_manager_->displayNoisyCarPosesFromGt(ground_truth_trajectory, noisy_obj_observations);
                        vis_manager_->displayOdomTrajectory(unoptimized_trajectory);
                        vis_manager_->displayNoisyCarPosesFromOdomTrajectory(unoptimized_trajectory,
                                                                             noisy_obj_observations);

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
                        vis_manager_->displayNoisyCarPosesFromEstTrajectory(node_poses_list, noisy_obj_observations);
//                        ros::Duration(2).sleep();
                    }
                        break;
                    case offline_optimization::VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION:
                        // Optionally display distribution intensity map (either over robot poses or over object poses)
                    {
//                    std::string car_class = "car_class";
//                    vis_manager_->displayMaxGpRegressorOutput(pose_graph->getMovableObjKde(car_class), 0.1, -10.0, 15,
//                                                              -5, 20);
                    }
                        break;
                    default:
                        break;
                }
            }
        }

        ceres::IterationCallback* createCeresIterationCallback(
                const pose_graph::NodeId &node_id,
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
                const std::vector<std::vector<pose::Pose2d>> noisy_observations) {
            if (run_visualization_) {
                return new offline_optimization::CeresVisualizationCallback2d(
                        pose_graph, vis_manager_, node_id, noisy_observations);
            } else {
                return nullptr;
            }
        }

        std::unordered_map<pose_graph::NodeId, pose::Pose2d> runSyntheticProblem(const std::vector<pose::Pose2d> &ground_truth_trajectory,
                const std::unordered_map<std::string, std::vector<pose::Pose2d>> &movable_object_poses_by_class,
                const std::unordered_map<std::string, std::vector<pose::Pose2d>> &past_movable_object_poses,
                const SyntheticProblemNoiseConfig2d &noise_config,
                const pose_optimization::CostFunctionParameters &cost_function_params,
                const pose_optimization::PoseOptimizationParameters &pose_optimization_params) {

            OfflineProblemDataType offline_problem_data;

            // Compute true odometry and add noise to generate constraints ----------------------------------
            std::vector<pose::Pose2d> true_odometry;
            for (size_t i = 1; i < ground_truth_trajectory.size(); i++) {
                true_odometry.emplace_back(pose::getPoseOfObj1RelToObj2(ground_truth_trajectory[i],
                                                                        ground_truth_trajectory[i-1]));
            }

            util_random::Random random_generator;
            std::vector<pose::Pose2d> noisy_odometry;
            for (const pose::Pose2d &true_odom : true_odometry) {
                noisy_odometry.emplace_back(pose::addRelativeGaussianNoise(true_odom, noise_config.odometry_x_std_dev_,
                                                             noise_config.odometry_y_std_dev_,
                                                             noise_config.odometry_yaw_std_dev_,
                                                             random_generator));
            }

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
            pose::Pose2d prev_pose = pose::createPose2d(0, 0, 0);
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

            // For each pose in the trajectory, get the true position of the objects relative to the robot (for objects within
            // some radius)
            std::vector<pose::Pose2d> all_movable_obj_gt_poses;
            for (const auto &movable_obj_info_for_type : movable_object_poses_by_class) {
                for (const pose::Pose2d &obj_pose : movable_obj_info_for_type.second) {
                    all_movable_obj_gt_poses.emplace_back(obj_pose);
                }
            }

            std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> true_transforms_at_bot_poses;
            for (const auto &movable_obj_info_for_type : movable_object_poses_by_class) {
                true_transforms_at_bot_poses[movable_obj_info_for_type.first] = {};
                for (size_t i = 0; i < ground_truth_trajectory.size(); i++) {
                    std::vector<pose::Pose2d> true_obj_transforms;
                    for (const pose::Pose2d &obj_pose : movable_obj_info_for_type.second) {
                        if ((ground_truth_trajectory[i].first - obj_pose.first).norm()
                        <= noise_config.max_observable_moving_obj_distance_) {
                            true_obj_transforms.emplace_back(pose::getPoseOfObj1RelToObj2(obj_pose, ground_truth_trajectory[i]));
                        }
                    }
                    true_transforms_at_bot_poses[movable_obj_info_for_type.first].emplace_back(true_obj_transforms);
                }
            }

            std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> noisy_observations;
            for (const auto &true_transforms_for_type : true_transforms_at_bot_poses) {
                noisy_observations[true_transforms_for_type.first] = {};
                for (const std::vector<pose::Pose2d> &true_observations : true_transforms_for_type.second) {
                    std::vector<pose::Pose2d> noisy_at_pose;
                    for (const pose::Pose2d &true_obs : true_observations) {
                        pose::Pose2d noisy_obs = pose::addRelativeGaussianNoise(true_obs, noise_config.movable_observation_x_std_dev_,
                                                                  noise_config.movable_observation_y_std_dev_,
                                                                  noise_config.movable_observation_yaw_std_dev_,
                                                                  random_generator);
                        noisy_at_pose.emplace_back(noisy_obs);
                    }
                    noisy_observations[true_transforms_for_type.first].emplace_back(noisy_at_pose);
                }
            }

            Eigen::Matrix3d movable_obs_cov_mat;
            movable_obs_cov_mat(0, 0) = pow(noise_config.movable_observation_x_std_dev_, 2);
            movable_obs_cov_mat(1, 1) = pow(noise_config.movable_observation_y_std_dev_, 2);
            movable_obs_cov_mat(2, 2) = pow(noise_config.movable_observation_yaw_std_dev_, 2);
            for (const auto &noisy_observations_for_type : noisy_observations) {
                for (pose_graph::NodeId node_num = 0; node_num < noisy_observations_for_type.second.size(); node_num++) {
                    std::vector<pose::Pose2d> observations_at_node = noisy_observations_for_type.second[node_num];
                    for (const pose::Pose2d &observation : observations_at_node) {
                        pose_graph::MovableObservation<2, double, 3> pg_observation;
                        pg_observation.semantic_class_ = noisy_observations_for_type.first;
                        pg_observation.observation_transl_ = observation.first;
                        pg_observation.observation_orientation_ = observation.second;
                        pg_observation.observation_covariance_ = movable_obs_cov_mat;

                        pose_graph::MovableObservationFactor<2, double, 3> factor(node_num, pg_observation);
                        offline_problem_data.movable_observation_factors_.emplace_back(factor);
                    }
                }
            }

            // Add map observations to offline problem data -------------------------------------------------
            for (const auto &map_observations_by_type : past_movable_object_poses) {
                for (const pose::Pose2d &detected_pose : map_observations_by_type.second) {
                    pose_graph::MapObjectObservation<2, double> observation;
                    observation.semantic_class_ = map_observations_by_type.first;
                    observation.transl_ = detected_pose.first;
                    observation.orientation_ = detected_pose.second;
                    offline_problem_data.map_object_observations_.emplace_back(observation);
                }
            }

            std::vector<std::vector<pose::Pose2d>> noisy_observations_for_all_types;
            for (pose_graph::NodeId node_num = 0; node_num < initial_node_positions.size(); node_num++) {
                std::vector<pose::Pose2d> noisy_observations_at_node;
                for (const auto &noisy_obs_by_type : noisy_observations) {
                    noisy_observations_at_node.insert(noisy_observations_at_node.end(),
                                                      noisy_obs_by_type.second[node_num].begin(),
                                                      noisy_obs_by_type.second[node_num].end());
                }
                noisy_observations_for_all_types.emplace_back(noisy_observations_at_node);
            }

            offline_optimization::OfflinePoseOptimizer<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3> offline_optimizer;
            return offline_optimizer.runOfflineOptimization(
                    offline_problem_data, cost_function_params, pose_optimization_params,
                    SyntheticProblemRunner2d::createPoseGraph,
                    std::bind(&SyntheticProblemRunner2d::createCeresIterationCallback, this, std::placeholders::_1,
                              std::placeholders::_2, noisy_observations_for_all_types),
                    std::bind(&SyntheticProblemRunner2d::runOptimizationVisualization, this,
                              std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                              ground_truth_trajectory, initial_node_positions, all_movable_obj_gt_poses,
                              noisy_observations_for_all_types));
        }

    private:
        std::shared_ptr<visualization::VisualizationManager> vis_manager_;

        bool run_visualization_;
    };
}

#endif //AUTODIFF_GP_SYNTHETIC_PROBLEM_RUNNER_2D_H
