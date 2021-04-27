#include <glog/logging.h>
#include <pose_optimization/odometry_3d_cost_functor.h>
#include <pose_optimization/pose_3d_factor_graph.h>
#include <pose_optimization/pose_graph_optimizer.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <util/random.h>

#include <gaussian_process/kernel_density_estimator.h>

#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_2d.h>
#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_3d.h>
#include <synthetic_problem/synthetic_problem_runner_2d.h>

#include <base_lib/pose_reps.h>
#include <visualization/ros_visualization.h>
#include <pose_optimization/offline/offline_problem_runner.h>

#include <ros/ros.h>

#include <synthetic_problem/parking_lot_simulator.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <fstream>
#include <synthetic_problem/uncertainty_aware/uncertainty_aware_synthetic_problem.h>

using namespace pose;

pose::Pose2d addGaussianNoise(const pose::Pose2d &original_pose_2d, const double &x_std_dev,
                              const double &y_std_dev, const double &theta_std_dev,
                              util_random::Random &rand_gen) {
    return std::make_pair(Eigen::Vector2d(rand_gen.Gaussian(original_pose_2d.first.x(), x_std_dev),
                                          rand_gen.Gaussian(original_pose_2d.first.y(), y_std_dev)),
                          rand_gen.Gaussian(original_pose_2d.second, theta_std_dev));
}

// TODO sample angle from Gaussian in SO(3)
Pose3d addGaussianNoise(const Pose3d &original_pose_3d, const float &x_std_dev, const float &y_std_dev,
                        const float &z_std_dev, const float &yaw_std_dev, util_random::Random &rand_gen) {
    Eigen::Vector3d transl(rand_gen.Gaussian(original_pose_3d.first.x(), x_std_dev),
                           rand_gen.Gaussian(original_pose_3d.first.y(), y_std_dev),
                           rand_gen.Gaussian(original_pose_3d.first.z(), z_std_dev));
    Eigen::Quaterniond noise_rot;
    noise_rot = Eigen::AngleAxisd(rand_gen.Gaussian(0, yaw_std_dev), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rot = original_pose_3d.second * noise_rot;
    return std::make_pair(transl, rot);
}


std::vector<Pose2d> createGroundTruthPoses() {
    std::vector<Pose2d> poses;
    poses.emplace_back(createPose2d(0, 0, 0)); // Should test how this deals with non-zero-origins?
    poses.emplace_back(createPose2d(0.1, 1, 0));
    poses.emplace_back(createPose2d(0, 4, M_PI_2));
    poses.emplace_back(createPose2d(-.04, 7, M_PI_2));
    poses.emplace_back(createPose2d(0, 10, M_PI_2));
//    poses.emplace_back(createPose2d(0.3, 13, M_PI_2));
//    poses.emplace_back(createPose2d(0.7, 15, M_PI_4));
//    poses.emplace_back(createPose2d(2, 17, 0));
//    poses.emplace_back(createPose2d(4, 18, 0));
//    poses.emplace_back(createPose2d(7, 18, 0));
//    // TODO uncomment
//    poses.emplace_back(createPose2d(10, 17.5, 0));
//    poses.emplace_back(createPose2d(12, 15, -M_PI_4));
//    poses.emplace_back(createPose2d(12, 12, -M_PI_2));
//    poses.emplace_back(createPose2d(11.5, 9, -M_PI_2));
//    poses.emplace_back(createPose2d(11.7, 6, -M_PI_2));
//    poses.emplace_back(createPose2d(11.3, 3, -M_PI_2));
//    poses.emplace_back(createPose2d(11, -1, -(M_PI_2 + M_PI_4)));
//    poses.emplace_back(createPose2d(9, -0.0, M_PI));
//    poses.emplace_back(createPose2d(6, -0.9, M_PI));
//    poses.emplace_back(createPose2d(3, -0.7, M_PI));

//    poses.emplace_back(createPose2d(0, 0, 0)); // Should test how this deals with non-zero-origins?
//    poses.emplace_back(createPose2d(0.1, 1, 0));
//    poses.emplace_back(createPose2d(0, 4, 0));
//    poses.emplace_back(createPose2d(-.04, 7, 0));
//    poses.emplace_back(createPose2d(0, 10, 0));
//    poses.emplace_back(createPose2d(0.3, 13, 0));
//    poses.emplace_back(createPose2d(0.7, 15, 0));
//    poses.emplace_back(createPose2d(2, 17, 0));
//    poses.emplace_back(createPose2d(4, 18, 0));
//    poses.emplace_back(createPose2d(7, 18, 0));
//    poses.emplace_back(createPose2d(10, 17.5, 0));
//    poses.emplace_back(createPose2d(12, 15, 0));
//    poses.emplace_back(createPose2d(12, 12, 0));
//    poses.emplace_back(createPose2d(11.5, 9, 0));
//    poses.emplace_back(createPose2d(11.7, 6, 0));
//    poses.emplace_back(createPose2d(11.3, 3, 0));
//    poses.emplace_back(createPose2d(11, -1, 0));
//    poses.emplace_back(createPose2d(9, -0.0, 0));
//    poses.emplace_back(createPose2d(6, -0.9, 0));
//    poses.emplace_back(createPose2d(3, -0.7, 0));
    return poses;
}

std::vector<Pose2d> createHighLevelTrajectory() {
    std::vector<Pose2d> poses;
//    poses.emplace_back(pose::createPose2d(-18, -31, 3.0 * M_PI / 4.0 ));

    poses.emplace_back(pose::createPose2d(-12, -31, -M_PI));
    poses.emplace_back(pose::createPose2d(-15, -25, M_PI / 2.0));
    poses.emplace_back(pose::createPose2d(-15, 25, M_PI / 2.0));
//    poses.emplace_back(pose::createPose2d(-18, 31, M_PI / 4.0 ));
    poses.emplace_back(pose::createPose2d(-12, 31, 0));
    poses.emplace_back(pose::createPose2d(10, 31, 0));
//    poses.emplace_back(pose::createPose2d(18, 31, - M_PI / 4.0 ));
    poses.emplace_back(pose::createPose2d(12, 25, -M_PI / 2.0));
    poses.emplace_back(pose::createPose2d(12, -25, -M_PI / 2.0));
//    poses.emplace_back(pose::createPose2d(18, -31, -3.0 * M_PI / 4.0 ));
    poses.emplace_back(pose::createPose2d(10, -31, -M_PI));

////    poses.emplace_back(pose::createPose2d(-18, -31, 3.0 * M_PI / 4.0 ));
//    poses.emplace_back(pose::createPose2d(0, 0, 0));
//    poses.emplace_back(pose::createPose2d(0, -5, -M_PI_2));
//    poses.emplace_back(pose::createPose2d(0, -25, -M_PI_2));
//    poses.emplace_back(pose::createPose2d(-5, -31, -M_PI));
//    poses.emplace_back(pose::createPose2d(-18, -25,  M_PI / 2.0));
//    poses.emplace_back(pose::createPose2d(-18, -25,  M_PI / 2.0));
//    poses.emplace_back(pose::createPose2d(-18, 25,  M_PI / 2.0));
////    poses.emplace_back(pose::createPose2d(-18, 31, M_PI / 4.0 ));
//    poses.emplace_back(pose::createPose2d(-15, 31, 0 ));
//    poses.emplace_back(pose::createPose2d(15, 31, 0 ));
////    poses.emplace_back(pose::createPose2d(18, 31, - M_PI / 4.0 ));
//    poses.emplace_back(pose::createPose2d(18, 25,  -M_PI / 2.0));
//    poses.emplace_back(pose::createPose2d(18, -25,  -M_PI / 2.0));
////    poses.emplace_back(pose::createPose2d(18, -31, -3.0 * M_PI / 4.0 ));
//    poses.emplace_back(pose::createPose2d(15, -31, -M_PI ));
//    poses.emplace_back(pose::createPose2d(-15, -31, -M_PI));
    return poses;
}

std::vector<Pose2d>
createIntermediateTrajectory(std::vector<Pose2d> high_level_trajectory, double min_dist_between_nodes,
                             double max_transl_variation, double max_rot_variation) {
    std::vector<Pose2d> poses;
//    poses.emplace_back(high_level_trajectory[0]);

    Pose2d last_added_pose;
    double remaining_dist = 0;
    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
    for (size_t i = 0; i < high_level_trajectory.size(); i++) {
        // Pseudocode:
        // Get previous pose and next pose
        Pose2d prev_pose = high_level_trajectory[i];
        Pose2d next_pose;
        if (i == (high_level_trajectory.size() - 1)) {
            next_pose = high_level_trajectory[0];
        } else {
            next_pose = high_level_trajectory[i + 1];
        }

        // Using remaining distance from prev iter, construct a node that uses up what's left
        Eigen::Vector2d vector_to_second = next_pose.first - prev_pose.first;
        double vector_to_second_dist = vector_to_second.norm();
        Eigen::Vector2d unit_vector = vector_to_second / vector_to_second_dist;

        Eigen::Vector2d next_position = prev_pose.first + (remaining_dist * unit_vector);

        double angle_diff_prev_to_next = math_util::AngleDiff(next_pose.second, prev_pose.second);
        double angle_diff_for_next_pos = angle_diff_prev_to_next * (remaining_dist / vector_to_second_dist);
        double new_angle = math_util::AngleMod(prev_pose.second + angle_diff_for_next_pos);
        last_added_pose = createPose2d(next_position.x(), next_position.y(), new_angle);

//        LOG(INFO) << "Adding pose " << last_added_pose.first.x() << ", " << last_added_pose.first.y() << ", " << last_added_pose.second;
        pose::Pose2d imperfect_pose = createPose2d(
                last_added_pose.first.x() + random_generator.UniformRandom(-max_transl_variation, max_transl_variation),
                last_added_pose.first.y() + random_generator.UniformRandom(-max_transl_variation, max_transl_variation),
                last_added_pose.second + random_generator.UniformRandom(-max_rot_variation, max_rot_variation));
        poses.emplace_back(imperfect_pose);

        // Get the distance, vector, and angle change from that node to the next higher level trajectory node
        Eigen::Vector2d vector_from_last_added = next_pose.first - last_added_pose.first;
        double dist = vector_from_last_added.norm();
        Eigen::Vector2d unit_vector_to_second = vector_from_last_added / dist;
        double angle_diff_last_to_next = math_util::AngleDiff(next_pose.second, last_added_pose.second);

        // Determine how many full trajectory steps can fit
        int num_nodes = dist / min_dist_between_nodes;

        // Add the full trajectory steps, adding the distance specified in the direction of the unit vector to the last
        // added node
        // On the last full node, get the distance remaining to the next high level node.
        for (int j = 0; j < num_nodes; j++) {
            double angle_change = angle_diff_last_to_next * (min_dist_between_nodes / dist);
            double interpolated_angle = math_util::AngleMod(last_added_pose.second + angle_change);
            Pose2d new_pose = std::make_pair(last_added_pose.first + (unit_vector_to_second * min_dist_between_nodes),
                                             interpolated_angle);
            last_added_pose = new_pose;
            LOG(INFO) << "Adding pose " << last_added_pose.first.x() << ", " << last_added_pose.first.y() << ", "
                      << last_added_pose.second;
            imperfect_pose = createPose2d(last_added_pose.first.x() +
                                          random_generator.UniformRandom(-max_transl_variation, max_transl_variation),
                                          last_added_pose.first.y() +
                                          random_generator.UniformRandom(-max_transl_variation, max_transl_variation),
                                          last_added_pose.second +
                                          random_generator.UniformRandom(-max_rot_variation, max_rot_variation));
            poses.emplace_back(imperfect_pose);
            if (j == (num_nodes - 1)) {
                remaining_dist = min_dist_between_nodes - (next_pose.first - new_pose.first).norm();
            }
        }
    }
    return poses;
}

std::vector<Pose2d> createParkedCarPoses() {
    std::vector<Pose2d> poses;
    poses.emplace_back(createPose2d(-3, 2, -(M_PI * 5.0 / 6.0)));
    poses.emplace_back(createPose2d(-3, 4, -(M_PI * 5.0 / 6.0)));
//    poses.emplace_back(createPose2d(-7, 6, -(M_PI / 6.0)));

    poses.emplace_back(createPose2d(3, 2, -(M_PI / 6.0)));
    poses.emplace_back(createPose2d(3, 6, -(M_PI / 6.0)));
//    poses.emplace_back(createPose2d(3, 8, -(M_PI /6.0)));
//    poses.emplace_back(createPose2d(3, 14, -(M_PI /6.0)));

//    poses.emplace_back(createPose2d(7, 12, -(M_PI * 5.0 /6.0)));
//    poses.emplace_back(createPose2d(7, 10, -(M_PI * 5.0/6.0)));
    poses.emplace_back(createPose2d(7, 6, -(M_PI * 5.0 / 6.0)));
    poses.emplace_back(createPose2d(7, 2, -(M_PI * 5.0 / 6.0)));
    return poses;
}

std::vector<std::pair<Pose2d, unsigned int>> createParkedCarPosesWithFrequency() {
    std::vector<std::pair<Pose2d, unsigned int>> poses;
//     TODO uncomment
    poses.emplace_back(std::make_pair(createPose2d(-3, 2, -(M_PI * 5.0 / 6.0)), 10));
    poses.emplace_back(std::make_pair(createPose2d(-3, 4, -(M_PI * 5.0 / 6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(-7, 6, -(M_PI / 6.0)), 2));

    poses.emplace_back(std::make_pair(createPose2d(3, 2, -(M_PI / 6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(3, 6, -(M_PI / 6.0)), 3));
    poses.emplace_back(std::make_pair(createPose2d(3, 8, -(M_PI / 6.0)), 2));
    poses.emplace_back(std::make_pair(createPose2d(3, 14, -(M_PI / 6.0)), 1));

    poses.emplace_back(std::make_pair(createPose2d(7, 12, -(M_PI * 5.0 / 6.0)), 3));
    poses.emplace_back(std::make_pair(createPose2d(7, 10, -(M_PI * 5.0 / 6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(7, 6, -(M_PI * 5.0 / 6.0)), 2));
    poses.emplace_back(std::make_pair(createPose2d(7, 2, -(M_PI * 5.0 / 6.0)), 2));
    return poses;
}

synthetic_problem::ObjectPlacementConfiguration<pose::Pose2d, 3>
createCarPlacementConfiguration(const Eigen::Vector3d &object_pose_variance,
                                const std::vector<std::pair<Pose2d, unsigned int>> &parking_spots_and_frequency) {
    synthetic_problem::ObjectPlacementConfiguration<pose::Pose2d, 3> obj_placement_config;
    for (const std::pair<pose::Pose2d, unsigned int> &parking_spot_and_freq : parking_spots_and_frequency) {
        synthetic_problem::ObjectOccurrenceParams<pose::Pose2d, 3> object_occurrence;
        object_occurrence.object_pose_variance_ = object_pose_variance;
        object_occurrence.object_canonical_pose_ = parking_spot_and_freq.first;
        object_occurrence.relative_frequency_ = parking_spot_and_freq.second;
        obj_placement_config.obj_placement_config_.emplace_back(object_occurrence);
    }
    return obj_placement_config;
}

std::vector<Eigen::Vector2d> createNegativeObservations() {
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> exclude_regions;
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2d(-9, 0), Eigen::Vector2d(1, 10)));
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2d(1, 0), Eigen::Vector2d(9, 16)));

    std::vector<Eigen::Vector2d> negative_observations;
    for (int x = -15; x <= 20; x++) {
        for (int y = -5; y <= 25; y++) {
            bool in_exclude_region = false;
            for (const std::pair<Eigen::Vector2d, Eigen::Vector2d> &exclude_region : exclude_regions) {
                double x_min = exclude_region.first.x();
                double x_max = exclude_region.second.x();
                double y_min = exclude_region.first.y();
                double y_max = exclude_region.second.y();

                if ((y >= y_min) && (y <= y_max) && (x >= x_min) && (x <= x_max)) {
                    in_exclude_region = true;
                    break;
                }
            }

            if (!in_exclude_region) {
                negative_observations.emplace_back(Eigen::Vector2d(x, y));
            }
        }
    }
    return negative_observations;
}

//void outputToCsv(const std::string &file_name, const std::vector<Pose2d> &poses) {
//// TODO
//}

std::unordered_map<pose_graph::NodeId, pose::Pose2d> callSyntheticProblem(
        const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
        const synthetic_problem::ParkingLotConfigurationParams &parking_lot_configuration_params,
        const synthetic_problem::SyntheticProblemNoiseConfig2d &noise_config,
        const std::vector<pose::Pose2d> &ground_truth_trajectory,
        bool show_visualization,
        const pose_optimization::PoseOptimizationParameters &optimization_params) {

//    LOG(INFO) << "Setting up synthetic problem";
    synthetic_problem::SyntheticProblemRunner2d synthetic_prob_runner(vis_manager, show_visualization);

    std::vector<pose::Pose2d> past_car_poses;
    std::vector<pose::Pose2d> current_car_poses;
    synthetic_problem::createPastAndPresentObservations(
            parking_lot_configuration_params.parking_spots_and_relative_frequency_,
            parking_lot_configuration_params.parking_lot_std_dev_x_,
            parking_lot_configuration_params.parking_lot_std_dev_y_,
            parking_lot_configuration_params.parking_lot_std_dev_yaw_,
            parking_lot_configuration_params.num_samples_multiplier_,
            parking_lot_configuration_params.parking_lot_percent_filled_,
            past_car_poses, current_car_poses);

    std::string car_class = "car";
    std::unordered_map<std::string, std::vector<pose::Pose2d>> past_mov_obj_positions_by_class = {{car_class, past_car_poses}};
    std::unordered_map<std::string, std::vector<pose::Pose2d>> curr_mov_obj_positions_by_class = {{car_class, current_car_poses}};

    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());

    // TODO move back to synthetic problem?
    // Compute true odometry and add noise to generate constraints ----------------------------------
    std::vector<pose::Pose2d> true_odometry;
    for (size_t i = 1; i < ground_truth_trajectory.size(); i++) {
        true_odometry.emplace_back(pose::getPoseOfObj1RelToObj2(ground_truth_trajectory[i],
                                                                ground_truth_trajectory[i - 1]));
    }

//            util_random::Random random_generator;
    std::vector<pose::Pose2d> noisy_odometry;
    for (const pose::Pose2d &true_odom : true_odometry) {
        noisy_odometry.emplace_back(pose::addRelativeGaussianNoise(true_odom, noise_config.odometry_x_std_dev_,
                                                                   noise_config.odometry_y_std_dev_,
                                                                   noise_config.odometry_yaw_std_dev_,
                                                                   random_generator));
    }

    return synthetic_prob_runner.runSyntheticProblem(
            ground_truth_trajectory,
            noisy_odometry,
            curr_mov_obj_positions_by_class,
            {},// TODO make this different
//            past_mov_obj_positions_by_class, //mov_obj_positions_by_class,
            {}, // TODO
            noise_config,
            optimization_params,
            random_generator);
}

double computeATE(const std::unordered_map<pose_graph::NodeId, pose::Pose2d> &ground_truth_trajectory,
                  const std::unordered_map<pose_graph::NodeId, pose::Pose2d> &optimized_trajectory) {
    int num_poses = 0;
    double squared_error_sum = 0;
    for (const auto &ground_truth_info : ground_truth_trajectory) {
        if (optimized_trajectory.find(ground_truth_info.first) != optimized_trajectory.end()) {
            pose::Pose2d optimized = optimized_trajectory.at(ground_truth_info.first);
            pose::Pose2d ground_truth = ground_truth_info.second;
            squared_error_sum += (optimized.first - ground_truth.first).squaredNorm();
            num_poses++;
        } else {
            LOG(INFO) << "Node " << ground_truth_info.first << " in ground truth not in optimized trajectory";
        }
    }
    if (num_poses == 0) {
        return 0;
    }
    return squared_error_sum / num_poses;
}

double runSingleSyntheticProblem(const std::shared_ptr<visualization::VisualizationManager> &vis_manager) {
    synthetic_problem::ParkingLotSpacingConfigurationParams parking_lot_spacing_config;
    parking_lot_spacing_config.seed_spot_x = 3.0;
    parking_lot_spacing_config.seed_spot_y = 3.0;
    parking_lot_spacing_config.min_x = -20;
    parking_lot_spacing_config.max_x = 20;
    parking_lot_spacing_config.min_y = -30;
    parking_lot_spacing_config.max_y = 30;
    parking_lot_spacing_config.parking_lot_spacing_x = 4;
    parking_lot_spacing_config.parking_lot_spacing_y = 3;
    parking_lot_spacing_config.parking_lot_aisle_gap_x = 8;
    parking_lot_spacing_config.parking_lot_aisle_gap_y = 6;
    parking_lot_spacing_config.spots_between_gaps_y = 5;
    parking_lot_spacing_config.right_parking_angle = -(M_PI / 6.0);
    parking_lot_spacing_config.left_parking_angle = -(5.0 * M_PI / 6.0);
    parking_lot_spacing_config.max_frequency_ratio = 10;

    synthetic_problem::ParkingLotConfigurationParams parking_lot_config;
    parking_lot_config.parking_lot_std_dev_x_ = 0.4;
    parking_lot_config.parking_lot_std_dev_y_ = 0.4;
    parking_lot_config.parking_lot_std_dev_yaw_ = 0.15;
    parking_lot_config.num_samples_multiplier_ = 4;
    parking_lot_config.parking_lot_percent_filled_ = 0.6;
//    parking_lot_config.parking_spots_and_relative_frequency_ = createParkedCarPosesWithFrequency();
    parking_lot_config.parking_spots_and_relative_frequency_ = synthetic_problem::generateParkingSpotsAndFrequency(
            parking_lot_spacing_config);

    synthetic_problem::SyntheticProblemNoiseConfig2d noise_config;
    noise_config.add_additional_initial_noise_ = false;
    noise_config.odometry_x_std_dev_ = 0.15;
    noise_config.odometry_y_std_dev_ = 0.15;
    noise_config.odometry_yaw_std_dev_ = 0.15;
    noise_config.max_observable_moving_obj_distance_ = 8.0;
    noise_config.movable_observation_x_std_dev_ = 0.1;
    noise_config.movable_observation_y_std_dev_ = 0.1;
    noise_config.movable_observation_yaw_std_dev_ = 0.1;

    pose_optimization::CostFunctionParameters cost_function_params;

    pose_optimization::PoseOptimizationParameters optimization_params;
    optimization_params.cost_function_params_ = cost_function_params;

//    std::vector<pose::Pose2d> ground_truth_poses = createGroundTruthPoses();
    std::vector<pose::Pose2d> high_level_trajectory = createHighLevelTrajectory();
    LOG(INFO) << "High level trajectory size " << high_level_trajectory.size();
//    std::vector<pose::Pose2d> ground_truth_poses = createIntermediateTrajectory(high_level_trajectory, 1, 0.125, 0.125);

    std::vector<pose::Pose2d> ground_truth_poses = createIntermediateTrajectory(high_level_trajectory, 1, 0.2, 0.4);
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> ground_truth_poses_as_map;
    for (size_t i = 0; i < ground_truth_poses.size(); i++) {
        ground_truth_poses_as_map[i] = ground_truth_poses[i];
    }
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = callSyntheticProblem(
            vis_manager, parking_lot_config, noise_config, ground_truth_poses, true, optimization_params);
    return computeATE(ground_truth_poses_as_map, optimization_results);
}


void outputResultsHeader(const std::string &file_name) {
    std::ofstream csv_file(file_name, std::ios::app);
    csv_file << "position_kernel_len" << ", " << "orientation_kernel_len"
             << ", " << "odometry_x_std_dev" << ", " << "odometry_y_std_dev" << ", "
             << "max_observable_moving_obj_distance" << ", "
             << "odometry_yaw_std_dev" << ", " << "movable_observation_x_std_dev" << ", "
             << "movable_observation_y_std_dev" << ", "
             << "movable_observation_yaw_std_dev" << ", "
             << "parking_lot_std_dev_x" << ", "
             << "parking_lot_std_dev_y" << ", "
             << "parking_lot_std_dev_yaw" << ", "
             << "parking_lot_percent_filled" << ", " << "absolute_trajectory_error" << "\n";
    csv_file.close();
}

void outputResults(const std::string &file_name,
                   const synthetic_problem::ParkingLotConfigurationParams &parking_lot_configuration_params,
                   const synthetic_problem::SyntheticProblemNoiseConfig2d &noise_config,
                   const pose_optimization::PoseOptimizationParameters &optimization_params,
                   const double &absolute_trajectory_error) {
    // TODO
    std::ofstream csv_file(file_name, std::ios::app);
    csv_file << optimization_params.cost_function_params_.mean_position_kernel_len_
             << ", " << optimization_params.cost_function_params_.mean_orientation_kernel_len_
             << ", " << noise_config.odometry_x_std_dev_ << ", " << noise_config.odometry_y_std_dev_ << ", "
             << noise_config.max_observable_moving_obj_distance_ << ", "
             << noise_config.odometry_yaw_std_dev_ << ", " << noise_config.movable_observation_x_std_dev_ << ", "
             << noise_config.movable_observation_y_std_dev_ << ", "
             << noise_config.movable_observation_yaw_std_dev_ << ", "
             << parking_lot_configuration_params.parking_lot_std_dev_x_ << ", "
             << parking_lot_configuration_params.parking_lot_std_dev_y_ << ", "
             << parking_lot_configuration_params.parking_lot_std_dev_yaw_ << ", "
             << parking_lot_configuration_params.parking_lot_percent_filled_ << ", " << absolute_trajectory_error
             << "\n";
    csv_file.close();
}

void runSyntheticProblemAndOutputResults(
        const std::string &results_file_name,
        const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
        const synthetic_problem::ParkingLotConfigurationParams &parking_lot_configuration_params,
        const synthetic_problem::SyntheticProblemNoiseConfig2d &noise_config,
        const std::vector<pose::Pose2d> &ground_truth_trajectory,
        const pose_optimization::PoseOptimizationParameters &optimization_params) {
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results = callSyntheticProblem(
            vis_manager, parking_lot_configuration_params, noise_config, ground_truth_trajectory, true,
            optimization_params);
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> ground_truth_poses_as_map;
    for (size_t i = 0; i < ground_truth_trajectory.size(); i++) {
        ground_truth_poses_as_map[i] = ground_truth_trajectory[i];
    }
    double absolute_trajectory_error = computeATE(ground_truth_poses_as_map, optimization_results);
    outputResults(results_file_name, parking_lot_configuration_params, noise_config,
                  optimization_params, absolute_trajectory_error);
}


void runSyntheticProblemWithConfigVariations(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                             const std::vector<std::pair<pose::Pose2d, unsigned int>> &parking_spots_and_relative_frequency,
                                             const std::vector<pose::Pose2d> &ground_truth_trajectory,
                                             const std::string &results_file_name) {

    outputResultsHeader(results_file_name);

    synthetic_problem::ParkingLotConfigurationParams parking_lot_configuration_params;
    parking_lot_configuration_params.parking_spots_and_relative_frequency_ = parking_spots_and_relative_frequency;

    synthetic_problem::SyntheticProblemNoiseConfig2d noise_config;
    pose_optimization::CostFunctionParameters cost_function_params;

    noise_config.max_observable_moving_obj_distance_ = 8.0;
    parking_lot_configuration_params.num_samples_multiplier_ = 5.0;
    noise_config.add_additional_initial_noise_ = false;

    std::vector<double> position_kernel_len_opts = {cost_function_params.mean_position_kernel_len_};
    std::vector<double> orientation_kernel_len_opts = {cost_function_params.mean_orientation_kernel_len_};

    pose_optimization::PoseOptimizationParameters optimization_params;
    optimization_params.cost_function_params_ = cost_function_params;

//    std::vector<double> odom_transl_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005};
//    std::vector<double> odom_rot_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005};
//    std::vector<double> obs_transl_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> obs_rot_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> parking_spot_transl_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> parking_spot_rot_std_dev_opts = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};

//    std::vector<double> odom_transl_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005};
//    std::vector<double> odom_rot_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005};
//    std::vector<double> obs_transl_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> obs_rot_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> parking_spot_transl_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};
//    std::vector<double> parking_spot_rot_std_dev_opts = {0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05, 0.01, 0.005, 0.001};

//    std::vector<double> odom_transl_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};
//    std::vector<double> odom_rot_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};
//    std::vector<double> obs_transl_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};
//    std::vector<double> obs_rot_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};
//    std::vector<double> parking_spot_transl_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};
//    std::vector<double> parking_spot_rot_std_dev_opts = {0.005, 0.01, 0.025, 0.05, 0.075, 0.1, 0.2, 0.3};


    std::vector<double> odom_transl_std_dev_opts = {0.1};
    std::vector<double> odom_rot_std_dev_opts = {0.15};
    std::vector<double> obs_transl_std_dev_opts = {0.01};
    std::vector<double> obs_rot_std_dev_opts = {0.01};
    std::vector<double> parking_spot_transl_std_dev_opts = {0.1};
    std::vector<double> parking_spot_rot_std_dev_opts = {0.05};
    std::vector<double> percent_spots_filled_opts = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    for (const double &position_kernel_len : position_kernel_len_opts) {
        LOG(INFO) << "Setting position_kernel_len to " << position_kernel_len;
        cost_function_params.mean_position_kernel_len_ = position_kernel_len;
        for (const double &orientation_kernel_len : position_kernel_len_opts) {
            LOG(INFO) << "Setting orientation_kernel_len to " << orientation_kernel_len;
            cost_function_params.mean_orientation_kernel_len_ = orientation_kernel_len;
            for (const double &odom_transl_std_dev : odom_transl_std_dev_opts) {
                LOG(INFO) << "Setting odom_transl_std_dev to " << odom_transl_std_dev;
                noise_config.odometry_x_std_dev_ = odom_transl_std_dev;
                noise_config.odometry_y_std_dev_ = odom_transl_std_dev;
                for (const double &odom_rot_std_dev : odom_rot_std_dev_opts) {
                    LOG(INFO) << "Setting odom_rot_std_dev to " << odom_rot_std_dev;
                    noise_config.odometry_yaw_std_dev_ = odom_rot_std_dev;
                    for (const double &obs_transl_std_dev : obs_transl_std_dev_opts) {
                        LOG(INFO) << "Setting obs_transl_std_dev to " << obs_transl_std_dev;
                        noise_config.movable_observation_x_std_dev_ = obs_transl_std_dev;
                        noise_config.movable_observation_y_std_dev_ = obs_transl_std_dev;
                        for (const double &obs_rot_std_dev : obs_rot_std_dev_opts) {
                            LOG(INFO) << "Setting obs_rot_std_dev to " << obs_rot_std_dev;
                            noise_config.movable_observation_yaw_std_dev_ = obs_rot_std_dev;
                            for (const double &parking_spot_transl_std_dev : parking_spot_transl_std_dev_opts) {
                                LOG(INFO) << "Setting parking_spot_transl_std_dev to " << parking_spot_transl_std_dev;
                                parking_lot_configuration_params.parking_lot_std_dev_x_ = parking_spot_transl_std_dev;
                                parking_lot_configuration_params.parking_lot_std_dev_y_ = parking_spot_transl_std_dev;
                                for (const double &parking_spot_rot_std_dev : parking_spot_rot_std_dev_opts) {
                                    LOG(INFO) << "Setting parking_spot_rot_std_dev to " << parking_spot_rot_std_dev;
                                    parking_lot_configuration_params.parking_lot_std_dev_yaw_ = parking_spot_rot_std_dev;
                                    for (const double &percent_spots_filled : percent_spots_filled_opts) {
                                        LOG(INFO) << "Setting percent_spots_filled to " << percent_spots_filled;
                                        parking_lot_configuration_params.parking_lot_percent_filled_ = percent_spots_filled;
                                        runSyntheticProblemAndOutputResults(
                                                results_file_name, vis_manager, parking_lot_configuration_params,
                                                noise_config, ground_truth_trajectory, optimization_params);
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

//    void runSyntheticProblemAndOutputResults(
//            const std::string &results_file_name,
//            const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
//            const synthetic_problem::ParkingLotConfigurationParams &parking_lot_configuration_params,
//            const synthetic_problem::SyntheticProblemNoiseConfig2d &noise_config,
//            const std::vector<pose::Pose2d> &ground_truth_trajectory,
//            const pose_optimization::CostFunctionParameters &cost_function_params,
//            const pose_optimization::PoseOptimizationParameters &optimization_params) {
}

double runSyntheticProblemWithUncertainty(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                          const unsigned int &num_prev_trajectories) {

    // Synthetic problem config -------------------------------------------------------------------
    std::string car_class = "car";

    synthetic_problem::SyntheticProblemNoiseConfig2d noise_config;
    noise_config.add_additional_initial_noise_ = false;
    noise_config.odometry_x_std_dev_ = 0.15;
    noise_config.odometry_y_std_dev_ = 0.15;
    noise_config.odometry_yaw_std_dev_ = 0.15;
    noise_config.max_observable_moving_obj_distance_ = 8.0;
    noise_config.movable_observation_x_std_dev_ = 0.1;
    noise_config.movable_observation_y_std_dev_ = 0.1;
    noise_config.movable_observation_yaw_std_dev_ = 0.1;

    pose_optimization::CostFunctionParameters cost_function_params;
    cost_function_params.mean_position_kernel_len_ = 0.5;
    cost_function_params.mean_orientation_kernel_len_ = 0.4; // TODO fix
    cost_function_params.mean_orientation_kernel_len_ = 10000;
//    cost_function_params.mean_position_kernel_var_ = 30;
//    cost_function_params.mean_orientation_kernel_var_ = 30;

    cost_function_params.mean_position_kernel_var_ = 9;
    cost_function_params.mean_orientation_kernel_var_ = 1;


    cost_function_params.default_obj_probability_input_variance_for_mean_ = 10;

    cost_function_params.var_position_kernel_len_ = 1.8;
    cost_function_params.var_orientation_kernel_len_ = 1000;

    cost_function_params.var_position_kernel_var_ = 100;
    cost_function_params.var_orientation_kernel_var_ = 100;

    cost_function_params.default_obj_probability_input_variance_for_var_ = 10;
    pose_optimization::PoseOptimizationParameters pose_optimization_params;
    pose_optimization_params.cost_function_params_ = cost_function_params;

    // TODO consider moving out to some config
    Eigen::Vector3d trajectory_pose_variance;
    trajectory_pose_variance(0) = 0.75;
    trajectory_pose_variance(1) = 0.75;
    trajectory_pose_variance(2) = 0.75;

    synthetic_problem::ObjectPlacementConfigurationAllClasses<pose::Pose2d, 3> object_configurations_for_all_classes;
//    Eigen::Vector3d obj_occurrence_variance = Eigen::Vector3d(0.00005, 0.00005, 0.00005);
    Eigen::Vector3d obj_occurrence_variance = Eigen::Vector3d(0.005, 0.005, 0.005);
    object_configurations_for_all_classes.obj_placement_configs_by_class_[car_class] = createCarPlacementConfiguration(
            obj_occurrence_variance, createParkedCarPosesWithFrequency());

    for (const auto &entry : object_configurations_for_all_classes.obj_placement_configs_by_class_) {
        LOG(INFO) << "Config size for class " << entry.first << ": " << entry.second.obj_placement_config_.size();
    }

    std::unordered_map<std::string, std::pair<double, double>> valid_percent_filled_range;
    valid_percent_filled_range[car_class] = std::make_pair(0.3, 0.8);
//    valid_percent_filled_range[car_class] = std::make_pair(1.0, 1.0);

    // TODO make more noisy
    Eigen::Vector3d object_detection_variance_per_detection_len;
    object_detection_variance_per_detection_len(0) = 0.005;
    object_detection_variance_per_detection_len(1) = 0.005;
    object_detection_variance_per_detection_len(2) = 0.005;

    double max_obj_detection_dist = 10.0;
    synthetic_problem::ScanGenerationParams2d scan_gen_params;
    scan_gen_params.min_angle_ = -0.75 * M_PI;
    scan_gen_params.max_angle_ = 0.75 * M_PI;
//    scan_gen_params.max_range_ = 20.0;
    scan_gen_params.max_range_ = 10.0;
    scan_gen_params.min_range_ = 0.2;
    scan_gen_params.num_beams_ = 100; // TODO should probably increase this to be more realistic eventually

    std::unordered_map<std::string, pose_optimization::SampleGeneratorParams2d> sample_gen_params_by_class;
    sample_gen_params_by_class[car_class].num_samples_per_beam_ = 1;
    // TODO increase this
    sample_gen_params_by_class[car_class].percent_beams_per_scan_ = 0.05;
    sample_gen_params_by_class[car_class].percent_poses_to_include_ = 1.0;

//    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
    util_random::Random random_generator;

    // TODO replace with something more complicated once this is working
    std::vector<pose::Pose2d> ground_truth_trajectory = createGroundTruthPoses();

    // Synthetic problem setup / execution -------------------------------------------------------------------
    synthetic_problem::SyntheticProblemRunner2d synthetic_prob_runner(vis_manager, true);

    // Compute true odometry and add noise to generate constraints ----------------------------------
    std::vector<pose::Pose2d> true_odometry;
    for (size_t i = 1; i < ground_truth_trajectory.size(); i++) {
        true_odometry.emplace_back(pose::getPoseOfObj1RelToObj2(ground_truth_trajectory[i],
                                                                ground_truth_trajectory[i - 1]));
    }

//            util_random::Random random_generator;
    std::vector<pose::Pose2d> noisy_odometry;
    for (const pose::Pose2d &true_odom : true_odometry) {
        noisy_odometry.emplace_back(pose::addRelativeGaussianNoise(true_odom, noise_config.odometry_x_std_dev_,
                                                                   noise_config.odometry_y_std_dev_,
                                                                   noise_config.odometry_yaw_std_dev_,
                                                                   random_generator));
    }


    std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>> gt_object_placements = synthetic_problem::getObjectInstantiationsFromConfiguration(
            object_configurations_for_all_classes, num_prev_trajectories + 1, valid_percent_filled_range,
            random_generator);

    LOG(INFO) << "Past placements ";
    std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>> past_object_placements(
            gt_object_placements.begin() + 1, gt_object_placements.end());
    for (const std::unordered_map<std::string, std::vector<pose::Pose2d>> &past_objs : past_object_placements) {
        for (const pose::Pose2d &pose : past_objs.at(car_class)) {
            LOG(INFO) << pose.first.x() << ", " << pose.first.y() << ", " << pose.second;
        }
    }

    std::unordered_map<std::string, std::vector<pose::Pose2d>> object_gt_poses = gt_object_placements.front();
    LOG(INFO) << "Object gt pose size: " << object_gt_poses[car_class].size();

    // Make other trajectories to give historical data
    std::vector<std::vector<pose::Pose2d>> past_trajectories =
            synthetic_problem::generateSimilarTrajectories(ground_truth_trajectory, trajectory_pose_variance,
                                                           num_prev_trajectories, random_generator);

    std::vector<std::vector<std::pair<pose::Pose2d, SensorInfo<pose::Pose2d, 3, sensor_msgs::LaserScan>>>> sensor_info_for_past_trajectories =
            synthetic_problem::generateSensorInfoForTrajectories<pose::Pose2d, 3, sensor_msgs::LaserScan, synthetic_problem::ScanGenerationParams2d>(
                    past_trajectories, past_object_placements, object_detection_variance_per_detection_len,
                    max_obj_detection_dist, scan_gen_params, random_generator);

    std::function<double(
            const double &)> pdf_squashing_function = pose_optimization::squashPdfValueToZeroToOneRangeExponential;
    std::function<double(const pose::Pose2d &,
                         const std::vector<ObjectDetectionRelRobot<pose::Pose2d, 3>> &)> sample_value_generator =
            std::bind(&pose_optimization::computeValueForSample<pose::Pose2d, 3>, std::placeholders::_1,
                      std::placeholders::_2, pdf_squashing_function);

    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories = generateSamplesForPastTrajectories(
            sensor_info_for_past_trajectories, sample_value_generator,
            sample_gen_params_by_class,
            random_generator);

    std::unordered_map<std::string, std::vector<std::vector<pose_optimization::ObjectDetectionRelRobot<pose::Pose2d, 3>>>> movable_object_detections =
            synthetic_problem::generateObjectDetectionsForTrajectory(
                    ground_truth_trajectory, max_obj_detection_dist, object_gt_poses,
                    object_detection_variance_per_detection_len, random_generator);

    LOG(INFO) << "Running synthetic problem";
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> optimization_results =
            synthetic_prob_runner.runSyntheticProblem(ground_truth_trajectory,
                                                      noisy_odometry, // TODO remove?
                                                      object_gt_poses,
                                                      movable_object_detections, samples_for_prev_trajectories,
                                                      noise_config, pose_optimization_params, random_generator);
    std::unordered_map<pose_graph::NodeId, pose::Pose2d> ground_truth_poses_as_map;
    for (size_t i = 0; i < ground_truth_trajectory.size(); i++) {
        ground_truth_poses_as_map[i] = ground_truth_trajectory[i];
    }
    return computeATE(ground_truth_poses_as_map, optimization_results);
}

void GPCTest() {

    // Rows = 3
    // Cols = # samples

    int larger_sample_multiplier = 10;
    int smaller_num_samples = 25;
    int larger_num_samples = larger_sample_multiplier * smaller_num_samples;
    Eigen::MatrixXd larger_inputs = Eigen::MatrixXd(3, larger_num_samples);
    Eigen::MatrixXd larger_outputs = Eigen::MatrixXd(1, larger_num_samples);

    Eigen::MatrixXd smaller_inputs = Eigen::MatrixXd(3, smaller_num_samples);
    Eigen::MatrixXd smaller_outputs = Eigen::MatrixXd(1, smaller_num_samples);

    util_random::Random random_generator;
    for (int i = 0; i < smaller_num_samples; i++) {
        smaller_inputs(0, i) = random_generator.UniformRandom(0, 1);
        smaller_inputs(1, i) = random_generator.UniformRandom(0, 1);
        smaller_inputs(2, i) = random_generator.UniformRandom(0, 2 * M_PI);
        smaller_outputs(0, i) = random_generator.UniformRandom(0, 1);
        LOG(INFO) << "Sample " << i;
        LOG(INFO) << smaller_inputs(0, i);
        LOG(INFO) << smaller_inputs(1, i);
        LOG(INFO) << smaller_inputs(2, i);
        LOG(INFO) << smaller_outputs(0, i);

        for (int j = 0; j < larger_sample_multiplier; j++) {
            larger_inputs.col(larger_sample_multiplier * i + j) = smaller_inputs.col(i);
            larger_outputs.col(larger_sample_multiplier * i + j) = smaller_outputs.col(i);
        }
    }

    LOG(INFO) << "Smaller inputs";
    LOG(INFO) << smaller_inputs;
    LOG(INFO) << "Smaller outputs";
    LOG(INFO) << smaller_outputs;

//    cost_function_params.mean_position_kernel_len_ = 0.5;
//    cost_function_params.mean_orientation_kernel_len_ = 10000;
//    cost_function_params.mean_position_kernel_var_ = 30;
//    cost_function_params.mean_orientation_kernel_var_ = 30;

//    cost_function_params.mean_position_kernel_var_ = 9;
//    cost_function_params.mean_orientation_kernel_var_ = 1;


//    cost_function_params.default_obj_probability_input_variance_for_mean_ = 10;

//    cost_function_params.var_position_kernel_len_ = 1.8;
//    cost_function_params.var_orientation_kernel_len_ = 1000;

//    cost_function_params.var_position_kernel_var_ = 100;
//    cost_function_params.var_orientation_kernel_var_ = 100;

//    cost_function_params.default_obj_probability_input_variance_for_var_ = 10;

    // TODO set values for these
    double prior_mean = 0.1;
    double input_variance_for_mean = 10;
    double mean_pos_kernel_len = 0.5;
    double mean_pos_kernel_var = 9;
    double mean_orientation_kernel_len = 10000;
    double mean_orientation_kernel_var = 1;

    double var_pos_kernel_len = 1.8;
    double var_orientation_kernel_len = 1000;

    double input_variance_for_var = 10;
    double var_pos_kernel_var = 100;
    double var_orientation_kernel_var = 100;

    double input_var_for_var_2 = 10;
    double var_pos_kernel_var_2 = 10000;
    double var_orientation_kernel_var_2 = 10000;

    double input_var_for_var_3 = 10;
    double var_pos_kernel_var_3 = 5000;
    double var_orientation_kernel_var_3 = 5000;

    gp_kernel::GaussianKernel<2> mean_position_kernel(mean_pos_kernel_len, mean_pos_kernel_var);
    gp_kernel::PeriodicGaussianKernel<1> mean_orientation_kernel(M_PI * 2,
                                                                 mean_orientation_kernel_var,
                                                                 mean_orientation_kernel_len);
    gp_kernel::Pose2dKernel mean_pose_2d_kernel(mean_position_kernel, mean_orientation_kernel);


    gp_kernel::GaussianKernel<2> var_position_kernel(var_pos_kernel_len,
                                                     var_pos_kernel_var);
    gp_kernel::PeriodicGaussianKernel<1> var_orientation_kernel(M_PI * 2,
                                                                var_orientation_kernel_var,
                                                                var_orientation_kernel_len);
    gp_kernel::Pose2dKernel var_pose_2d_kernel(var_position_kernel, var_orientation_kernel);

    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    larger_inputs, larger_outputs,
                    prior_mean, input_variance_for_mean,
                    input_variance_for_var, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel);

    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc_smaller_samples =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    smaller_inputs, smaller_outputs,
                    prior_mean, input_variance_for_mean,
                    input_variance_for_var, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel);



    gp_kernel::GaussianKernel<2> var_position_kernel_2(var_pos_kernel_len,
                                                     var_pos_kernel_var_2);
    gp_kernel::PeriodicGaussianKernel<1> var_orientation_kernel_2(M_PI * 2,
                                                                var_orientation_kernel_var_2,
                                                                var_orientation_kernel_len);
    gp_kernel::Pose2dKernel var_pose_2d_kernel_2(var_position_kernel_2, var_orientation_kernel_2);

    LOG(INFO) << "Creating GP params 2, larger data set";
    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> larger_gpc_params_2 =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    larger_inputs, larger_outputs,
                    prior_mean, input_variance_for_mean,
                    input_var_for_var_2, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel_2);

    LOG(INFO) << "Creating GP params 2, smaller data set";
    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc_smaller_samples_params_2 =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    smaller_inputs, smaller_outputs,
                    prior_mean, input_variance_for_mean,
                    input_var_for_var_2, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel_2);

    gp_kernel::GaussianKernel<2> var_position_kernel_3(var_pos_kernel_len,
                                                       var_pos_kernel_var_3);
    gp_kernel::PeriodicGaussianKernel<1> var_orientation_kernel_3(M_PI * 2,
                                                                  var_orientation_kernel_var_3,
                                                                  var_orientation_kernel_len);
    gp_kernel::Pose2dKernel var_pose_2d_kernel_3(var_position_kernel_3, var_orientation_kernel_3);

    LOG(INFO) << "Creating GP params 3, larger data set";
    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> larger_gpc_params_3 =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    larger_inputs, larger_outputs,
                    prior_mean, input_variance_for_mean,
                    input_var_for_var_3, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel_3);

    LOG(INFO) << "Creating GP params 3, smaller data set";
    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc_smaller_samples_params_3 =
            std::make_shared<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                    smaller_inputs, smaller_outputs,
                    prior_mean, input_variance_for_mean,
                    input_var_for_var_3, &mean_pose_2d_kernel,
                    &var_pose_2d_kernel_3);

    Eigen::Matrix<double, 3, Eigen::Dynamic> query_input(3, 1);
    query_input << 0.5, 0.5, M_PI;
    LOG(INFO) << "Output val larger samples: beta: " << input_variance_for_var << ", position kern var " << var_pos_kernel_var << ", orientation kern var " << var_orientation_kernel_var;
    LOG(INFO) << gpc->Inference(query_input)(0, 0);
    LOG(INFO) << "Output val smaller samples: ";
    LOG(INFO) << gpc_smaller_samples->Inference(query_input)(0, 0);

    LOG(INFO) << "Output val larger samples params 2: : beta: " << input_var_for_var_2 << ", position kern var " << var_pos_kernel_var_2 << ", orientation kern var " << var_orientation_kernel_var_2;
    LOG(INFO) << larger_gpc_params_2->Inference(query_input)(0, 0);
    LOG(INFO) << "Output val smaller samples params 2: ";
    LOG(INFO) << gpc_smaller_samples_params_2->Inference(query_input)(0, 0);

    LOG(INFO) << "Output val larger samples params 3: beta: " << input_var_for_var_3 << ", position kern var " << var_pos_kernel_var_3 << ", orientation kern var " << var_orientation_kernel_var_3;
    LOG(INFO) << larger_gpc_params_3->Inference(query_input)(0, 0);
    LOG(INFO) << "Output val smaller samples params 3: ";
    LOG(INFO) << gpc_smaller_samples_params_3->Inference(query_input)(0, 0);


}

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "momo_demo_2");
    ros::NodeHandle n;
    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;


    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
    std::string time_str = oss.str();
    std::string csv_file_name = "results/noise_eval_" + time_str + ".csv";
//    GPCTest();
    LOG(INFO) << runSyntheticProblemWithUncertainty(manager, 5);
//    LOG(INFO) << runSingleSyntheticProblem(manager);
//    runSyntheticProblemWithConfigVariations(manager, createParkedCarPosesWithFrequency(), createGroundTruthPoses(),
//                                            csv_file_name);

    return 0;
}