//
// Created by amanda on 3/18/21.
//

#ifndef AUTODIFF_GP_UNCERTAINTY_AWARE_SYNTHETIC_PROBLEM_H
#define AUTODIFF_GP_UNCERTAINTY_AWARE_SYNTHETIC_PROBLEM_H

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/LaserScan.h>

#include <util/random.h>

#include <base_lib/pose_reps.h>
#include <pose_optimization/uncertainty_aware/uncertainty_aware_pose_opt_utils.h>

using namespace pose_optimization;

namespace synthetic_problem {

    template<typename PoseType, int PoseParamCount>
    struct ObjectOccurrenceParams {
        PoseType object_canonical_pose_;
        unsigned int relative_frequency_;

        /**
         * Contains the variance of the object's pose (consistency in how it is placed).
         *
         * Will be XYZ, RPY for 3D and X, Y, Yaw for 2D.
         */
        Eigen::Matrix<double, PoseParamCount, 1> object_pose_variance_;
    };

    template<typename PoseType, int PoseParamCount>
    struct ObjectPlacementConfiguration {
        std::vector<ObjectOccurrenceParams<PoseType, PoseParamCount>> obj_placement_config_;
    };

    template<typename PoseType, int PoseParamCount>
    struct ObjectPlacementConfigurationAllClasses {
        std::unordered_map<std::string, ObjectPlacementConfiguration<PoseType, PoseParamCount>> obj_placement_configs_by_class_;
    };

    struct ScanGenerationParams2d {
        double max_range_ = 0;
        double min_range_ = 0;
        double min_angle_ = 0;
        double max_angle_ = 0;
        unsigned int num_beams_ = 0;
    };

    // Subtask: Generate actual trajectories (including current trajectory on which to localize)
    // Input: Canonical trajectory, variance in each dimension for how much poses can spread
    // Output: Vector of vector of poses
    // Question: How do we want to apply noise? Noise per edge or noise around each pose in trajectory?
    // Scope: Any synthetic problem
    template<typename PoseType, int PoseParamCount>
    std::vector<std::vector<PoseType>> generateSimilarTrajectories(const std::vector<PoseType> &base_trajectory_poses,
            const Eigen::Matrix<double, PoseParamCount, 1> &trajectory_pose_variance,
            const unsigned int &num_trajectories,
            util_random::Random &random_generator) {
        std::vector<std::vector<PoseType>> trajectories;
        for (unsigned int i = 0; i < num_trajectories; i++) {
            std::vector<PoseType> trajectory;
            for (const PoseType &base_trajectory_pose : base_trajectory_poses) {
                trajectory.emplace_back(pose::addGaussianNoise(base_trajectory_pose, trajectory_pose_variance, random_generator));

            }
            trajectories.emplace_back(trajectory);
        }
        return trajectories;
    }

    // Sub-subtask: Generate object poses given the % filled
    // Input: object placement config, % of spots filled
    // Output: list of object poses
    template<typename PoseType, int PoseParamCount>
    std::vector<PoseType> getObjectInstantiationsFromConfiguration(
            const ObjectPlacementConfiguration<PoseType, PoseParamCount> &object_configuration,
            double percent_spots_filled, util_random::Random &random_generator) {

        std::vector<size_t> open_spots_scaled_by_likelihood;
        for (size_t i = 0; i < object_configuration.obj_placement_config_.size(); i++) {
            ObjectOccurrenceParams obj_occurrence_params = object_configuration.obj_placement_config_[i];
            for (size_t j = 0; j < obj_occurrence_params.relative_frequency_; j++) {
                open_spots_scaled_by_likelihood.emplace_back(i);
            }
        }

        LOG(INFO) << "Number of spots: " << object_configuration.obj_placement_config_.size();
        std::vector<PoseType> result_poses;
        unsigned int num_filled_spots = ((double) object_configuration.obj_placement_config_.size()) * percent_spots_filled;
        LOG(INFO) << "Number of spots to fill " << num_filled_spots;
        for (unsigned int i = 0; i < num_filled_spots; i++) {
            size_t filled_spot_list_index = std::rand() % open_spots_scaled_by_likelihood.size();
            size_t spot_index = open_spots_scaled_by_likelihood[filled_spot_list_index];
            result_poses.emplace_back(pose::addGaussianNoise(
                    object_configuration.obj_placement_config_[spot_index].object_canonical_pose_,
                    object_configuration.obj_placement_config_[spot_index].object_pose_variance_, random_generator));
            open_spots_scaled_by_likelihood.erase(std::remove(open_spots_scaled_by_likelihood.begin(),
                                                              open_spots_scaled_by_likelihood.end(), spot_index),
                                                  open_spots_scaled_by_likelihood.end());
        }
        return result_poses;
    }


    // Subtask: Generate object configuration per trajectory
    // Input: Number of trajectories (add 1 for the current trajectory), map of object class to object placement config, map of object class to valid range of % object placements filled
    // Output: list of map of object class to object poses (item 1 in list corresponds to trajectory 1)
    // Scope: Any synthetic problem
    // Question: Should this output objects by class or just a list of objects?
    template<typename PoseType, int PoseParamCount>
    std::vector<std::unordered_map<std::string, std::vector<PoseType>>> getObjectInstantiationsFromConfiguration(
            const ObjectPlacementConfigurationAllClasses<PoseType, PoseParamCount> &object_configuration,
            unsigned int num_configurations,
            std::unordered_map<std::string, std::pair<double, double>> &valid_percent_filled_range,
            util_random::Random &random_generator) {

        for (const auto &entry : object_configuration.obj_placement_configs_by_class_) {
            LOG(INFO) << "(In helper) Config size for class " << entry.first << ": " << entry.second.obj_placement_config_.size();
        }

        std::vector<std::unordered_map<std::string, std::vector<PoseType>>> instantiated_object_poses;
        for (unsigned int i = 0; i < num_configurations; i++) {
            LOG(INFO) << "Creating object config " << i;
            std::unordered_map<std::string, std::vector<PoseType>> poses_for_config_i;
            for (const auto &obj_config_and_class : object_configuration.obj_placement_configs_by_class_) {
                std::string obj_class = obj_config_and_class.first;
                ObjectPlacementConfiguration<PoseType, PoseParamCount> placement_config = obj_config_and_class.second;

                std::pair<double, double> valid_percent_filled_range_for_class = std::make_pair(0, 1);
                if (valid_percent_filled_range.find(obj_class) != valid_percent_filled_range.end()) {
                    std::pair<double, double> retrieved_range = valid_percent_filled_range.at(obj_class);
                    valid_percent_filled_range_for_class.first =
                            std::max(valid_percent_filled_range_for_class.first, retrieved_range.first);
                    valid_percent_filled_range_for_class.second =
                            std::max(valid_percent_filled_range_for_class.first,
                                     std::min(valid_percent_filled_range_for_class.second, retrieved_range.second));
                }

                LOG(INFO) << "Valid percent filled range " << valid_percent_filled_range_for_class.first << ", " << valid_percent_filled_range_for_class.second;
                double percent_filled = random_generator.UniformRandom(
                        valid_percent_filled_range_for_class.first, valid_percent_filled_range_for_class.second);

                LOG(INFO) << "Percent filled " << percent_filled;
                poses_for_config_i[obj_class] =
                        getObjectInstantiationsFromConfiguration(placement_config, percent_filled, random_generator);
                LOG(INFO) << "Poses count for class  " << obj_class << ": " << poses_for_config_i[obj_class].size();

            }
            instantiated_object_poses.emplace_back(poses_for_config_i);
        }

        return instantiated_object_poses; // TODO verify this works as expected
    }

    // Subsubtask: Get detections relative to robot
    // Input: Maximum detection range, object positions for trajectory, robot pose
    // Output: Object detections relative to robot + noise
    template<typename PoseType, int PoseParamCount>
    std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> getObjectDetectionsRelativeToRobot(
            const PoseType &robot_pose,
            const double &max_detection_range,
            const std::vector<PoseType> gt_object_poses,
            const Eigen::Matrix<double, PoseParamCount, 1> &object_detection_variances,
            util_random::Random &rand_gen) {
        std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> relative_object_poses;
        for (const PoseType &obj_pose : gt_object_poses) {
            if ((obj_pose.first - robot_pose.first).norm() > max_detection_range) {
                continue;
            }
            PoseType relative_pose = pose::getPoseOfObj1RelToObj2(obj_pose, robot_pose);
            std::pair<PoseType, Eigen::Matrix<double, PoseParamCount, 1>> noisy_pose_and_distance_relative_variance =
                    pose::addPositionRelativeGaussianNoiseAndGetVariance(relative_pose, object_detection_variances,
                                                                         rand_gen);
            ObjectDetectionRelRobot<PoseType, PoseParamCount> obj_detection;
            obj_detection.pose_ = noisy_pose_and_distance_relative_variance.first;
            obj_detection.object_pose_variance_ = noisy_pose_and_distance_relative_variance.second;
            relative_object_poses.emplace_back(obj_detection);
        }
        return relative_object_poses;
    }

    // Subsubtask: Generate fake lidar scan
    // Input: Max range, min angle, max angle, beam count
    // Output: Scan data (need to determine format for this)
    // TODO
    sensor_msgs::LaserScan generateFakeLaserScan(const ScanGenerationParams2d &scan_params) {
        sensor_msgs::LaserScan laser_scan;
        laser_scan.range_min = scan_params.min_range_;
        laser_scan.range_max = scan_params.max_range_;
        laser_scan.angle_increment = (scan_params.max_angle_ - scan_params.min_angle_) / (scan_params.num_beams_ - 1);
        laser_scan.angle_min = scan_params.min_angle_;
        laser_scan.angle_max = scan_params.max_angle_;

        for (unsigned int i = 0; i < scan_params.num_beams_; i++) {
            laser_scan.ranges.emplace_back(scan_params.max_range_);
        }
        return laser_scan;
    }

    // Subtask: Generate current detections
    // Input: Current trajectory poses, current trajectory object pose configuration (map of class to list of poses), detection noise parameters
    // Output: vector of map of object class to list of detections (relative pose to respective pose in trajectory + noise)
    // Scope: Synthetic
    template <typename PoseType, int PoseParamCount>
    std::unordered_map<std::string, std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>> generateObjectDetectionsForPose(
            const PoseType &robot_pose,
            const double &max_obj_detection_dist,
            std::unordered_map<std::string, std::vector<PoseType>> &object_configurations_by_class,
            const Eigen::Matrix<double, PoseParamCount, 1> &object_detection_variance_per_detection_len,
            util_random::Random &rand_gen) {
        std::unordered_map<std::string, std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>> relative_object_poses;
        for (const auto &object_config_and_class : object_configurations_by_class) {
            relative_object_poses[object_config_and_class.first] =
                    getObjectDetectionsRelativeToRobot(robot_pose, max_obj_detection_dist,
                                                       object_config_and_class.second,
                                                       object_detection_variance_per_detection_len,
                                                       rand_gen);
        }
        return relative_object_poses;
    }

    template <typename PoseType, int PoseParamCount>
    std::unordered_map<std::string, std::vector<std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>>> generateObjectDetectionsForTrajectory(
            const std::vector<PoseType> &robot_poses,
            const double &max_obj_detection_dist,
            std::unordered_map<std::string, std::vector<PoseType>> &object_configurations_by_class,
            const Eigen::Matrix<double, PoseParamCount, 1> &object_detection_variance_per_detection_len,
            util_random::Random &rand_gen) {
        std::unordered_map<std::string, std::vector<std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>>> detections_for_trajectory;

        for (const auto &obj_config_types : object_configurations_by_class) {
            detections_for_trajectory[obj_config_types.first] = {};
        }
        for (const PoseType &robot_pose : robot_poses) {
            std::unordered_map<std::string, std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>> detections_for_pose =
                    generateObjectDetectionsForPose(robot_pose, max_obj_detection_dist, object_configurations_by_class,
                                                    object_detection_variance_per_detection_len, rand_gen);

            for (const auto &obj_config_types : object_configurations_by_class) {
                std::string obj_class = obj_config_types.first;
                std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> detections_for_class = {};
                if (detections_for_pose.find(obj_class) != detections_for_pose.end()) {
                    detections_for_class = detections_for_pose[obj_class];
                }
                detections_for_trajectory[obj_class].emplace_back(detections_for_class);
            }
        }
        return detections_for_trajectory;
    }


    // Subtask: Generate sensor info for each pose in each previous trajectory
    // Input: Previous trajectories (vector of vector of poses),  list of map of object class to object poses (item 1 in list corresponds to trajectory 1), object detection noise (variance in x, y, theta), maximum object detection distance, laser max range, laser max angle, laser min angle, laser angle resolution, robot poses,
    // Output: Vector of vector of sensor info -- each sensor info has laser scan (will just be max range for now), map of object class to list of detections (relative pose to respective pose in trajectory + noise)
    // Scope: Any synthetic problem

    template<typename PoseType, int PoseParamCount, typename ScanType, typename ScanGenParamsType>
    std::vector<std::vector<std::pair<PoseType, SensorInfo<PoseType, PoseParamCount, ScanType>>>> generateSensorInfoForTrajectories(
            const std::vector<std::vector<PoseType>> &trajectories,
            const std::vector<std::unordered_map<std::string, std::vector<PoseType>>> &object_configuration_per_trajectory,
            const Eigen::Matrix<double, PoseParamCount, 1> &object_detection_variance_per_detection_len,
            const double &max_obj_detection_dist,
            const ScanGenParamsType &scan_gen_params,
            util_random::Random &rand_gen) {
        std::vector<std::vector<std::pair<PoseType, SensorInfo<PoseType, PoseParamCount, ScanType>>>> sensor_data_for_trajectories;
        for (unsigned int traj_num = 0; traj_num < trajectories.size(); traj_num++) {
            std::vector<std::pair<PoseType, SensorInfo<PoseType, PoseParamCount, ScanType>>> sensor_data_for_trajectory;
            std::vector<PoseType> trajectory = trajectories[traj_num];
            for (unsigned int pose_num = 0; pose_num < trajectories[traj_num].size(); pose_num++) {
                PoseType robot_pose = trajectory[pose_num];
                SensorInfo<PoseType, PoseParamCount, ScanType> sensor_info;
                sensor_info.scan_info_ = generateFakeLaserScan(scan_gen_params);

                std::unordered_map<std::string, std::vector<PoseType>> object_configurations_by_class = object_configuration_per_trajectory[traj_num];

                sensor_info.object_detections_ = generateObjectDetectionsForPose<PoseType, PoseParamCount>(
                        robot_pose, max_obj_detection_dist, object_configurations_by_class,
                        object_detection_variance_per_detection_len, rand_gen);
                sensor_data_for_trajectory.emplace_back(std::make_pair(robot_pose, sensor_info));
            }
            sensor_data_for_trajectories.emplace_back(sensor_data_for_trajectory);
        }
        return sensor_data_for_trajectories;
    }







    // Similar to generating for past detections

    // Subtask: add noise to true trajectory
    // Input: Ground truth poses, odometry variance (x, y, yaw)
    // Output: Poses (with noise added)
    // Scope: Synthetic

    // Subtask: map samples onto real line
    // Input: Samples, mapping function
    // Output: mapped sample values
}

#endif //AUTODIFF_GP_UNCERTAINTY_AWARE_SYNTHETIC_PROBLEM_H
