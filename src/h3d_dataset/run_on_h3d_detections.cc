//
// Created by amanda on 2/13/21.
//

#ifndef AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC
#define AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC

#include <glog/logging.h>
#include <ros/ros.h>

#include <h3d_dataset/lidar_odom.h>
#include <h3d_dataset/h3d_file_operations.h>
#include <h3d_dataset/gps.h>
#include <h3d_dataset/obj_detections.h>
#include <pose_optimization/offline/offline_problem_runner.h>
#include <visualization/ros_visualization.h>
#include <pose_optimization/offline/ceres_visualization_callback_2d.h>

#include <h3d_dataset/dataset_odom.h>


DEFINE_string(scenario_num, "xxx", "3 digit scenario string to use");
DEFINE_bool(use_overlapping_trajectories, false, "True if this should find and use overlapping trajectories to "
                                                 "create the distribution of object poses, false (default) if this "
                                                 "should use the observations from the same trajectory that is being "
                                                 "optimized");

namespace h3d {


    static std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> createPoseGraph(const pose_optimization::CostFunctionParameters &cost_function_params) {

        gp_kernel::GaussianKernel<2> position_kernel(cost_function_params.position_kernel_len_,
                                                     cost_function_params.position_kernel_var_);
        gp_kernel::PeriodicGaussianKernel<1> orientation_kernel(M_PI * 2, cost_function_params.orientation_kernel_var_,
                                                                cost_function_params.orientation_kernel_len_);
        gp_kernel::Pose2dKernel pose_2d_kernel(position_kernel, orientation_kernel);
        return std::make_shared<pose_graph::PoseGraph2dMovObjDistribution2d>(pose_2d_kernel);
    }

    ceres::IterationCallback* createCeresIterationCallback(
            const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
            const pose_graph::NodeId &node_id,
            const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
            const std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> noisy_observations_by_class) {
        return new offline_optimization::CeresVisualizationCallback2d(
                    pose_graph, vis_manager, node_id, noisy_observations_by_class);
    }

    void runOptimizationVisualization(
            const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
            const pose_graph::NodeId &max_node_id,
            const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> &pose_graph,
            const offline_optimization::VisualizationTypeEnum &vis_stage,
            const std::vector<pose::Pose2d> &ground_truth_trajectory,
            const std::vector<pose::Pose2d> &unoptimized_trajectory,
            const std::vector<std::vector<pose::Pose2d>> &noisy_obj_observations) {

        // TODO make this more generic (not specifically car class)
        std::string car_class = "car";
        switch (vis_stage) {
            case offline_optimization::VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION:

                vis_manager->displayTrueTrajectory(ground_truth_trajectory);
                vis_manager->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obj_observations, car_class);
                vis_manager->displayOdomTrajectory(unoptimized_trajectory);
                vis_manager->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
                                                                     noisy_obj_observations, car_class);
                {
                    std::vector<pose::Pose2d> poses_global_frame;
                    for (size_t node = 0; node < ground_truth_trajectory.size(); node++) {
                        pose::Pose2d robot_pose = ground_truth_trajectory[node];
                        for (const pose::Pose2d &obj_pose : noisy_obj_observations[node]) {
                            poses_global_frame.emplace_back(pose::combinePoses(robot_pose, obj_pose));
                        }
                    }

                    std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
                            visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(poses_global_frame);
                    vis_manager->displayMaxGpRegressorOutput(pose_graph->getMovableObjKde(car_class), 0.6, // TODO revert
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
                vis_manager->displayEstTrajectory(node_poses_list);
                vis_manager->displayObjObservationsFromEstTrajectory(node_poses_list, noisy_obj_observations, car_class);

                vis_manager->displayTrueTrajectory(ground_truth_trajectory);
                vis_manager->displayObjObservationsFromGtTrajectory(ground_truth_trajectory, noisy_obj_observations, car_class);
                vis_manager->displayOdomTrajectory(unoptimized_trajectory);
                vis_manager->displayObjObservationsFromOdomTrajectory(unoptimized_trajectory,
                                                                      noisy_obj_observations, car_class);
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
//                vis_manager->displayMaxGpRegressorOutput(pose_graph->getMovableObjKde(car_class), 0.6, // TODO revert
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

    bool isScenarioCloseButNotAtCloseTime(const std::vector<GPSData> &curr_scenario_gps,
                                          const std::vector<GPSData> &compare_scenario_gps,
                                          const double &max_lat_distance,
                                          const double &max_long_distance,
                                          const double &min_time_distance) {
        double min_time_curr_scenario = curr_scenario_gps.front().timestamp_;
        double max_time_curr_scenario = curr_scenario_gps.back().timestamp_;
        double min_time_compare_scenario = compare_scenario_gps.front().timestamp_;
        double max_time_compare_scenario = compare_scenario_gps.back().timestamp_;

        if (max_time_compare_scenario < min_time_curr_scenario) {
            // Compare scenario is before current

            if ((min_time_curr_scenario - max_time_compare_scenario) < min_time_distance) {
                return false;
            }
        } else {
            // Current scenario is before compare

            if ((min_time_compare_scenario - max_time_curr_scenario) < min_time_distance) {
                return false;
            }
        }

        // Check if GPS coordinates are sufficiently overlapping
        for (const GPSData &curr_scenario_gps_entry : curr_scenario_gps) {
            for (const GPSData &compare_scenario_gps_entry : compare_scenario_gps) {
                if ((abs(compare_scenario_gps_entry.abs_lat_ - curr_scenario_gps_entry.abs_lat_) < max_lat_distance) &&
                (abs(compare_scenario_gps_entry.abs_long_ - curr_scenario_gps_entry.abs_long_) < max_long_distance)) {
                    return true;
                }
            }
        }
        return false;
    }

    void getGpsDataForNearbyScenarios(const std::string &dataset_directory,
                                      const std::vector<std::string> &other_scenario_num_strs,
                                      const std::vector<GPSData> &curr_scenario_gps,
                                      const double &max_lat_distance,
                                      const double &max_long_distance,
                                      const double &min_time_distance,
                                      std::unordered_map<std::string, std::vector<GPSData>> &gps_data_for_nearby_trajectories) {

        for (const std::string &other_scenario_num_str : other_scenario_num_strs) {
            std::vector<GPSData> gps_data_for_other_scenario = getFirstGpsDataForEachFileNumInScenario(
                    dataset_directory, other_scenario_num_str);
            if (isScenarioCloseButNotAtCloseTime(curr_scenario_gps, gps_data_for_other_scenario, max_lat_distance,
                                                 max_long_distance, min_time_distance)) {
                LOG(INFO) << "Nearby scenario " << other_scenario_num_str;
                gps_data_for_nearby_trajectories[other_scenario_num_str] = gps_data_for_other_scenario;
            }
        }

    }

    void getOtherScenarioInCurrScenarioOdomFrame(const std::vector<GPSData> &other_scenario_gps,
                                                 const std::string &compare_scenario_dir,
                                                 const double &origin_lat,
                                                 const double &origin_long,
                                                 const double &origin_yaw,
                                                 const pose::Pose2d &velodyne_pos_in_gps_frame,
                                                 std::vector<pose::Pose2d> &odom_frame_poses) {

        // For now, we'll find the pose of the first element in the other scenario trajectory using its GPS coordinates
        // and then use the odometry of the other scenario to find the pose of the rest of the trajectory in the original coordinate frame
        // However, if the GPS of the first coordinate is off, then this will forward propagate that error, in which
        // case it might be better just to have the (theoretically) independent errors between individual GPS poses
        // for the second trajectory -- need to test this out and see if using odometry is problematic

        std::vector<pose::Pose2d> trajectory_2_rel_to_pose_0_in_traj_2;
        pose::Pose2d first_pose;

        for (int i = 0; i < (int) other_scenario_gps.size(); i++) {
            std::vector<h3d::RawDatasetOdomData> dataset_odom_data = h3d::readRawDatasetOdomForFileNum(
                    compare_scenario_dir, i);

            // Store all of these relative to the pose at node=0 (so that the pose at node=0 is identity)
            if (i == 0) {
                trajectory_2_rel_to_pose_0_in_traj_2.emplace_back(pose::createPose2d(0, 0, 0));
                first_pose = pose::createPose2d(dataset_odom_data[0].transl_x_, dataset_odom_data[0].transl_y_,
                                                dataset_odom_data[0].yaw_);
            } else {
                trajectory_2_rel_to_pose_0_in_traj_2.emplace_back(pose::getPoseOfObj1RelToObj2(
                        pose::createPose2d(dataset_odom_data[0].transl_x_, dataset_odom_data[0].transl_y_,
                                           dataset_odom_data[0].yaw_), first_pose));
            }
        }

        // Get the position of the GPS coordinate of the first entry for scenario 2 relative to the origin
        Eigen::Vector2d gps_2_in_gps_1_metric = gpsToMetric(other_scenario_gps[0].abs_lat_, other_scenario_gps[0].abs_long_,
                                                            origin_lat, origin_long);
        pose::Pose2d gps_2_rel_gps_1 = pose::combinePoses(pose::createPose2d(0, 0, -origin_yaw),
                                                          pose::combinePoses(std::make_pair(gps_2_in_gps_1_metric, 0),
                                                                             pose::createPose2d(0, 0, other_scenario_gps[0].tilt_yaw_))); // TODO is this right?

        pose::Pose2d first_pose_t2_rel_t1 = pose::combinePoses(pose::getPoseOfObj1RelToObj2(gps_2_rel_gps_1, velodyne_pos_in_gps_frame), velodyne_pos_in_gps_frame);

        for (const pose::Pose2d &traj_2_pose : trajectory_2_rel_to_pose_0_in_traj_2){
            odom_frame_poses.emplace_back(pose::combinePoses(first_pose_t2_rel_t1, traj_2_pose));
        }
    }

    void getMapObservations(const std::string &dataset_directory,
                            const std::string &current_scenario_str,
                            const double &max_lat_distance,
                            const double &max_long_distance,
                            const double &min_time_distance,
                            const double &origin_lat, // Latitude of first node in trajectory that we're using
                            const double &origin_long, // Longitude of first node in trajectory that we're using
                            const double &origin_yaw,
                            const pose::Pose2d &velodyne_pos_in_gps_frame,
                            const std::unordered_set<std::string> &obj_types,
                            std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                            std::vector<pose_graph::MapObjectObservation2d> &observations) {

        // Find all scenarios
        std::vector<std::string> all_scenario_strs = getAllScenarios(dataset_directory);

        // Exclude current scenario
        std::vector<std::string> all_other_scenarios = all_scenario_strs;
        all_other_scenarios.erase(
                std::remove(all_other_scenarios.begin(), all_other_scenarios.end(), current_scenario_str),
                all_other_scenarios.end());

        std::string curr_scenario_dir = h3d::getScenarioDirectory(dataset_directory, current_scenario_str);

        std::vector<GPSData> gps_data_for_curr_scenario = getFirstGpsDataForEachFileNumInScenario(dataset_directory,
                                                                                                  current_scenario_str);

        // Get trajectories in appropriate lat/long range of any in current trajectory and within distant enough time
        std::unordered_map<std::string, std::vector<GPSData>> gps_data_for_nearby_trajectories;
        getGpsDataForNearbyScenarios(dataset_directory, all_other_scenarios, gps_data_for_curr_scenario,
                                     max_lat_distance, max_long_distance, min_time_distance,
                                     gps_data_for_nearby_trajectories);

        std::vector<pose::Pose2d> other_trajectory_poses;
        for (const auto &gps_data_nearby_trajectory : gps_data_for_nearby_trajectories) {
            LOG(INFO) << "Adding trajectory info for scenario " << gps_data_nearby_trajectory.first;
            std::string scenario_dir = h3d::getScenarioDirectory(dataset_directory, gps_data_nearby_trajectory.first);

            // Convert GPS for other trajectories to same frame as odom
            std::vector<pose::Pose2d> odom_frame_poses;
            getOtherScenarioInCurrScenarioOdomFrame(gps_data_nearby_trajectory.second, scenario_dir, origin_lat, origin_long, origin_yaw, velodyne_pos_in_gps_frame, odom_frame_poses);
            other_trajectory_poses.insert(other_trajectory_poses.end(), odom_frame_poses.begin(), odom_frame_poses.end());

            // Convert observations to same frame as odom
            for (size_t i = 0; i < odom_frame_poses.size(); i++) {
                std::vector<h3d::RawObjectDetection> full_obj_detection =
                        h3d::readObjDetectionDataFromFile(scenario_dir, i);
                std::vector<h3d::RawObjectDetection> filtered_obj_detections =
                        h3d::getStaticObjDetectionsWithTypes(full_obj_detection, obj_types);
                std::vector<pose::Pose2d> obs_at_pose;
                pose::Pose2d node_pose = odom_frame_poses[i];

                for (const h3d::RawObjectDetection &obj_detection : filtered_obj_detections) {
                     pose_graph::MapObjectObservation<2, double> observation;

                     pose::Pose2d map_frame_obj_detection = pose::combinePoses(
                             node_pose, pose::createPose2d(obj_detection.centroid_x_, obj_detection.centroid_y_,
                                                           obj_detection.yaw_));

                     observation.transl_.x() = map_frame_obj_detection.first.x();
                     observation.transl_.y() = map_frame_obj_detection.first.y();
                     observation.orientation_ = map_frame_obj_detection.second;
                     observation.semantic_class_ = obj_detection.label_;

                     observations.emplace_back(observation);
                }
            }
        }
        vis_manager->displayOtherTrajectory(other_trajectory_poses);
    }
}

int main(int argc, char** argv) {


    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    ros::init(argc, argv,
              "run_on_h3d_detections");
    ros::NodeHandle n;

    std::string scenario_number_str = FLAGS_scenario_num;
    LOG(INFO) << "Running for scenario " << scenario_number_str;

    std::string h3d_extracted_data_dir = "/home/amanda/momo_testing/h3d/";
    std::string h3d_dataset_directory = "/home/amanda/datasets/h3d/icra_benchmark_20200103_with_odom";
    std::string dataset_preprocessed_data_dir = h3d_dataset_directory + "/augmentation";
    std::string lidar_odom_file = dataset_preprocessed_data_dir + "/scenario_" + scenario_number_str + "/legoloam_odom_output_file.csv";

    double max_lat_distance = 0.001; // TODO reconsider this, seems like a decent starting point
    double max_long_distance = 0.0015; // TODO reconsider this, seems like a decent starting point
    double min_time_distance = 75; // TODO reconsider this, seems like a decent starting point
//    double min_time_distance = 0; // TODO reconsider this, seems like a decent starting point
//
//    double obj_detection_variance_for_transl_per_dist = 0.1; // TODO is this reasonable?
//    double obj_detection_yaw_variance = 0.1; // TODO is this reasonable?

    double obj_detection_variance_for_transl_per_dist = 1e-5; // TODO is this reasonable?
    double obj_detection_yaw_variance = 1e-5; // TODO is this reasonable?

    // TODO this is a guess and may not be right
    pose::Pose2d velodyne_pose_rel_gps = pose::createPose2d(0, 0, M_PI_2);

    std::unordered_map<pose_graph::NodeId, double> timestamps_by_node_id;
    std::vector<h3d::GaussianBinaryFactor2d> lidar_odom_factors;

    h3d::getLidar2dConstraintsFromLidarFile(lidar_odom_file, timestamps_by_node_id,
                                            lidar_odom_factors);

    LOG(INFO) << "Num odom factors " << lidar_odom_factors.size();
    LOG(INFO) << "Timestamps by node id len " << timestamps_by_node_id.size();

    // Read in lidar odom
    // Compute transforms relative to each other
    // TODO how to handle if we have multiple files? (for now, just assume one)

    // Read observations in global frame

    // For each timestamp in lidar odom, find relevant current observations
    std::string scenario_dir_str = h3d::getScenarioDirectory(h3d_dataset_directory, scenario_number_str);
    std::string preprocessed_scenario_dir_str = h3d::getScenarioDirectory(dataset_preprocessed_data_dir, scenario_number_str);
    std::vector<std::pair<std::string, double>> timestamps_by_filenum_str = h3d::getTimestampsForFileNums(scenario_dir_str);

    std::unordered_map<pose_graph::NodeId, std::string> filenum_str_for_node_id;
    size_t start_search_index = 0;
    for (size_t node_num = 0; node_num < timestamps_by_node_id.size(); node_num++) {
        std::string filenum_str;
        size_t found_index;
        h3d::getFileNumCorrespondingToTimestamp(timestamps_by_filenum_str, timestamps_by_node_id[node_num], start_search_index, filenum_str, found_index);
        start_search_index = found_index;
        filenum_str_for_node_id[node_num] = filenum_str;

        LOG(INFO) << "Filenum str for node id " << node_num << ": " << filenum_str;
    }


    std::shared_ptr<visualization::VisualizationManager> vis_manager = std::make_shared<visualization::VisualizationManager>(n);

    std::vector<pose_graph::MovableObservationFactor<2, double, 3>> movable_observation_factors;

    std::string car_class = "car";
    std::unordered_set<std::string> obj_types = {car_class};

    std::unordered_map<pose_graph::NodeId, pose::Pose2d> true_poses;
    std::unordered_map<pose_graph::NodeId, std::vector<h3d::RawObjectDetection>> object_detections_at_nodes;
    std::vector<std::vector<pose::Pose2d>> obs_at_poses;

    pose::Pose2d first_pose;


    for (size_t node_num = 0; node_num < timestamps_by_node_id.size(); node_num++) {
        std::string filenum_str_for_node = filenum_str_for_node_id[node_num];

//        std::vector<h3d::PreprocessedGPSWithXY> preprocessed_gps_at_filenum_str = h3d::readPreprocessedGpsDataForFileNum(
//                preprocessed_scenario_dir_str, filenum_str_for_node);

        // Use the first entry as the "true pose"
        std::vector<h3d::RawDatasetOdomData> dataset_odom_data = h3d::readRawDatasetOdomForFileNum(scenario_dir_str, filenum_str_for_node);

        // Store all of these relative to the pose at node=0 (so that the pose at node=0 is identity)
        if (node_num == 0) {
            true_poses[node_num] = pose::createPose2d(0, 0, 0);
            first_pose = pose::createPose2d(dataset_odom_data[0].transl_x_, dataset_odom_data[0].transl_y_,
                                                      dataset_odom_data[0].yaw_);
        } else {
            true_poses[node_num] = pose::getPoseOfObj1RelToObj2(
                    pose::createPose2d(dataset_odom_data[0].transl_x_, dataset_odom_data[0].transl_y_,
                                       dataset_odom_data[0].yaw_), first_pose);
        }
//        true_poses[node_num] = pose::combinePoses(pose::createPose2d(preprocessed_gps_at_filenum_str[0].rel_x_,
//                                                  preprocessed_gps_at_filenum_str[0].rel_y_,
//                                                  preprocessed_gps_at_filenum_str[0].orig_gps_data_.tilt_yaw_), velodyne_pose_rel_gps);

        std::vector<h3d::RawObjectDetection> full_obj_detection = h3d::readObjDetectionDataFromFile(
                scenario_dir_str, filenum_str_for_node);
        std::vector<h3d::RawObjectDetection> filtered_obj_detections =
                h3d::getStaticObjDetectionsWithTypes(full_obj_detection, obj_types);
        object_detections_at_nodes[node_num] = filtered_obj_detections;

        std::vector<pose::Pose2d> obs_at_pose;

        for (const h3d::RawObjectDetection &obj_detection : filtered_obj_detections) {
            pose_graph::MovableObservation<2, double, 3> observation;
            observation.observation_transl_.x() = obj_detection.centroid_x_;
            observation.observation_transl_.y() = obj_detection.centroid_y_;
            observation.observation_orientation_ = obj_detection.yaw_;
            observation.semantic_class_ = obj_detection.label_;


            obs_at_pose.emplace_back(
                        std::make_pair(observation.observation_transl_, observation.observation_orientation_));

            Eigen::Matrix<double, 3, 3> obs_cov_mat = Eigen::Matrix<double, 3, 3>::Zero();
            obs_cov_mat(0, 0) = obj_detection_variance_for_transl_per_dist * abs(observation.observation_transl_.x());
            obs_cov_mat(1, 1) = obj_detection_variance_for_transl_per_dist * abs(observation.observation_transl_.y());
            obs_cov_mat(2, 2) = obj_detection_yaw_variance;

            observation.observation_covariance_ = obs_cov_mat;

            pose_graph::MovableObservationFactor<2, double, 3> factor(node_num, observation);
            movable_observation_factors.emplace_back(factor);
        }
        obs_at_poses.emplace_back(obs_at_pose);
    }
    LOG(INFO) << "Num movable object observations " << movable_observation_factors.size();



    std::vector<pose_graph::MapObjectObservation<2, double>> map_object_observations;
    if (FLAGS_use_overlapping_trajectories) {

        std::string filenum_str_for_node_0 = filenum_str_for_node_id[0];
        h3d::GPSData gps_data_for_node_0 = h3d::readGpsDataForFileNum(scenario_dir_str, filenum_str_for_node_0)[0];
        double origin_lat = gps_data_for_node_0.abs_lat_;
        double origin_long = gps_data_for_node_0.abs_long_;
        double origin_yaw = gps_data_for_node_0.tilt_yaw_;

        h3d::getMapObservations(h3d_dataset_directory, scenario_number_str, max_lat_distance, max_long_distance,
                                min_time_distance, origin_lat, origin_long, origin_yaw, velodyne_pose_rel_gps,
                                obj_types, vis_manager, map_object_observations);

//        return 0;
    } else {
        for (size_t node_num = 0; node_num < timestamps_by_node_id.size(); node_num++) {
            pose::Pose2d node_pose_gps = true_poses[node_num];
            std::vector<h3d::RawObjectDetection> obj_detections_at_pose = object_detections_at_nodes[node_num];

            for (const h3d::RawObjectDetection &obj_detection : obj_detections_at_pose) {
                pose_graph::MapObjectObservation<2, double> observation;
                pose::Pose2d map_frame_obj_detection = pose::combinePoses(node_pose_gps,
                                                                          pose::createPose2d(obj_detection.centroid_x_,
                                                                                             obj_detection.centroid_y_,
                                                                                             obj_detection.yaw_));
                observation.transl_.x() = map_frame_obj_detection.first.x();
                observation.transl_.y() = map_frame_obj_detection.first.y();
                observation.orientation_ = map_frame_obj_detection.second;
                observation.semantic_class_ = obj_detection.label_;

                map_object_observations.emplace_back(observation);
            }
        }
    }

    // Create initial positions based on true pose of the first node in the trajectory and the odom constraints
    std::vector<pose_graph::Node<2, double>> initial_node_positions;

    pose_graph::Node<2, double> prev_node;
    pose::Pose2d prev_node_pose = true_poses.at(0);
    prev_node.id_ = 0;
    prev_node.est_position_ = std::make_shared<Eigen::Vector2d>(prev_node_pose.first);
    prev_node.est_orientation_ = std::make_shared<double>(prev_node_pose.second);
    initial_node_positions.emplace_back(prev_node);
    std::vector<pose::Pose2d> simple_init_trajectory;
    simple_init_trajectory.emplace_back(prev_node_pose);

    for (pose_graph::NodeId i = 1; i < timestamps_by_node_id.size(); i++) {
        // Relying on these being in order
        h3d::GaussianBinaryFactor2d lidar_odom_factor = lidar_odom_factors[i-1];

        pose_graph::Node<2, double> new_node;
        new_node.id_ = i;
        pose::Pose2d est_pose = pose::combinePoses(
                std::make_pair(*(prev_node.est_position_), *(prev_node.est_orientation_)),
                std::make_pair(lidar_odom_factor.translation_change_, lidar_odom_factor.orientation_change_));
        new_node.est_position_ = std::make_shared<Eigen::Vector2d>(est_pose.first);
        new_node.est_orientation_ = std::make_shared<double>(est_pose.second);

        simple_init_trajectory.emplace_back(est_pose);
        initial_node_positions.emplace_back(new_node);
        prev_node = new_node;
    }

    LOG(INFO) << "Map object observations size " << map_object_observations.size();

    offline_optimization::OfflineProblemData<2, double, 3, 2, double> offline_problem_data;
    offline_problem_data.odometry_factors_ = lidar_odom_factors;
    offline_problem_data.map_object_observations_ = map_object_observations;
    offline_problem_data.movable_observation_factors_ = movable_observation_factors;
    offline_problem_data.initial_node_positions_ = initial_node_positions;

    std::vector<pose::Pose2d> gt_vec;
    for (size_t node_num = 0; node_num < true_poses.size(); node_num++) {
        gt_vec.emplace_back(true_poses.at(node_num));
//        LOG(INFO) << "GT: Node " << node_num << ": " << gt_vec.back().first << ", " << gt_vec.back().second;
    }


    pose_optimization::CostFunctionParameters cost_function_params;
//    cost_function_params.odometry_enabled_ = false;
//    cost_function_params.position_kernel_len_ = cost_function_params.position_kernel_len_ / 5;
//    cost_function_params.orientation_kernel_len_ = cost_function_params.orientation_kernel_len_ / 5;
//    cost_function_params.orientation_kernel_len_ = 100000;
//    cost_function_params.position_kernel_len_ = 10000000;
    pose_optimization::PoseOptimizationParameters pose_optimization_params;
    pose_optimization_params.cost_function_params_ = cost_function_params;
    offline_optimization::OfflinePoseOptimizer<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3> offline_optimizer;

    std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> object_detections_by_type = {{car_class, obs_at_poses}};
    LOG(INFO) << offline_optimizer.runOfflineOptimization(
            offline_problem_data, pose_optimization_params,
            h3d::createPoseGraph,
            std::bind(h3d::createCeresIterationCallback, vis_manager, std::placeholders::_1,
                      std::placeholders::_2, object_detections_by_type),
            std::bind(h3d::runOptimizationVisualization, vis_manager,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                      gt_vec, simple_init_trajectory,
                      obs_at_poses)).size();


}

#endif //AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC
