//
// Created by amanda on 12/21/20.
//

#ifndef AUTODIFF_GP_ROS_VISUALIZATION_H
#define AUTODIFF_GP_ROS_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_3d.h>

#include <base_lib/pose_reps.h>
#include <gaussian_process/gp_classifier.h>

namespace visualization {

    enum TrajectoryType {
        GROUND_TRUTH,
        ODOM_ONLY,
        ESTIMATED
    };

    class VisualizationManager {
    public:

        VisualizationManager(ros::NodeHandle &node_handle, const std::string &prefix = "") : node_handle_(node_handle),
                                                                                             prefix_(prefix) {
            gt_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>(prefix + "gt_visualization_marker",
                                                                                10000);
            other_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>(
                    prefix + "other_visualization_marker", 10000);
            est_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>(prefix + "est_pos_marker", 10000);
            odom_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>(prefix + "odom_pos_marker", 10000);
            regressor_max_val_for_pos_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                    prefix + "regressor_max_val_for_pos", 2);
            variance_max_val_for_pos_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                    prefix + "variance_max_val_for_pos",
                    2);
            classifier_max_val_for_pos_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                    prefix + "classifier_max_val_for_pos", 2);
            robot_pose_max_val_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(prefix + "robot_pose_max_val", 2);

            waypoint_pub_ = node_handle_.advertise<visualization_msgs::Marker>(prefix + "waypoints", 500);

            parking_spot_3d_pub_ = node_handle_.advertise<visualization_msgs::Marker>(prefix + "parking_spots_3d",
                                                                                      10000);
            LOG(INFO) << "Vis manager 1";
            ros::Duration(2).sleep();
            LOG(INFO) << "Vis manager 2";
//            for (int id = 0; id < kRobotEstPosesMax; id++) {
//                LOG(INFO) << "Vis manager 2a";
//                removeMarker(id, gt_marker_pub_);
//                removeMarker(id, other_marker_pub_);
//                removeMarker(id, est_marker_pub_);
//                removeMarker(id, odom_marker_pub_);
//                LOG(INFO) << "Vis manager 2b";
//                ros::Duration(0.0001).sleep();
//                LOG(INFO) << "Vis manager 2c";
//            }
            LOG(INFO) << "Vis manager 3";
            ros::Duration(1).sleep();
            LOG(INFO) << "Vis manager 4";

            for (int i = -5; i <= 6; i++) {
                std::string angle_name;
                if (i < 0) {
                    angle_name = "neg_";
                }
                angle_name += std::to_string(abs(30 * i));

                std::string regressor_heat_topic_name = "regressor_" + angle_name + "_val";
                std::string classifier_heat_topic_name = "variance_" + angle_name + "_val";
                std::string variance_heat_topic_name = "classifier_" + angle_name + "_val";
                std::string robot_pose_topic_name = "robot_pose_" + angle_name + "_val";
                classifier_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                        prefix + classifier_heat_topic_name, 2);
                regressor_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                        prefix + regressor_heat_topic_name, 2);
                variance_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                        prefix + variance_heat_topic_name, 2);
                robot_pose_pub_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(
                        prefix + robot_pose_topic_name, 2);
            }
            LOG(INFO) << "Vis manager 5";
            ros::Duration(2).sleep();
            LOG(INFO) << "Vis manager 6";
            // TODO consider removing all existing markers trajectory topics
        }

        void displayTrueTrajectory(const std::vector<pose::Pose3d> &true_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            publishTrajectory(gt_marker_pub_, color, true_trajectory, kGtTrajectoryId, kRobotGtPosesMin,
                              kRobotGtPosesMax);
        }

        void displayTrueObjPoses(const std::vector<pose::Pose3d> &true_car_poses, const std::string &obj_class) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(GROUND_TRUTH, obj_class, pub);

            int32_t car_poses_count = true_car_poses.size();
            for (int32_t i = 0; i < car_poses_count; i++) {
                pose::Pose3d pose = true_car_poses[i];
                publishCarPoses(pub, pose, color, kCarGtPosesMin + i);
            }

//            for (int32_t i = kCarGtPosesMin + car_poses_count; i <= kCarGtPosesMax; i++) {
//                removeMarker(i, pub);
//            }
        }

        void displayParkingSpots(const std::vector<pose::Pose3d> &parking_spots_3d, const double &spot_x_dim,
                                 const double &spot_y_dim) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            int32_t car_poses_count = parking_spots_3d.size();
            for (int32_t i = 0; i < car_poses_count; i++) {
                pose::Pose3d pose = parking_spots_3d[i];
                LOG(INFO) << "Publishing spot " << i;
                publishParkingSpot3d(parking_spot_3d_pub_, pose, color, kCarGtPosesMin + i, spot_x_dim, spot_y_dim);
            }
        }

        void displayObjObservationsFromGtTrajectory(const std::vector<pose::Pose3d> &gt_trajectory,
                                                    const std::vector<std::vector<pose::Pose3d>> &relative_car_poses,
                                                    const std::string &obj_class) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.g = 1.0;
            color.b = 1.0;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(GROUND_TRUTH, obj_class, pub);

            publishLinesToCarDetections(pub, gt_trajectory, relative_car_poses, color,
                                        kObservedFromGtCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, gt_trajectory, relative_car_poses, color,
                                                kObservedFromGtCarDetectionsMin, kObservedFromGtCarDetectionsMax);
        }

        void displayObjObservationsFromEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory,
                                                     const std::vector<std::vector<pose::Pose3d>> &relative_car_poses,
                                                     const std::string &obj_class) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.r = 1.0;
            color.g = 0.7;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ESTIMATED, obj_class, pub);

            publishLinesToCarDetections(pub, est_trajectory, relative_car_poses, color,
                                        kObservedFromEstCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, est_trajectory, relative_car_poses, color,
                                                kObservedFromEstCarDetectionsMin, kObservedFromEstCarDetectionsMax);
        }

        void displaySemanticPointObsFromEstTrajectory(const std::vector<pose::Pose2d> &est_trajectory,
                                                      const std::vector<std::vector<std::vector<Eigen::Vector2d>>> &relative_semantic_points,
                                                      const std::string &obj_class,
                                                      const std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> &relative_object_samples_for_cluster = {},
                                                      const Eigen::Vector2d &dimensions_for_samples = Eigen::Vector2d()) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.r = 1.0;
            color.g = 0.7;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ESTIMATED, obj_class, pub);

            displaySemanticPointObsFromTrajectory(est_trajectory, relative_semantic_points, 0.15,
                                                kObservedFromEstCarDetectionLines, color, pub,
                                                relative_object_samples_for_cluster, dimensions_for_samples);
        }

        void displayObjObservationsFromOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory,
                                                      const std::vector<std::vector<pose::Pose3d>> &relative_car_poses,
                                                      const std::string &obj_class) {

            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.b = 1.0;
            color.r = 0.7;
            color.g = 0.2;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ODOM_ONLY, obj_class, pub);

            publishLinesToCarDetections(pub, odom_trajectory, relative_car_poses, color,
                                        kObservedFromOdomCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, odom_trajectory, relative_car_poses, color,
                                                kObservedFromOdomCarDetectionsMin, kObservedFromOdomCarDetectionsMax);
        }

        void displaySemanticPointObsFromOdomTrajectory(const std::vector<pose::Pose2d> &odom_trajectory,
                                                     const std::vector<std::vector<std::vector<Eigen::Vector2d>>> &relative_semantic_points,
                                                     const std::string &obj_class,
                                                     const std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> &relative_object_samples_for_cluster = {},
                                                     const Eigen::Vector2d &dimensions_for_samples = Eigen::Vector2d()) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.b = 1.0;
            color.r = 0.7;
            color.g = 0.2;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ODOM_ONLY, obj_class, pub);

            displaySemanticPointObsFromTrajectory(odom_trajectory, relative_semantic_points, 0.15,
                                                kObservedFromOdomCarDetectionLines, color, pub,
                                                relative_object_samples_for_cluster, dimensions_for_samples);
        }

        void displayOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;

            publishTrajectory(odom_marker_pub_, color, odom_trajectory, kOdomTrajectoryId, kRobotOdomPosesMin,
                              kRobotOdomPosesMax);
        }

        void displayEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;

            publishTrajectory(est_marker_pub_, color, est_trajectory, kEstTrajectoryId, kRobotEstPosesMin,
                              kRobotEstPosesMax);
        }

        //        void displayMaxGpRegressorOutput(
//                std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> regressor,
//                const double &resolution, const double &x_min, const double &x_max, const double &y_min,
//                const double &y_max) {
        void displayMaxGpRegressorOutput(
                std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc,
                const double &resolution, const double &x_min, const double &x_max, const double &y_min,
                const double &y_max) {
            int64_t x_min_unscaled = floor(x_min / resolution);
            int64_t x_max_unscaled = ceil(x_max / resolution);
            int64_t y_min_unscaled = floor(y_min / resolution);
            int64_t y_max_unscaled = ceil(y_max / resolution);

            LOG(INFO) << "Creating best occ grid";

            nav_msgs::OccupancyGrid best_variance_grid;
            best_variance_grid.header.frame_id = kVizFrame;
            best_variance_grid.header.stamp = ros::Time::now();
            best_variance_grid.info.resolution = resolution;
            best_variance_grid.info.origin.position.z = -3;
            best_variance_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_variance_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_variance_grid.info.origin.orientation.w = 1.0;
            best_variance_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_variance_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_variance_grid.data.resize(best_variance_grid.info.width * best_variance_grid.info.height);

            nav_msgs::OccupancyGrid best_regressor_grid;
            best_regressor_grid.header.frame_id = kVizFrame;
            best_regressor_grid.header.stamp = ros::Time::now();
            best_regressor_grid.info.resolution = resolution;
            best_regressor_grid.info.origin.position.z = -3;
            best_regressor_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_regressor_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_regressor_grid.info.origin.orientation.w = 1.0;
            best_regressor_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_regressor_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_regressor_grid.data.resize(best_regressor_grid.info.width * best_regressor_grid.info.height);

            nav_msgs::OccupancyGrid best_occ_grid;
            best_occ_grid.header.frame_id = kVizFrame;
            best_occ_grid.header.stamp = ros::Time::now();
            best_occ_grid.info.resolution = resolution;
            best_occ_grid.info.origin.position.z = -3;
            best_occ_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_occ_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_occ_grid.info.origin.orientation.w = 1.0;
            best_occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_occ_grid.data.resize(best_occ_grid.info.width * best_occ_grid.info.height);


            nav_msgs::OccupancyGrid test_grid;
            test_grid = best_occ_grid;

            std::unordered_map<int, nav_msgs::OccupancyGrid> occ_grids_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> regressor_grids_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> variance_grids_by_angle;
            LOG(INFO) << "Creating occ grids";

            double min_value = std::numeric_limits<double>::infinity();
            double max_value = -std::numeric_limits<double>::infinity();

            double variance_min_value = std::numeric_limits<double>::infinity();
            double variance_max_value = -std::numeric_limits<double>::infinity();
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats;
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats_regressor_val;
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats_variance;

            for (int i = -5; i <= 6; i++) {


                nav_msgs::OccupancyGrid occ_grid_variance;
                occ_grid_variance.header.frame_id = kVizFrame;
                occ_grid_variance.header.stamp = ros::Time::now();
                occ_grid_variance.info.resolution = resolution;
                occ_grid_variance.info.origin.position.z = -10 + i;
                occ_grid_variance.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_variance.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_variance.info.origin.orientation.w = 1.0;
                occ_grid_variance.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_variance.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_variance.data.resize(occ_grid_variance.info.width * occ_grid_variance.info.height);
                variance_grids_by_angle[i] = occ_grid_variance;
                output_mats_variance[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                                      best_occ_grid.info.height);


                nav_msgs::OccupancyGrid occ_grid_regressor;
                occ_grid_regressor.header.frame_id = kVizFrame;
                occ_grid_regressor.header.stamp = ros::Time::now();
                occ_grid_regressor.info.resolution = resolution;
                occ_grid_regressor.info.origin.position.z = -10 + i;
                occ_grid_regressor.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_regressor.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_regressor.info.origin.orientation.w = 1.0;
                occ_grid_regressor.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_regressor.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_regressor.data.resize(occ_grid_regressor.info.width * occ_grid_regressor.info.height);
                regressor_grids_by_angle[i] = occ_grid_regressor;
                output_mats_regressor_val[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                                           best_occ_grid.info.height);

                nav_msgs::OccupancyGrid occ_grid;
                occ_grid.header.frame_id = kVizFrame;
                occ_grid.header.stamp = ros::Time::now();
                occ_grid.info.resolution = resolution;
                occ_grid.info.origin.position.z = -10 + i;
                occ_grid.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid.info.origin.orientation.w = 1.0;
                occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);
                occ_grids_by_angle[i] = occ_grid;
                output_mats[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                             best_occ_grid.info.height);
            }

            LOG(INFO) << "Looping through vals";

            int size = best_occ_grid.info.width * best_occ_grid.info.height;
            std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_regressor = gpc->getUnderlyingGpRegressor();
            LOG(INFO) << "Got underlying gp regessor";

            for (int y_val = y_min_unscaled; y_val <= y_max_unscaled; y_val++) {
                for (int x_val = x_min_unscaled; x_val <= x_max_unscaled; x_val++) {

                    long data_index = (best_occ_grid.info.width * (y_val - y_min_unscaled)) + x_val -
                                      x_min_unscaled; // Should I switch x and y?
                    for (int i = -5; i <= 6; i++) {

                        double yaw = i * M_PI / 6;

                        Eigen::Matrix<double, 3, Eigen::Dynamic> object_pose_2d(3, 1);
                        object_pose_2d << (x_val * resolution), (y_val * resolution), yaw;
                        Eigen::Matrix<double, 1, Eigen::Dynamic> gpc_output = gpc->Inference(object_pose_2d);
                        output_mats[i].col(data_index) = gpc_output;

//                        LOG(INFO) << "Getting data from regressor";
                        std::pair<Eigen::Matrix<double, 1, Eigen::Dynamic>, Eigen::Matrix<double, 1, Eigen::Dynamic>> regressor_out =
                                gp_regressor->Inference(object_pose_2d);

                        output_mats_regressor_val[i].col(data_index) = ((regressor_out.first.array() * -1).exp() +
                                                                        1).inverse().matrix();
                        output_mats_variance[i].col(data_index) = regressor_out.second;
//                        LOG(INFO) << "Done getting data from regressor";

                        min_value = std::min(min_value, gpc_output.minCoeff());
                        max_value = std::max(max_value, gpc_output.maxCoeff());

                        variance_min_value = std::min(variance_min_value, regressor_out.second.minCoeff());
                        variance_max_value = std::max(variance_max_value, regressor_out.second.maxCoeff());
                    }
                }
            }

            LOG(INFO) << "Setting occupancy grid data";
            for (int i = 0; i < size; i++) {
                int8_t best_val = 0.0;
                int8_t regressor_val_for_best_val = 0.0;
                int8_t variance_val_for_best_val = 0.0;
                for (int j = -5; j <= 6; j++) {
//                    ros::Publisher pub_for_angle = classifier_for_angle_mult_[i];
                    double inf_val = output_mats[j](0, i);
                    if (inf_val < min_value) {
                        inf_val = min_value;
                    }
                    double regressor_val = (int8_t) 100 * output_mats_regressor_val[j](0, i);
                    double variance_val = (int8_t) (100) * ((output_mats_variance[j](0, i) - variance_min_value) /
                                                            (variance_max_value - variance_min_value));
//                    LOG(INFO) << "Variance val: " << variance_val;

                    if (inf_val > 1) {
                        LOG(INFO) << "Inf val " << inf_val;
                    }
//                    int8_t value = (int8_t) (100) * ((inf_val - min_value) / (max_value - min_value));

                    int8_t value = (int8_t) 100 * inf_val;
                    if (value > best_val) {
//                        LOG(INFO) << "Variance val for best val " << variance_val;
                        best_val = value;
                        regressor_val_for_best_val = regressor_val;
                        variance_val_for_best_val = variance_val;
                    }

                    nav_msgs::OccupancyGrid occ_grid_for_angle = occ_grids_by_angle[j];
                    occ_grid_for_angle.data[i] = value;
                    occ_grids_by_angle[j] = occ_grid_for_angle;

                    nav_msgs::OccupancyGrid regressor_occ_grid_for_angle = regressor_grids_by_angle[j];
                    regressor_occ_grid_for_angle.data[i] = regressor_val;
                    regressor_grids_by_angle[j] = regressor_occ_grid_for_angle;

                    nav_msgs::OccupancyGrid var_occ_grid_for_angle = variance_grids_by_angle[j];
                    var_occ_grid_for_angle.data[i] = variance_val;
                    variance_grids_by_angle[j] = var_occ_grid_for_angle;
                }
                best_occ_grid.data[i] = (int8_t) best_val;
                best_variance_grid.data[i] = (int8_t) variance_val_for_best_val;
                best_regressor_grid.data[i] = (int8_t) regressor_val_for_best_val;
            }

            LOG(INFO) << "Publishing occ grid data";
            std::string occ_data;
            for (size_t i = 0; i < best_occ_grid.data.size(); i++) {
                occ_data += std::to_string(best_occ_grid.data[i]);
                occ_data += ", ";
            }
            LOG(INFO) << "Occ " << occ_data;
            classifier_max_val_for_pos_pub_.publish(best_occ_grid);
            regressor_max_val_for_pos_pub_.publish(best_regressor_grid);
            variance_max_val_for_pos_pub_.publish(best_variance_grid);

            for (int i = -5; i <= 6; i++) {
                ros::Publisher publisher = classifier_for_angle_mult_[i];
                publisher.publish(occ_grids_by_angle[i]);

                ros::Publisher regressor_pub = regressor_for_angle_mult_[i];
                regressor_pub.publish(regressor_grids_by_angle[i]);

                ros::Publisher variance_pub = variance_for_angle_mult_[i];
                variance_pub.publish(variance_grids_by_angle[i]);
            }
        }

        void displayMaxGpRegressorOutput(
                std::function<std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                        const Eigen::Vector2d &)> gpc_creator,
                const double &resolution, const double &x_min, const double &x_max, const double &y_min,
                const double &y_max) {
            int64_t x_min_unscaled = floor(x_min / resolution);
            int64_t x_max_unscaled = ceil(x_max / resolution);
            int64_t y_min_unscaled = floor(y_min / resolution);
            int64_t y_max_unscaled = ceil(y_max / resolution);

            LOG(INFO) << "Creating best occ grid";

            nav_msgs::OccupancyGrid best_variance_grid;
            best_variance_grid.header.frame_id = kVizFrame;
            best_variance_grid.header.stamp = ros::Time::now();
            best_variance_grid.info.resolution = resolution;
            best_variance_grid.info.origin.position.z = -3;
            best_variance_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_variance_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_variance_grid.info.origin.orientation.w = 1.0;
            best_variance_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_variance_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_variance_grid.data.resize(best_variance_grid.info.width * best_variance_grid.info.height);

            nav_msgs::OccupancyGrid best_regressor_grid;
            best_regressor_grid.header.frame_id = kVizFrame;
            best_regressor_grid.header.stamp = ros::Time::now();
            best_regressor_grid.info.resolution = resolution;
            best_regressor_grid.info.origin.position.z = -3;
            best_regressor_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_regressor_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_regressor_grid.info.origin.orientation.w = 1.0;
            best_regressor_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_regressor_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_regressor_grid.data.resize(best_regressor_grid.info.width * best_regressor_grid.info.height);

            nav_msgs::OccupancyGrid best_occ_grid;
            best_occ_grid.header.frame_id = kVizFrame;
            best_occ_grid.header.stamp = ros::Time::now();
            best_occ_grid.info.resolution = resolution;
            best_occ_grid.info.origin.position.z = -3;
            best_occ_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_occ_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_occ_grid.info.origin.orientation.w = 1.0;
            best_occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_occ_grid.data.resize(best_occ_grid.info.width * best_occ_grid.info.height);


            nav_msgs::OccupancyGrid test_grid;
            test_grid = best_occ_grid;

            std::unordered_map<int, nav_msgs::OccupancyGrid> occ_grids_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> regressor_grids_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> variance_grids_by_angle;
            LOG(INFO) << "Creating occ grids";

            double min_value = std::numeric_limits<double>::infinity();
            double max_value = -std::numeric_limits<double>::infinity();

            double variance_min_value = std::numeric_limits<double>::infinity();
            double variance_max_value = -std::numeric_limits<double>::infinity();
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats;
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats_regressor_val;
            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats_variance;
            Eigen::Matrix<double, 1, Eigen::Dynamic> close_to_parking_spots_mat = Eigen::Matrix<double, 1, Eigen::Dynamic>(
                    1, best_occ_grid.info.width *
                       best_occ_grid.info.height);

            std::vector<Eigen::Vector2d> parking_spots;
            parking_spots.emplace_back(Eigen::Vector2d(5.430979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(8.630979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(11.830979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(15.030979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(1.160000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(3.894000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(6.628000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(9.362000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(12.096000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(14.830000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(17.564000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(20.298000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(23.032000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(25.766000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(28.400000, 10.500000));
            parking_spots.emplace_back(Eigen::Vector2d(18.230979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(21.430979, -4.871888));
            parking_spots.emplace_back(Eigen::Vector2d(2.900000, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(5.987500, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(9.075000, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(12.162500, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(15.250000, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(18.337500, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(21.425000, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(24.512500, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(27.600000, 4.870000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.610000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(1.129000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(3.868000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(6.607000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(9.346000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(12.085000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(14.825000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(17.564000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(20.303000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(23.042000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(25.781000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(28.520000, 25.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-15.160000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-18.240000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-21.320000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-24.400000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-27.480000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-30.560000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-33.640000, 20.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-27.220000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-30.270000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-33.320000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-36.370000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-39.420000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-42.470000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-45.520000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-48.570000, 8.240000));
            parking_spots.emplace_back(Eigen::Vector2d(-15.510000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-18.500000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-21.500000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-24.490000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-27.480000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-30.470000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-33.470000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-36.460000, 25.230000));
            parking_spots.emplace_back(Eigen::Vector2d(-39.770000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-36.680000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-33.590000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-30.500000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-27.410000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-24.320000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-21.230000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-18.140000, 34.940000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 32.870000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 35.730000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 38.590000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 41.450000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 44.308414));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 47.170000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 50.030000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 52.893948));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 55.750000));
            parking_spots.emplace_back(Eigen::Vector2d(-14.703172, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-17.846000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-20.992000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-24.138000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-27.284000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-30.431709, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-33.576000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-36.722000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-39.868000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-43.014000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-46.160000, 50.821438));
            parking_spots.emplace_back(Eigen::Vector2d(-15.380000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-18.438442, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-21.480000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-24.530000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-27.580000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-30.640000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-33.690000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-36.740000, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-39.803191, 39.666271));
            parking_spots.emplace_back(Eigen::Vector2d(-42.812000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-39.660316, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-36.508000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-33.356000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-30.204000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-27.052000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-23.900000, 55.475323));
            parking_spots.emplace_back(Eigen::Vector2d(-34.020012, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-30.948000, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-27.876000, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-24.804000, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-21.732000, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-18.665812, 65.287094));
            parking_spots.emplace_back(Eigen::Vector2d(-56.997650, 55.184795));
            parking_spots.emplace_back(Eigen::Vector2d(-55.450000, 56.956000));
            parking_spots.emplace_back(Eigen::Vector2d(-53.904000, 58.728000));
            parking_spots.emplace_back(Eigen::Vector2d(-52.357000, 60.499000));
            parking_spots.emplace_back(Eigen::Vector2d(-50.810000, 62.270000));
            parking_spots.emplace_back(Eigen::Vector2d(-49.263000, 64.040000));
            parking_spots.emplace_back(Eigen::Vector2d(-47.716000, 65.814000));
            parking_spots.emplace_back(Eigen::Vector2d(-46.169000, 67.585000));
            parking_spots.emplace_back(Eigen::Vector2d(-44.622000, 69.357000));
            parking_spots.emplace_back(Eigen::Vector2d(-43.076000, 71.130000));
            parking_spots.emplace_back(Eigen::Vector2d(-41.529000, 72.899800));
            parking_spots.emplace_back(Eigen::Vector2d(-39.982000, 74.671000));
            parking_spots.emplace_back(Eigen::Vector2d(-38.435000, 76.443000));
            parking_spots.emplace_back(Eigen::Vector2d(-36.888000, 78.214000));
            parking_spots.emplace_back(Eigen::Vector2d(-35.341000, 79.986000));
            parking_spots.emplace_back(Eigen::Vector2d(-33.794000, 81.757000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 66.510000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 69.410000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 72.310000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 75.210000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 78.110000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 81.010000));
            parking_spots.emplace_back(Eigen::Vector2d(-1.717181, 83.910000));

            for (int i = -5; i <= 6; i++) {


                nav_msgs::OccupancyGrid occ_grid_variance;
                occ_grid_variance.header.frame_id = kVizFrame;
                occ_grid_variance.header.stamp = ros::Time::now();
                occ_grid_variance.info.resolution = resolution;
                occ_grid_variance.info.origin.position.z = -10 + i;
                occ_grid_variance.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_variance.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_variance.info.origin.orientation.w = 1.0;
                occ_grid_variance.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_variance.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_variance.data.resize(occ_grid_variance.info.width * occ_grid_variance.info.height);
                variance_grids_by_angle[i] = occ_grid_variance;
                output_mats_variance[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                                      best_occ_grid.info.height);


                nav_msgs::OccupancyGrid occ_grid_regressor;
                occ_grid_regressor.header.frame_id = kVizFrame;
                occ_grid_regressor.header.stamp = ros::Time::now();
                occ_grid_regressor.info.resolution = resolution;
                occ_grid_regressor.info.origin.position.z = -10 + i;
                occ_grid_regressor.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_regressor.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_regressor.info.origin.orientation.w = 1.0;
                occ_grid_regressor.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_regressor.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_regressor.data.resize(occ_grid_regressor.info.width * occ_grid_regressor.info.height);
                regressor_grids_by_angle[i] = occ_grid_regressor;
                output_mats_regressor_val[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                                           best_occ_grid.info.height);

                nav_msgs::OccupancyGrid occ_grid;
                occ_grid.header.frame_id = kVizFrame;
                occ_grid.header.stamp = ros::Time::now();
                occ_grid.info.resolution = resolution;
                occ_grid.info.origin.position.z = -10 + i;
                occ_grid.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid.info.origin.orientation.w = 1.0;
                occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);
                occ_grids_by_angle[i] = occ_grid;
                output_mats[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                             best_occ_grid.info.height);


                output_mats[i] = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, best_occ_grid.info.width *
                                                                             best_occ_grid.info.height);
            }

            LOG(INFO) << "Looping through vals";

            int size = best_occ_grid.info.width * best_occ_grid.info.height;
            LOG(INFO) << "Got underlying gp regessor";

            for (int y_val = y_min_unscaled; y_val <= y_max_unscaled; y_val++) {
                for (int x_val = x_min_unscaled; x_val <= x_max_unscaled; x_val++) {

                    long data_index = (best_occ_grid.info.width * (y_val - y_min_unscaled)) + x_val -
                                      x_min_unscaled; // Should I switch x and y?

                    bool close_to_parking_spot = false;
                    for (const Eigen::Vector2d &parking_spot : parking_spots) {
                        if ((parking_spot - Eigen::Vector2d((x_val * resolution), (y_val * resolution))).norm() <
                            0.25) {
                            close_to_parking_spot = true;
                            continue;
                        }
                    }
                    close_to_parking_spots_mat(0, data_index) = close_to_parking_spot ? 1 : 0;

                    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gpc = gpc_creator(
                            Eigen::Vector2d(x_val * resolution, y_val * resolution));
                    std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_regressor;
                    if (gpc) {
                        gp_regressor = gpc->getUnderlyingGpRegressor();
                    }
                    for (int i = -5; i <= 6; i++) {

                        double yaw = i * M_PI / 6;

                        Eigen::Matrix<double, 3, Eigen::Dynamic> object_pose_2d(3, 1);
                        object_pose_2d << (x_val * resolution), (y_val * resolution), yaw;
                        Eigen::Matrix<double, 1, Eigen::Dynamic> gpc_output;
                        if (gpc) {
                            gpc_output = gpc->Inference(object_pose_2d);
                        } else {
                            gpc_output = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, 1);
                            gpc_output << 0.0;
                        }
                        output_mats[i].col(data_index) = gpc_output;

                        std::pair<Eigen::Matrix<double, 1, Eigen::Dynamic>, Eigen::Matrix<double, 1, Eigen::Dynamic>> regressor_out;
                        if (gpc) {
                            regressor_out = gp_regressor->Inference(object_pose_2d);
                        } else {
                            Eigen::Matrix<double, 1, Eigen::Dynamic> regressor_val(1, 1);
                            regressor_val << 0.0;
                            Eigen::Matrix<double, 1, Eigen::Dynamic> variance_val(1, 1);
                            variance_val << 0.0;
                            regressor_out = std::make_pair(regressor_val, variance_val);
                        }

                        output_mats_regressor_val[i].col(data_index) = ((regressor_out.first.array() * -1).exp() +
                                                                        1).inverse().matrix();
                        output_mats_variance[i].col(data_index) = regressor_out.second;
                        if (close_to_parking_spot) {
                            LOG(INFO) << "Class, reg, var : " << gpc_output << ", "
                                      << output_mats_regressor_val[i].col(data_index) << ", "
                                      << output_mats_variance[i].col(data_index);
                        }

                        if (gpc) {
                            min_value = std::min(min_value, gpc_output.minCoeff());
                            max_value = std::max(max_value, gpc_output.maxCoeff());

                            variance_min_value = std::min(variance_min_value, regressor_out.second.minCoeff());
                            variance_max_value = std::max(variance_max_value, regressor_out.second.maxCoeff());
                        }
                    }
                }
            }

            LOG(INFO) << "Setting occupancy grid data";
            for (int i = 0; i < size; i++) {
                int8_t best_val = 0.0;
                int8_t regressor_val_for_best_val = 0.0;
                int8_t variance_val_for_best_val = 0.0;
                for (int j = -5; j <= 6; j++) {
//                    ros::Publisher pub_for_angle = classifier_for_angle_mult_[i];
                    double inf_val = output_mats[j](0, i);
                    double regressor_val = (int8_t) 100 * output_mats_regressor_val[j](0, i);
                    double variance_val =
                            (int8_t) (100) * std::max(0.0, ((output_mats_variance[j](0, i) - variance_min_value) /
                                                            (variance_max_value - variance_min_value)));


//                    LOG(INFO) << "Variance val: " << variance_val;

                    if (inf_val > 1) {
                        LOG(INFO) << "Inf val " << inf_val;
                    }
//                    int8_t value = (int8_t) (100) * ((inf_val - min_value) / (max_value - min_value));

                    int8_t value = (int8_t) 100 * inf_val;
                    if (value > best_val) {
//                        LOG(INFO) << "Variance val for best val " << variance_val;
                        best_val = value;
                        regressor_val_for_best_val = regressor_val;
                        variance_val_for_best_val = variance_val;
                    }

                    nav_msgs::OccupancyGrid occ_grid_for_angle = occ_grids_by_angle[j];
                    occ_grid_for_angle.data[i] = value;
                    occ_grids_by_angle[j] = occ_grid_for_angle;

                    nav_msgs::OccupancyGrid regressor_occ_grid_for_angle = regressor_grids_by_angle[j];
                    regressor_occ_grid_for_angle.data[i] = regressor_val;
                    regressor_grids_by_angle[j] = regressor_occ_grid_for_angle;

                    nav_msgs::OccupancyGrid var_occ_grid_for_angle = variance_grids_by_angle[j];
                    var_occ_grid_for_angle.data[i] = variance_val;
                    variance_grids_by_angle[j] = var_occ_grid_for_angle;
                }
                best_occ_grid.data[i] = (int8_t) best_val;
                best_variance_grid.data[i] = (int8_t) variance_val_for_best_val;
                best_regressor_grid.data[i] = (int8_t) regressor_val_for_best_val;
            }

            LOG(INFO) << "Publishing occ grid data";
            std::string occ_data;
            for (size_t i = 0; i < best_occ_grid.data.size(); i++) {
                occ_data += std::to_string(best_occ_grid.data[i]);
                occ_data += ", ";
            }
            LOG(INFO) << "Occ " << occ_data;
            classifier_max_val_for_pos_pub_.publish(best_occ_grid);
            regressor_max_val_for_pos_pub_.publish(best_regressor_grid);
            variance_max_val_for_pos_pub_.publish(best_variance_grid);

            for (int i = -5; i <= 6; i++) {
                ros::Publisher publisher = classifier_for_angle_mult_[i];
                publisher.publish(occ_grids_by_angle[i]);

                ros::Publisher regressor_pub = regressor_for_angle_mult_[i];
                regressor_pub.publish(regressor_grids_by_angle[i]);

                ros::Publisher variance_pub = variance_for_angle_mult_[i];
                variance_pub.publish(variance_grids_by_angle[i]);
            }
        }

//        void displayPoseResiduals(pose_optimization::MovableObservationCostFunctor cost_functor,
        template<typename FactorType>
        void displayPoseResiduals(FactorType cost_functor,
                                  const double &resolution, const double &x_min, const double &x_max,
                                  const double &y_min,
                                  const double &y_max) {
            int64_t x_min_unscaled = floor(x_min / resolution);
            int64_t x_max_unscaled = ceil(x_max / resolution);
            int64_t y_min_unscaled = floor(y_min / resolution);
            int64_t y_max_unscaled = ceil(y_max / resolution);

            nav_msgs::OccupancyGrid best_occ_grid;
            best_occ_grid.header.frame_id = kVizFrame;
            best_occ_grid.header.stamp = ros::Time::now();
            best_occ_grid.info.resolution = resolution;
            best_occ_grid.info.origin.position.z = -18;
            best_occ_grid.info.origin.position.x = x_min_unscaled * resolution;
            best_occ_grid.info.origin.position.y = y_min_unscaled * resolution;
            best_occ_grid.info.origin.orientation.w = 1.0;
            best_occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
            best_occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
            best_occ_grid.data.resize(best_occ_grid.info.width * best_occ_grid.info.height);

            nav_msgs::OccupancyGrid test_grid;
            test_grid = best_occ_grid;

            std::unordered_map<int, nav_msgs::OccupancyGrid> occ_grids_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> occ_grids_resolution_by_angle;
            std::unordered_map<int, nav_msgs::OccupancyGrid> occ_grids_variance_by_angle;

            for (int i = -5; i <= 6; i++) {
                nav_msgs::OccupancyGrid occ_grid_regressor;
                occ_grid_regressor.header.frame_id = kVizFrame;
                occ_grid_regressor.header.stamp = ros::Time::now();
                occ_grid_regressor.info.resolution = resolution;
                occ_grid_regressor.info.origin.position.z = -25 + i;
                occ_grid_regressor.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_regressor.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_regressor.info.origin.orientation.w = 1.0;
                occ_grid_regressor.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_regressor.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_regressor.data.resize(occ_grid_regressor.info.width * occ_grid_regressor.info.height);
                occ_grids_resolution_by_angle[i] = occ_grid_regressor;

                nav_msgs::OccupancyGrid occ_grid_variance;
                occ_grid_variance.header.frame_id = kVizFrame;
                occ_grid_variance.header.stamp = ros::Time::now();
                occ_grid_variance.info.resolution = resolution;
                occ_grid_variance.info.origin.position.z = -25 + i;
                occ_grid_variance.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid_variance.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid_variance.info.origin.orientation.w = 1.0;
                occ_grid_variance.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid_variance.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid_variance.data.resize(occ_grid_variance.info.width * occ_grid_variance.info.height);
                occ_grids_variance_by_angle[i] = occ_grid_variance;

                nav_msgs::OccupancyGrid occ_grid;
                occ_grid.header.frame_id = kVizFrame;
                occ_grid.header.stamp = ros::Time::now();
                occ_grid.info.resolution = resolution;
                occ_grid.info.origin.position.z = -25 + i;
                occ_grid.info.origin.position.x = x_min_unscaled * resolution;
                occ_grid.info.origin.position.y = y_min_unscaled * resolution;
                occ_grid.info.origin.orientation.w = 1.0;
                occ_grid.info.width = x_max_unscaled - x_min_unscaled + 1;
                occ_grid.info.height = y_max_unscaled - y_min_unscaled + 1;
                occ_grid.data.resize(occ_grid.info.width * occ_grid.info.height);
                occ_grids_by_angle[i] = occ_grid;
            }

            int size = best_occ_grid.info.width * best_occ_grid.info.height;

            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats;
            for (int i = -5; i <= 6; i++) {

                double yaw = i * M_PI / 6;
                LOG(INFO) << "Grid yaw for i=" << i << ": " << yaw;

                Eigen::Quaterniond quat(cos(yaw / 2), 0, 0, sin(yaw / 2));
//                LOG(INFO) << "Quaternion " << quat.coeffs();
                Eigen::Matrix<double, 1, Eigen::Dynamic> output_mat = Eigen::Matrix<double, 1, Eigen::Dynamic>(1, size);
                for (int y_val = y_min_unscaled; y_val <= y_max_unscaled; y_val++) {
                    for (int x_val = x_min_unscaled; x_val <= x_max_unscaled; x_val++) {

                        long data_index = (best_occ_grid.info.width * (y_val - y_min_unscaled)) + x_val -
                                          x_min_unscaled; // Should I switch x and y?

//                    long data_index = (best_occ_grid.info.height * (x_val - x_min_unscaled)) + y_val - y_min_unscaled; // Should I switch x and y?


                        Eigen::Matrix<double, 3, 1> object_pose_2d;
                        double residual;
                        Eigen::Vector3d transl(x_val * resolution, y_val * resolution, 0);
//                        LOG(INFO) << "Transl for robot: " << transl;


                        cost_functor(transl.data(), quat.coeffs().data(), &residual);
//
//                        if (residual < 0) {
//                            LOG(INFO) << "Neg residual " << residual;
//                        }
//
//                        output_mat(0, data_index) = std::exp(-residual);
                        if (residual < 100) {

                            output_mat(0, data_index) = residual;
                        } else {
                            output_mat(0, data_index) = 100;
                        }
//                        mats_by_angle[i] = mat_for_angle;


                    }
                }
                output_mats[i] = output_mat;
            }

            for (int i = 0; i < size; i++) {
                double best_val = 100.0;
                for (int j = -5; j <= 6; j++) {
                    ros::Publisher pub_for_angle = classifier_for_angle_mult_[i];
                    double inf_val = output_mats[j](0, i);
                    if (inf_val < best_val) {
                        best_val = inf_val;
                    }

//                    if (inf_val > 1) {
//                        LOG(INFO) << "Inf val " << inf_val;
//                    }


                    nav_msgs::OccupancyGrid occ_grid_for_angle = occ_grids_by_angle[j];
//                    LOG(INFO) << "Double val " << inf_val;
//                    LOG(INFO) << std::to_string(((int8_t) (100 * inf_val)));
//                    occ_grid_for_angle.data[i] = (int8_t) (100 * inf_val);
                    occ_grid_for_angle.data[i] = (int8_t) (inf_val);
//                    LOG(INFO) << std::to_string(occ_grid_for_angle.data[i]);
                    occ_grids_by_angle[j] = occ_grid_for_angle;
                }

                best_occ_grid.data[i] = (int8_t) (best_val);
//                best_occ_grid.data[i] = (int8_t) (100 * best_val);
            }

//            LOG(INFO) << "Publishing occ grid data";
            std::string occ_data;
            for (size_t i = 0; i < best_occ_grid.data.size(); i++) {
                occ_data += std::to_string(best_occ_grid.data[i]);
                occ_data += ", ";
            }
//            LOG(INFO) << "Occ " << occ_data;
            robot_pose_max_val_pub_.publish(best_occ_grid);
            for (int i = -5; i <= 6; i++) {
                ros::Publisher publisher = robot_pose_pub_for_angle_mult_[i];

                publisher.publish(occ_grids_by_angle[i]);
            }
        }

        // 2D Variants ------------------------------------------------------------------------------------------------
        void displayTrueTrajectory(const std::vector<pose::Pose2d> &true_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            publishTrajectory(gt_marker_pub_, color, convert2DPosesTo3D(true_trajectory), kGtTrajectoryId,
                              kRobotGtPosesMin, kRobotGtPosesMax);
        }

        void displayOtherTrajectory(const std::vector<pose::Pose2d> &other_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;
            color.r = 1.0;

            // TODO reconsider the id situations here since this isn't ground truth (getting around it now since
            publishTrajectory(other_marker_pub_, color, convert2DPosesTo3D(other_trajectory), kGtTrajectoryId,
                              kRobotGtPosesMin, kRobotGtPosesMax);
        }

        void displayTrueObjPoses(const std::vector<pose::Pose2d> &true_car_poses, const std::string &obj_class) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(GROUND_TRUTH, obj_class, pub);

            int32_t car_poses_count = true_car_poses.size();
            for (int32_t i = 0; i < car_poses_count; i++) {
                pose::Pose3d pose = pose::toPose3d(true_car_poses[i]);
                publishCarPoses(pub, pose, color, kCarGtPosesMin + i);
            }

//            for (int32_t i = kCarGtPosesMin + car_poses_count; i <= kCarGtPosesMax; i++) {
//                removeMarker(i, gt_marker_pub_);
//            }
        }

        void displayObjObservationsFromGtTrajectory(const std::vector<pose::Pose2d> &gt_trajectory,
                                                    const std::vector<std::vector<pose::Pose2d>> &relative_car_poses,
                                                    const std::string &obj_class) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.g = 1.0;
            color.b = 1.0;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(GROUND_TRUTH, obj_class, pub);

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(pub, convert2DPosesTo3D(gt_trajectory), relative_car_poses_3d, color,
                                        kObservedFromGtCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, convert2DPosesTo3D(gt_trajectory),
                                                relative_car_poses_3d, color,
                                                kObservedFromGtCarDetectionsMin, kObservedFromGtCarDetectionsMax);
        }

        void displaySemanticPointObsFromGtTrajectory(const std::vector<pose::Pose2d> &gt_trajectory,
                                                     const std::vector<std::vector<std::vector<Eigen::Vector2d>>> &relative_semantic_points,
                                                     const std::string &obj_class,
                                                     const std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> &relative_object_samples_for_cluster = {},
                                                     const Eigen::Vector2d &dimensions_for_samples = Eigen::Vector2d()) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.g = 1.0;
            color.b = 1.0;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(GROUND_TRUTH, obj_class, pub);

            displaySemanticPointObsFromTrajectory(gt_trajectory, relative_semantic_points, 0.15,
                                                  kObservedFromGtCarDetectionLines, color, pub,
                                                  relative_object_samples_for_cluster, dimensions_for_samples);
        }

        void displayPastSampleValues(const std::string &obj_class,
                                     const std::vector<std::pair<pose::Pose2d, double>> &sample_poses_and_values) {
            ros::Publisher publisher;
            getOrCreatePublisherForSamples(obj_class, publisher);

            visualization_msgs::Marker marker_msg;
            marker_msg.type = visualization_msgs::Marker::Type::TRIANGLE_LIST;

            geometry_msgs::Point triangle_corner_1;
            triangle_corner_1.x = -0.05;
            triangle_corner_1.y = -0.05;
            geometry_msgs::Point triangle_corner_2;
            triangle_corner_2.x = -0.05;
            triangle_corner_2.y = 0.05;
            geometry_msgs::Point triangle_corner_3;
            triangle_corner_3.x = 0.1;

            for (size_t i = 0; i < sample_poses_and_values.size(); i++) {
                std::pair<pose::Pose2d, double> pose_and_value = sample_poses_and_values.at(i);
                std_msgs::ColorRGBA color;
                color.a = 1.0;
                color.b = 1.0 - pose_and_value.second;
                color.r = pose_and_value.second;
                Eigen::Vector2d sample_position = pose_and_value.first.first;
                double sample_orientation = pose_and_value.first.second;
                double sin_orientation = sin(sample_orientation);
                double cos_orientation = cos(sample_orientation);
                geometry_msgs::Point p1;
                p1.x = sample_position.x() + (triangle_corner_1.x * cos_orientation) -
                       (sin_orientation * triangle_corner_1.y);
                p1.y = sample_position.y() + (triangle_corner_1.x * sin_orientation) +
                       (triangle_corner_1.y * cos_orientation);

                geometry_msgs::Point p2;
                p2.x = sample_position.x() + (triangle_corner_2.x * cos_orientation) -
                       (triangle_corner_2.y * sin_orientation);
                p2.y = sample_position.y() + (triangle_corner_2.x * sin_orientation) +
                       (triangle_corner_2.y * cos_orientation);

                geometry_msgs::Point p3;
                p3.x = sample_position.x() + (triangle_corner_3.x * cos_orientation) -
                       (triangle_corner_3.y * sin_orientation);
                p3.y = sample_position.y() + (triangle_corner_3.x * sin_orientation) +
                       (triangle_corner_3.y * cos_orientation);

                for (int j = 0; j < 6; j++) {
                    marker_msg.colors.emplace_back(color);
                }
                marker_msg.points.emplace_back(p1);
                marker_msg.points.emplace_back(p2);
                marker_msg.points.emplace_back(p3);
                marker_msg.points.emplace_back(p1);
                marker_msg.points.emplace_back(p3);
                marker_msg.points.emplace_back(p2);
            }
            marker_msg.scale.x = 1.0;
            marker_msg.scale.y = 1.0;
            marker_msg.scale.z = 1.0;

            marker_msg.id = 1;
            marker_msg.pose.orientation.w = 1.0;

            publishMarker(marker_msg, publisher);
        }

        void displayObjObservationsFromEstTrajectory(const std::vector<pose::Pose2d> &est_trajectory,
                                                     const std::vector<std::vector<pose::Pose2d>> &relative_car_poses,
                                                     const std::string &obj_class) {
            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.r = 1.0;
            color.g = 0.7;

            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ESTIMATED, obj_class, pub);

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(pub, convert2DPosesTo3D(est_trajectory),
                                        relative_car_poses_3d, color, kObservedFromEstCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, convert2DPosesTo3D(est_trajectory),
                                                relative_car_poses_3d, color,
                                                kObservedFromEstCarDetectionsMin, kObservedFromEstCarDetectionsMax);
        }

        void displayObjObservationsFromOdomTrajectory(const std::vector<pose::Pose2d> &odom_trajectory,
                                                      const std::vector<std::vector<pose::Pose2d>> &relative_car_poses,
                                                      const std::string &obj_class) {

            std_msgs::ColorRGBA color;
            color.a = 0.5;
            color.b = 1.0;
            color.r = 0.7;
            color.g = 0.2;


            ros::Publisher pub;
            getOrCreatePublisherForTrajTypeAndClass(ODOM_ONLY, obj_class, pub);

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(pub, convert2DPosesTo3D(odom_trajectory), relative_car_poses_3d, color,
                                        kObservedFromOdomCarDetectionLines);
            publishObjDetectionsRelToRobotPoses(pub, convert2DPosesTo3D(odom_trajectory), relative_car_poses_3d, color,
                                                kObservedFromOdomCarDetectionsMin, kObservedFromOdomCarDetectionsMax);
        }

        void displayOdomTrajectory(const std::vector<pose::Pose2d> &odom_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;

            publishTrajectory(odom_marker_pub_, color, convert2DPosesTo3D(odom_trajectory), kOdomTrajectoryId,
                              kRobotOdomPosesMin, kRobotOdomPosesMax);
        }

        void displayEstTrajectory(const std::vector<pose::Pose2d> &est_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;

            publishTrajectory(est_marker_pub_, color, convert2DPosesTo3D(est_trajectory), kEstTrajectoryId,
                              kRobotEstPosesMin, kRobotEstPosesMax);
        }

        static std::pair<Eigen::Vector2d, Eigen::Vector2d>
        getMinMaxCornersForDistributionVisualization(const std::vector<pose::Pose2d> &map_frame_obj_poses) {
            double min_x = std::numeric_limits<double>::infinity();
            double max_x = -std::numeric_limits<double>::infinity();
            double min_y = std::numeric_limits<double>::infinity();
            double max_y = -std::numeric_limits<double>::infinity();

            for (const pose::Pose2d &pose_2d : map_frame_obj_poses) {
                min_x = std::min(pose_2d.first.x(), min_x);
                max_x = std::max(pose_2d.first.x(), max_x);
                min_y = std::min(pose_2d.first.y(), min_y);
                max_y = std::max(pose_2d.first.y(), max_y);
            }

            LOG(INFO) << "Min x, max x, min y, max y: " << min_x << ", " << max_x << ", " << min_y << ", " << max_y;

            return std::make_pair(Eigen::Vector2d(min_x - kExtraMarginDistribution, min_y - kExtraMarginDistribution),
                                  Eigen::Vector2d(max_x + kExtraMarginDistribution, max_y + kExtraMarginDistribution));
        }

        void plotWaypoints(const std::vector<std::vector<std::vector<pose::Pose2d>>> &waypoints) {

            int32_t next_robot_pose_id = kWaypointsPosesMin;

            std::vector<std_msgs::ColorRGBA> colors;
            for (size_t i = 0; i < waypoints.size(); i++) {

                std_msgs::ColorRGBA color;
                color.a = 1.0;
                color.g = 1.0;
                std::vector<std::vector<pose::Pose2d>> poses_for_traj = waypoints[i];

                size_t half_poses = poses_for_traj.size() / 2;
                for (size_t j = 0; j < poses_for_traj.size(); j++) {
                    if (j % 2) {
                        color.b = ((double) (j / 2)) / half_poses;
                        color.g = 1.0;
                    } else {
                        color.b = 1.0;
                        color.g = ((double) ((poses_for_traj.size() - j) / 2)) / half_poses;
                    }
                    LOG(INFO) << "j: " << j << ", color: " << color;
                    std::vector<pose::Pose3d> poses_3d = convert2DPosesTo3D(poses_for_traj[j]);
                    for (pose::Pose3d &pose_3d : poses_3d) {
                        pose_3d = std::make_pair(
                                Eigen::Vector3d(pose_3d.first.x(), pose_3d.first.y(), kWaypoint2DHeight),
                                pose_3d.second);
                        publishRobotPose(waypoint_pub_, pose_3d, color, next_robot_pose_id++, true);
                    }
                }
            }
        }

        void publishEstimatedTrajectories(const std::vector<std::vector<pose::Pose2d>> &trajectory_poses) {
            std_msgs::ColorRGBA base_color;
//            base_color.a = 0.3;
            base_color.a = 1.0;
            base_color.r = 1.0;

            std::vector<std::vector<pose::Pose3d>> trajectory_poses_3d;

            std::vector<std_msgs::ColorRGBA> colors;
            for (size_t i = 0; i < trajectory_poses.size(); i++) {
                std_msgs::ColorRGBA color = base_color;
                color.b = ((double) i) / (trajectory_poses.size() - 1);
                colors.emplace_back(color);

                trajectory_poses_3d.emplace_back(convert2DPosesTo3D(trajectory_poses[i]));
            }

            publishTrajectories(est_marker_pub_, colors, trajectory_poses_3d, kEstTrajectoryId, kRobotEstPosesMin,
                                kRobotEstPosesMax);
        }

    private:

        const std::string kVizFrame = "map";

        static constexpr const double kExtraMarginDistribution = 2.5;

        const uint32_t kObservationPubQueueSize = 100000;

        const double kTrajectoryScaleX = 0.05;

        const int32_t kEstTrajectoryId = 1;

        const int32_t kOdomTrajectoryId = 2;

        const int32_t kGtTrajectoryId = 3;

        const int32_t kObservedFromGtCarDetectionLines = 4;
        const int32_t kObservedFromOdomCarDetectionLines = 5;
        const int32_t kObservedFromEstCarDetectionLines = 6;

        // TODO might need to increase these
        const int32_t kMaxObservationsToDisplay = 2000;
        const int32_t kMaxTrajectoryLen = 50000;
        const int32_t kObservedFromGtCarDetectionsMin = 100;
        const int32_t kObservedFromOdomCarDetectionsMin = kObservedFromGtCarDetectionsMin + kMaxObservationsToDisplay;
        const int32_t kObservedFromEstCarDetectionsMin = kObservedFromOdomCarDetectionsMin + kMaxObservationsToDisplay;
        const int32_t kCarGtPosesMin = kObservedFromEstCarDetectionsMin + kMaxObservationsToDisplay;
        const int32_t kRobotGtPosesMin = kCarGtPosesMin + kMaxObservationsToDisplay;
        const int32_t kRobotOdomPosesMin = kRobotGtPosesMin + kMaxTrajectoryLen;
        const int32_t kRobotEstPosesMin = kRobotOdomPosesMin + kMaxTrajectoryLen;
        const int32_t kWaypointsPosesMin = kRobotEstPosesMin + kMaxTrajectoryLen;
        const int32_t kSemanticPointsMin = kWaypointsPosesMin + kMaxTrajectoryLen;

        const int32_t kObservedFromGtCarDetectionsMax = kObservedFromOdomCarDetectionsMin - 1;
        const int32_t kObservedFromOdomCarDetectionsMax = kObservedFromEstCarDetectionsMin - 1;
        const int32_t kObservedFromEstCarDetectionsMax = kCarGtPosesMin - 1;
        const int32_t kCarGtPosesMax = kRobotGtPosesMin - 1;
        const int32_t kRobotGtPosesMax = kRobotOdomPosesMin - 1;
        const int32_t kRobotOdomPosesMax = kRobotEstPosesMin - 1;
        const int32_t kRobotEstPosesMax = kWaypointsPosesMin - 1;
        const int32_t kWaypointsPosesMax = kWaypointsPosesMin + kMaxTrajectoryLen - 1;
        const int32_t kSemanticPointsMax = kSemanticPointsMin + kMaxTrajectoryLen - 1;

        const double kWaypoint2DHeight = 0.2;


        /**
         * Node handle.
         */
        ros::NodeHandle node_handle_;

        std::string prefix_;

        ros::Publisher other_marker_pub_;
        ros::Publisher gt_marker_pub_;
        ros::Publisher est_marker_pub_;
        ros::Publisher odom_marker_pub_;

        std::unordered_map<TrajectoryType, std::unordered_map<std::string, ros::Publisher>> obs_marker_pubs_by_class_by_traj_type_;
        std::unordered_map<std::string, ros::Publisher> sample_pubs_by_class_;

        std::unordered_map<int32_t, ros::Publisher> classifier_for_angle_mult_;
        std::unordered_map<int32_t, ros::Publisher> variance_for_angle_mult_;
        std::unordered_map<int32_t, ros::Publisher> regressor_for_angle_mult_;
        std::unordered_map<int32_t, ros::Publisher> robot_pose_pub_for_angle_mult_;

        ros::Publisher robot_pose_max_val_pub_;
        ros::Publisher waypoint_pub_;

        ros::Publisher parking_spot_3d_pub_;

        ros::Publisher regressor_max_val_for_pos_pub_;
        ros::Publisher classifier_max_val_for_pos_pub_;
        ros::Publisher variance_max_val_for_pos_pub_;

        std::string getStringRepForTrajType(const TrajectoryType &trajectory_type) {
            switch (trajectory_type) {
                case ODOM_ONLY:
                    return "odom";
                case GROUND_TRUTH:
                    return "gt";
                case ESTIMATED:
                    return "est";
                default:
                    LOG(WARNING) << "Invalid trajectory type " << trajectory_type;
                    return "";
            }
        }

        void
        getOrCreatePublisherForTrajTypeAndClass(const TrajectoryType &trajectory_type, const std::string &obj_class,
                                                ros::Publisher &pub) {
            std::unordered_map<std::string, ros::Publisher> pubs_for_traj_type_;
            if (obs_marker_pubs_by_class_by_traj_type_.find(trajectory_type) !=
                obs_marker_pubs_by_class_by_traj_type_.end()) {
                pubs_for_traj_type_ = obs_marker_pubs_by_class_by_traj_type_[trajectory_type];
            }

            if (pubs_for_traj_type_.find(obj_class) == pubs_for_traj_type_.end()) {
                std::string topic_name =
                        prefix_ + getStringRepForTrajType(trajectory_type) + "_" + obj_class + "_obs_marker";
                ros::Publisher new_pub_for_class = node_handle_.advertise<visualization_msgs::Marker>(topic_name,
                                                                                                      kObservationPubQueueSize);
                pubs_for_traj_type_[obj_class] = new_pub_for_class;
                obs_marker_pubs_by_class_by_traj_type_[trajectory_type] = pubs_for_traj_type_;
                ros::Duration(2).sleep();
                for (int id = 0; id < kRobotEstPosesMax; id++) {
                    removeMarker(id, new_pub_for_class);
                    ros::Duration(0.0001).sleep();
                }
                ros::Duration(1).sleep();
                // TODO consider removing all existing markers on this topic
            }

            pub = obs_marker_pubs_by_class_by_traj_type_[trajectory_type][obj_class];
        }

        void getOrCreatePublisherForSamples(const std::string &obj_class, ros::Publisher &pub) {

            if (sample_pubs_by_class_.find(obj_class) == sample_pubs_by_class_.end()) {
                std::string topic_name = prefix_ + obj_class + "_past_sample_markers";
                ros::Publisher new_pub_for_class = node_handle_.advertise<visualization_msgs::Marker>(topic_name,
                                                                                                      50000);
                sample_pubs_by_class_[obj_class] = new_pub_for_class;
                ros::Duration(2).sleep();
                for (int id = 0; id < kRobotEstPosesMax; id++) {
                    removeMarker(id, new_pub_for_class);
                    ros::Duration(0.0001).sleep();
                }
                ros::Duration(1).sleep();
                // TODO consider removing all existing markers on this topic
            }

            pub = sample_pubs_by_class_[obj_class];
        }

        std::vector<pose::Pose3d> convert2DPosesTo3D(const std::vector<pose::Pose2d> &poses_2d) {
            std::vector<pose::Pose3d> poses_3d;
            for (const pose::Pose2d &pose_2d : poses_2d) {
                poses_3d.emplace_back(pose::toPose3d(pose_2d));
            }
            return poses_3d;
        }

        std::vector<Eigen::Vector3d> convert2DPointsTo3D(const std::vector<Eigen::Vector2d> &points_2d) {
            std::vector<Eigen::Vector3d> points_3d;
            for (const Eigen::Vector2d &point_2d : points_2d) {
                points_3d.emplace_back(pose::toPoint3d(point_2d));
            }
            return points_3d;
        }

        void publishMarker(visualization_msgs::Marker &marker_msg, ros::Publisher &marker_pub) {
            if (marker_msg.header.frame_id.empty()) {
                marker_msg.header.frame_id = kVizFrame;
            }
            marker_msg.header.stamp = ros::Time();
            marker_msg.ns = "momo_demo";
            marker_msg.action = visualization_msgs::Marker::ADD;
//            LOG(INFO) << "Publishing vis msg";
            marker_pub.publish(marker_msg);
        }

        void removeMarker(const int32_t id_to_remove, ros::Publisher &marker_pub) {
            visualization_msgs::Marker marker_msg;

            marker_msg.id = id_to_remove;
            marker_msg.header.frame_id = kVizFrame;
            marker_msg.header.stamp = ros::Time();
            marker_msg.ns = "momo_demo";
            marker_msg.action = visualization_msgs::Marker::DELETE;
//            LOG(INFO) << "Publishing vis msg";
            marker_pub.publish(marker_msg);
        }

        void publishTrajectory(ros::Publisher &marker_pub, const std_msgs::ColorRGBA &color,
                               const std::vector<pose::Pose3d> &trajectory_poses,
                               const int32_t &trajectory_id, const int32_t &min_pose_id, const int32_t &max_pose_id) {
            visualization_msgs::Marker marker_msg;
            marker_msg.id = trajectory_id;
            marker_msg.color = color;

            marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
            marker_msg.scale.x = kTrajectoryScaleX;
            for (const pose::Pose3d &traj_pose : trajectory_poses) {
                geometry_msgs::Point point;
                point.x = traj_pose.first.x();
                point.y = traj_pose.first.y();
                point.z = traj_pose.first.z();
                marker_msg.points.emplace_back(point);
            }
//            LOG(INFO) << "Trajectory len " << marker_msg.points.size();

            marker_msg.pose.orientation.w = 1.0;

            publishMarker(marker_msg, marker_pub);

            // Also publish box for each pose in the trajectory
            publishRobotPoses(marker_pub, trajectory_poses, color, min_pose_id, max_pose_id);
        }

        void publishTrajectories(ros::Publisher &marker_pub, const std::vector<std_msgs::ColorRGBA> &colors,
                                 const std::vector<std::vector<pose::Pose3d>> &trajectory_poses,
                                 const int32_t &first_trajectory_id, const int32_t &min_pose_id,
                                 const int32_t &max_pose_id) {

            int32_t next_robot_pose_id = min_pose_id;
            for (size_t i = 0; i < trajectory_poses.size(); i++) {

                visualization_msgs::Marker marker_msg;
                marker_msg.id = first_trajectory_id + i;
                marker_msg.color = colors[i];

                marker_msg.type = visualization_msgs::Marker::LINE_STRIP;
                marker_msg.scale.x = kTrajectoryScaleX;
                for (const pose::Pose3d &traj_pose : trajectory_poses[i]) {
                    geometry_msgs::Point point;
                    point.x = traj_pose.first.x();
                    point.y = traj_pose.first.y();
                    point.z = traj_pose.first.z();
                    marker_msg.points.emplace_back(point);
                }
//            LOG(INFO) << "Trajecto
//            LOG(INFO) << "Trajectory len " << marker_msg.points.size();

                marker_msg.pose.orientation.w = 1.0;

                publishMarker(marker_msg, marker_pub);

                // Also publish box for each pose in the trajectory
                publishRobotPoses(marker_pub, trajectory_poses[i], colors[i], next_robot_pose_id, max_pose_id);
                next_robot_pose_id += trajectory_poses[i].size();
            }
        }

        void publishRobotPoses(ros::Publisher &marker_pub, const std::vector<pose::Pose3d> &robot_poses,
                               const std_msgs::ColorRGBA &color, const int32_t min_id, const int32_t max_id) {

            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];
                publishRobotPose(marker_pub, robot_pose, color, min_id + i);
            }


//            for (int32_t i = robot_poses.size() + min_id; i <= max_id; i++) {
//                removeMarker(i, marker_pub);
//            }
        }

        void publishRobotPose(ros::Publisher &marker_pub, pose::Pose3d &robot_pose, const std_msgs::ColorRGBA &color,
                              const int32_t id, bool bigger = false) {

            visualization_msgs::Marker marker_msg;

            marker_msg.scale.x = 0.6;
            marker_msg.scale.y = 0.4;
            marker_msg.scale.z = 0.15;

            if (bigger) {
                marker_msg.scale.x = 0.8;
                marker_msg.scale.y = 0.6;
                marker_msg.scale.z = 0.3;
            }

            marker_msg.pose.position.x = robot_pose.first.x();
            marker_msg.pose.position.y = robot_pose.first.y();
            marker_msg.pose.position.z = robot_pose.first.z();

            marker_msg.pose.orientation.w = robot_pose.second.w();
            marker_msg.pose.orientation.x = robot_pose.second.x();
            marker_msg.pose.orientation.y = robot_pose.second.y();
            marker_msg.pose.orientation.z = robot_pose.second.z();

            marker_msg.type = visualization_msgs::Marker::CUBE;

            marker_msg.color = color;
            marker_msg.id = id;

            publishMarker(marker_msg, marker_pub);
        }

        void publishCarPoses(ros::Publisher &marker_pub, pose::Pose3d &car_pose, const std_msgs::ColorRGBA &color,
                             const int32_t id) {

            visualization_msgs::Marker marker_msg;

            marker_msg.scale.x = 0.5;
            marker_msg.scale.y = 0.3;
            marker_msg.scale.z = 0.15;

            marker_msg.pose.position.x = car_pose.first.x();
            marker_msg.pose.position.y = car_pose.first.y();
            marker_msg.pose.position.z = car_pose.first.z();

            marker_msg.pose.orientation.w = car_pose.second.w();
            marker_msg.pose.orientation.x = car_pose.second.x();
            marker_msg.pose.orientation.y = car_pose.second.y();
            marker_msg.pose.orientation.z = car_pose.second.z();

            marker_msg.type = visualization_msgs::Marker::CUBE;

            marker_msg.color = color;
            marker_msg.id = id;

            publishMarker(marker_msg, marker_pub);
        }

        void publishObjectSample(ros::Publisher &marker_pub, pose::Pose3d &sample_pose, const std_msgs::ColorRGBA &color,
                             const int32_t id,
                             const Eigen::Vector2d &shape_dim) {

            visualization_msgs::Marker marker_msg;

            marker_msg.scale.x = shape_dim.x();
            marker_msg.scale.y = shape_dim.y();
            marker_msg.scale.z = 0.15;

            marker_msg.pose.position.x = sample_pose.first.x();
            marker_msg.pose.position.y = sample_pose.first.y();
            marker_msg.pose.position.z = sample_pose.first.z();

            marker_msg.pose.orientation.w = sample_pose.second.w();
            marker_msg.pose.orientation.x = sample_pose.second.x();
            marker_msg.pose.orientation.y = sample_pose.second.y();
            marker_msg.pose.orientation.z = sample_pose.second.z();

            marker_msg.type = visualization_msgs::Marker::CUBE;

            marker_msg.color = color;
            marker_msg.id = id;

            publishMarker(marker_msg, marker_pub);
        }

        void publishParkingSpot3d(ros::Publisher &marker_pub, pose::Pose3d &car_pose, const std_msgs::ColorRGBA &color,
                                  const int32_t id, const double &spot_x_dim, const double &spot_y_dim) {

            visualization_msgs::Marker marker_msg_1;
            marker_msg_1.header.frame_id = "parking_lot";

            marker_msg_1.scale.x = spot_x_dim;
            marker_msg_1.scale.y = spot_y_dim;
            marker_msg_1.scale.z = 0.1;

            marker_msg_1.pose.position.x = car_pose.first.x();
            marker_msg_1.pose.position.y = car_pose.first.y();
            marker_msg_1.pose.position.z = car_pose.first.z();

            marker_msg_1.pose.orientation.w = car_pose.second.w();
            marker_msg_1.pose.orientation.x = car_pose.second.x();
            marker_msg_1.pose.orientation.y = car_pose.second.y();
            marker_msg_1.pose.orientation.z = car_pose.second.z();

            marker_msg_1.type = visualization_msgs::Marker::CUBE;

            marker_msg_1.color = color;
            marker_msg_1.color.a = 0.8;
            marker_msg_1.id = id;

            publishMarker(marker_msg_1, marker_pub);


            visualization_msgs::Marker marker_msg_2;

            marker_msg_2.header.frame_id = "parking_lot";
            marker_msg_2.scale.x = 0.5;
            marker_msg_2.scale.y = 0.1;
            marker_msg_2.scale.z = 0.1;

            marker_msg_2.pose.position.x = car_pose.first.x();
            marker_msg_2.pose.position.y = car_pose.first.y();
            marker_msg_2.pose.position.z = car_pose.first.z();

            marker_msg_2.pose.orientation.w = car_pose.second.w();
            marker_msg_2.pose.orientation.x = car_pose.second.x();
            marker_msg_2.pose.orientation.y = car_pose.second.y();
            marker_msg_2.pose.orientation.z = car_pose.second.z();

            marker_msg_2.type = visualization_msgs::Marker::ARROW;

            marker_msg_2.color = color;
            marker_msg_2.id = id + 5000;

            publishMarker(marker_msg_2, marker_pub);
        }

        void
        publishObjDetectionsRelToRobotPoses(ros::Publisher &marker_pub, const std::vector<pose::Pose3d> &robot_poses,
                                            const std::vector<std::vector<pose::Pose3d>> &car_detections,
                                            const std_msgs::ColorRGBA &color, const int32_t min_id,
                                            const int32_t max_id) {

            int marker_num = min_id;
            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];
                for (const pose::Pose3d &car_detection : car_detections[i]) {
                    if (marker_num > max_id) {
                        break;
                    }
                    pose::Pose3d car_pose = pose::combinePoses(robot_pose, car_detection);
                    publishCarPoses(marker_pub, car_pose, color, marker_num++);
                }
            }


//            for (int32_t i = robot_poses.size() + min_id; i <= max_id; i++) {
//                removeMarker(i, marker_pub);
//            }
        }

        void publishSemanticPointDetectionsRelToRobotPoses(ros::Publisher &marker_pub,
                                                           const std::vector<pose::Pose3d> &robot_poses,
                                                           const std::vector<std::vector<Eigen::Vector3d>> &point_detections,
                                                           const std_msgs::ColorRGBA &color, const int32_t marker_id) {
            visualization_msgs::Marker marker_msg;

            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];

                for (const Eigen::Vector3d &semantic_point : point_detections[i]) {
                    Eigen::Vector3d global_point_3d = pose::transformPoint(robot_pose, semantic_point);

                    geometry_msgs::Point point_msg;
                    point_msg.x = global_point_3d.x();
                    point_msg.y = global_point_3d.y();
                    point_msg.z = global_point_3d.z();

                    marker_msg.points.emplace_back(point_msg);
                }
            }

            marker_msg.pose.orientation.w = 1.0;
            marker_msg.scale.x = 0.175;
            marker_msg.type = visualization_msgs::Marker::SPHERE_LIST;

            marker_msg.id = marker_id;
            marker_msg.color = color;

            publishMarker(marker_msg, marker_pub);
        }

        void publishLinesToCarDetections(ros::Publisher &marker_pub, const std::vector<pose::Pose3d> &robot_poses,
                                         const std::vector<std::vector<pose::Pose3d>> &car_detections,
                                         const std_msgs::ColorRGBA &color, const int32_t id) {
            visualization_msgs::Marker marker_msg;

            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];

                geometry_msgs::Point point_1;
                point_1.x = robot_pose.first.x();
                point_1.y = robot_pose.first.y();
                point_1.z = robot_pose.first.z();

                for (const pose::Pose3d &car_detection : car_detections[i]) {
                    pose::Pose3d pose_3d = pose::combinePoses(robot_pose, car_detection);

                    geometry_msgs::Point point_2;
                    point_2.x = pose_3d.first.x();
                    point_2.y = pose_3d.first.y();
                    point_2.z = pose_3d.first.z();

                    marker_msg.points.emplace_back(point_1);
                    marker_msg.points.emplace_back(point_2);
                }
            }

            marker_msg.pose.orientation.w = 1.0;
            marker_msg.scale.x = 0.05;
            marker_msg.type = visualization_msgs::Marker::LINE_LIST;

            marker_msg.id = id;
            marker_msg.color = color;

            publishMarker(marker_msg, marker_pub);
        }

        void publishLinesToSemanticPointDetections(ros::Publisher &marker_pub,
                                                   const std::vector<pose::Pose3d> &robot_poses,
                                                   const std::vector<std::vector<Eigen::Vector3d>> &semantic_points,
                                                   const std_msgs::ColorRGBA &color, const int32_t id) {
            visualization_msgs::Marker marker_msg;

            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];

                geometry_msgs::Point point_1;
                point_1.x = robot_pose.first.x();
                point_1.y = robot_pose.first.y();
                point_1.z = robot_pose.first.z();

                for (const Eigen::Vector3d &semantic_point_for_pose : semantic_points[i]) {
                    Eigen::Vector3d point_global = pose::transformPoint(robot_pose, semantic_point_for_pose);

                    geometry_msgs::Point point_2;
                    point_2.x = point_global.x();
                    point_2.y = point_global.y();
                    point_2.z = point_global.z();

                    marker_msg.points.emplace_back(point_1);
                    marker_msg.points.emplace_back(point_2);
                }
            }

            marker_msg.pose.orientation.w = 1.0;
            marker_msg.scale.x = 0.005;
            marker_msg.type = visualization_msgs::Marker::LINE_LIST;

            marker_msg.id = id;
            marker_msg.color = color;
            publishMarker(marker_msg, marker_pub);
        }

        void publishSampledObjPoseRelToRobotPoses(ros::Publisher &marker_pub, const std::vector<pose::Pose3d> &robot_poses,
                                                  const std::unordered_map<size_t, std::unordered_map<size_t, std::vector<pose::Pose3d>>> &samples,
                                                  const std_msgs::ColorRGBA &color,
                                            const Eigen::Vector2d &shape_dim,
                                            const int32_t min_id,
                                            const int32_t max_id) {

            int marker_num = min_id;
            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];
                if (samples.find(i) != samples.end()) {
                    for (const auto &cluster_info : samples.at(i)) {
                        for (const pose::Pose3d &sample_pose_rel : cluster_info.second) {
                            if (marker_num > max_id) {
                                break;
                            }
                            pose::Pose3d sample_pose = pose::combinePoses(robot_pose, sample_pose_rel);
                            publishObjectSample(marker_pub, sample_pose, color, marker_num++, shape_dim);
                        }
                    }
                }
            }
        }

        void displaySemanticPointObsFromTrajectory(const std::vector<pose::Pose2d> &trajectory,
                                                 const std::vector<std::vector<std::vector<Eigen::Vector2d>>> &relative_semantic_points,
                                                 const double &samples_color_a,
                                                 const int32_t &lines_to_obs_id,
                                                 std_msgs::ColorRGBA color,
                                                 ros::Publisher &pub,
                                                 const std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> &relative_object_samples_for_cluster = {},
                                                 const Eigen::Vector2d &dimensions_for_samples = Eigen::Vector2d()) {


            std::vector<std::vector<Eigen::Vector3d>> relative_points_3d;
            for (const std::vector<std::vector<Eigen::Vector2d>> &relative_semantic_points_for_pose : relative_semantic_points) {
                std::vector<Eigen::Vector3d> points_for_pose;
                for (const std::vector<Eigen::Vector2d> &points_for_object : relative_semantic_points_for_pose) {
                    std::vector<Eigen::Vector3d> points_for_obj_3d = convert2DPointsTo3D(points_for_object);
                    points_for_pose.insert(points_for_pose.end(), points_for_obj_3d.begin(), points_for_obj_3d.end());
                }
                relative_points_3d.emplace_back(points_for_pose);
            }

            std::vector<pose::Pose3d> trajectory_3d = convert2DPosesTo3D(trajectory);

            publishLinesToSemanticPointDetections(pub, trajectory_3d, relative_points_3d, color, lines_to_obs_id);
            publishSemanticPointDetectionsRelToRobotPoses(pub, trajectory_3d, relative_points_3d, color,
                                                          kSemanticPointsMin);

            if (!relative_object_samples_for_cluster.empty()) {

                color.a = samples_color_a;

                std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose3d>>> relative_object_samples_for_cluster_3d;
                for (const auto &samples_for_node : relative_object_samples_for_cluster) {
                    for (const auto &samples_for_cluster : samples_for_node.second) {
                        relative_object_samples_for_cluster_3d[samples_for_node.first][samples_for_cluster.first] = convert2DPosesTo3D(
                                samples_for_cluster.second);
                    }
                }

                publishSampledObjPoseRelToRobotPoses(pub, trajectory_3d,
                                                     relative_object_samples_for_cluster_3d,
                                                     color,
                                                     dimensions_for_samples,
                                                     kSemanticPointsMin + 1,
                                                     kSemanticPointsMax);
            }
        }
    };
}

#endif //AUTODIFF_GP_ROS_VISUALIZATION_H
