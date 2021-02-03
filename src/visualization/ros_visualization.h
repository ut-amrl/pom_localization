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

namespace visualization {

    class VisualizationManager {
    public:

        VisualizationManager(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
            gt_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("gt_visualization_marker", 250);
            est_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("est_pos_marker", 250);
            odom_marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("odom_pos_marker", 250);
            regressor_max_val_for_pos_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("regressor_max_val_for_pos", 2);
            robot_pose_max_val_pub_  = node_handle_.advertise<nav_msgs::OccupancyGrid>("robot_pose_max_val", 2);
            for (int i = -5; i <= 6; i++) {
                std::string angle_name;
                if (i < 0) {
                    angle_name = "neg_";
                }
                angle_name += std::to_string(abs(30 * i));

                std::string obj_heat_topic_name = "regressor_" + angle_name + "_val";
                std::string robot_pose_topic_name = "robot_pose_" + angle_name + "_val";
                pub_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(obj_heat_topic_name, 2);
                robot_pose_pub_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(robot_pose_topic_name, 2);
            }
            ros::Duration(5).sleep();
        }

        void displayTrueTrajectory(const std::vector<pose::Pose3d> &true_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            publishTrajectory(gt_marker_pub_, color, true_trajectory, kGtTrajectoryId);
        }

        void displayTrueCarPoses(const std::vector<pose::Pose3d> &true_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            int32_t car_poses_count = true_car_poses.size();
            for (int32_t i = 0; i < car_poses_count; i++) {
                pose::Pose3d pose = true_car_poses[i];
                publishCarPoses(gt_marker_pub_, pose, color, kCarGtPosesMin + i);
            }

//            for (int32_t i = kCarGtPosesMin + car_poses_count; i <= kCarGtPosesMax; i++) {
//                removeMarker(i, gt_marker_pub_);
//            }
        }

        void displayNoisyCarPosesFromGt(const std::vector<pose::Pose3d> &gt_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;
            color.b = 1.0;

            publishLinesToCarDetections(gt_marker_pub_, gt_trajectory, relative_car_poses, color, kObservedFromGtCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(gt_marker_pub_, gt_trajectory, relative_car_poses, color,
                                                kObservedFromGtCarDetectionsMin, kObservedFromGtCarDetectionsMax);
        }

        void displayNoisyCarPosesFromEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;
            color.b = 0.7;

            publishLinesToCarDetections(est_marker_pub_, est_trajectory, relative_car_poses, color, kObservedFromEstCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(est_marker_pub_, est_trajectory, relative_car_poses, color,
                                                kObservedFromEstCarDetectionsMin, kObservedFromEstCarDetectionsMax);
        }

        void displayNoisyCarPosesFromOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;
            color.g = 0.7;

            publishLinesToCarDetections(odom_marker_pub_, odom_trajectory, relative_car_poses, color, kObservedFromOdomCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(odom_marker_pub_, odom_trajectory, relative_car_poses, color,
                                                kObservedFromOdomCarDetectionsMin, kObservedFromOdomCarDetectionsMax);
        }

        void displayOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;

            publishTrajectory(odom_marker_pub_, color, odom_trajectory, kOdomTrajectoryId);
        }

        void displayEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;

            publishTrajectory(est_marker_pub_, color, est_trajectory, kEstTrajectoryId);
        }
        //        void displayMaxGpRegressorOutput(
//                std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> regressor,
//                const double &resolution, const double &x_min, const double &x_max, const double &y_min,
//                const double &y_max) {
        void displayMaxGpRegressorOutput(
                std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> kde,
                const double &resolution, const double &x_min, const double &x_max, const double &y_min,
                const double &y_max) {
            int64_t x_min_unscaled = floor(x_min / resolution);
            int64_t x_max_unscaled = ceil(x_max / resolution);
            int64_t y_min_unscaled = floor(y_min / resolution);
            int64_t y_max_unscaled = ceil(y_max / resolution);

            LOG(INFO) << "Creating best occ grid";

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
            LOG(INFO) << "Creating occ grids";

            for (int i = -5; i <= 6; i++) {
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
            }

            LOG(INFO) << "Looping through vals";

            int size = best_occ_grid.info.width * best_occ_grid.info.height;

//            std::vector<double, grid

            std::unordered_map<int, Eigen::Matrix<double, 3, Eigen::Dynamic>> mats_by_angle;
            for (int i = -5; i <= 6; i++) {
                mats_by_angle[i] = Eigen::Matrix<double, 3, Eigen::Dynamic>(3, size);
            }



            LOG(INFO) << "Creating input matrices for different angles";
            for (int y_val = y_min_unscaled; y_val <= y_max_unscaled; y_val++) {
                for (int x_val = x_min_unscaled; x_val <= x_max_unscaled; x_val++) {

                    long data_index = (best_occ_grid.info.width * (y_val - y_min_unscaled)) + x_val - x_min_unscaled; // Should I switch x and y?

//                    long data_index = (best_occ_grid.info.height * (x_val - x_min_unscaled)) + y_val - y_min_unscaled; // Should I switch x and y?

                    for (int i = -5; i <= 6; i++) {
                        Eigen::Matrix<double, 3, Eigen::Dynamic> mat_for_angle = mats_by_angle[i];
                        double yaw = i * M_PI / 6;

                        Eigen::Matrix<double, 3, 1> object_pose_2d;
                        object_pose_2d << (x_val * resolution), (y_val * resolution), yaw;
                        mat_for_angle.col(data_index) = object_pose_2d;
                        mats_by_angle[i] = mat_for_angle;
                    }
                }
            }

            std::unordered_map<int, Eigen::Matrix<double, 1, Eigen::Dynamic>> output_mats;
            for (int i = -5; i <= 6; i++) {
                LOG(INFO) << "Getting regression value for angle index " << i;

                LOG(INFO) << "Input size " << mats_by_angle[i].rows() << ", " << mats_by_angle[i].cols();
//                output_mats[i] = regressor->Inference(mats_by_angle[i]);
                output_mats[i] = kde->Inference(mats_by_angle[i]);
                LOG(INFO) << "Output size " << output_mats[i].rows() << ", " << output_mats[i].cols();
                LOG(INFO) << "Output " << output_mats[i];
            }

            LOG(INFO) << "Setting occupancy grid data";

            for (int i = 0; i < size; i++) {
                double best_val = 0.0;
                for (int j = -5; j <= 6; j++) {
                    ros::Publisher pub_for_angle = pub_for_angle_mult_[i];
                    double inf_val = output_mats[j](0, i);
                    if (inf_val > best_val) {
                        best_val = inf_val;
                    }

                    if (inf_val > 1) {
                        LOG(INFO) << "Inf val " << inf_val;
                    }

                    nav_msgs::OccupancyGrid occ_grid_for_angle = occ_grids_by_angle[j];
//                    LOG(INFO) << "Double val " << inf_val;
//                    LOG(INFO) << std::to_string(((int8_t) (100 * inf_val)));
//                    occ_grid_for_angle.data[i] = (int8_t) (100 * inf_val);
                    occ_grid_for_angle.data[i] = (int8_t) (1000 * inf_val);
//                    LOG(INFO) << std::to_string(occ_grid_for_angle.data[i]);
                    occ_grids_by_angle[j] = occ_grid_for_angle;
                }
                best_occ_grid.data[i] = (int8_t) (100 * best_val);
            }

            LOG(INFO) << "Publishing occ grid data";
            std::string occ_data;
            for (size_t i = 0; i < best_occ_grid.data.size(); i++) {
                occ_data += std::to_string(best_occ_grid.data[i]);
                occ_data += ", ";
            }
            LOG(INFO) << "Occ " << occ_data;
            regressor_max_val_for_pos_pub_.publish(best_occ_grid);
            for (int i = -5; i <= 6; i++) {
                ros::Publisher publisher = pub_for_angle_mult_[i];

                publisher.publish(occ_grids_by_angle[i]);
            }
        }

//        void displayPoseResiduals(pose_optimization::MovableObservationCostFunctor cost_functor,
        template <typename FactorType>
        void displayPoseResiduals(FactorType cost_functor,
                                  const double &resolution, const double &x_min, const double &x_max, const double &y_min,
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

            for (int i = -5; i <= 6; i++) {
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

                Eigen::Quaterniond quat(cos(yaw/2), 0, 0, sin(yaw/2));
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
                    ros::Publisher pub_for_angle = pub_for_angle_mult_[i];
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

            publishTrajectory(gt_marker_pub_, color, convert2DPosesTo3D(true_trajectory), kGtTrajectoryId);
        }

        void displayTrueCarPoses(const std::vector<pose::Pose2d> &true_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            int32_t car_poses_count = true_car_poses.size();
            for (int32_t i = 0; i < car_poses_count; i++) {
                pose::Pose3d pose = pose::toPose3d(true_car_poses[i]);
                publishCarPoses(gt_marker_pub_, pose, color, kCarGtPosesMin + i);
            }

//            for (int32_t i = kCarGtPosesMin + car_poses_count; i <= kCarGtPosesMax; i++) {
//                removeMarker(i, gt_marker_pub_);
//            }
        }

        void displayNoisyCarPosesFromGt(const std::vector<pose::Pose2d> &gt_trajectory, const std::vector<std::vector<pose::Pose2d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;
            color.b = 1.0;

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(gt_marker_pub_, convert2DPosesTo3D(gt_trajectory), relative_car_poses_3d, color, kObservedFromGtCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(gt_marker_pub_, convert2DPosesTo3D(gt_trajectory), relative_car_poses_3d, color,
                                                kObservedFromGtCarDetectionsMin, kObservedFromGtCarDetectionsMax);
        }

        void displayNoisyCarPosesFromEstTrajectory(const std::vector<pose::Pose2d> &est_trajectory, const std::vector<std::vector<pose::Pose2d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;
            color.b = 0.7;

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(est_marker_pub_, convert2DPosesTo3D(est_trajectory),
                                        relative_car_poses_3d, color, kObservedFromEstCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(est_marker_pub_, convert2DPosesTo3D(est_trajectory),
                                                relative_car_poses_3d, color,
                                                kObservedFromEstCarDetectionsMin, kObservedFromEstCarDetectionsMax);
        }

        void displayNoisyCarPosesFromOdomTrajectory(const std::vector<pose::Pose2d> &odom_trajectory, const std::vector<std::vector<pose::Pose2d>> &relative_car_poses) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;
            color.g = 0.7;

            std::vector<std::vector<pose::Pose3d>> relative_car_poses_3d;
            for (const std::vector<pose::Pose2d> &relative_car_pose_list : relative_car_poses) {
                relative_car_poses_3d.emplace_back(convert2DPosesTo3D(relative_car_pose_list));
            }

            publishLinesToCarDetections(odom_marker_pub_, convert2DPosesTo3D(odom_trajectory), relative_car_poses_3d, color, kObservedFromOdomCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(odom_marker_pub_, convert2DPosesTo3D(odom_trajectory), relative_car_poses_3d, color,
                                                kObservedFromOdomCarDetectionsMin, kObservedFromOdomCarDetectionsMax);
        }

        void displayOdomTrajectory(const std::vector<pose::Pose2d> &odom_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;

            publishTrajectory(odom_marker_pub_, color, convert2DPosesTo3D(odom_trajectory), kOdomTrajectoryId);
        }

        void displayEstTrajectory(const std::vector<pose::Pose2d> &est_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;

            publishTrajectory(est_marker_pub_, color, convert2DPosesTo3D(est_trajectory), kEstTrajectoryId);
        }

    private:

        const std::string kVizFrame = "map";

        const double kTrajectoryScaleX = 0.05;

        const int32_t kEstTrajectoryId = 1;

        const int32_t kOdomTrajectoryId = 2;

        const int32_t kGtTrajectoryId = 3;

        const int32_t kObservedFromGtCarDetectionLines = 4;
        const int32_t kObservedFromOdomCarDetectionLines = 5;
        const int32_t kObservedFromEstCarDetectionLines = 6;

        // TODO might need to increase these
        const int32_t kMaxObservationsToDisplay = 200;
        const int32_t kObservedFromGtCarDetectionsMin = 100;
        const int32_t kObservedFromOdomCarDetectionsMin = kObservedFromGtCarDetectionsMin + kMaxObservationsToDisplay;
        const int32_t kObservedFromEstCarDetectionsMin = kObservedFromOdomCarDetectionsMin + kMaxObservationsToDisplay;
        const int32_t kCarGtPosesMin = kObservedFromEstCarDetectionsMin + kMaxObservationsToDisplay;

        const int32_t kObservedFromGtCarDetectionsMax = kObservedFromOdomCarDetectionsMin - 1;
        const int32_t kObservedFromOdomCarDetectionsMax = kObservedFromEstCarDetectionsMin - 1;
        const int32_t kObservedFromEstCarDetectionsMax = kCarGtPosesMin - 1;
        const int32_t kCarGtPosesMax = kCarGtPosesMin + kMaxObservationsToDisplay - 1;

        /**
         * Node handle.
         */
        ros::NodeHandle node_handle_;

        ros::Publisher gt_marker_pub_;
        ros::Publisher est_marker_pub_;
        ros::Publisher odom_marker_pub_;


        std::unordered_map<int32_t, ros::Publisher> pub_for_angle_mult_;
        std::unordered_map<int32_t, ros::Publisher> robot_pose_pub_for_angle_mult_;

        ros::Publisher robot_pose_max_val_pub_;

        ros::Publisher regressor_max_val_for_pos_pub_;

        std::vector<pose::Pose3d> convert2DPosesTo3D(const std::vector<pose::Pose2d> &poses_2d) {
            std::vector<pose::Pose3d> poses_3d;
            for (const pose::Pose2d &pose_2d : poses_2d) {
                poses_3d.emplace_back(pose::toPose3d(pose_2d));
            }
            return poses_3d;
        }

        void publishMarker(visualization_msgs::Marker &marker_msg, ros::Publisher &marker_pub) {
            marker_msg.header.frame_id = kVizFrame;
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

        void publishTrajectory(ros::Publisher &marker_pub, const std_msgs::ColorRGBA &color, const std::vector<pose::Pose3d> &trajectory_poses,
                               const int32_t &id) {
            visualization_msgs::Marker marker_msg;
            marker_msg.id = id;
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
        }

        void publishCarPoses(ros::Publisher &marker_pub, pose::Pose3d &car_pose, const std_msgs::ColorRGBA &color, const int32_t id) {

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

        void publishCarDetectionsRelToRobotPoses(ros::Publisher &marker_pub, const std::vector<pose::Pose3d> &robot_poses,
                                                 const std::vector<std::vector<pose::Pose3d>> &car_detections,
                                                 const std_msgs::ColorRGBA &color, const int32_t min_id, const int32_t max_id) {

            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];
                for (const pose::Pose3d &car_detection : car_detections[i]) {
                    pose::Pose3d car_pose = pose::combinePoses(robot_pose, car_detection);
                    publishCarPoses(marker_pub, car_pose, color, min_id + i);
                }
            }


//            for (int32_t i = robot_poses.size() + min_id; i <= max_id; i++) {
//                removeMarker(i, marker_pub);
//            }
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
    };
}

#endif //AUTODIFF_GP_ROS_VISUALIZATION_H
