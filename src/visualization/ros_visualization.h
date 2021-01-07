//
// Created by amanda on 12/21/20.
//

#ifndef AUTODIFF_GP_ROS_VISUALIZATION_H
#define AUTODIFF_GP_ROS_VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>

#include <base_lib/pose_reps.h>

namespace visualization {

    class VisualizationManager {
    public:

        VisualizationManager(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
            marker_pub_ = node_handle_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
            regressor_max_val_for_pos_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("regressor_max_val_for_pos", 2);
            for (int i = -5; i <= 6; i++) {
                std::string angle_name;
                if (i < 0) {
                    angle_name = "neg_";
                }
                angle_name += std::to_string(abs(30 * i));

                std::string topic_name = "regressor_" + angle_name + "_val";
                pub_for_angle_mult_[i] = node_handle_.advertise<nav_msgs::OccupancyGrid>(topic_name, 2);
            }
            test_occ_pub_ = node_handle_.advertise<visualization_msgs::Marker>("test_occ_pub", 2);
            ros::Duration(5).sleep();
        }

        void displayTrueTrajectory(const std::vector<pose::Pose3d> &true_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            publishTrajectory(color, true_trajectory, kGtTrajectoryId);
        }

        void displayTrueCarPoses(const std::vector<pose::Pose3d> &true_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;

            publishCarPoses(true_car_poses, color, kCarGtPoses);
        }

        void displayNoisyCarPosesFromGt(const std::vector<pose::Pose3d> &gt_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.g = 1.0;
            color.b = 1.0;

            publishLinesToCarDetections(gt_trajectory, relative_car_poses, color, kObservedFromGtCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(gt_trajectory, relative_car_poses, color, kObservedFromGtCarDetections);
        }

        void displayNoisyCarPosesFromEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {
            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;
            color.b = 0.7;

            publishLinesToCarDetections(est_trajectory, relative_car_poses, color, kObservedFromEstCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(est_trajectory, relative_car_poses, color, kObservedFromEstCarDetections);
        }

        void displayNoisyCarPosesFromOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory, const std::vector<std::vector<pose::Pose3d>> &relative_car_poses) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;
            color.g = 0.7;

            publishLinesToCarDetections(odom_trajectory, relative_car_poses, color, kObservedFromOdomCarDetectionLines);
            publishCarDetectionsRelToRobotPoses(odom_trajectory, relative_car_poses, color, kObservedFromOdomCarDetections);
        }

        void displayOdomTrajectory(const std::vector<pose::Pose3d> &odom_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.b = 1.0;

            publishTrajectory(color, odom_trajectory, kOdomTrajectoryId);
        }

        void displayEstTrajectory(const std::vector<pose::Pose3d> &est_trajectory) {

            std_msgs::ColorRGBA color;
            color.a = 1.0;
            color.r = 1.0;

            publishTrajectory(color, est_trajectory, kEstTrajectoryId);
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
                    occ_grid_for_angle.data[i] = (int8_t) (100 * inf_val);
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

    private:

        const std::string kVizFrame = "map";

        const double kTrajectoryScaleX = 0.05;

        const int32_t kEstTrajectoryId = 1;

        const int32_t kOdomTrajectoryId = 2;

        const int32_t kGtTrajectoryId = 3;

        const int32_t kCarGtPoses = 4;

        const int32_t kObservedFromGtCarDetections = 5;

        const int32_t kObservedFromGtCarDetectionLines = 6;


        const int32_t kObservedFromOdomCarDetections = 7;

        const int32_t kObservedFromOdomCarDetectionLines = 8;

        const int32_t kObservedFromEstCarDetections = 9;

        const int32_t kObservedFromEstCarDetectionLines = 10;

        /**
         * Node handle.
         */
        ros::NodeHandle node_handle_;

        ros::Publisher marker_pub_;


        std::unordered_map<int32_t, ros::Publisher> pub_for_angle_mult_;


        ros::Publisher regressor_max_val_for_pos_pub_;
        ros::Publisher test_occ_pub_;
//
//        ros::Publisher regressor_val_for_neg_150_pub_;
//        ros::Publisher regressor_val_for_neg_120_pub_;
//        ros::Publisher regressor_val_for_neg_90_pub_;
//        ros::Publisher regressor_val_for_neg_60_pub_;
//        ros::Publisher regressor_val_for_neg_30_pub_;
//        ros::Publisher regressor_val_for_0_pub_;
//        ros::Publisher regressor_val_for_30_pub_;
//        ros::Publisher regressor_val_for_60_pub_;
//        ros::Publisher regressor_val_for_90_pub_;
//        ros::Publisher regressor_val_for_120_pub_;
//        ros::Publisher regressor_val_for_150_pub_;
//        ros::Publisher regressor_val_for_180_pub_;


        void publishMarker(visualization_msgs::Marker &marker_msg) {
            marker_msg.header.frame_id = kVizFrame;
            marker_msg.header.stamp = ros::Time();
            marker_msg.ns = "momo_demo";
            marker_msg.action = visualization_msgs::Marker::ADD;
            LOG(INFO) << "Publishing vis msg";
            marker_pub_.publish(marker_msg);
        }

        void publishTrajectory(const std_msgs::ColorRGBA &color, const std::vector<pose::Pose3d> &trajectory_poses,
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
            LOG(INFO) << "Trajectory len " << marker_msg.points.size();

            marker_msg.pose.orientation.w = 1.0;

            publishMarker(marker_msg);
        }

        void publishCarPoses(const std::vector<pose::Pose3d> &car_poses, const std_msgs::ColorRGBA &color, const int32_t id) {

            visualization_msgs::Marker marker_msg;

            marker_msg.scale.x = 0.5;
            marker_msg.scale.y = 0.3;
            marker_msg.scale.z = 0.15;

            marker_msg.type = visualization_msgs::Marker::CUBE_LIST;
            marker_msg.pose.orientation.w = 1.0;

            for (const pose::Pose3d &car_pose : car_poses) {
                geometry_msgs::Point point;
                point.x = car_pose.first.x();
                point.y = car_pose.first.y();
                point.z = car_pose.first.z();
                marker_msg.points.emplace_back(point);
            }

            marker_msg.color = color;
            marker_msg.id = id;

            publishMarker(marker_msg);
        }

        void publishCarDetectionsRelToRobotPoses(const std::vector<pose::Pose3d> &robot_poses,
                                                 const std::vector<std::vector<pose::Pose3d>> &car_detections,
                                                 const std_msgs::ColorRGBA &color, const int32_t id) {
            std::vector<pose::Pose3d> car_poses_map_frame;
            for (size_t i = 0; i < robot_poses.size(); i++) {
                pose::Pose3d robot_pose = robot_poses[i];
                for (const pose::Pose3d &car_detection : car_detections[i]) {
                    car_poses_map_frame.emplace_back(pose::combinePoses(robot_pose, car_detection));
                }
            }

            publishCarPoses(car_poses_map_frame, color, id);
        }

        void publishLinesToCarDetections(const std::vector<pose::Pose3d> &robot_poses,
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

            publishMarker(marker_msg);
        }
    };
}

#endif //AUTODIFF_GP_ROS_VISUALIZATION_H
