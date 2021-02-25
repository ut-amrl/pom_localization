//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_LIDAR_ODOM_H
#define AUTODIFF_GP_LIDAR_ODOM_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <pose_optimization/pose_graph_generic.h>
#include <base_lib/pose_reps.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace h3d {

    typedef pose_graph::GaussianBinaryFactor<2, double, 3> GaussianBinaryFactor2d;

    struct RawAbsoluteLidarOdomNode3D {
        double timestamp_;
        double transl_x_;
        double transl_y_;
        double transl_z_;
        double quat_w_;
        double quat_x_;
        double quat_y_;
        double quat_z_;
        double noise_variance_1_;
        double noise_variance_2_;
        double noise_variance_3_;
        double noise_variance_4_;
        double noise_variance_5_;
        double noise_variance_6_;
    };

    void readRawLidarOdomLine(const std::string &line_in_file, RawAbsoluteLidarOdomNode3D &lidar_odom_est_data) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        lidar_odom_est_data.timestamp_ = data[0];
        lidar_odom_est_data.transl_x_ = data[1];
        lidar_odom_est_data.transl_y_ = data[2];
        lidar_odom_est_data.transl_z_ = data[3];
        lidar_odom_est_data.quat_w_ = data[4];
        lidar_odom_est_data.quat_x_ = data[5];
        lidar_odom_est_data.quat_y_ = data[6];
        lidar_odom_est_data.quat_z_ = data[7];
        lidar_odom_est_data.noise_variance_1_ = 5e-1;
        lidar_odom_est_data.noise_variance_2_ = 5e-1;
        lidar_odom_est_data.noise_variance_3_ = 5e-1;
        lidar_odom_est_data.noise_variance_4_ = 5e-1;
        lidar_odom_est_data.noise_variance_5_ = 5e-1;
        lidar_odom_est_data.noise_variance_6_ = 5e-1;
//        lidar_odom_est_data.noise_variance_1_ = data[8];
//        lidar_odom_est_data.noise_variance_2_ = data[9];
//        lidar_odom_est_data.noise_variance_3_ = data[10];
//        lidar_odom_est_data.noise_variance_4_ = data[11];
//        lidar_odom_est_data.noise_variance_5_ = data[12];
//        lidar_odom_est_data.noise_variance_6_ = data[13];
        LOG(INFO) << "Lidar odom quat " << lidar_odom_est_data.quat_w_ << ", " << lidar_odom_est_data.quat_x_ << ", "
        << lidar_odom_est_data.quat_y_ << ", " << lidar_odom_est_data.quat_z_;
        LOG(INFO) << "Lidar odom transl " << lidar_odom_est_data.transl_x_ << ", " << lidar_odom_est_data.transl_y_ << ", " << lidar_odom_est_data.transl_z_;
    }

    void readRawLidarOdomFromFile(const std::string &file_name, std::vector<RawAbsoluteLidarOdomNode3D> &odom_ests_in_abs_frame) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            RawAbsoluteLidarOdomNode3D lidar_odom_est_data;
            readRawLidarOdomLine(line, lidar_odom_est_data);
            odom_ests_in_abs_frame.emplace_back(lidar_odom_est_data);
        }
    }

    void getLidar2dConstraintsFromLidarFile(const std::string &file_name,
                                            std::unordered_map<pose_graph::NodeId, double> &timestamps_by_node_id,
                                            std::vector<GaussianBinaryFactor2d> &lidar_odom) {
        std::vector<RawAbsoluteLidarOdomNode3D> odom_ests_map_frame_3d;
        readRawLidarOdomFromFile(file_name, odom_ests_map_frame_3d);

//        for (size_t i = 0; i < odom_ests_map_frame_3d.size(); i++) {
        for (size_t i = 0; i < std::min((size_t) 25, odom_ests_map_frame_3d.size()); i++) {
            RawAbsoluteLidarOdomNode3D odom_est = odom_ests_map_frame_3d[i];
            timestamps_by_node_id[i] = odom_est.timestamp_;
        }

        RawAbsoluteLidarOdomNode3D prev_est = odom_ests_map_frame_3d[0];
        pose::Pose3d prev_pose_3d = std::make_pair(
                Eigen::Vector3d(prev_est.transl_x_, prev_est.transl_y_, prev_est.transl_z_),
                Eigen::Quaterniond(prev_est.quat_w_, prev_est.quat_x_, prev_est.quat_y_, prev_est.quat_z_));

//        for (size_t i = 1; i < odom_ests_map_frame_3d.size(); i++) {
        for (size_t i = 1; i < std::min((size_t) 25, odom_ests_map_frame_3d.size()); i++) {
            LOG(INFO) << prev_pose_3d.second.w() << ", " << prev_pose_3d.second.x() << ", " << prev_pose_3d.second.y() << ", " << prev_pose_3d.second.z();
            RawAbsoluteLidarOdomNode3D curr_est = odom_ests_map_frame_3d[i];

            pose::Pose3d curr_pose_3d = std::make_pair(
                    Eigen::Vector3d(curr_est.transl_x_, curr_est.transl_y_, curr_est.transl_z_),
                    Eigen::Quaterniond(curr_est.quat_w_, curr_est.quat_x_, curr_est.quat_y_, curr_est.quat_z_));

            pose::Pose3d pose_difference_3d = pose::getPoseOfObj1RelToObj2(curr_pose_3d, prev_pose_3d);
            LOG(INFO) << "Node " << i-1 << "to " << i << "pose diff: ("
            << pose_difference_3d.first.x() << ", " << pose_difference_3d.first.y() << ", "<< pose_difference_3d.first.z() << "), ("
            << pose_difference_3d.second.w() << ", "<< pose_difference_3d.second.x() << ", " << pose_difference_3d.second.y() << ", "<< pose_difference_3d.second.z() << ")";

            // Get the 2D components

            Eigen::Quaterniond quat = pose_difference_3d.second;
            // Using this version because the eigen rotation matrix -> euler conversion seemed to frequently return the
            // set of equivalent euler angles close to 180, 180, 180 instead of 0, 0, 0
            // This way so far has avoided that problem
            double yaw = atan2((2 * ((quat.w() * quat.z()) + (quat.y() * quat.x()))), (1 - 2 * (pow(quat.z(), 2) + pow(quat.y(), 2))));
            pose::Pose2d pose_difference_2d = pose::createPose2d(
                    pose_difference_3d.first.x(), pose_difference_3d.first.y(), yaw);


            LOG(INFO) << "Node " << i-1 << "to " << i << "pose diff 2d: "
                      << pose_difference_2d.first.x() << ", " << pose_difference_2d.first.y() << ", "<< pose_difference_2d.second;

            // GTSAM noise order appears to be roll, pitch, yaw, x, y, z
            double variance_x = prev_est.noise_variance_4_;
            double variance_y = prev_est.noise_variance_5_;
            double variance_yaw = prev_est.noise_variance_3_;

            GaussianBinaryFactor2d factor_2d;
            factor_2d.from_node_ = i - 1;
            factor_2d.to_node_ = i;
            factor_2d.translation_change_ = pose_difference_2d.first;
            factor_2d.orientation_change_ = pose_difference_2d.second;
            Eigen::Matrix<double, 3, 3> odom_cov_mat = Eigen::Matrix<double, 3, 3>::Zero();
            odom_cov_mat(0, 0) = variance_x;
            odom_cov_mat(1, 1) = variance_y;
            odom_cov_mat(2, 2) = variance_yaw;

            Eigen::Matrix<double, 3, 3> odom_sqrt_information_mat = odom_cov_mat.inverse().sqrt();
            factor_2d.sqrt_information_ = odom_sqrt_information_mat;

            lidar_odom.emplace_back(factor_2d);

            prev_est = curr_est;
            prev_pose_3d = curr_pose_3d;
        }
    }
}

#endif //AUTODIFF_GP_LIDAR_ODOM_H
