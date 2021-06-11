//
// Created by amanda on 6/8/21.
//

#ifndef AUTODIFF_GP_KITTI_POSES_H
#define AUTODIFF_GP_KITTI_POSES_H

#include <string>
#include <eigen3/Eigen/Dense>
#include <fstream>

namespace file_io {

    struct KittiFormattedPose {
        Eigen::Affine3d relative_pose;
    };

    void readRawKittiPoseLine(const std::string &line_in_file, KittiFormattedPose &kitti_pose) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ' ');
            data.push_back(std::stod(substr));
        }
        Eigen::Vector3d transl(data[3], data[7], data[11]);
        Eigen::Matrix3d rot;
        rot << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];

        kitti_pose.relative_pose.translation() = transl;
        kitti_pose.relative_pose.linear() = rot;
    }

    void readRawKittiPosesFromFile(const std::string &file_name, std::vector<KittiFormattedPose> &odom_ests_in_abs_frame) {
        std::ifstream file_obj(file_name);
        std::string line;
        while (std::getline(file_obj, line)) {

            KittiFormattedPose kitti_pose;
            readRawKittiPoseLine(line, kitti_pose);
            odom_ests_in_abs_frame.emplace_back(kitti_pose);
        }
    }
}

#endif //AUTODIFF_GP_KITTI_POSES_H
