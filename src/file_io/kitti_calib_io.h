//
// Created by amanda on 6/10/21.
//

#ifndef AUTODIFF_GP_KITTI_CALIB_IO_H
#define AUTODIFF_GP_KITTI_CALIB_IO_H

#include <glog/logging.h>

#include <string>
#include <eigen3/Eigen/Dense>
#include <fstream>

namespace file_io {
//    calib.txt: Calibration data for the cameras: P0/P1 are the 3x4 projection
//            matrices after rectification. Here P0 denotes the left and P1 denotes the
//    right camera. Tr transforms a point from velodyne coordinates into the
//            left rectified camera coordinate system. In order to map a point X from the
//            velodyne scanner to a point x in the i'th image plane, you thus have to
//    transform it like:
//
//    x = Pi * Tr * X

    /**
     * Get the transform that gives the pose of the velodyne in the camera frame.
     *
     * @param kitti_calib_file file containing kitti calibration.
     *
     * @return Transform that gives the pose of the velodyne in the camera frame. In other words, transforms points in
     * velodyne frame to camera frame.
     */
    Eigen::Affine3d getVelodyneToCameraTf(const std::string &kitti_calib_file) {
        std::ifstream file_obj(kitti_calib_file);
        std::string last_line;
        std::string second_to_last_line;
        while (std::getline(file_obj, last_line)) {
            second_to_last_line = last_line;
        }
        std::string trimmed_line = second_to_last_line.substr(4, second_to_last_line.length() - 4);
        std::stringstream ss(trimmed_line);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ' ');
            data.push_back(std::stod(substr));
        }
        Eigen::Vector3d transl(data[3], data[7], data[11]);
        Eigen::Matrix3d rot;
        rot << data[0], data[1], data[2], data[4], data[5], data[6], data[8], data[9], data[10];
        Eigen::Affine3d velodyne_to_camera;
        velodyne_to_camera.translation() = transl;
        velodyne_to_camera.linear() = rot;

        return velodyne_to_camera;
    }
}

#endif //AUTODIFF_GP_KITTI_CALIB_IO_H
