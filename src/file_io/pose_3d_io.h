//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_POSE_3D_IO_H
#define AUTODIFF_GP_POSE_3D_IO_H

#include <fstream>
#include <base_lib/pose_reps.h>

namespace file_io {
    void writePose3dHeaderToFile(std::ofstream &file_stream) {
        file_stream << "transl_x" << ", " <<
                    "transl_y" << ", " <<
                    "transl_z" << ", " <<
                    "quat_x" << ", " <<
                    "quat_y" << ", " <<
                    "quat_z" << ", " <<
                    "quat_w" << "\n";
    }


    void writePose3dLineToFile(const pose::Pose3d &pose_3d, std::ofstream &file_stream) {
        file_stream << pose_3d.first.x() << ", " <<
                    pose_3d.first.y() << ", " <<
                    pose_3d.first.z() << ", " <<
                    pose_3d.second.x() << ", " <<
                    pose_3d.second.y() << ", " <<
                    pose_3d.second.z() << ", " <<
                    pose_3d.second.w() << "\n";
    }

    void writePose3dsToFile(const std::string &file_name, const std::vector<pose::Pose3d> &pose_3ds) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writePose3dHeaderToFile(csv_file);
        for (const pose::Pose3d &pose_3d : pose_3ds) {
            writePose3dLineToFile(pose_3d, csv_file);
        }

        csv_file.close();
    }

    void readPose3dLine(const std::string &line_in_file, pose::Pose3d &pose_3d) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        bool first_entry = true;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        pose_3d = std::make_pair(Eigen::Vector3d(data[0], data[1], data[2]),
                                 Eigen::Quaterniond(data[6], data[3], data[4], data[5]));
    }

    void readPose3dsFromFile(const std::string &file_name, std::vector<pose::Pose3d> &pose_3ds) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            pose::Pose3d pose_3d;
            readPose3dLine(line, pose_3d);
            pose_3ds.emplace_back(pose_3d);
        }
    }
}

#endif //AUTODIFF_GP_POSE_3D_IO_H
