//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_TRAJECTORY_2D_IO_H
#define AUTODIFF_GP_TRAJECTORY_2D_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <base_lib/pose_reps.h>

namespace file_io {
    struct TrajectoryNode2d {
        uint64_t node_id_;
        double transl_x_;
        double transl_y_;
        double theta_;
    };

    void writeTrajectory2dHeaderToFile(std::ofstream &file_stream) {
        file_stream << "node_id" << ", " << "transl_x"
                 << ", " << "transl_y" << ", " << "theta" << "\n";
    }


    void writeTrajectory2dEntryLineToFile(const TrajectoryNode2d &trajectory_node, std::ofstream &file_stream) {
        file_stream << trajectory_node.node_id_ << ", " << trajectory_node.transl_x_
                    << ", " << trajectory_node.transl_y_ << ", " << trajectory_node.theta_ << "\n";
    }

    void writeTrajectory2dToFile(const std::string &file_name, const std::vector<TrajectoryNode2d> &trajectory_nodes) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeTrajectory2dHeaderToFile(csv_file);
        for (const TrajectoryNode2d &trajectory_node : trajectory_nodes) {
            writeTrajectory2dEntryLineToFile(trajectory_node, csv_file);
        }

        csv_file.close();
    }

    void readTrajectory2dLine(const std::string &line_in_file, TrajectoryNode2d &trajectory_node) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        bool first_entry = true;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (first_entry) {
                first_entry = false;
                std::istringstream stream(substr);
                stream >> trajectory_node.node_id_;
            } else {
                data.push_back(std::stod(substr));
            }
        }
        trajectory_node.transl_x_ = data[0];
        trajectory_node.transl_y_ = data[1];
        trajectory_node.theta_ = data[2];
    }

    void readRawTrajectory2dFromFile(const std::string &file_name, std::vector<TrajectoryNode2d> &trajectory2d) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            TrajectoryNode2d trajectory_node;
            readTrajectory2dLine(line, trajectory_node);
            trajectory2d.emplace_back(trajectory_node);
        }
    }

    std::vector<pose::Pose2d> readTrajFromFile(const std::string &file_name) {
        std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
        file_io::readRawTrajectory2dFromFile(file_name, trajectory_nodes);

        // Assuming trajectory nodes are in order
        std::vector<pose::Pose2d> init_traj_poses;
        for (const file_io::TrajectoryNode2d traj_node : trajectory_nodes) {
            init_traj_poses.emplace_back(pose::createPose2d(traj_node.transl_x_, traj_node.transl_y_, traj_node.theta_));
        }
        return init_traj_poses;
    }

}

#endif //AUTODIFF_GP_TRAJECTORY_2D_IO_H
