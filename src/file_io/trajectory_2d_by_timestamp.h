//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_TRAJECTORY_2D_BY_TIMESTAMP_IO_H
#define AUTODIFF_GP_TRAJECTORY_2D_BY_TIMESTAMP_IO_H

#include <cstdint>
#include <fstream>
#include <vector>

namespace file_io {
    struct TrajectoryNode2dWithTimestamp {
        uint32_t seconds_;
        uint32_t nano_seconds_;
        double transl_x_;
        double transl_y_;
        double theta_;
    };

    void writeTrajectory2dWithTimestmapHeaderToFile(std::ofstream &file_stream) {
        file_stream << "secs" << ", " << "nsecs" << ", " << "transl_x"
                 << ", " << "transl_y" << ", " << "theta" << "\n";
    }


    void writeTrajectory2dWithTimestampEntryLineToFile(const TrajectoryNode2dWithTimestamp &trajectory_node, std::ofstream &file_stream) {
        file_stream << trajectory_node.seconds_ << ", " << trajectory_node.nano_seconds_ << ", " << trajectory_node.transl_x_
                    << ", " << trajectory_node.transl_y_ << ", " << trajectory_node.theta_ << "\n";
    }

    void writeTrajectory2dWithTimestampsToFile(const std::string &file_name, const std::vector<TrajectoryNode2dWithTimestamp> &trajectory_nodes) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeTrajectory2dWithTimestmapHeaderToFile(csv_file);
        for (const TrajectoryNode2dWithTimestamp &trajectory_node : trajectory_nodes) {
            writeTrajectory2dWithTimestampEntryLineToFile(trajectory_node, csv_file);
        }

        csv_file.close();
    }

    void readTrajectory2dWithTimestampLine(const std::string &line_in_file, TrajectoryNode2dWithTimestamp &trajectory_node) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        bool still_need_to_process_first_entry = true;
        bool still_need_to_process_second_entry = true;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (still_need_to_process_first_entry) {
                still_need_to_process_first_entry = false;
                std::istringstream stream(substr);
                stream >> trajectory_node.seconds_;
            } else if (still_need_to_process_second_entry) {
                still_need_to_process_second_entry = false;
                std::istringstream stream(substr);
                stream >> trajectory_node.nano_seconds_;
            } else {
                data.push_back(std::stod(substr));
            }
        }
        trajectory_node.transl_x_ = data[0];
        trajectory_node.transl_y_ = data[1];
        trajectory_node.theta_ = data[2];
    }

    void readRawTrajectory2dWithTimestampFromFile(const std::string &file_name, std::vector<TrajectoryNode2dWithTimestamp> &trajectory2d) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            TrajectoryNode2dWithTimestamp trajectory_node;
            readTrajectory2dWithTimestampLine(line, trajectory_node);
            trajectory2d.emplace_back(trajectory_node);
        }
    }
}

#endif //AUTODIFF_GP_TRAJECTORY_2D_BY_TIMESTAMP_IO_H
