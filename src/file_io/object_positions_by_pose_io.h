//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_OBJECT_POSITIONS_BY_POSE_IO_H
#define AUTODIFF_GP_OBJECT_POSITIONS_BY_POSE_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace file_io {

    struct ObjectPositionByPose {
        uint64_t identifier_;
        uint64_t pose_number_;
        double transl_x_;
        double transl_y_;
        double theta_;
    };

    void readObjectPositionByPoseLine(const std::string &line_in_file, ObjectPositionByPose &object_position) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        bool first_entry_visited = false;
        bool second_entry_visited = false;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (!first_entry_visited) {
                first_entry_visited = true;
                std::istringstream stream(substr);
                stream >> object_position.identifier_;
            } else if (!second_entry_visited) {
                second_entry_visited = true;
                std::istringstream stream(substr);
                stream >> object_position.pose_number_;
            } else {
                data.push_back(std::stod(substr));
            }
        }
        object_position.transl_x_ = data[0];
        object_position.transl_y_ = data[1];
        object_position.theta_ = data[2];
    }

    void readObjectPositionsByPoseFromFile(const std::string &file_name,
                                           std::vector<ObjectPositionByPose> &object_positions) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            ObjectPositionByPose object_position;
            readObjectPositionByPoseLine(line, object_position);
            object_positions.emplace_back(object_position);
        }
    }

    void writeObjectPositionsByPoseHeaderToFile(std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({"instNum", "relativeToScan", "centroidX", "centroidY", "angle"},
                                             file_stream);
    }


    void writeObjectPositionsByPoseLineToFile(const ObjectPositionByPose &object_position, std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile(
                {std::to_string(object_position.identifier_), std::to_string(object_position.pose_number_),
                 std::to_string(object_position.transl_x_), std::to_string(object_position.transl_y_),
                 std::to_string(object_position.theta_)}, file_stream);
    }

    void writeObjectPositionsByPoseToFile(const std::string &file_name,
                                          const std::vector<ObjectPositionByPose> &object_positions) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeObjectPositionsByPoseHeaderToFile(csv_file);
        for (const ObjectPositionByPose &object_position : object_positions) {
            writeObjectPositionsByPoseLineToFile(object_position, csv_file);
        }

        csv_file.close();
    }
}

#endif //AUTODIFF_GP_OBJECT_POSITIONS_BY_POSE_IO_H
