//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_IO_H
#define AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

#include <file_io/object_positions_by_timestamp.h>
#include <file_io/file_io_utils.h>

namespace file_io {

    void
    readObjectPositionByTimestampLine(const std::string &line_in_file, ObjectPositionByTimestamp &object_position) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        std::string substr;
        getline(ss, substr, ',');
        object_position.label_ = substr;

        getline(ss, substr, ',');
        std::istringstream stream_identifier(substr);
        stream_identifier >> object_position.identifier_;

        getline(ss, substr, ',');
        std::istringstream stream_seconds(substr);
        stream_seconds >> object_position.seconds_;

        getline(ss, substr, ',');
        std::istringstream stream_nanoseconds(substr);
        stream_nanoseconds >> object_position.nano_seconds_;

        getline(ss, substr, ',');
        object_position.transl_x_ = std::stod(substr);

        getline(ss, substr, ',');
        object_position.transl_y_ = std::stod(substr);

        getline(ss, substr, ',');
        object_position.theta_ = std::stod(substr);
    }

    void readObjectPositionsByTimestampFromFile(const std::string &file_name,
                                                std::vector<ObjectPositionByTimestamp> &object_positions) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            ObjectPositionByTimestamp object_position;
            readObjectPositionByTimestampLine(line, object_position);
            object_positions.emplace_back(object_position);
        }
    }

    void writeObjectPositionsByTimestampHeaderToFile(std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile(
                {"label", "identifier", "seconds", "nanoseconds", "centroidX", "centroidY", "angle"},
                file_stream);
    }


    void writeObjectPositionsByTimestampLineToFile(const ObjectPositionByTimestamp &object_position,
                                                   std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile(
                {object_position.label_, std::to_string(object_position.identifier_),
                 std::to_string(object_position.seconds_), std::to_string(object_position.nano_seconds_),
                 std::to_string(object_position.transl_x_), std::to_string(object_position.transl_y_),
                 std::to_string(object_position.theta_)}, file_stream);
    }

    void writeObjectPositionsByTimestampToFile(const std::string &file_name,
                                               const std::vector<ObjectPositionByTimestamp> &object_positions) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeObjectPositionsByTimestampHeaderToFile(csv_file);
        for (const ObjectPositionByTimestamp &object_position : object_positions) {
            writeObjectPositionsByTimestampLineToFile(object_position, csv_file);
        }

        csv_file.close();
    }
}

#endif //AUTODIFF_GP_OBJECT_POSITIONS_BY_TIMESTAMP_IO_H
