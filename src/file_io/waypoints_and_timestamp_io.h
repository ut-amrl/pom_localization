//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_WAYPOINT_AND_TIMESTAMP_IO_H
#define AUTODIFF_GP_WAYPOINT_AND_TIMESTAMP_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace file_io {
    struct WaypointAndTimestamp {
        uint64_t waypoint;
        uint32_t seconds_;
        uint32_t nano_seconds_;
    };

    void writeWaypointAndTimestampHeaderToFile(std::ofstream &file_stream) {
        file_stream << "waypoint_id" << ", " << "seconds" << ", " << "nanoseconds" << "\n";
    }


    void writeWaypointAndTimestampEntryLineToFile(const WaypointAndTimestamp &waypoint_and_timestamp,
                                                std::ofstream &file_stream) {
        file_stream << waypoint_and_timestamp.waypoint << ", " << waypoint_and_timestamp.seconds_ << ", "
                    << waypoint_and_timestamp.nano_seconds_ << "\n";
    }

    void writeWaypointsAndTimestampsToFile(const std::string &file_name,
                                       const std::vector<WaypointAndTimestamp> &waypoints_with_timestamps) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeWaypointAndTimestampHeaderToFile(csv_file);
        for (const WaypointAndTimestamp &waypoint_with_timestamp : waypoints_with_timestamps) {
            writeWaypointAndTimestampEntryLineToFile(waypoint_with_timestamp, csv_file);
        }

        csv_file.close();
    }

    void readWaypointAndTimestampLine(const std::string &line_in_file, WaypointAndTimestamp &waypoint_and_timestamp) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        std::string substr;
        getline(ss, substr, ',');
        std::istringstream stream_identifier(substr);
        stream_identifier >> waypoint_and_timestamp.waypoint;

        getline(ss, substr, ',');
        std::istringstream stream_seconds(substr);
        stream_seconds >> waypoint_and_timestamp.seconds_;

        getline(ss, substr, ',');
        std::istringstream stream_nanoseconds(substr);
        stream_nanoseconds >> waypoint_and_timestamp.nano_seconds_;
    }

    void readWaypointsAndTimestampsFromFile(const std::string &file_name, std::vector<WaypointAndTimestamp> &waypoints_and_timestamps) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            WaypointAndTimestamp waypoint_and_timestamp;
            readWaypointAndTimestampLine(line, waypoint_and_timestamp);
            waypoints_and_timestamps.emplace_back(waypoint_and_timestamp);
        }
    }
}

#endif //AUTODIFF_GP_WAYPOINT_AND_TIMESTAMP_IO_H
