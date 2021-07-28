//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_WAYPOINT_AND_NODE_ID_IO_H
#define AUTODIFF_GP_WAYPOINT_AND_NODE_ID_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace file_io {

    void writeWaypointAndNodeIdHeaderToFile(std::ofstream &file_stream) {
        file_stream << "waypoint_id" << ", " << "node_id" << "\n";
    }


    void writeWaypointAndNodeIdEntryLineToFile(const std::pair<uint64_t, uint64_t> &waypoint_and_node_id,
                                               std::ofstream &file_stream) {
        file_stream << waypoint_and_node_id.first << ", " << waypoint_and_node_id.second << "\n";
    }

    void writeWaypointsAndNodeIdsToFile(const std::string &file_name,
                                           const std::vector<std::pair<uint64_t, uint64_t>> &waypoints_with_node_ids) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeWaypointAndNodeIdHeaderToFile(csv_file);
        for (const std::pair<uint64_t, uint64_t> &waypoint_with_node_id : waypoints_with_node_ids) {
            writeWaypointAndNodeIdEntryLineToFile(waypoint_with_node_id, csv_file);
        }

        csv_file.close();
    }

    void
    readWaypointAndNodeIdLine(const std::string &line_in_file, std::pair<uint64_t, uint64_t> &waypoint_and_node_id) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        std::string substr;
        getline(ss, substr, ',');
        std::istringstream stream_identifier(substr);
        stream_identifier >> waypoint_and_node_id.first;

        getline(ss, substr, ',');
        std::istringstream stream_seconds(substr);
        stream_seconds >> waypoint_and_node_id.second;
    }

    void readWaypointsAndNodeIdsFromFile(const std::string &file_name,
                                            std::vector<std::pair<uint64_t, uint64_t>> &waypoints_and_node_ids) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            std::pair<uint64_t, uint64_t> waypoint_and_node_id;
            readWaypointAndNodeIdLine(line, waypoint_and_node_id);
            waypoints_and_node_ids.emplace_back(waypoint_and_node_id);
        }
    }
}

#endif //AUTODIFF_GP_WAYPOINT_AND_NODE_ID_IO_H
