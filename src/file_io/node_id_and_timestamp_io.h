//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_NODE_ID_AND_TIMESTAMP_IO_H
#define AUTODIFF_GP_NODE_ID_AND_TIMESTAMP_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace file_io {
    struct NodeIdAndTimestamp {
        uint64_t node_id_;
        uint32_t seconds_;
        uint32_t nano_seconds_;
    };

    void writeNodeIdAndTimestampHeaderToFile(std::ofstream &file_stream) {
        file_stream << "node_id" << ", " << "seconds" << ", " << "nanoseconds" << "\n";
    }


    void writeNodeIdAndTimestampEntryLineToFile(const NodeIdAndTimestamp &node_id_and_timestamp,
                                                std::ofstream &file_stream) {
        file_stream << node_id_and_timestamp.node_id_ << ", " << node_id_and_timestamp.seconds_ << ", "
                    << node_id_and_timestamp.nano_seconds_ << "\n";
    }

    void writeNodeIdsAndTimestampsToFile(const std::string &file_name,
                                       const std::vector<NodeIdAndTimestamp> &nodes_with_timestamps) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeNodeIdAndTimestampHeaderToFile(csv_file);
        for (const NodeIdAndTimestamp &node_with_timestamp : nodes_with_timestamps) {
            writeNodeIdAndTimestampEntryLineToFile(node_with_timestamp, csv_file);
        }

        csv_file.close();
    }

    void readNodeIdAndTimestampLine(const std::string &line_in_file, NodeIdAndTimestamp &node_and_timestamp) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        std::string substr;
        getline(ss, substr, ',');
        std::istringstream stream_identifier(substr);
        stream_identifier >> node_and_timestamp.node_id_;

        getline(ss, substr, ',');
        std::istringstream stream_seconds(substr);
        stream_seconds >> node_and_timestamp.seconds_;

        getline(ss, substr, ',');
        std::istringstream stream_nanoseconds(substr);
        stream_nanoseconds >> node_and_timestamp.nano_seconds_;
    }

    void readNodeIdsAndTimestampsFromFile(const std::string &file_name, std::vector<NodeIdAndTimestamp> &nodes_and_timestamps) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            NodeIdAndTimestamp node_and_timestamp;
            readNodeIdAndTimestampLine(line, node_and_timestamp);
            nodes_and_timestamps.emplace_back(node_and_timestamp);
        }
    }
}

#endif //AUTODIFF_GP_NODE_ID_AND_TIMESTAMP_IO_H
