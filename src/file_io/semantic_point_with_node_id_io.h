//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_SEMANTIC_POINT_WITH_NODE_ID_IO_H
#define AUTODIFF_GP_SEMANTIC_POINT_WITH_NODE_ID_IO_H

#include <stdint.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace semantic_segmentation {
    struct SemanticallyLabeledPointWithNodeIdInfo {

        /**
         * X coordinate of the point in the lidar frame.
         */
        double point_x;

        /**
         * Y coordinate of the point in the lidar frame.
         */
        double point_y;

        /**
         * Z coordinate of the point in the lidar frame.
         */
        double point_z;

        unsigned short semantic_label; // Should this be an unsigned short or a string?

        uint64_t node_id;

        /**
         * Identifier for the cluster.
         */
        uint32_t cluster_label;
    };

    void writeSemanticallyLabeledPointWithNodeIdInfoHeaderToFile(std::ofstream &file_stream) {
        file_io::writeCommaSeparatedStringsLineToFile(
                {"point_x", "point_y", "point_z", "semantic_label", "node_id", "cluster_label"}, file_stream);
    }


    void writeSemanticallyLabeledPointWithNodeIdInfoEntryLineToFile(
            const SemanticallyLabeledPointWithNodeIdInfo &semantic_point,
            std::ofstream &file_stream) {
        file_io::writeCommaSeparatedStringsLineToFile({std::to_string(semantic_point.point_x),
                                                       std::to_string(semantic_point.point_y),
                                                       std::to_string(semantic_point.point_z),
                                                       std::to_string(semantic_point.semantic_label),
                                                       std::to_string(semantic_point.node_id),
                                                       std::to_string(semantic_point.cluster_label)}, file_stream);
    }

    void writeSemanticallyLabeledPointWithNodeIdInfosToFile(const std::string &file_name,
                                                            const std::vector<SemanticallyLabeledPointWithNodeIdInfo> &semantic_points) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeSemanticallyLabeledPointWithNodeIdInfoHeaderToFile(csv_file);
        for (const SemanticallyLabeledPointWithNodeIdInfo &node_with_timestamp : semantic_points) {
            writeSemanticallyLabeledPointWithNodeIdInfoEntryLineToFile(node_with_timestamp, csv_file);
        }

        csv_file.close();
    }

    void readSemanticallyLabeledPointWithNodeIdInfoLine(const std::string &line_in_file,
                                                        SemanticallyLabeledPointWithNodeIdInfo &semantic_point) {
        std::stringstream ss(line_in_file);
        std::string substr;

        getline(ss, substr, ',');
        std::istringstream stream_px(substr);
        stream_px >> semantic_point.point_x;

        getline(ss, substr, ',');
        std::istringstream stream_py(substr);
        stream_py >> semantic_point.point_y;

        getline(ss, substr, ',');
        std::istringstream stream_pz(substr);
        stream_py >> semantic_point.point_z;

        getline(ss, substr, ',');
        std::istringstream stream_semantic_label(substr);
        stream_semantic_label >> semantic_point.semantic_label;

        getline(ss, substr, ',');
        std::istringstream stream_node_id(substr);
        stream_node_id >> semantic_point.node_id;

        getline(ss, substr, ',');
        std::istringstream stream_cluster_label(substr);
        stream_cluster_label >> semantic_point.cluster_label;
    }

    void readSemanticallyLabeledPointWithNodeIdInfoFromFile(const std::string &file_name,
                                                            std::vector<SemanticallyLabeledPointWithNodeIdInfo> &semantic_points) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            SemanticallyLabeledPointWithNodeIdInfo semantic_point;
            readSemanticallyLabeledPointWithNodeIdInfoLine(line, semantic_point);
            semantic_points.emplace_back(semantic_point);
        }
    }
}

#endif //AUTODIFF_GP_SEMANTIC_POINT_WITH_NODE_ID_IO_H