//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_SEMANTIC_POINT_WITH_TIMESTAMP_IO_H
#define AUTODIFF_GP_SEMANTIC_POINT_WITH_TIMESTAMP_IO_H

#include <stdint.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace semantic_segmentation {
    struct SemanticallyLabeledPointWithTimestampInfo {

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

        /**
         * Seconds component of the timestamp of the frame that the semantic label was taken from.
         *
         * TODO should this be the camera frame or the lidar frame. Probably image? Easier to interpolate 3d points
         * than image.
         */
        uint32_t seconds;

        /**
         * Seconds component of the timestamp of the frame that the semantic label was taken from.
         *
         * TODO should this be the camera frame or the lidar frame. Probably image? Easier to interpolate 3d points
         * than image.
         */
        uint32_t nano_seconds;

        /**
         * Identifier for the cluster.
         */
        uint32_t cluster_label;
    };

    void writeSemanticallyLabeledPointWithTimestampInfoHeaderToFile(std::ofstream &file_stream) {
        file_io::writeCommaSeparatedStringsLineToFile(
                {"point_x", "point_y", "point_z", "semantic_label", "seconds", "nanoseconds", "cluster_label"},
                file_stream);
    }


    void writeSemanticallyLabeledPointWithTimestampInfoEntryLineToFile(
            const SemanticallyLabeledPointWithTimestampInfo &semantic_point,
            std::ofstream &file_stream) {
        file_stream << semantic_point.point_x << ", " << semantic_point.point_y << ", " << semantic_point.point_z <<
                    ", " << semantic_point.semantic_label << ", " << semantic_point.seconds << ", "
                    << semantic_point.nano_seconds << ", " << semantic_point.cluster_label << "\n";
    }

    void writeSemanticallyLabeledPointWithTimestampInfosToFile(const std::string &file_name,
                                                               const std::vector<SemanticallyLabeledPointWithTimestampInfo> &semantic_points) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeSemanticallyLabeledPointWithTimestampInfoHeaderToFile(csv_file);
        for (const SemanticallyLabeledPointWithTimestampInfo &node_with_timestamp : semantic_points) {
            writeSemanticallyLabeledPointWithTimestampInfoEntryLineToFile(node_with_timestamp, csv_file);
        }

        csv_file.close();
    }

    void readSemanticallyLabeledPointWithTimestampInfoLine(const std::string &line_in_file,
                                                           SemanticallyLabeledPointWithTimestampInfo &semantic_point) {
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
        std::istringstream stream_seconds(substr);
        stream_seconds >> semantic_point.seconds;

        getline(ss, substr, ',');
        std::istringstream stream_nsec(substr);
        stream_nsec >> semantic_point.nano_seconds;

        getline(ss, substr, ',');
        std::istringstream stream_cluster_label(substr);
        stream_cluster_label >> semantic_point.cluster_label;
    }

    void readSemanticallyLabeledPointWithTimestampInfoFromFile(const std::string &file_name,
                                                               std::vector<SemanticallyLabeledPointWithTimestampInfo> &semantic_points) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            SemanticallyLabeledPointWithTimestampInfo semantic_point;
            readSemanticallyLabeledPointWithTimestampInfoLine(line, semantic_point);
            semantic_points.emplace_back(semantic_point);
        }
    }
}

#endif //AUTODIFF_GP_SEMANTIC_POINT_WITH_TIMESTAMP_IO_H
