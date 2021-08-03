//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_WAYPOINT_CONSISTENCY_RESULTS_IO_H
#define AUTODIFF_GP_WAYPOINT_CONSISTENCY_RESULTS_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

namespace file_io {
    struct WaypointConsistencyResult {
        uint64_t waypoint;
        double transl_deviation_;
        double rot_deviation_;
    };

    void writeWaypointConsistencyResultHeaderToFile(std::ofstream &file_stream) {
        file_stream << "waypoint_id" << ", " << "transl_deviation" << ", " << "rot_deviation" << "\n";
    }


    void writeWaypointConsistencyResultEntryLineToFile(const WaypointConsistencyResult &waypoint_consistency_result,
                                                       std::ofstream &file_stream) {
        file_stream << waypoint_consistency_result.waypoint << ", " << waypoint_consistency_result.transl_deviation_
                    << ", "
                    << waypoint_consistency_result.rot_deviation_ << "\n";
    }

    void writeWaypointConsistencyResultsToFile(const std::string &file_name,
                                               const std::vector<WaypointConsistencyResult> &waypoint_consistency_results) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeWaypointConsistencyResultHeaderToFile(csv_file);
        for (const WaypointConsistencyResult &waypoint_consistency_result : waypoint_consistency_results) {
            writeWaypointConsistencyResultEntryLineToFile(waypoint_consistency_result, csv_file);
        }

        csv_file.close();
    }

    void readWaypointConsistencyResultLine(const std::string &line_in_file,
                                           WaypointConsistencyResult &waypoint_consistency_result) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        std::string substr;
        getline(ss, substr, ',');
        std::istringstream stream_identifier(substr);
        stream_identifier >> waypoint_consistency_result.waypoint;

        getline(ss, substr, ',');
        waypoint_consistency_result.transl_deviation_ = std::stod(substr);

        getline(ss, substr, ',');
        waypoint_consistency_result.rot_deviation_ = std::stod(substr);
    }

    void readWaypointConsistencyResultsFromFile(const std::string &file_name,
                                                std::vector<WaypointConsistencyResult> &waypoint_consistency_results) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            WaypointConsistencyResult waypoint_consistency_result;
            readWaypointConsistencyResultLine(line, waypoint_consistency_result);
            waypoint_consistency_results.emplace_back(waypoint_consistency_result);
        }
    }
}

#endif //AUTODIFF_GP_WAYPOINT_CONSISTENCY_RESULTS_IO_H
