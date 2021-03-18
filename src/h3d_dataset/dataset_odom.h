//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_DATASET_ODOM_H
#define AUTODIFF_GP_DATASET_ODOM_H

#include <fstream>
#include <vector>
#include <sstream>

#include <h3d_dataset/h3d_file_operations.h>

namespace h3d {

    struct RawDatasetOdomData {
        double transl_x_;
        double transl_y_;
        double transl_z_;
        double roll_;
        double pitch_;
        double yaw_;
    };

    void readRawDatasetOdomLine(const std::string &line_in_file, RawDatasetOdomData &dataset_odom_data) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ' ');
            data.push_back(std::stod(substr));
        }
        dataset_odom_data.transl_x_ = data[0];
        dataset_odom_data.transl_y_ = data[1];
        dataset_odom_data.transl_z_ = data[2];
        dataset_odom_data.roll_ = data[3];
        dataset_odom_data.pitch_ = data[4];
        dataset_odom_data.yaw_ = data[5];
    }

    void readRawDatasetOdomFromFile(const std::string &file_name, std::vector<RawDatasetOdomData> &odom_ests_in_abs_frame) {
        std::ifstream file_obj(file_name);
        std::string line;
        while (std::getline(file_obj, line)) {
            RawDatasetOdomData dataset_odom_data;
            readRawDatasetOdomLine(line, dataset_odom_data);
            odom_ests_in_abs_frame.emplace_back(dataset_odom_data);
        }
    }

    std::vector<RawDatasetOdomData> readRawDatasetOdomForFileNum(const std::string &scenario_dir, const std::string &file_num_str) {
        std::string gps_filename = scenario_dir;
        gps_filename += kDatasetOdomFilePrefix;
        gps_filename += file_num_str;
        gps_filename += kTxtFileSuffix;

        std::vector<RawDatasetOdomData> odom_data;

        readRawDatasetOdomFromFile(gps_filename, odom_data);
        return odom_data;
    }

    std::vector<RawDatasetOdomData> readRawDatasetOdomForFileNum(const std::string &scenario_dir, const int &file_num) {
        return readRawDatasetOdomForFileNum(scenario_dir, convertFileNumToFileStrSuffix(file_num));
    }
}

#endif //AUTODIFF_GP_DATASET_ODOM_H
