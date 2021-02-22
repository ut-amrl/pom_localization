//
// Created by amanda on 2/13/21.
//

#ifndef AUTODIFF_GP_H3D_FILE_OPERATIONS_H
#define AUTODIFF_GP_H3D_FILE_OPERATIONS_H

#include <string>
#include <experimental/filesystem>
#include <glog/logging.h>
#include <iomanip>

namespace h3d {

    static const std::string kScenarioNameStr = "scenario_";

    static const std::string kPointCloudFilePrefix = "pointcloud1_";
    static const std::string kGpsFilePrefix = "gps_";
    static const std::string kPreprocessedGpsPrefix = "augmented_";
    static const std::string kObjDetectionFilePrefix = "labels_3d1_";

    static const std::string kPointCloudFileExt = ".ply";
    static const std::string kCsvFileSuffix = ".csv";
    static const std::string kTxtFileSuffix = ".txt";

    int getMaxFileNumForScenario(const std::string &scenario_directory, const std::string &filetype_prefix) {
        int max_file_num = -1;
        for (const auto &entry : std::experimental::filesystem::directory_iterator(scenario_directory)) {
            std::string filename = entry.path().filename();
            LOG(INFO) << "filename " << filename;
            bool starts_with_prefix = filename.size() >= filetype_prefix.size() && (0 == filename.compare(0, filetype_prefix.size(), filetype_prefix));
            if (starts_with_prefix) {
                LOG(INFO) << "starts with prefix!";
                std::string num_str = filename.substr(filetype_prefix.size(), 3);
                LOG(INFO) << "Num str " << num_str;
                int num = std::stoi(num_str);
                LOG(INFO) << "Num " << num;
                max_file_num = std::max(num, max_file_num);
            }
        }
        LOG(INFO) << "Max file num " << max_file_num;
        return max_file_num;
    }

    std::string convertFileNumToFileStrSuffix(const int file_num) {
        std::string file_num_str = std::to_string(file_num);
        if (file_num < 10) {
            file_num_str = "0" + file_num_str;
        }
        if (file_num < 100) {
            file_num_str = "0" + file_num_str;
        }
        return file_num_str;
    }

    std::string getScenarioDirectory(const std::string &dataset_directory, const std::string scenario_number_string) {
        std::string scenario_directory_str = dataset_directory;
        if (scenario_directory_str.back() != '/') {
            scenario_directory_str += "/";
        }
        scenario_directory_str += kScenarioNameStr;
        scenario_directory_str += scenario_number_string;
        scenario_directory_str += "/";
        return scenario_directory_str;
    }

    void getFileNumCorrespondingToTimestamp(const std::vector<std::pair<std::string, double>> timestamps_and_filenum,
                                            const double &query_timestamp, const size_t &start_search_num,
                                            std::string &filenum_string, size_t &result_found_index) {
        for (size_t i = start_search_num; i < timestamps_and_filenum.size(); i++) {
            std::pair<std::string, double> timestamp_and_filenum = timestamps_and_filenum[i];
            if (timestamp_and_filenum.second == query_timestamp) {
                result_found_index = i;
                filenum_string = timestamp_and_filenum.first;
                break;
            }
        }
    }
}

#endif //AUTODIFF_GP_H3D_FILE_OPERATIONS_H
