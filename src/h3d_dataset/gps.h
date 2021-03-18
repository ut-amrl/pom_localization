//
// Created by amanda on 2/13/21.
//

#ifndef AUTODIFF_GP_GPS_H
#define AUTODIFF_GP_GPS_H

#include <string>
#include <math/math_util.h>

namespace h3d {

    // Earth geoid parameters from WGS 84 system
    // https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
    // a = Semimajor (Equatorial) axis
    static constexpr double wgs_84_a = 6378137.0;
    // b = Semiminor (Polar) axis
    static constexpr double wgs_84_b = 6356752.314245;

    struct GPSData {
        GPSData() = default;
        double timestamp_ = 0;
        double long_rel_ = 0;
        double lat_rel_ = 0;
        double in_height_ = 0;
        double tilt_roll_ = 0;
        double tilt_pitch_ = 0;
        double tilt_yaw_ = 0;
        double vel_x_ = 0;
        double vel_y_ = 0;
        double vel_z_ = 0;
        double std_dev_x_ = 0;
        double std_dev_y_ = 0;
        double std_dev_z_ = 0;
        double std_dev_roll_ = 0;
        double std_dev_pitch_ = 0;
        double std_dev_yaw_ = 0;
        double std_dev_vel_x_ = 0;
        double std_dev_vel_y_ = 0;
        double std_dev_vel_z_ = 0;
        double abs_lat_ = 0;
        double abs_long_ = 0;
    };

    struct PreprocessedGPSWithXY {
        GPSData orig_gps_data_;
        double origin_lat_;
        double origin_long_;
        double rel_x_;
        double rel_y_;
    };

    GPSData readLineFromGPSFile(const std::string &gps_file_line) {
        std::stringstream ss(gps_file_line);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        GPSData gps_data;
        gps_data.timestamp_ = data[0];
        gps_data.long_rel_ = data[1];
        gps_data.lat_rel_ = data[2];
        gps_data.in_height_ = data[3];
        gps_data.tilt_roll_ = math_util::DegToRad(data[4]);
        gps_data.tilt_pitch_ = math_util::DegToRad(data[5]);
        gps_data.tilt_yaw_ = math_util::DegToRad(data[6]);
        gps_data.vel_x_ = data[7];
        gps_data.vel_y_ = data[8];
        gps_data.vel_z_ = data[9];
        gps_data.std_dev_x_ = data[10];
        gps_data.std_dev_y_ = data[11];
        gps_data.std_dev_z_ = data[12];
        gps_data.std_dev_roll_ = data[13];
        gps_data.std_dev_pitch_ = data[14];
        gps_data.std_dev_yaw_ = data[15];
        gps_data.std_dev_vel_x_ = data[16];
        gps_data.std_dev_vel_y_ = data[17];
        gps_data.std_dev_vel_z_ = data[18];
        gps_data.abs_lat_ = data[19];
        gps_data.abs_long_ = data[20];
        return gps_data;
    }

    PreprocessedGPSWithXY readLineFromPreprocessedGPSFile(const std::string &gps_file_line) {
        std::stringstream ss(gps_file_line);
        std::vector<double> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        int index = 0;
        GPSData gps_data;
        gps_data.timestamp_ = data[index++];
        gps_data.long_rel_ = data[index++];
        gps_data.lat_rel_ = data[index++];
        gps_data.in_height_ = data[index++];
        gps_data.tilt_roll_ = math_util::DegToRad(data[index++]);
        gps_data.tilt_pitch_ = math_util::DegToRad(data[index++]);
        gps_data.tilt_yaw_ = math_util::DegToRad(data[index++]);
        gps_data.vel_x_ = data[index++];
        gps_data.vel_y_ = data[index++];
        gps_data.vel_z_ = data[index++];
        gps_data.std_dev_x_ = data[index++];
        gps_data.std_dev_y_ = data[index++];
        gps_data.std_dev_z_ = data[index++];
        gps_data.std_dev_roll_ = data[index++];
        gps_data.std_dev_pitch_ = data[index++];
        gps_data.std_dev_yaw_ = data[index++];
        gps_data.std_dev_vel_x_ = data[index++];
        gps_data.std_dev_vel_y_ = data[index++];
        gps_data.std_dev_vel_z_ = data[index++];
        gps_data.abs_lat_ = data[index++];
        gps_data.abs_long_ = data[index++];

        PreprocessedGPSWithXY preprocessed_gps;
        preprocessed_gps.orig_gps_data_ = gps_data;
        preprocessed_gps.origin_lat_ = data[index++];
        preprocessed_gps.origin_long_ = data[index++];
        preprocessed_gps.rel_x_ = data[index++];
        preprocessed_gps.rel_y_ = data[index++];
        return preprocessed_gps;
    }

    std::vector<GPSData> readGpsDataFromFile(const std::string &gps_filename) {
        std::ifstream gps_file_obj(gps_filename);
        std::string line;
        bool first_line = true;
        std::vector<GPSData> gps_data;
        while (std::getline(gps_file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            gps_data.emplace_back(readLineFromGPSFile(line));
        }
        return gps_data;
    }

    std::vector<GPSData> readGpsDataForFileNum(const std::string &scenario_dir, const std::string &file_num_str) {
        std::string gps_filename = scenario_dir;
        gps_filename += kGpsFilePrefix;
        gps_filename += file_num_str;
        gps_filename += kCsvFileSuffix;

        return readGpsDataFromFile(gps_filename);
    }

    std::vector<GPSData> readGpsDataForFileNum(const std::string &scenario_dir, const int file_num) {
        return readGpsDataForFileNum(scenario_dir, convertFileNumToFileStrSuffix(file_num));
    }

    std::vector<PreprocessedGPSWithXY> readPreprocessedGpsDataFromFile(const std::string &gps_filename) {
        std::ifstream gps_file_obj(gps_filename);
        std::string line;
        bool first_line = true;
        std::vector<PreprocessedGPSWithXY> gps_data;
        while (std::getline(gps_file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            gps_data.emplace_back(readLineFromPreprocessedGPSFile(line));
        }
        return gps_data;
    }

    std::vector<PreprocessedGPSWithXY> readPreprocessedGpsDataForFileNum(const std::string &preproc_scenario_dir, const std::string file_num_str) {
        std::string gps_filename = preproc_scenario_dir;
        gps_filename += kPreprocessedGpsPrefix;
        gps_filename += kGpsFilePrefix;
        gps_filename += file_num_str;
        gps_filename += kCsvFileSuffix;

        return readPreprocessedGpsDataFromFile(gps_filename);
    }

    std::vector<std::pair<std::string, double>> getTimestampsForFileNums(const std::vector<std::pair<std::string, std::vector<GPSData>>> &gps_data_by_filenums) {
        std::vector<std::pair<std::string, double>> filenum_and_timestamp;
        for (const auto &gps_data_with_filenum : gps_data_by_filenums) {
            filenum_and_timestamp.emplace_back(std::make_pair(gps_data_with_filenum.first, gps_data_with_filenum.second[0].timestamp_));
        }
        return filenum_and_timestamp;
    }

    std::vector<std::pair<std::string, double>> getTimestampsForFileNums(const std::string &scenario_dir_str) {
        int max_file_num_for_scenario = h3d::getMaxFileNumForScenario(scenario_dir_str, h3d::kPointCloudFilePrefix);

        std::vector<std::pair<std::string, std::vector<GPSData>>> gps_data_by_filenums;
        for (int i = 0; i <= max_file_num_for_scenario; i++) {
            std::string file_num_str = convertFileNumToFileStrSuffix(i);
            gps_data_by_filenums.emplace_back(std::make_pair(file_num_str, h3d::readGpsDataForFileNum(scenario_dir_str, i)));
        }

        return getTimestampsForFileNums(gps_data_by_filenums);
    }

    std::vector<GPSData> getFirstGpsDataForEachFileNumInScenario(const std::string &dataset_dir, const std::string &scenario_num_str) {
        std::string scenario_dir = h3d::getScenarioDirectory(dataset_dir, scenario_num_str);
        int max_file_num_for_scenario = h3d::getMaxFileNumForScenario(scenario_dir, h3d::kPointCloudFilePrefix);

        std::vector<GPSData> gps_data;
        for (int i = 0; i <= max_file_num_for_scenario; i++) {
            std::vector<GPSData> gps_data_for_filenum = readGpsDataForFileNum(scenario_dir, i);
            gps_data.emplace_back(gps_data_for_filenum[0]);
        }
        return gps_data;
    }

    Eigen::Vector2d gpsToMetric(const double latitude, const double longitude,
                                const double &origin_latitude, const double &origin_longitude) {
        const double theta = math_util::DegToRad(latitude);
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        const double r = sqrt(math_util::Sq(wgs_84_a * wgs_84_b) / (math_util::Sq(c * wgs_84_b) + math_util::Sq(s * wgs_84_a)));
        const double dlat = math_util::DegToRad(latitude - origin_latitude);
        const double dlong = math_util::DegToRad(longitude - origin_longitude);
        const double r1 = r * c;
        const double x = r1 * dlong;
        const double y = r * dlat;
        return Eigen::Vector2d(x, y);
    }
}
#endif //AUTODIFF_GP_GPS_H
