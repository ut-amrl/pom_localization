//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_SYNTHETIC_DISTRIBUTION_GENERATION_CONFIG_MULTIPLE_CAR_SELECTION_PROFILES_IO_H
#define AUTODIFF_GP_SYNTHETIC_DISTRIBUTION_GENERATION_CONFIG_MULTIPLE_CAR_SELECTION_PROFILES_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace file_io {

    struct CarPlacementNoiseProfile {

        double parking_spot_generation_std_dev_x_;
        double parking_spot_generation_std_dev_y_;
        double parking_spot_generation_std_dev_theta_;
    };

    struct SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles {

        uint32_t num_trajectories_;
        uint32_t num_samples_per_beam_;
        uint32_t scan_num_beams_;

        double percent_parking_spots_filled_;

        double trajectory_variation_std_dev_x_;
        double trajectory_variation_std_dev_y_;
        double trajectory_variation_std_dev_theta_;

        double object_detection_variance_per_len_x_;
        double object_detection_variance_per_len_y_;
        double object_detection_variance_per_len_theta_;

        double max_object_detection_distance_;

        double scan_min_range_;
        double scan_max_range_;
        double scan_min_angle_;
        double scan_max_angle_;

        double percent_beams_per_scan_;
        double percent_poses_to_sample_;

        std::vector<std::pair<double, CarPlacementNoiseProfile>> car_placement_profile;
    };

    void readSyntheticDistributionConfigFromLine(const std::string &line_in_file,
                                                 SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles &synthetic_dist_config) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        int loop_count = 0;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (loop_count == 0) {
                std::istringstream stream(substr);
                stream >> synthetic_dist_config.num_trajectories_;
                LOG(INFO) << "Num trajectories in config " << synthetic_dist_config.num_trajectories_;
            } else if (loop_count == 1) {
                std::istringstream stream(substr);
                stream >> synthetic_dist_config.num_samples_per_beam_;
            } else if (loop_count == 2) {
                std::istringstream stream(substr);
                stream >> synthetic_dist_config.scan_num_beams_;
            } else {
                data.push_back(std::stod(substr));
            }
            loop_count++;
        }

        size_t data_index = 0;
        synthetic_dist_config.percent_parking_spots_filled_ = data[data_index++];

        synthetic_dist_config.trajectory_variation_std_dev_x_ = data[data_index++];
        synthetic_dist_config.trajectory_variation_std_dev_y_ = data[data_index++];
        synthetic_dist_config.trajectory_variation_std_dev_theta_ = data[data_index++];

        synthetic_dist_config.object_detection_variance_per_len_x_ = data[data_index++];
        synthetic_dist_config.object_detection_variance_per_len_y_ = data[data_index++];
        synthetic_dist_config.object_detection_variance_per_len_theta_ = data[data_index++];

        synthetic_dist_config.max_object_detection_distance_ = data[data_index++];

        synthetic_dist_config.scan_min_range_ = data[data_index++];
        synthetic_dist_config.scan_max_range_ = data[data_index++];
        synthetic_dist_config.scan_min_angle_ = data[data_index++];
        synthetic_dist_config.scan_max_angle_ = data[data_index++];

        synthetic_dist_config.percent_beams_per_scan_ = data[data_index++];
        synthetic_dist_config.percent_poses_to_sample_ = data[data_index++];

        std::vector<std::pair<double, CarPlacementNoiseProfile>> unnormalized_profiles;
        double accumulated_ratios = 0;

        while (data_index < data.size()) {
            double ratio = data[data_index++];
            accumulated_ratios += ratio;
            CarPlacementNoiseProfile car_placement_noise;
            car_placement_noise.parking_spot_generation_std_dev_x_ = data[data_index++];
            car_placement_noise.parking_spot_generation_std_dev_y_ = data[data_index++];
            car_placement_noise.parking_spot_generation_std_dev_theta_ = data[data_index++];

            unnormalized_profiles.emplace_back(std::make_pair(ratio, car_placement_noise));
        }

        for (size_t j = 0; j < unnormalized_profiles.size(); j++) {
            std::pair<double, CarPlacementNoiseProfile> orig_profile = unnormalized_profiles[j];
            synthetic_dist_config.car_placement_profile.emplace_back(std::make_pair((orig_profile.first / accumulated_ratios), orig_profile.second));
        }
    }

    void readSyntheticDistributionConfigFromFile(const std::string &file_name,
                                                 SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles &synthetic_distribution_config) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            // There should only be one line after the header
            readSyntheticDistributionConfigFromLine(line, synthetic_distribution_config);
        }
    }

    void writeSyntheticDistributionConfigToFile(const std::string &file_name,
                                                const SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles &config) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeCommaSeparatedStringsLineToFile(
                {"num_trajectories", "num_samples_per_beam", "scan_num_beams",
                 "percent_parking_spots_filled", "trajectory_variation_std_dev_x", "trajectory_variation_std_dev_y",
                 "trajectory_variation_std_dev_theta", "object_detection_variance_per_len_x",
                 "object_detection_variance_per_len_y", "object_detection_variance_per_len_theta",
                 "max_object_detection_distance", "scan_min_range", "scan_max_range", "scan_min_angle",
                 "scan_max_angle", "percent_beams_per_scan", "percent_poses_to_sample", "sampling_ratio",
                 "parking_spot_generation_std_dev_x", "parking_spot_generation_std_dev_y",
                 "parking_spot_generation_std_dev_theta"}, csv_file);
        std::vector<std::string> data_strings = {std::to_string(config.num_trajectories_),
                                                 std::to_string(config.num_samples_per_beam_),
                                                 std::to_string(config.scan_num_beams_),
                                                 std::to_string(config.percent_parking_spots_filled_),
                                                 std::to_string(config.trajectory_variation_std_dev_x_),
                                                 std::to_string(config.trajectory_variation_std_dev_y_),
                                                 std::to_string(config.trajectory_variation_std_dev_theta_),
                                                 std::to_string(config.object_detection_variance_per_len_x_),
                                                 std::to_string(config.object_detection_variance_per_len_y_),
                                                 std::to_string(config.object_detection_variance_per_len_theta_),
                                                 std::to_string(config.max_object_detection_distance_),
                                                 std::to_string(config.scan_min_range_),
                                                 std::to_string(config.scan_max_range_),
                                                 std::to_string(config.scan_min_angle_),
                                                 std::to_string(config.scan_max_angle_),
                                                 std::to_string(config.percent_beams_per_scan_),
                                                 std::to_string(config.percent_poses_to_sample_)};
        for (const auto &parking_spot_placement : config.car_placement_profile) {
            data_strings.emplace_back(std::to_string(parking_spot_placement.first));
            data_strings.emplace_back(std::to_string(parking_spot_placement.second.parking_spot_generation_std_dev_x_));
            data_strings.emplace_back(std::to_string(parking_spot_placement.second.parking_spot_generation_std_dev_y_));
            data_strings.emplace_back(
                    std::to_string(parking_spot_placement.second.parking_spot_generation_std_dev_theta_));
        }
        writeCommaSeparatedStringsLineToFile(data_strings, csv_file);
        csv_file.close();
    }
}

#endif //AUTODIFF_GP_SYNTHETIC_DISTRIBUTION_GENERATION_CONFIG_MULTIPLE_CAR_SELECTION_PROFILES_IO_H
