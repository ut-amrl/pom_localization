//
// Created by amanda on 1/31/21.
//

#ifndef AUTODIFF_GP_PARKING_LOT_SIMULATOR_H
#define AUTODIFF_GP_PARKING_LOT_SIMULATOR_H

#include <vector>
#include <base_lib/pose_reps.h>
#include <chrono>

namespace synthetic_problem {

    struct ParkingLotSpacingConfigurationParams {
        double min_x;
        double min_y;
        double max_x;
        double max_y;
        double seed_spot_x;
        double seed_spot_y;
        double parking_lot_spacing_x;
        double parking_lot_spacing_y;
        double parking_lot_aisle_gap_x;
        double parking_lot_aisle_gap_y;
        int spots_between_gaps_y;
        double right_parking_angle;
        double left_parking_angle;
        unsigned int max_frequency_ratio;
    };

    struct ParkingLotConfigurationParams {
        std::vector<std::pair<pose::Pose2d, unsigned int>> parking_spots_and_relative_frequency_;
        double parking_lot_std_dev_x_;
        double parking_lot_std_dev_y_;
        double parking_lot_std_dev_yaw_;
        int32_t num_samples_multiplier_;
        double parking_lot_percent_filled_; // 0 to 1
    };

    std::vector<std::pair<pose::Pose2d, unsigned int>> generateParkingSpotsAndFrequency(
            ParkingLotSpacingConfigurationParams spacing_config) {
        std::vector<std::pair<pose::Pose2d, unsigned int>> spots_and_frequency;
        std::vector<double> right_lane_x_vals;
        std::vector<double> left_lane_x_vals;
        std::vector<double> y_vals;

        double right_lane_next_x = spacing_config.seed_spot_x;
        while (right_lane_next_x <= spacing_config.max_x) {
            right_lane_x_vals.emplace_back(right_lane_next_x);
            right_lane_next_x += (spacing_config.parking_lot_spacing_x + spacing_config.parking_lot_aisle_gap_x);
        }
        right_lane_next_x = spacing_config.seed_spot_x - (spacing_config.parking_lot_spacing_x + spacing_config.parking_lot_aisle_gap_x);
        while (right_lane_next_x >= spacing_config.min_x) {
            right_lane_x_vals.emplace_back(right_lane_next_x);
            right_lane_next_x -= (spacing_config.parking_lot_spacing_x + spacing_config.parking_lot_aisle_gap_x);
        }

        double left_lane_next_x = spacing_config.seed_spot_x + spacing_config.parking_lot_spacing_x;
        while (left_lane_next_x <= spacing_config.max_x) {
            left_lane_x_vals.emplace_back(left_lane_next_x);
            left_lane_next_x += (spacing_config.parking_lot_spacing_x + spacing_config.parking_lot_aisle_gap_x);
        }
        left_lane_next_x = spacing_config.seed_spot_x - spacing_config.parking_lot_aisle_gap_x;
        while (left_lane_next_x >= spacing_config.min_x) {
            left_lane_x_vals.emplace_back(left_lane_next_x);
            left_lane_next_x -= (spacing_config.parking_lot_spacing_x + spacing_config.parking_lot_aisle_gap_x);
        }

        double next_y = spacing_config.seed_spot_y;
        int parking_spots_counter = 0;
        while (next_y <= spacing_config.max_y) {
            y_vals.emplace_back(next_y);
            parking_spots_counter++;
            if ((parking_spots_counter % spacing_config.spots_between_gaps_y) == 0) {
                next_y += spacing_config.parking_lot_aisle_gap_y;
            } else {
                next_y += spacing_config.parking_lot_spacing_y;
            }
        }
        next_y = spacing_config.seed_spot_y - spacing_config.parking_lot_aisle_gap_y;
        parking_spots_counter = 0;
        while (next_y >= spacing_config.min_y) {
            y_vals.emplace_back(next_y);
            parking_spots_counter++;
            if ((parking_spots_counter % spacing_config.spots_between_gaps_y) == 0) {
                next_y -= spacing_config.parking_lot_aisle_gap_y;
            } else {
                next_y -= spacing_config.parking_lot_spacing_y;
            }
        }

        std::string y_str;
        for (double y_for_spot : y_vals) {
            y_str += std::to_string(y_for_spot);
            y_str += ", ";
        }
        LOG(INFO) << "Y spots " << y_str;
        std::string right_x_str;
        for (double right_x : right_lane_x_vals) {
            right_x_str += std::to_string(right_x);
            right_x_str += ", ";
        }
        LOG(INFO) << "Right x spots " << right_x_str; // << right_lane_x_vals;

        std::string left_x_str;
        for (double left_x : left_lane_x_vals) {
            left_x_str += std::to_string(left_x);
            left_x_str += ", ";
        }
        LOG(INFO) << "Left x spots " << left_x_str;

        util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
        for (double y_for_spot : y_vals) {
            // Add right lane spots
            for (double right_lane_x : right_lane_x_vals) {
//                LOG(INFO) << "Adding spot " << right_lane_x << ", " << y_for_spot;
                spots_and_frequency.emplace_back(std::make_pair(
                        pose::createPose2d(right_lane_x, y_for_spot, spacing_config.right_parking_angle),
//                        1));
                        (unsigned int) random_generator.RandomInt(1, (int) spacing_config.max_frequency_ratio + 1)));
            }

            for (double left_lane_x : left_lane_x_vals) {

//                LOG(INFO) << "Adding spot " << left_lane_x << ", " << y_for_spot;
                spots_and_frequency.emplace_back(std::make_pair(
                        pose::createPose2d(left_lane_x, y_for_spot, spacing_config.left_parking_angle),
//                        1));
                        (unsigned int) random_generator.RandomInt(1, (int) spacing_config.max_frequency_ratio + 1)));
            }

        }

        return spots_and_frequency;
    }

    void createPastAndPresentObservations(
            const std::vector<std::pair<pose::Pose2d, unsigned int>> &poses_and_relative_frequncy,
            const double &parking_lot_std_dev_x, const double &parking_lot_std_dev_y,
            const double &parking_lot_std_dev_yaw,
            const int32_t num_samples_multiplier,
            const double &percent_filled,
            std::vector<pose::Pose2d> &past_observations,
            std::vector<pose::Pose2d> &current_placements) {

//        util_random::Random random_generator;
        util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

        for (const std::pair<pose::Pose2d, unsigned int> &pose_and_relative_count : poses_and_relative_frequncy) {
            for (unsigned int i = 0; i < pose_and_relative_count.second; i++) {
                for (int j = 0; j < num_samples_multiplier; j++) {
                    past_observations.emplace_back(pose::addGaussianNoise(
                            pose_and_relative_count.first, parking_lot_std_dev_x, parking_lot_std_dev_y,
                            parking_lot_std_dev_yaw, random_generator));
                }
            }
        }


        std::vector<size_t> open_spots_scaled_by_likelihood;
        for (size_t i = 0; i < poses_and_relative_frequncy.size(); i++) {
            for (size_t j = 0; j < (poses_and_relative_frequncy[i]).second; j++) {
                open_spots_scaled_by_likelihood.emplace_back(i);
            }
        }

        unsigned int num_filled_spots = ((double) poses_and_relative_frequncy.size()) * percent_filled;
        for (unsigned int i = 0; i < num_filled_spots; i++) {
            size_t filled_spot_list_index = std::rand() % open_spots_scaled_by_likelihood.size();
            size_t spot_index = open_spots_scaled_by_likelihood[filled_spot_list_index];
            current_placements.emplace_back(pose::addGaussianNoise((poses_and_relative_frequncy[spot_index]).first,
                                                                   parking_lot_std_dev_x, parking_lot_std_dev_y,
                                                                   parking_lot_std_dev_yaw, random_generator));
            open_spots_scaled_by_likelihood.erase(std::remove(open_spots_scaled_by_likelihood.begin(),
                                                              open_spots_scaled_by_likelihood.end(), spot_index),
                                                  open_spots_scaled_by_likelihood.end());
        }
    }
}

#endif //AUTODIFF_GP_PARKING_LOT_SIMULATOR_H
