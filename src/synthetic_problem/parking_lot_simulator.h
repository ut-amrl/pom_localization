//
// Created by amanda on 1/31/21.
//

#ifndef AUTODIFF_GP_PARKING_LOT_SIMULATOR_H
#define AUTODIFF_GP_PARKING_LOT_SIMULATOR_H

#include <vector>
#include <base_lib/pose_reps.h>
#include <chrono>

namespace synthetic_problem {

    struct ParkingLotConfigurationParams {
        std::vector<std::pair<pose::Pose2d, unsigned int>> parking_spots_and_relative_frequency_;
        double parking_lot_std_dev_x_;
        double parking_lot_std_dev_y_;
        double parking_lot_std_dev_yaw_;
        int32_t num_samples_multiplier_;
        double parking_lot_percent_filled_; // 0 to 1
    };

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
