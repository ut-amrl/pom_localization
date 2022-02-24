//
// Created by amanda on 2/4/22.
//

#ifndef AUTODIFF_GP_RECTANGLE_SAMPLER_H
#define AUTODIFF_GP_RECTANGLE_SAMPLER_H

#include <base_lib/pose_reps.h>
#include <shared/util/random.h>
#include <shared/math/math_util.h>
#include <unordered_set>

namespace semantic_point_pom {

    template <typename T>
    T AngleModSymmetric(T angle) {
        angle -= T(M_PI) * rint(angle / T(M_PI));
        return angle;
    }

    bool compareSizesOfBins(std::pair<size_t, std::pair<int, std::pair<int, int>>> a,
                            std::pair<size_t, std::pair<int, std::pair<int, int>>> b) {
        return a.first > b.first;
    };

    pose::Pose2d generateConsistentRectangleSample(const Eigen::Vector2d &point_on_rect,
                                                   const double &rectangle_width,
                                                   const double &rectangle_height,
                                                   const double &point_std_dev,
                                                   util_random::Random &random_generator) {
        Eigen::Vector2d point_to_use = point_on_rect;
        if (point_std_dev > 0) {
            point_to_use.x() = random_generator.Gaussian(point_to_use.x(), point_std_dev);
            point_to_use.y() = random_generator.Gaussian(point_to_use.y(), point_std_dev);
        }
        double angle_to_point = atan2(point_to_use.y(), point_to_use.x());

        double rect_perim = 2.0 * (rectangle_width + rectangle_height);
        double point_on_surface_rand = random_generator.UniformRandom(0, rect_perim);
        if (point_on_surface_rand == rect_perim) {
            point_on_surface_rand = 0;
        }

        Eigen::Vector2d point_in_rect_frame;
        double theta_modifier;
        if (point_on_surface_rand < rectangle_width) {
            point_in_rect_frame = Eigen::Vector2d(point_on_surface_rand - (rectangle_width / 2), -rectangle_height / 2);
            theta_modifier = 0;
        } else if (point_on_surface_rand < (rectangle_height + rectangle_width)) {
            point_in_rect_frame = Eigen::Vector2d(rectangle_width / 2,
                                                  point_on_surface_rand - rectangle_width - (rectangle_height / 2));
            theta_modifier = -M_PI_2;
        } else if (point_on_surface_rand < ((2 * rectangle_width) + rectangle_height)) {
            point_in_rect_frame = Eigen::Vector2d(
                    ((2 * rectangle_width) + rectangle_height) - point_on_surface_rand - (rectangle_width / 2),
                    rectangle_height / 2);
            theta_modifier = M_PI;
        } else {
            point_in_rect_frame = Eigen::Vector2d(-rectangle_width / 2,
                                                  rect_perim - point_on_surface_rand - rectangle_height / 2);
            theta_modifier = M_PI_2;
        }

        double rand_angle = random_generator.UniformRandom(0, M_PI);
        if (rand_angle >= M_PI) {
            rand_angle = 0;
        }

        double rectangle_angle = math_util::AngleMod(-rand_angle + angle_to_point + theta_modifier);

        double rect_center_x = point_to_use.x() - (point_in_rect_frame.x() * cos(rectangle_angle)) +
                               (point_in_rect_frame.y() * sin(rectangle_angle));
        double rect_center_y = point_to_use.y() - (point_in_rect_frame.x() * sin(rectangle_angle)) -
                               (point_in_rect_frame.y() * cos(rectangle_angle));

        return pose::createPose2d(rect_center_x, rect_center_y, rectangle_angle);
    }

    std::vector<pose::Pose2d> generateConsistentSamples(const std::vector<Eigen::Vector2d> &points_for_object,
                                                        const size_t &num_samples_to_generate,
                                                        const uint64_t &samples_per_point,
                                                        const double &position_bin_size,
                                                        const double &orientation_bin_size,
                                                        const double &rect_width,
                                                        const double &rect_height,
                                                        const double &point_std_dev,
                                                        const double &min_fraction_of_points_repped_in_bin,
                                                        const uint16_t &minimum_bins,
                                                        util_random::Random &random_generator) {
        std::vector<std::pair<size_t, pose::Pose2d>> unfiltered_candidate_rects;

        std::unordered_map<int, std::unordered_map<int,
                std::unordered_map<int, std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>>>>> binned_candidates;

        LOG(INFO) << "Generating samples from " << points_for_object.size() << " points";
        for (size_t i = 0; i < points_for_object.size(); i++) {
            Eigen::Vector2d object_point = points_for_object[i];
            for (uint64_t j = 0; j < samples_per_point; j++) {
                unfiltered_candidate_rects.emplace_back(
                        std::make_pair(i, generateConsistentRectangleSample(object_point, rect_width, rect_height,
                                                                            point_std_dev, random_generator)));
            }
        }

        LOG(INFO) << "Binning samples";
        for (const std::pair<size_t, pose::Pose2d> &candidate : unfiltered_candidate_rects) {
            int truncated_x = candidate.second.first.x() / position_bin_size;
            int truncated_y = candidate.second.first.y() / position_bin_size;
            // We should bin so that things that are 180 degrees from each other are in the same bin
            int truncated_theta = AngleModSymmetric(candidate.second.second) / orientation_bin_size;

            std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>> map_entry_for_candidate = std::make_pair(
                    (std::unordered_set<size_t>) {candidate.first}, (std::vector<pose::Pose2d>) {candidate.second});
            if (binned_candidates.find(truncated_x) != binned_candidates.end()) {
                if (binned_candidates[truncated_x].find(truncated_y) != binned_candidates[truncated_x].end()) {
                    if (binned_candidates[truncated_x][truncated_y].find(truncated_theta) !=
                        binned_candidates[truncated_x][truncated_y].end()) {
                        std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>> old_entry_for_candidate = binned_candidates[truncated_x][truncated_y][truncated_theta];
                        std::unordered_set<size_t> new_origin_points_set = old_entry_for_candidate.first;
                        new_origin_points_set.insert(candidate.first);
                        std::vector<pose::Pose2d> new_candidates_set = old_entry_for_candidate.second;
                        new_candidates_set.emplace_back(candidate.second);
                        map_entry_for_candidate = std::make_pair(new_origin_points_set, new_candidates_set);
                    }
                }
            }
            binned_candidates[truncated_x][truncated_y][truncated_theta] = map_entry_for_candidate;
        }

        uint64_t bins_over_threshold = 0;
        std::vector<std::pair<size_t, std::pair<int, std::pair<int, int>>>> size_with_bins;
        std::vector<pose::Pose2d> refined_candidates;
        for (const auto &outer_loop_map : binned_candidates) {
            for (const auto &inner_loop_map : outer_loop_map.second) {
                for (const auto &innermost_loop_map : inner_loop_map.second) {
                    std::unordered_set<size_t> points_repped_in_bin = innermost_loop_map.second.first;
                    std::vector<pose::Pose2d> entries_in_bin = innermost_loop_map.second.second;
                    size_with_bins.emplace_back(std::make_pair(points_repped_in_bin.size(),
                                                               std::make_pair(outer_loop_map.first,
                                                                              std::make_pair(inner_loop_map.first,
                                                                                             innermost_loop_map.first))));
                    double fraction_of_repped_points =
                            ((double) points_repped_in_bin.size()) / points_for_object.size();
                    if (fraction_of_repped_points > min_fraction_of_points_repped_in_bin) {
                        refined_candidates.insert(refined_candidates.end(), entries_in_bin.begin(),
                                                  entries_in_bin.end());
                        bins_over_threshold++;
                    }
                }
            }
        }

        size_t min_effective_bins = std::min(size_with_bins.size(), (size_t) minimum_bins);
        if (bins_over_threshold < min_effective_bins) {
            LOG(WARNING) << "Augmenting bins list because not enough met minimum represented bins ("
                         << min_effective_bins
                         << "). consider decreasing minimum bin threshold or switching to larger bins";

            std::partial_sort(size_with_bins.begin(), size_with_bins.begin() + min_effective_bins, size_with_bins.end(),
                              compareSizesOfBins);
            int added_bins = 0;
            for (size_t i = bins_over_threshold; i < min_effective_bins; i++) {
                LOG(INFO) << i << "th most common bin had pose rep percent "
                          << ((double) size_with_bins[i].first / points_for_object.size());
                std::pair<int, std::pair<int, int>> bin_indices = size_with_bins[i].second;
                std::vector<pose::Pose2d> entries_in_bin = binned_candidates[bin_indices.first][bin_indices.second.first][bin_indices.second.second].second;
                refined_candidates.insert(refined_candidates.end(), entries_in_bin.begin(),
                                          entries_in_bin.end());
                added_bins++;
            }
        }

        LOG(INFO) << "Generating samples from " << refined_candidates.size() << " samples";
        std::vector<pose::Pose2d> samples;
        int sample_range_end = refined_candidates.size() - 1;
        for (size_t i = 0; i < num_samples_to_generate; i++) {
            size_t sample_index = random_generator.RandomInt((int) 0, sample_range_end);
            samples.emplace_back(refined_candidates[sample_index]);
        }
        return samples;
    }
}

#endif //AUTODIFF_GP_RECTANGLE_SAMPLER_H
