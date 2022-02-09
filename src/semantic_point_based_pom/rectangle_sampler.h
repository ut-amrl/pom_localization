//
// Created by amanda on 2/4/22.
//

#ifndef AUTODIFF_GP_RECTANGLE_SAMPLER_H
#define AUTODIFF_GP_RECTANGLE_SAMPLER_H

#include <base_lib/pose_reps.h>
#include <shared/util/random.h>
#include <shared/math/math_util.h>

namespace semantic_point_pom {

    pose::Pose2d generateConsistentRectangleSample(const Eigen::Vector2d &point_on_rect,
                                                   const double &rectangle_width,
                                                   const double &rectangle_height,
                                                   const double &point_std_dev,
                                                   util_random::Random &random_generator) {
        Eigen::Vector2d point_to_use = point_on_rect;
        if (point_std_dev > 0) {
            point_to_use.x() = random_generator.Gaussian(point_to_use.x(), point_std_dev);
            point_to_use.y() = random_generator.Gaussian(point_to_use.x(), point_std_dev);
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
            point_in_rect_frame = Eigen::Vector2d(point_on_surface_rand, 0);
            theta_modifier = -M_PI;
        } else if (point_on_surface_rand < (rectangle_height + rectangle_width)) {
            point_in_rect_frame = Eigen::Vector2d(rectangle_width, point_on_surface_rand - rectangle_width);
            theta_modifier = M_PI_2;
        } else if (point_on_surface_rand < ((2 * rectangle_width) + rectangle_height)) {
            point_in_rect_frame = Eigen::Vector2d(((2 * rectangle_width) + rectangle_height) - point_on_surface_rand,
                                                  rectangle_height);
            theta_modifier = 0;
        } else {
            point_in_rect_frame = Eigen::Vector2d(0, rect_perim - point_on_surface_rand);
            theta_modifier = -M_PI_2;
        }

        double rand_angle = random_generator.UniformRandom(0, M_PI);
        if (rand_angle >= M_PI) {
            rand_angle = 0;
        }

        double rectangle_angle = math_util::AngleMod(rand_angle + angle_to_point + theta_modifier);

        double rect_center_x = point_to_use.x() - (point_in_rect_frame.x() * cos(rectangle_angle)) +
                               (point_in_rect_frame.y() * sin(rectangle_angle));
        double rect_center_y = point_to_use.y() - (point_in_rect_frame.x() * sin(rectangle_angle)) -
                               (point_in_rect_frame.y() * cos(rectangle_angle));

        return pose::createPose2d(rect_center_x, rect_center_y, rectangle_angle);
    }

    std::vector<pose::Pose2d> generateConsistentSamples(const std::vector<Eigen::Vector2d> &points_for_object,
                                                        const size_t &num_samples_to_generate,
                                                        const int &samples_per_point,
                                                        const double &position_bin_size,
                                                        const double &orientation_bin_size,
                                                        const double &rect_width,
                                                        const double &rect_height,
                                                        const double &point_std_dev,
                                                        const size_t &min_bin_count,
                                                        util_random::Random &random_generator) {

        std::vector<std::pair<size_t, pose::Pose2d>> unfiltered_candidate_rects;

        std::unordered_map<int, std::unordered_map<int,
        std::unordered_map<int, std::vector<std::pair<size_t, pose::Pose2d>>>>> binned_candidates;

        for (size_t i = 0; i < points_for_object.size(); i++) {
            Eigen::Vector2d object_point = points_for_object[i];
            for (int j = 0; j < samples_per_point; j++) {
                unfiltered_candidate_rects.emplace_back(
                        std::make_pair(i, generateConsistentRectangleSample(object_point, rect_width, rect_height,
                                                                            point_std_dev, random_generator)));
            }
        }

        for (const std::pair<size_t, pose::Pose2d> &candidate : unfiltered_candidate_rects) {
            int truncated_x = candidate.second.first.x() / position_bin_size;
            int truncated_y = candidate.second.first.y() / position_bin_size;
            int truncated_theta = candidate.second.second / orientation_bin_size;

            binned_candidates[truncated_x][truncated_y][truncated_theta].emplace_back(candidate);
        }

        std::vector<std::pair<size_t, pose::Pose2d>> refined_candidates;
        for (const auto &outer_loop_map : binned_candidates) {
            for (const auto &inner_loop_map : outer_loop_map.second) {
                for (const auto &innermost_loop_map : inner_loop_map.second) {
                    std::vector<std::pair<size_t, pose::Pose2d>> bin_vec;
                    if (bin_vec.size() >= min_bin_count) {
                        refined_candidates.insert(refined_candidates.end(), bin_vec.begin(), bin_vec.end());
                    }
                }
            }
        }

        std::vector<pose::Pose2d> samples;
        for (size_t i = 0; i < num_samples_to_generate; i++) {
            size_t sample_index = random_generator.RandomInt((int) 0, (int) refined_candidates.size());
            samples.emplace_back(refined_candidates[sample_index].second);
        }
        return samples;
    }
}

#endif //AUTODIFF_GP_RECTANGLE_SAMPLER_H
