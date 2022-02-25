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

    const size_t kMaxPointsForSample = 2000;

    template<typename T>
    T AngleModSymmetric(T angle) {
        angle -= T(M_PI) * rint(angle / T(M_PI));
        return angle;
    }

    bool compareSizesOfBins(std::pair<size_t, std::pair<int, std::pair<int, int>>> a,
                            std::pair<size_t, std::pair<int, std::pair<int, int>>> b) {
        return a.first > b.first;
    };

    std::pair<Eigen::Vector2d, double> samplePointsProportionalToPerim(
            const double &rectangle_width,
            const double &rectangle_height,
            util_random::Random &random_generator) {
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
        return std::make_pair(point_in_rect_frame, theta_modifier);
    }

    std::pair<Eigen::Vector2d, double> samplePointsProportionalToSide(
            const double &rectangle_width,
            const double &rectangle_height,
            util_random::Random &random_generator) {
        int rect_side = random_generator.RandomInt(0, 3);
        double point_on_side_rand = random_generator.UniformRandom();
        if (point_on_side_rand == 1) {
            point_on_side_rand = 0;
        }
        Eigen::Vector2d point_in_rect_frame;
        double theta_modifier;
        if (rect_side == 0) {
            point_in_rect_frame = Eigen::Vector2d((point_on_side_rand * rectangle_width) - (rectangle_width / 2),
                                                  -rectangle_height / 2);
            theta_modifier = 0;
        } else if (rect_side == 1) {
            point_in_rect_frame = Eigen::Vector2d(rectangle_width / 2,
                                                  (point_on_side_rand * rectangle_height) - (rectangle_height / 2));
            theta_modifier = -M_PI_2;
        } else if (rect_side == 2) {
            point_in_rect_frame = Eigen::Vector2d(
                    ((1 - point_on_side_rand) * rectangle_width) - (rectangle_width / 2),
                    rectangle_height / 2);
            theta_modifier = M_PI;
        } else {
            point_in_rect_frame = Eigen::Vector2d(-rectangle_width / 2,
                                                  ((1 - point_on_side_rand) * rectangle_height) -
                                                  (rectangle_height / 2));
            theta_modifier = M_PI_2;
        }
        if ((abs(point_in_rect_frame.x()) > (rectangle_width / 2)) ||
            (abs(point_in_rect_frame.y()) > (rectangle_height / 2))) {
            LOG(INFO) << "Bad point " << point_in_rect_frame;
            exit(1);
        }
        return std::make_pair(point_in_rect_frame, theta_modifier);

    }

    pose::Pose2d generateConsistentRectangleSample(const Eigen::Vector2d &point_on_rect,
                                                   const double &rectangle_width,
                                                   const double &rectangle_height,
                                                   const double &point_std_dev,
                                                   util_random::Random &random_generator,
                                                   const std::function<std::pair<Eigen::Vector2d, double>(
                                                           const double &,
                                                           const double &,
                                                           util_random::Random &)> &point_proposer = samplePointsProportionalToSide) {
        Eigen::Vector2d point_to_use = point_on_rect;
        if (point_std_dev > 0) {
            point_to_use.x() = random_generator.Gaussian(point_to_use.x(), point_std_dev);
            point_to_use.y() = random_generator.Gaussian(point_to_use.y(), point_std_dev);
        }
        double angle_to_point = atan2(point_to_use.y(), point_to_use.x());

        std::pair<Eigen::Vector2d, double> point_hit_info = point_proposer(rectangle_width,
                                                                           rectangle_height,
                                                                           random_generator);
        Eigen::Vector2d point_in_rect_frame = point_hit_info.first;
        double theta_modifier = point_hit_info.second;

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

    std::pair<Eigen::Vector2d, bool> generateLineSegMidpointOnRectPropLen(
            const double &line_seg_len,
            const double &rectangle_width,
            const double &rectangle_height,
            util_random::Random &random_generator) {

        double shorter_side_len = std::min(rectangle_width, rectangle_height);
        double rect_width_min_len = rectangle_width - line_seg_len;
        double rect_height_min_len = rectangle_height - line_seg_len;
        bool hit_x_dim_side = true;

        double p_x = rectangle_width / 2;
        double p_y = -rectangle_height / 2;
        if (line_seg_len > shorter_side_len) {
            if (rectangle_width > rectangle_height) {
                hit_x_dim_side = true;
                if (rect_width_min_len < 0) {
                    p_x = 0;
                } else {
                    p_x = random_generator.UniformRandom(-rect_width_min_len / 2, rect_width_min_len / 2);
                    if (p_x == rect_width_min_len / 2) {
                        p_x = -p_x;
                    }
                }
            } else {
                hit_x_dim_side = false;
                if (rect_height_min_len < 0) {
                    p_y = 0;
                } else {
                    p_y = random_generator.UniformRandom(-rect_height_min_len / 2, rect_height_min_len / 2);
                    if (p_y == rect_height_min_len / 2) {
                        p_y = -p_y;
                    }
                }
            }
        } else {
            double point_along_l = random_generator.UniformRandom(0, rect_width_min_len + rect_height_min_len);
            if (point_along_l == (rect_width_min_len - rect_height_min_len)) {
                point_along_l = 0;
            }
            if (point_along_l < rect_width_min_len) {
                p_x = point_along_l - (rect_width_min_len / 2);
                hit_x_dim_side = true;
            } else {
                p_y = (point_along_l - rect_width_min_len) - (rect_height_min_len / 2);
                hit_x_dim_side = false;
            }
        }

        Eigen::Vector2d midpoint_loc_on_rect(p_x, p_y);

        return std::make_pair(midpoint_loc_on_rect, hit_x_dim_side);
    }

    std::pair<Eigen::Vector2d, bool> generateLineSegMidpointOnRectPropSide(
            const double &line_seg_len,
            const double &rectangle_width,
            const double &rectangle_height,
            util_random::Random &random_generator) {

        bool hit_x_dim_side = true;
        double shorter_side_len = std::min(rectangle_width, rectangle_height);
        if (line_seg_len > shorter_side_len) {
            hit_x_dim_side = true;
            if (rectangle_height > rectangle_width) {
                hit_x_dim_side = false;
            }
        } else {
            int rect_side = random_generator.RandomInt(0, 1);
            hit_x_dim_side = rect_side == 0;
        }

        double using_side_len = (hit_x_dim_side) ? rectangle_width : rectangle_height;

        Eigen::Vector2d midpoint_loc_on_rect;

        double loc_on_side = 0;
        if (line_seg_len < using_side_len) {
            double remaining_side_len = using_side_len - line_seg_len;
            loc_on_side = random_generator.UniformRandom(-remaining_side_len / 2, remaining_side_len / 2);
        }
        if (hit_x_dim_side) {
            midpoint_loc_on_rect = Eigen::Vector2d(loc_on_side, -rectangle_height / 2);
        } else {
            midpoint_loc_on_rect = Eigen::Vector2d(rectangle_width / 2, loc_on_side);
        }

        return std::make_pair(midpoint_loc_on_rect, hit_x_dim_side);
    }

    pose::Pose2d generateConsistentSampleTwoPoint(const Eigen::Vector2d &point_on_rect_1,
                                                  const Eigen::Vector2d &point_on_rect_2,
                                                  const double &rectangle_width,
                                                  const double &rectangle_height,
                                                  const double &point_std_dev,
                                                  util_random::Random &random_generator,
                                                  const std::function<std::pair<Eigen::Vector2d, bool>(
                                                          const double &,
                                                          const double &,
                                                          const double &,
                                                          util_random::Random &)> point_proposer) {


        Eigen::Vector2d point_to_use_1 = point_on_rect_1;
        Eigen::Vector2d point_to_use_2 = point_on_rect_2;

        if (point_on_rect_1.norm() > point_on_rect_2.norm()) {
            point_to_use_1 = point_on_rect_2;
            point_to_use_2 = point_on_rect_1;
        }

        if (point_std_dev > 0) {
            point_to_use_1.x() = random_generator.Gaussian(point_to_use_1.x(), point_std_dev);
            point_to_use_1.y() = random_generator.Gaussian(point_to_use_1.y(), point_std_dev);
        }
        if (point_std_dev > 0) {
            point_to_use_2.x() = random_generator.Gaussian(point_to_use_2.x(), point_std_dev);
            point_to_use_2.y() = random_generator.Gaussian(point_to_use_2.y(), point_std_dev);
        }

        double theta_p1 = atan2(point_to_use_1.y(), point_to_use_1.x());
        Eigen::Vector2d rotated_p2 = Eigen::Rotation2Dd(-theta_p1) * point_to_use_2;

        Eigen::Vector2d midpoint = (point_to_use_1 + point_to_use_2) / 2;
        Eigen::Vector2d displacement = point_to_use_2 - point_to_use_1;
        double len = displacement.norm();
        double theta_rect = atan2(displacement.y(), displacement.x());

        std::pair<Eigen::Vector2d, bool> midpoint_on_rect_info = point_proposer(len, rectangle_width, rectangle_height,
                                                                                random_generator);

        Eigen::Vector2d midpoint_loc_on_rect = midpoint_on_rect_info.first;

        bool hit_on_x_side = midpoint_on_rect_info.second;
        if (rotated_p2.y() >= 0) {
            // If hit on width, hit must be on top
            if (hit_on_x_side) {
                midpoint_loc_on_rect = Eigen::Vector2d(-midpoint_loc_on_rect.x(), rectangle_height / 2);
            } else {
                // If hit on height, hit must be on left side
                midpoint_loc_on_rect = Eigen::Vector2d(-rectangle_width / 2, -midpoint_loc_on_rect.y());
            }
        }
//        else {
//            // If hit on width, hit must be on bottom (what is given by proposer)
//            // If hit on height, hit must be on right side (what is given by proposer)
//        }


        if (!hit_on_x_side) {
            theta_rect = math_util::AngleMod(theta_rect - M_PI_2);
        }

        double rect_center_x = midpoint.x() - (midpoint_loc_on_rect.x() * cos(theta_rect)) +
                               (midpoint_loc_on_rect.y() * sin(theta_rect));
        double rect_center_y = midpoint.y() - (midpoint_loc_on_rect.x() * sin(theta_rect)) -
                               (midpoint_loc_on_rect.y() * cos(theta_rect));

        return pose::createPose2d(rect_center_x, rect_center_y, theta_rect);
    }

    std::vector<pose::Pose2d> generateConsistentSamplesTwoPoint(const std::vector<Eigen::Vector2d> &full_points_for_object,
                                                                const size_t &num_samples_to_generate,
                                                                const uint64_t &samples_per_point,
                                                                const double &position_bin_size,
                                                                const double &orientation_bin_size,
                                                                const double &rect_width,
                                                                const double &rect_height,
                                                                const double &point_std_dev,
                                                                const double &min_fraction_of_points_repped_in_bin,
                                                                const uint16_t &minimum_bins,
                                                                util_random::Random &random_generator,
                                                                const std::function<std::pair<Eigen::Vector2d, bool>(
                                                                        const double &,
                                                                        const double &,
                                                                        const double &,
                                                                        util_random::Random &)> point_proposer = generateLineSegMidpointOnRectPropLen,
                                                                const bool &sample_prop_to_bin_count = false,
                                                                const bool &sample_prop_to_squared_bin_rep = false,
                                                                const size_t &max_points_to_use = kMaxPointsForSample) {
//        std::vector<std::pair<std::pair<size_t, size_t>, pose::Pose2d>> unfiltered_candidate_rects;
        std::vector<std::pair<size_t, pose::Pose2d>> unfiltered_candidate_rects;

        std::unordered_map<int, std::unordered_map<int,
                std::unordered_map<int, std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>>>>> binned_candidates;

        std::vector<Eigen::Vector2d> points_for_object;
        if (full_points_for_object.size() > max_points_to_use) {
            LOG(INFO) << "Max points exceeded " << full_points_for_object.size() << "vs " << max_points_to_use;
            std::sample(full_points_for_object.begin(), full_points_for_object.end(), std::back_inserter(points_for_object), max_points_to_use, std::default_random_engine());
        } else {
            points_for_object = full_points_for_object;
        }

        std::vector<std::vector<size_t>> neighbors;
        neighbors.resize(points_for_object.size());

        std::vector<std::pair<size_t, double>> closest_to_point_idx_and_norm;
        for (size_t p1_idx = 0; p1_idx < points_for_object.size(); p1_idx++) {
            size_t init_closest_idx = (p1_idx + 1) % points_for_object.size();
            double init_closest_norm = (points_for_object[p1_idx] - points_for_object[init_closest_idx]).norm();
            closest_to_point_idx_and_norm.emplace_back(std::make_pair(init_closest_idx, init_closest_norm));
        }
        for (size_t p1_idx = 0; p1_idx < points_for_object.size(); p1_idx++) {
            Eigen::Vector2d p1 = points_for_object[p1_idx];
            double closest_idx_p1_norm = closest_to_point_idx_and_norm[p1_idx].second;
            for (size_t p2_idx = p1_idx + 1; p2_idx < points_for_object.size(); p2_idx++) {
                double closest_idx_p2_norm = closest_to_point_idx_and_norm[p2_idx].second;
                double norm = (p1 - points_for_object[p2_idx]).norm();
                if ((p1 - points_for_object[p2_idx]).norm() <= 0.5) { // TODO make configurable
                    neighbors[p1_idx].emplace_back(p2_idx);
                    neighbors[p2_idx].emplace_back(p1_idx);
                }
                if (closest_idx_p1_norm > norm) {
                    closest_to_point_idx_and_norm[p1_idx] = std::make_pair(p2_idx, norm);
                }
                if (closest_idx_p2_norm > norm) {
                    closest_to_point_idx_and_norm[p2_idx] = std::make_pair(p1_idx, norm);
                }
            }
        }
        for (size_t p1_idx = 0; p1_idx < points_for_object.size(); p1_idx++) {
            if (neighbors[p1_idx].empty()) {
                LOG(INFO) << "Initially no neighbors for p1. Using closest point as only neighbor";
                neighbors[p1_idx].emplace_back(closest_to_point_idx_and_norm[p1_idx].first);
            }
        }

        LOG(INFO) << "Generating samples from " << points_for_object.size() << " points";
        for (size_t i = 0; i < points_for_object.size(); i++) {
            Eigen::Vector2d object_point = points_for_object[i];
            std::vector<size_t> neighbor_indices_for_point = neighbors[i];
            if (neighbor_indices_for_point.empty()) {
                LOG(INFO) << "No neighbors for point with idx " << i;
                continue;
            }
            for (uint64_t j = 0; j < samples_per_point; j++) {
                uint64_t other_point_idx = neighbor_indices_for_point[random_generator.RandomInt(0,
                                                                                                 (int) neighbor_indices_for_point.size() -
                                                                                                 1)];
                Eigen::Vector2d other_point = points_for_object[other_point_idx];
                unfiltered_candidate_rects.emplace_back(
//                        std::make_pair(std::make_pair(i, other_point_idx),
                        std::make_pair(i,
                                       generateConsistentSampleTwoPoint(object_point, other_point, rect_width,
                                                                        rect_height,
                                                                        point_std_dev, random_generator,
                                                                        point_proposer)));
            }
        }

        LOG(INFO) << "Binning samples";
//        for (const std::pair<std::pair<size_t, size_t>, pose::Pose2d> &candidate : unfiltered_candidate_rects) {
        for (const std::pair<size_t, pose::Pose2d> &candidate : unfiltered_candidate_rects) {
            int truncated_x = candidate.second.first.x() / position_bin_size;
            int truncated_y = candidate.second.first.y() / position_bin_size;
            // We should bin so that things that are 180 degrees from each other are in the same bin
            int truncated_theta = AngleModSymmetric(candidate.second.second) / orientation_bin_size;

            std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>> map_entry_for_candidate = std::make_pair(
//                    (std::unordered_set<size_t>) {candidate.first.first, candidate.first.second},
                    (std::unordered_set<size_t>) {candidate.first},
                    (std::vector<pose::Pose2d>) {candidate.second});
            if (binned_candidates.find(truncated_x) != binned_candidates.end()) {
                if (binned_candidates[truncated_x].find(truncated_y) != binned_candidates[truncated_x].end()) {
                    if (binned_candidates[truncated_x][truncated_y].find(truncated_theta) !=
                        binned_candidates[truncated_x][truncated_y].end()) {
                        std::pair<std::unordered_set<size_t>, std::vector<pose::Pose2d>> old_entry_for_candidate = binned_candidates[truncated_x][truncated_y][truncated_theta];
                        std::unordered_set<size_t> new_origin_points_set = old_entry_for_candidate.first;
//                        new_origin_points_set.insert(candidate.first.first);
//                        new_origin_points_set.insert(candidate.first.second);
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
        std::vector<double> fraction_of_repped_points_incl;
        std::vector<std::pair<int, std::pair<int, int>>> bin_count_sample_vec;
        std::vector<std::pair<int, std::pair<int, int>>> retained_bins;
        std::vector<double> squared_ratio_for_retained_bins;
        for (const auto &outer_loop_map : binned_candidates) {
            for (const auto &inner_loop_map : outer_loop_map.second) {
                for (const auto &innermost_loop_map : inner_loop_map.second) {
                    std::unordered_set<size_t> points_repped_in_bin = innermost_loop_map.second.first;
                    std::vector<pose::Pose2d> entries_in_bin = innermost_loop_map.second.second;
                    std::pair<int, std::pair<int, int>> bin_key = std::make_pair(outer_loop_map.first,
                                                                                 std::make_pair(inner_loop_map.first,
                                                                                                innermost_loop_map.first));
                    size_with_bins.emplace_back(std::make_pair(points_repped_in_bin.size(), bin_key));
                    double fraction_of_repped_points =
                            ((double) points_repped_in_bin.size()) / points_for_object.size();
                    if (fraction_of_repped_points > min_fraction_of_points_repped_in_bin) {
                        refined_candidates.insert(refined_candidates.end(), entries_in_bin.begin(),
                                                  entries_in_bin.end());
                        fraction_of_repped_points_incl.emplace_back(fraction_of_repped_points);
                        retained_bins.emplace_back(bin_key);
                        squared_ratio_for_retained_bins.emplace_back(pow(fraction_of_repped_points, 2));
                        for (size_t bin_count_num = 0; bin_count_num < points_repped_in_bin.size(); bin_count_num++) {
                            bin_count_sample_vec.emplace_back(bin_key);
                        }
                        bins_over_threshold++;
                    }
                }
            }
        }
        LOG(INFO) << "Bins over threshold " << bins_over_threshold;
        size_t min_effective_bins = std::min(size_with_bins.size(), (size_t) minimum_bins);
        if (bins_over_threshold < min_effective_bins) {
            LOG(WARNING) << "Augmenting bins list because not enough met minimum represented bins ("
                         << min_effective_bins
                         << "). consider decreasing minimum bin threshold or switching to larger bins";

            std::partial_sort(size_with_bins.begin(), size_with_bins.begin() + min_effective_bins, size_with_bins.end(),
                              compareSizesOfBins);
            int added_bins = 0;
            for (size_t i = bins_over_threshold; i < min_effective_bins; i++) {
                double fraction_of_repped_points = ((double) size_with_bins[i].first / points_for_object.size());
                LOG(INFO) << i << "th most common bin had pose rep percent "
                          << fraction_of_repped_points;
                std::pair<int, std::pair<int, int>> bin_indices = size_with_bins[i].second;
                std::vector<pose::Pose2d> entries_in_bin = binned_candidates[bin_indices.first][bin_indices.second.first][bin_indices.second.second].second;
                refined_candidates.insert(refined_candidates.end(), entries_in_bin.begin(),
                                          entries_in_bin.end());
                retained_bins.emplace_back(bin_indices);
                squared_ratio_for_retained_bins.emplace_back(pow(fraction_of_repped_points, 2));
                for (size_t bin_count_num = 0; bin_count_num < size_with_bins[i].first; bin_count_num++) {
                    bin_count_sample_vec.emplace_back(bin_indices);
                }
                added_bins++;
            }
        }


        std::ostringstream oss;
        std::copy(fraction_of_repped_points_incl.begin(), fraction_of_repped_points_incl.end(),
                  std::ostream_iterator<double>(oss, ", "));
        LOG(INFO) << "Retained bin fractions " << oss.str();

        LOG(INFO) << "Generating samples from " << refined_candidates.size() << " samples";
        std::vector<pose::Pose2d> samples;
        if (sample_prop_to_bin_count) {
            for (size_t i = 0; i < num_samples_to_generate; i++) {
                size_t bin_sample_idx = random_generator.RandomInt(0, (int) bin_count_sample_vec.size() - 1);
                std::pair<int, std::pair<int, int>> bin_indices = bin_count_sample_vec[bin_sample_idx];
                std::vector<pose::Pose2d> entries_in_bin = binned_candidates[bin_indices.first][bin_indices.second.first][bin_indices.second.second].second;
                size_t sample_in_bin_idx = random_generator.RandomInt(0, (int) entries_in_bin.size() - 1);
                samples.emplace_back(entries_in_bin[sample_in_bin_idx]);
            }
        } else if (sample_prop_to_squared_bin_rep) {
            LOG(INFO) << "Prop squared bin rep";
            double prev_bin_weight_threshold = 0;
            std::vector<double> bin_weight_threshold;
            for (size_t bin_num = 0; bin_num < retained_bins.size(); bin_num++) {
                prev_bin_weight_threshold = squared_ratio_for_retained_bins[bin_num] + prev_bin_weight_threshold;
                bin_weight_threshold.emplace_back(prev_bin_weight_threshold);
            }
            double rand_num_inc = prev_bin_weight_threshold / num_samples_to_generate;
            double rand_num_for_bin_select = random_generator.UniformRandom(0, prev_bin_weight_threshold);
            size_t bin_search_idx = 0;
            for (size_t i = 0; i < num_samples_to_generate; i++) {
                while (rand_num_for_bin_select > bin_weight_threshold[bin_search_idx]) {
                    bin_search_idx++;
                }
                std::pair<int, std::pair<int, int>> bin_indices = retained_bins[bin_search_idx];
                std::vector<pose::Pose2d> entries_in_bin = binned_candidates[bin_indices.first][bin_indices.second.first][bin_indices.second.second].second;
                size_t sample_in_bin_idx = random_generator.RandomInt(0, (int) entries_in_bin.size() - 1);
                samples.emplace_back(entries_in_bin[sample_in_bin_idx]);
                rand_num_for_bin_select += rand_num_inc;
                if (rand_num_for_bin_select > prev_bin_weight_threshold) {
                    bin_search_idx = 0;
                    rand_num_for_bin_select -= prev_bin_weight_threshold;
                }
            }

        } else {
            int sample_range_end = refined_candidates.size() - 1;
            for (size_t i = 0; i < num_samples_to_generate; i++) {
                size_t sample_index = random_generator.RandomInt((int) 0, sample_range_end);
                samples.emplace_back(refined_candidates[sample_index]);
            }
        }

        return samples;
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
                                                        util_random::Random &random_generator,
                                                        const std::function<std::pair<Eigen::Vector2d, double>(
                                                                const double &,
                                                                const double &,
                                                                util_random::Random &)> &point_proposer = samplePointsProportionalToSide) {
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
        std::vector<double> fraction_of_repped_points_incl;
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
                        fraction_of_repped_points_incl.emplace_back(fraction_of_repped_points);
                        bins_over_threshold++;
                    }
                }
            }
        }
        LOG(INFO) << "Bins over threshold " << bins_over_threshold;
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
        std::ostringstream oss;
        std::copy(fraction_of_repped_points_incl.begin(), fraction_of_repped_points_incl.end(),
                  std::ostream_iterator<double>(oss, ", "));
        LOG(INFO) << "Retained bin fractions " << oss.str();

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
