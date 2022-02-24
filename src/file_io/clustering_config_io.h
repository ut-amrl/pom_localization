//
// Created by amanda on 2/21/22.
//

#ifndef AUTODIFF_GP_CLUSTERING_CONFIG_IO_H
#define AUTODIFF_GP_CLUSTERING_CONFIG_IO_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <glog/logging.h>

namespace file_io {

    struct ClusteringConfig {
        double ground_plane_height;
        double ground_plane_band_thickness;
        double cluster_tolerance; // meters
        int min_cluster_size;
        int max_cluster_size;

        double max_cluster_span;
        double min_horiz_span;
        double max_z;
        double min_max_z;
        double z_merge_proximity;
        double max_height_over_horiz_span;

        double time_between_frames;

        double singular_value_threshold = 10;// Tentative -- gave it matrix that was 4 long and 1 wide and got ~10 to 1 singular values
        double ransac_line_distance_threshold = std::numeric_limits<double>::max(); // Make default high so it's effectively off
        int ransac_max_iterations = 1; // Make default minimal so it doesn't take up time when we don't intend to run it
        double min_rect_fit_inlier_percent_threshold = 0; // 0 is effectively off
        double perp_line_angle_epsilon = M_PI_2; // Effectively off
        double max_long_side_len = std::numeric_limits<double>::max(); // Effectively off

        double max_min_z = std::numeric_limits<double>::max(); // Effectively off
        double more_conservative_ground_plane_radius = std::numeric_limits<double>::max();
        double more_conservative_ground_plane_thickness = 0;

        double min_z_span = 0; // Effectively off by default

        double dedupe_point_for_line_fit_threshold = 0;
        double inlier_threshold_after_fit = std::numeric_limits<double>::max(); // Effectively off
        double min_avg_deviation_from_straight_line = std::numeric_limits<double>::max();

    };

    void readClusteringConfigFromLine(const std::string &line_in_file,
                                      ClusteringConfig &config) {
        std::vector<std::string> substrs = parseCommaSeparatedStrings(line_in_file);

        size_t next_substr = 0;
        config.ground_plane_height = std::stod(substrs[next_substr++]);
        config.ground_plane_band_thickness = std::stod(substrs[next_substr++]);
        config.cluster_tolerance = std::stod(substrs[next_substr++]);
        config.min_cluster_size = std::stoi(substrs[next_substr++]);
        config.max_cluster_size = std::stoi(substrs[next_substr++]);

        config.max_cluster_span = std::stod(substrs[next_substr++]);
        config.min_horiz_span = std::stod(substrs[next_substr++]);
        config.max_z = std::stod(substrs[next_substr++]);
        config.min_max_z = std::stod(substrs[next_substr++]);
        config.z_merge_proximity = std::stod(substrs[next_substr++]);

        config.max_height_over_horiz_span = std::stod(substrs[next_substr++]);
        config.time_between_frames = std::stod(substrs[next_substr++]);

        if (next_substr < substrs.size()) {
            config.singular_value_threshold = std::stod(substrs[next_substr++]);
            config.ransac_line_distance_threshold = std::stod(substrs[next_substr++]);
            config.ransac_max_iterations = std::stoi(substrs[next_substr++]);

            config.min_rect_fit_inlier_percent_threshold = std::stod(substrs[next_substr++]);
            config.perp_line_angle_epsilon = std::stod(substrs[next_substr++]);
            config.max_long_side_len = std::stod(substrs[next_substr++]);
            config.max_min_z = std::stod(substrs[next_substr++]);
            config.more_conservative_ground_plane_radius = std::stod(substrs[next_substr++]);

            config.more_conservative_ground_plane_thickness = std::stod(substrs[next_substr++]);
            config.min_z_span = std::stod(substrs[next_substr++]);
            config.dedupe_point_for_line_fit_threshold = std::stod(substrs[next_substr++]);
            config.inlier_threshold_after_fit = std::stod(substrs[next_substr++]);
            config.min_avg_deviation_from_straight_line = std::stod(substrs[next_substr++]);
        }
    }

    void readClusteringConfigFromFile(const std::string &file_name,
                                      ClusteringConfig &config) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            // There should only be one line after the header
            readClusteringConfigFromLine(line, config);
        }
        if (first_line) {
            LOG(ERROR) << "Empty config file " << file_name;
            exit(1);
        }
    }

    void writeClusteringConfigToFile(const std::string &file_name,
                                     const ClusteringConfig &config) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeCommaSeparatedStringsLineToFile({
                                                     "ground_plane_height",
                                                     "ground_plane_band_thickness",
                                                     "cluster_tolerance",
                                                     "min_cluster_size",
                                                     "max_cluster_size",
                                                     "max_cluster_span",
                                                     "min_horiz_span",
                                                     "max_z",
                                                     "min_max_z",
                                                     "z_merge_proximity",
                                                     "max_height_over_horiz_span",
                                                     "time_between_frames",
                                                     "singular_value_threshold",
                                                     "ransac_line_distance_threshold",
                                                     "ransac_max_iterations",
                                                     "min_rect_fit_inlier_percent_threshold",
                                                     "perp_line_angle_epsilon",
                                                     "max_long_side_len",
                                                     "max_min_z",
                                                     "more_conservative_ground_plane_radius",
                                                     "more_conservative_ground_plane_thickness",
                                                     "min_z_span",
                                                     "dedupe_point_for_line_fit_threshold",
                                                     "inlier_threshold_after_fit",
                                                     "min_avg_deviation_from_straight_line",
                                             }, csv_file);
        writeCommaSeparatedStringsLineToFile(
                {
                        std::to_string(config.ground_plane_height),
                        std::to_string(config.ground_plane_band_thickness),
                        std::to_string(config.cluster_tolerance),
                        std::to_string(config.min_cluster_size),
                        std::to_string(config.max_cluster_size),
                        std::to_string(config.max_cluster_span),
                        std::to_string(config.min_horiz_span),
                        std::to_string(config.max_z),
                        std::to_string(config.min_max_z),
                        std::to_string(config.z_merge_proximity),
                        std::to_string(config.max_height_over_horiz_span),
                        std::to_string(config.time_between_frames),
                        std::to_string(config.singular_value_threshold),
                        std::to_string(config.ransac_line_distance_threshold),
                        std::to_string(config.ransac_max_iterations),
                        std::to_string(config.min_rect_fit_inlier_percent_threshold),
                        std::to_string(config.perp_line_angle_epsilon),
                        std::to_string(config.max_long_side_len),
                        std::to_string(config.max_min_z),
                        std::to_string(config.more_conservative_ground_plane_radius),
                        std::to_string(config.more_conservative_ground_plane_thickness),
                        std::to_string(config.min_z_span),
                        std::to_string(config.dedupe_point_for_line_fit_threshold),
                        std::to_string(config.inlier_threshold_after_fit),
                        std::to_string(config.min_avg_deviation_from_straight_line)
                }, csv_file);
        csv_file.close();
    }

}

#endif //AUTODIFF_GP_CLUSTERING_CONFIG_IO_H
