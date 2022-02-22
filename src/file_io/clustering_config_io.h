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
    };

    void readClusteringConfigFromLine(const std::string &line_in_file,
                                      ClusteringConfig &config) {
        std::stringstream ss(line_in_file);

        std::string substr;
        getline(ss, substr, ',');
        config.ground_plane_height = std::stod(substr);

        getline(ss, substr, ',');
        config.ground_plane_band_thickness = std::stod(substr);


        getline(ss, substr, ',');
        config.cluster_tolerance = std::stod(substr);

        getline(ss, substr, ',');
        config.min_cluster_size = std::stoi(substr);

        getline(ss, substr, ',');
        config.max_cluster_size = std::stoi(substr);

        getline(ss, substr, ',');
        config.max_cluster_span = std::stod(substr);
        getline(ss, substr, ',');
        config.min_horiz_span = std::stod(substr);
        getline(ss, substr, ',');
        config.max_z = std::stod(substr);
        getline(ss, substr, ',');
        config.min_max_z = std::stod(substr);
        getline(ss, substr, ',');
        config.z_merge_proximity = std::stod(substr);
        getline(ss, substr, ',');
        config.max_height_over_horiz_span = std::stod(substr);

        getline(ss, substr, ',');
        config.time_between_frames = std::stod(substr);
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
                        std::to_string(config.time_between_frames)
                }, csv_file);
        csv_file.close();
    }
}

#endif //AUTODIFF_GP_CLUSTERING_CONFIG_IO_H
