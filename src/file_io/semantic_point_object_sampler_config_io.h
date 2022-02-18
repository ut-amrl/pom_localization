//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_SEMANTIC_POINT_OBJECT_SAMPLER_CONFIG_IO_H
#define AUTODIFF_GP_SEMANTIC_POINT_OBJECT_SAMPLER_CONFIG_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace file_io {

    struct SemanticPointObjectSamplerConfig {
        uint64_t samples_per_point;
        double position_bin_size;
        double orientation_bin_size;
        double point_std_dev;
        double min_points_repped_in_bin;
        uint16_t minimum_bins;
    };

    void readSemanticPointObjectSamplerConfigFromLine(const std::string &line_in_file,
                                                      SemanticPointObjectSamplerConfig &config) {
        std::stringstream ss(line_in_file);

        std::string substr;
        getline(ss, substr, ',');
        std::istringstream samples_per_point_stream(substr);
        samples_per_point_stream >> config.samples_per_point;

        getline(ss, substr, ',');
        config.position_bin_size = std::stod(substr);

        getline(ss, substr, ',');
        config.orientation_bin_size = std::stod(substr);

        getline(ss, substr, ',');
        config.point_std_dev = std::stod(substr);

        getline(ss, substr, ',');
        config.min_points_repped_in_bin = std::stod(substr);

        getline(ss, substr, ',');
        std::istringstream min_bins_stream(substr);
        min_bins_stream >> config.minimum_bins;
    }

    void readSemanticPointObjectSamplerConfigFromFile(const std::string &file_name,
                                                      SemanticPointObjectSamplerConfig &config) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            // There should only be one line after the header
            readSemanticPointObjectSamplerConfigFromLine(line, config);
        }
        if (first_line) {
            LOG(ERROR) << "Empty config file " << file_name;
            exit(1);
        }
    }

    void writeSemanticPointObjectSamplerConfigToFile(const std::string &file_name,
                                                     const SemanticPointObjectSamplerConfig &config) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeCommaSeparatedStringsLineToFile(
                {"samples_per_point", "position_bin_size", "orientation_bin_size", "point_std_dev", "min_points_repped_in_bin", "minimum_bins"},
                csv_file);
        writeCommaSeparatedStringsLineToFile({
                                                     std::to_string(config.samples_per_point),
                                                     std::to_string(config.position_bin_size),
                                                     std::to_string(config.orientation_bin_size),
                                                     std::to_string(config.point_std_dev),
                                                     std::to_string(config.min_points_repped_in_bin),
                                                     std::to_string(config.minimum_bins)}, csv_file);
        csv_file.close();
    }
}

#endif //AUTODIFF_GP_SEMANTIC_POINT_OBJECT_SAMPLER_CONFIG_IO_H
