//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_RUNTIME_PARAMS_CONFIG_IO_H
#define AUTODIFF_GP_RUNTIME_PARAMS_CONFIG_IO_H

#include <cstdint>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace file_io {

    struct RuntimeParamsConfig {

        uint64_t max_gpc_samples_;

        uint64_t num_nodes_in_optimization_window_;
        uint64_t full_optimization_interval_;

        uint64_t pose_sample_ratio_;

        double mean_position_kernel_len_;
        double mean_orientation_kernel_len_;

        double mean_position_kernel_var_;
        double mean_orientation_kernel_var_;

        double default_obj_probability_input_variance_for_mean_;

        double var_position_kernel_len_;
        double var_orientation_kernel_len_;

        double var_position_kernel_var_;
        double var_orientation_kernel_var_;

        double default_obj_probability_input_variance_for_var_;

        double detection_variance_transl_x_;
        double detection_variance_transl_y_;
        double detection_variance_theta_;

        double odom_k1_;
        double odom_k2_;
        double odom_k3_;
        double odom_k4_;
        double odom_k5_;
        double odom_k6_;

    };

    void readRuntimeParamsConfigFromLine(const std::string &line_in_file,
                                         RuntimeParamsConfig &config) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        int loop_count = 0;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (loop_count == 0) {
                std::istringstream stream(substr);
                stream >> config.max_gpc_samples_;
            } else if (loop_count == 1) {
                std::istringstream stream(substr);
                stream >> config.num_nodes_in_optimization_window_;
            } else if (loop_count == 2) {
                std::istringstream stream(substr);
                stream >> config.full_optimization_interval_;
            } else if (loop_count == 3) {
                std::istringstream stream(substr);
                stream >> config.pose_sample_ratio_;
            } else {
                data.push_back(std::stod(substr));
            }
            loop_count++;
        }

        config.mean_position_kernel_len_ = data[0];
        config.mean_orientation_kernel_len_ = data[1];

        config.mean_position_kernel_var_ = data[2];
        config.mean_orientation_kernel_var_ = data[3];

        config.default_obj_probability_input_variance_for_mean_ = data[4];

        config.var_position_kernel_len_ = data[5];
        config.var_orientation_kernel_len_ = data[6];

        config.var_position_kernel_var_ = data[7];
        config.var_orientation_kernel_var_ = data[8];

        config.default_obj_probability_input_variance_for_var_ = data[9];

        config.detection_variance_transl_x_ = data[10];
        config.detection_variance_transl_y_ = data[11];
        config.detection_variance_theta_ = data[12];

        config.odom_k1_ = data[13];
        config.odom_k2_ = data[14];
        config.odom_k3_ = data[15];
        config.odom_k4_ = data[16];
        config.odom_k5_ = data[17];
        config.odom_k6_ = data[18];
    }

    void readRuntimeParamsConfigFromFile(const std::string &file_name,
                                         RuntimeParamsConfig &config) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            // There should only be one line after the header
            readRuntimeParamsConfigFromLine(line, config);
        }
        if (first_line) {
            LOG(ERROR) << "Config file was empty or non existent";
            exit(1);
        }
    }

    void writeRuntimeParamsConfigToFile(const std::string &file_name,
                                        const RuntimeParamsConfig &config) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeCommaSeparatedStringsLineToFile(
                {

                        "max_gpc_samples", "num_nodes_in_optimization_window", "full_optimization_interval",
                        "pose_sample_ratio",
                        "mean_position_kernel_len", "mean_orientation_kernel_len", "mean_position_kernel_var",
                        "mean_orientation_kernel_var", "default_obj_probability_input_variance_for_mean",
                        "var_position_kernel_len", "var_orientation_kernel_len", "var_position_kernel_var",
                        "var_orientation_kernel_var", "default_obj_probability_input_variance_for_var",
                        "detection_variance_transl_x", "detection_variance_transl_y", "detection_variance_theta",
                        "odom_k1", "odom_k2", "odom_k3", "odom_k4", "odom_k5", "odom_k6"}, csv_file);
        writeCommaSeparatedStringsLineToFile(
                {
                        std::to_string(config.max_gpc_samples_),
                        std::to_string(config.num_nodes_in_optimization_window_),
                        std::to_string(config.full_optimization_interval_),
                        std::to_string(config.pose_sample_ratio_),
                        std::to_string(config.mean_position_kernel_len_),
                        std::to_string(config.mean_orientation_kernel_len_),
                        std::to_string(config.mean_position_kernel_var_),
                        std::to_string(config.mean_orientation_kernel_var_),
                        std::to_string(config.default_obj_probability_input_variance_for_mean_),
                        std::to_string(config.var_position_kernel_len_),
                        std::to_string(config.var_orientation_kernel_len_),
                        std::to_string(config.var_position_kernel_var_),
                        std::to_string(config.var_orientation_kernel_var_),
                        std::to_string(config.default_obj_probability_input_variance_for_var_),
                        std::to_string(config.detection_variance_transl_x_),
                        std::to_string(config.detection_variance_transl_y_),
                        std::to_string(config.detection_variance_theta_), std::to_string(config.odom_k1_),
                        std::to_string(config.odom_k2_), std::to_string(config.odom_k3_),
                        std::to_string(config.odom_k4_), std::to_string(config.odom_k5_),
                        std::to_string(config.odom_k6_)}, csv_file);
        csv_file.close();
    }
}

#endif //AUTODIFF_GP_RUNTIME_PARAMS_CONFIG_IO_H
