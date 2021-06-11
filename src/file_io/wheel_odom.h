//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_WHEEL_ODOM_H
#define AUTODIFF_GP_WHEEL_ODOM_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <pose_optimization/pose_graph_generic.h>
#include <base_lib/pose_reps.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace file_io {

    struct WheelOdom {
        uint32_t sec_;
        uint32_t nsec_;
        double transl_x_;
        double transl_y_;
        double theta_;
    };

    void readRawWheelOdomLine(const std::string &line_in_file, WheelOdom &wheel_odom) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        int loop_counter = 0;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (loop_counter < 2) {
                uint32_t val; // TODO
                if (loop_counter == 0) {
                    wheel_odom.sec_ = val;
                } else {
                    wheel_odom.nsec_ = val;
                }
            } else {
                data.push_back(std::stod(substr));
            }
            loop_counter++;
        }

        wheel_odom.transl_x_ = data[0];
        wheel_odom.transl_y_ = data[1];
        wheel_odom.theta_ = data[2];
    }

    void readRawWheelOdomFromFile(const std::string &file_name, std::vector<WheelOdom> &odom_ests_in_abs_frame) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            WheelOdom wheel_odom;
            readRawWheelOdomLine(line, wheel_odom);
            odom_ests_in_abs_frame.emplace_back(wheel_odom);
        }
    }
}

#endif //AUTODIFF_GP_WHEEL_ODOM_H
