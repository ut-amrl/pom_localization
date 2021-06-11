//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_PAST_SAMPLE_IO_H
#define AUTODIFF_GP_PAST_SAMPLE_IO_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <pose_optimization/pose_graph_generic.h>
#include <base_lib/pose_reps.h>

#include <unsupported/Eigen/MatrixFunctions>
#include "file_io_utils.h"

namespace file_io {

    struct PastSample2d {
        std::string semantic_class_;
        double transl_x_;
        double transl_y_;
        double theta_;

        /**
         * This is in the 0-1 range (exclusive -- don't want to get infinite values in the GP).
         */
        double value_;
    };


    void readPastSample2dLine(const std::string &line_in_file, PastSample2d &past_sample_data) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        bool first_entry = true;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (first_entry) {
                first_entry = false;
                past_sample_data.semantic_class_ = substr;
            } else {
                data.push_back(std::stod(substr));
            }
        }
        past_sample_data.transl_x_ = data[0];
        past_sample_data.transl_y_ = data[1];
        past_sample_data.theta_ = data[2];
        past_sample_data.value_ = data[3];
    }

    void readPastSample2dsFromFile(const std::string &file_name, std::vector<PastSample2d> &past_samples) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            PastSample2d past_sample;
            readPastSample2dLine(line, past_sample);
            past_samples.emplace_back(past_sample);
        }
    }

    void writePastSamplesHeaderToFile(std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({"semantic_class", "transl_x", "transl_y", "theta", "value"}, file_stream);
    }


    void writePastSample2dLineToFile(const PastSample2d &past_sample, std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({past_sample.semantic_class_, std::to_string(past_sample.transl_x_),
                                              std::to_string(past_sample.transl_y_), std::to_string(past_sample.theta_),
                                              std::to_string(past_sample.value_)}, file_stream);
    }

    void writePastSamples2dToFile(const std::string &file_name, const std::vector<PastSample2d> &past_samples) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writePastSamplesHeaderToFile(csv_file);
        for (const PastSample2d &past_sample : past_samples) {
            writePastSample2dLineToFile(past_sample, csv_file);
        }

        csv_file.close();
    }
}

#endif //AUTODIFF_GP_PAST_SAMPLE_IO_H
