//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_SHAPE_DIMENSIONS_BY_SEMANTIC_CLASS_2D_IO_H
#define AUTODIFF_GP_SHAPE_DIMENSIONS_BY_SEMANTIC_CLASS_2D_IO_H

#include <stdint.h>
#include <fstream>
#include <unordered_map>
#include <sstream>
#include <file_io/file_io_utils.h>
#include <eigen3/Eigen/Dense>

namespace file_io {

    void writeShapeDimensions2dBySemanticClassMapHeaderToFile(std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({"semantic_class", "rect_dim_x", "rect_dim_y"}, file_stream);
    }


    void writeShapeDimensions2dBySemanticClassEntryLineToFile(
            const std::pair<std::string, Eigen::Vector2d> &semantic_class_with_shape_dimensions,
            std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({semantic_class_with_shape_dimensions.first,
                                              std::to_string(semantic_class_with_shape_dimensions.second.x()),
                                              std::to_string(semantic_class_with_shape_dimensions.second.y())},
                                             file_stream);
    }

    void writeShapeDimensions2dBySemanticClassMapToFile(const std::string &file_name,
                                                        const std::unordered_map<std::string, Eigen::Vector2d>
                                                        &shape_dimensions_2d_by_semantic_class) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeShapeDimensions2dBySemanticClassMapHeaderToFile(csv_file);
        for (const auto &shape_dimension_with_semantic_class : shape_dimensions_2d_by_semantic_class) {
            writeShapeDimensions2dBySemanticClassEntryLineToFile(shape_dimension_with_semantic_class, csv_file);
        }

        csv_file.close();
    }

    void readShapeDimensions2dBySemanticClassEntryLine(const std::string &line_in_file,
                                                       std::pair<std::string, Eigen::Vector2d>
                                                       &semantic_class_with_shape_dimensions_2d) {
        std::stringstream ss(line_in_file);
        std::string substr;

        std::string semantic_string;
        getline(ss, semantic_string, ',');

        getline(ss, substr, ',');
        double x = std::stod(substr);

        getline(ss, substr, ',');
        double y = std::stod(substr);

        semantic_class_with_shape_dimensions_2d = std::make_pair(semantic_string, Eigen::Vector2d(x, y));
    }

    void readShapeDimensions2dBySemanticClassMapFromFile(const std::string &file_name,
                                                         std::unordered_map<std::string, Eigen::Vector2d>
                                                                 &shape_dimensions_2d_by_semantic_class) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            std::pair<std::string, Eigen::Vector2d> shape_dimensions_for_semantic_class_entry;
            readShapeDimensions2dBySemanticClassEntryLine(line, shape_dimensions_for_semantic_class_entry);
            shape_dimensions_2d_by_semantic_class[shape_dimensions_for_semantic_class_entry.first] =
                    shape_dimensions_for_semantic_class_entry.second;
        }
    }
}

#endif //AUTODIFF_GP_SHAPE_DIMENSIONS_BY_SEMANTIC_CLASS_2D_IO_H
