//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_OBJ_DETECTIONS_H
#define AUTODIFF_GP_OBJ_DETECTIONS_H

#include <fstream>
#include <string>
#include <unordered_set>

#include <h3d_dataset/h3d_file_operations.h>

namespace h3d {

    static const std::string kStaticObjLabel = "static";

    struct RawObjectDetection {
        std::string label_;
        std::string tracker_id_;
        std::string state_; // dynamic or static
        double centroid_x_;
        double centroid_y_;
        double centroid_z_;
        double length_x_;
        double length_y_;
        double length_z_;
        double yaw_;
    };

    RawObjectDetection readLineFromObjDetectionFile(const std::string &obj_detection_line) {
        std::stringstream ss(obj_detection_line);
        std::vector<std::string> data;
        while(ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(substr);
        }
        RawObjectDetection obj_detection_data;
        obj_detection_data.label_ = data[0];
        obj_detection_data.tracker_id_ = data[1];
        obj_detection_data.state_= data[2];
        obj_detection_data.centroid_x_ = std::stod(data[3]);
        obj_detection_data.centroid_y_= std::stod(data[4]);
        obj_detection_data.centroid_z_ = std::stod(data[5]);
        obj_detection_data.length_x_ = std::stod(data[6]);
        obj_detection_data.length_y_ = std::stod(data[7]);
        obj_detection_data.length_z_ = std::stod(data[8]);
        obj_detection_data.yaw_ = std::stod(data[9]);
        return obj_detection_data;
    }

    std::vector<RawObjectDetection> readObjDetectionDataFromFile(const std::string &obj_detection_filename) {
        std::ifstream file_obj(obj_detection_filename);
        std::string line;
        std::vector<RawObjectDetection> obj_detection_data;
        while (std::getline(file_obj, line)) {
            obj_detection_data.emplace_back(readLineFromObjDetectionFile(line));
        }
        return obj_detection_data;
    }


    std::vector<RawObjectDetection> readObjDetectionDataFromFile(const std::string &scenario_dir, const std::string & file_num_str) {
        std::string obj_detection_filename = scenario_dir;
        obj_detection_filename += kObjDetectionFilePrefix;
        obj_detection_filename += file_num_str;
        obj_detection_filename += kTxtFileSuffix;

        return readObjDetectionDataFromFile(obj_detection_filename);
    }

    std::vector<RawObjectDetection> readObjDetectionDataFromFile(const std::string &scenario_dir, const int file_num) {
        return readObjDetectionDataFromFile(scenario_dir, convertFileNumToFileStrSuffix(file_num));
    }

    std::vector<RawObjectDetection> getStaticObjDetectionsWithTypes(
            const std::vector<RawObjectDetection> &full_obj_detection_list,
            const std::unordered_set<std::string> classes_to_use) {
        std::vector<RawObjectDetection> filtered_obj_detections;
        for (RawObjectDetection obj_detection : full_obj_detection_list) {
            if (obj_detection.state_ == kStaticObjLabel) {
                if (classes_to_use.find(obj_detection.label_) != classes_to_use.end()) {
                    filtered_obj_detections.emplace_back(obj_detection);
                }
            }
        }
        return filtered_obj_detections;
    }

}

#endif //AUTODIFF_GP_OBJ_DETECTIONS_H
