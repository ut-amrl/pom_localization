//
// Created by amanda on 2/21/21.
//

#ifndef AUTODIFF_GP_OBJECT_INSTANCES_OBSERVED_AT_POSES_H
#define AUTODIFF_GP_OBJECT_INSTANCES_OBSERVED_AT_POSES_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <pose_optimization/pose_graph_generic.h>
#include <base_lib/pose_reps.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <file_io/file_io_utils.h>

namespace file_io {

    std::pair<uint64_t, std::vector<uint64_t>> readObservedAtLine(const std::string &line_in_file) {
        std::stringstream ss(line_in_file);
        uint64_t instance;
        std::vector<uint64_t> observed_at_poses;
        bool first_entry = true;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            if (first_entry) {
                first_entry = false;
                std::istringstream stream(substr);
                stream >> instance;
            } else {
                uint64_t pose_id;
                std::istringstream stream(substr);
                stream >> pose_id;
                observed_at_poses.push_back(pose_id);
            }
        }
        return std::make_pair(instance, observed_at_poses);
    }

    std::unordered_map<uint64_t, std::vector<uint64_t>>
    readInstancesObservedAtPosesFromFile(const std::string &file_name) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        std::unordered_map<uint64_t, std::vector<uint64_t>> instances_and_observed_at_poses;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }

            std::pair<uint64_t, std::vector<uint64_t>> instance_and_observed_at_poses = readObservedAtLine(line);
            instances_and_observed_at_poses[instance_and_observed_at_poses.first] = instance_and_observed_at_poses.second;
        }
        return instances_and_observed_at_poses;
    }
}

#endif //AUTODIFF_GP_OBJECT_INSTANCES_OBSERVED_AT_POSES_H
