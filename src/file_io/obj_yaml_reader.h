//
// Created by amanda on 6/13/21.
//

#ifndef AUTODIFF_GP_OBJ_YAML_READER_H
#define AUTODIFF_GP_OBJ_YAML_READER_H

#include <file_io/object_positions_by_timestamp.h>
#include <yaml-cpp/yaml.h>
#include <base_lib/pose_reps.h>

namespace file_io {
    std::vector<ObjectPositionByTimestamp> readObjectPositionsFromYaml(std::string &file) {
        YAML::Node node;
        try {
            node = YAML::LoadFile(file);
        }
        catch (YAML::Exception const &e) {
            LOG(WARNING) << "Failed to open " << file << ": " << e.msg;
            return {};
        }

        YAML::Node tracks = node["tracks"];
        std::vector<ObjectPositionByTimestamp> obj_positions;
        for (size_t i = 0; i < tracks.size(); ++i) {
            YAML::Node annotation = tracks[i];
            auto const id = annotation["id"].as<uint64_t>();
            YAML::Node t = annotation["track"];
            for (size_t j = 0; j < t.size(); ++j) {
                ObjectPositionByTimestamp obj_pos;
                YAML::Node inst = t[j];
                obj_pos.label_ = inst["label"].as<std::string>();

                YAML::Node header = inst["header"];
                obj_pos.seconds_ = header["stamp"]["secs"].as<uint32_t>();
                obj_pos.nano_seconds_ = header["stamp"]["nsecs"].as<uint32_t>();

                YAML::Node origin = inst["translation"];
                Eigen::Vector3d transl(origin["x"].as<double>(), origin["y"].as<double>(), origin["z"].as<double>());
                YAML::Node rotation = inst["rotation"];
                Eigen::Quaterniond rot(rotation["w"].as<double>(), rotation["x"].as<double>(),
                                       rotation["y"].as<double>(), rotation["z"].as<double>());

                pose::Pose3d pose_3d = std::make_pair(transl, rot);
                pose::Pose2d pose_2d = pose::toPose2d(pose_3d);
                obj_pos.transl_x_ = pose_2d.first.x();
                obj_pos.transl_y_ = pose_2d.first.y();
                obj_pos.theta_ = pose_2d.second;
                obj_pos.identifier_ = id;

                obj_positions.emplace_back(obj_pos);
            }
        }
        return obj_positions;
    }
}

#endif //AUTODIFF_GP_OBJ_YAML_READER_H
