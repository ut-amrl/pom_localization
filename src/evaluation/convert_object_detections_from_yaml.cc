#include <glog/logging.h>

#include <ros/ros.h>

#include <file_io/obj_yaml_reader.h>
#include <file_io/object_positions_by_timestamp_io.h>

#include <iostream>

const std::string kObjYamlFileParamName = "object_file_yaml";
const std::string kObjOutputFileParamName = "obj_output_file_name";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "obj_detection_converter");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string object_yaml_file;
    std::string obj_output_file;

    if (!n.getParam(kObjYamlFileParamName, object_yaml_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kObjYamlFileParamName;
        exit(1);
    }

    if (!n.getParam(kObjOutputFileParamName, obj_output_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kObjOutputFileParamName;
        exit(1);
    }

    std::vector<file_io::ObjectPositionByTimestamp> obj_poses = file_io::readObjectPositionsFromYaml(object_yaml_file);

    file_io::writeObjectPositionsByTimestampToFile(obj_output_file, obj_poses);

    return 0;
}