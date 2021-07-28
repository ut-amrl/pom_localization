#include <glog/logging.h>

#include <ros/ros.h>

#include <file_io/obj_yaml_reader.h>
#include <file_io/object_positions_by_timestamp_io.h>

#include <iostream>


DEFINE_string(param_prefix, "", "param_prefix");

const std::string kObjYamlFileParamName = "object_file_yaml";
const std::string kObjOutputFileParamName = "obj_output_file_name";

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string param_prefix = FLAGS_param_prefix;
    std::string node_prefix = FLAGS_param_prefix;
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "obj_detection_converter");
    ros::NodeHandle n;

    std::string object_yaml_file;
    std::string obj_output_file;

    if (!n.getParam(param_prefix + kObjYamlFileParamName, object_yaml_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kObjYamlFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kObjOutputFileParamName, obj_output_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kObjOutputFileParamName;
        exit(1);
    }

    std::vector<file_io::ObjectPositionByTimestamp> obj_poses = file_io::readObjectPositionsFromYaml(object_yaml_file);

    file_io::writeObjectPositionsByTimestampToFile(obj_output_file, obj_poses);

    return 0;
}