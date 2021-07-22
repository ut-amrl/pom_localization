#include <iostream>
#include <unordered_map>

#include <glog/logging.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <file_io/object_positions_by_pose_io.h>
#include <file_io/object_positions_by_timestamp_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_timestamp_io.h>
#include <file_io/waypoints_and_node_id.h>

const std::string kObjectDetectionsByTimestampFileParam = "object_detections_by_timestamp_file";
const std::string kObjectDetectionsByNodeFileParam = "object_detections_by_node_file";
const std::string kNodeIdAndTimestampOutputFileParam = "node_id_and_timestamp_file";
const std::string kWaypointsByTimestampsFile = "waypoints_by_timestamps_file";
const std::string kWaypointsByNodeFileParam = "waypoints_by_node_file";

struct pair_hash {
    template<class T1, class T2>
    std::size_t operator()(std::pair<T1, T2> const &pair) const {
        std::size_t h1 = std::hash<T1>()(pair.first);
        std::size_t h2 = std::hash<T2>()(pair.second);

        return h1 ^ h2;
    }
};

int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    ros::init(argc, argv,
              "find_node_ids_for_objs_with_timestamps");
    ros::NodeHandle n;

    std::string objs_by_timestamp_file_name;
    std::string objs_by_node_id_file_name;
    std::string node_id_and_timestamp_file_name;
    std::string waypoints_by_timestamp_file_name;
    std::string waypoints_by_node_id_file_name;

    if (!n.getParam(kObjectDetectionsByNodeFileParam, objs_by_node_id_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kObjectDetectionsByNodeFileParam;
        exit(1);
    }

    if (!n.getParam(kObjectDetectionsByTimestampFileParam, objs_by_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kObjectDetectionsByTimestampFileParam;
        exit(1);
    }

    if (!n.getParam(kNodeIdAndTimestampOutputFileParam, node_id_and_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kNodeIdAndTimestampOutputFileParam;
        exit(1);
    }

    if (!n.getParam(kWaypointsByTimestampsFile, waypoints_by_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kWaypointsByTimestampsFile;
        exit(1);
    }

    if (!n.getParam(kWaypointsByNodeFileParam, waypoints_by_node_id_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kWaypointsByNodeFileParam;
        exit(1);
    }

    std::vector<file_io::ObjectPositionByTimestamp> objs_by_timestamp;
    file_io::readObjectPositionsByTimestampFromFile(objs_by_timestamp_file_name, objs_by_timestamp);

    std::vector<file_io::WaypointAndTimestamp> waypoints_by_timestamp;
    file_io::readWaypointsAndTimestampsFromFile(waypoints_by_timestamp_file_name, waypoints_by_timestamp);

    std::unordered_map<std::pair<uint32_t, uint32_t>, uint64_t, pair_hash> nodes_by_timestamp;
    std::vector<file_io::NodeIdAndTimestamp> nodes_and_timestamps_vector;
    file_io::readNodeIdsAndTimestampsFromFile(node_id_and_timestamp_file_name, nodes_and_timestamps_vector);

    for (const file_io::NodeIdAndTimestamp &node_id_and_timestamp : nodes_and_timestamps_vector) {
        nodes_by_timestamp[std::make_pair(node_id_and_timestamp.seconds_,
                                          node_id_and_timestamp.nano_seconds_)] = node_id_and_timestamp.node_id_;
    }


    std::vector<file_io::ObjectPositionByPose> objs_by_pose;
    for (const file_io::ObjectPositionByTimestamp &obj_with_timestamp : objs_by_timestamp) {
        file_io::ObjectPositionByPose obj_with_pose_id;
        std::pair<uint32_t, uint32_t> timestamp = std::make_pair(obj_with_timestamp.seconds_,
                                                                 obj_with_timestamp.nano_seconds_);
        if (nodes_by_timestamp.find(timestamp) == nodes_by_timestamp.end()) {
            LOG(INFO) << "No node id found for timestamp " << timestamp.first << ", " << timestamp.second
                      << ". Skipping object detection";
            continue;
        }
        obj_with_pose_id.pose_number_ = nodes_by_timestamp[timestamp];
        obj_with_pose_id.identifier_ = obj_with_timestamp.identifier_;
        obj_with_pose_id.transl_x_ = obj_with_timestamp.transl_x_;
        obj_with_pose_id.transl_y_ = obj_with_timestamp.transl_y_;
        obj_with_pose_id.theta_ = obj_with_timestamp.theta_;

        objs_by_pose.emplace_back(obj_with_pose_id);
    }

    file_io::writeObjectPositionsByPoseToFile(objs_by_node_id_file_name, objs_by_pose);

    std::vector<std::pair<uint64_t, uint64_t>> waypoints_by_node;
    for (const file_io::WaypointAndTimestamp &waypoint_with_timestamp : waypoints_by_timestamp) {
        std::pair<uint32_t, uint32_t> timestamp = std::make_pair(waypoint_with_timestamp.seconds_,
                                                                 waypoint_with_timestamp.nano_seconds_);
        if (nodes_by_timestamp.find(timestamp) == nodes_by_timestamp.end()) {
            LOG(INFO) << "No node id found for timestamp " << timestamp.first << ", " << timestamp.second
                      << ". Skipping waypoint";
            continue;
        }
        std::pair<uint64_t, uint64_t> waypoint_by_node = std::make_pair(waypoint_with_timestamp.waypoint,
                                                                        nodes_by_timestamp[timestamp]);
        waypoints_by_node.push_back(waypoint_by_node);
    }

    file_io::readWaypointsAndNodeIdsFromFile(waypoints_by_node_id_file_name, waypoints_by_node);

    return 0;
}