#include <iostream>
#include <unordered_map>

#include <glog/logging.h>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <file_io/semantic_point_with_node_id_io.h>
#include <file_io/semantic_point_with_timestamp_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_timestamp_io.h>
#include <file_io/waypoints_and_node_id.h>

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kSemanticPointDetectionsByTimestampFileParam = "semantic_point_detections_by_timestamp_file";
const std::string kSemanticPointDetectionsByNodeFileParam = "semantic_point_detections_by_node_file";
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
              node_prefix + "find_node_ids_for_semantic_points_with_timestamps");
    ros::NodeHandle n;

    std::string semantic_points_by_timestamp_file_name;
    std::string semantic_points_by_node_id_file_name;
    std::string node_id_and_timestamp_file_name;
    std::string waypoints_by_timestamp_file_name;
    std::string waypoints_by_node_id_file_name;

    bool include_semantic_point_times = false;
    if (n.getParam(param_prefix + kSemanticPointDetectionsByTimestampFileParam,
                   semantic_points_by_timestamp_file_name)) {
        include_semantic_point_times = true;
    }

    if (include_semantic_point_times) {
        if (!n.getParam(param_prefix + kSemanticPointDetectionsByNodeFileParam, semantic_points_by_node_id_file_name)) {
            LOG(INFO) << "No parameter value set for parameter with name "
                      << param_prefix + kSemanticPointDetectionsByNodeFileParam;
            exit(1);
        }
    }

    if (!n.getParam(param_prefix + kNodeIdAndTimestampOutputFileParam, node_id_and_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kNodeIdAndTimestampOutputFileParam;
        exit(1);
    }

    if (!n.getParam(param_prefix + kWaypointsByTimestampsFile, waypoints_by_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kWaypointsByTimestampsFile;
        exit(1);
    }

    if (!n.getParam(param_prefix + kWaypointsByNodeFileParam, waypoints_by_node_id_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kWaypointsByNodeFileParam;
        exit(1);
    }

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantic_points_by_timestamp;
    if (include_semantic_point_times) {
        semantic_segmentation::readSemanticallyLabeledPointWithTimestampInfoFromFile(
                semantic_points_by_timestamp_file_name, semantic_points_by_timestamp);
    }

    std::vector<file_io::WaypointAndTimestamp> waypoints_by_timestamp;
    file_io::readWaypointsAndTimestampsFromFile(waypoints_by_timestamp_file_name, waypoints_by_timestamp);

    std::unordered_map<std::pair<uint32_t, uint32_t>, uint64_t, pair_hash> nodes_by_timestamp;
    std::vector<file_io::NodeIdAndTimestamp> nodes_and_timestamps_vector;
    file_io::readNodeIdsAndTimestampsFromFile(node_id_and_timestamp_file_name, nodes_and_timestamps_vector);


    std::unordered_map<uint64_t, std::vector<std::pair<uint32_t, uint32_t>>> timestamps_for_nodes;
    std::unordered_map<uint64_t, std::vector<uint32_t>> max_cluster_label_plus_one_for_next_timestamps_for_node;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cluster_label_addition_for_next_timestamps_for_node;
    std::unordered_map<std::pair<uint32_t, uint32_t>, size_t, pair_hash> idx_of_timestamps_for_node;
    for (const file_io::NodeIdAndTimestamp &node_id_and_timestamp : nodes_and_timestamps_vector) {
        std::pair<uint32_t, uint32_t> timestamp = std::make_pair(node_id_and_timestamp.seconds_,
                                                                 node_id_and_timestamp.nano_seconds_);
        nodes_by_timestamp[timestamp] = node_id_and_timestamp.node_id_;
        std::vector<std::pair<uint32_t, uint32_t>> timestamps_for_node;
        if (timestamps_for_nodes.find(node_id_and_timestamp.node_id_) != timestamps_for_nodes.end()) {
            timestamps_for_node = timestamps_for_nodes.at(node_id_and_timestamp.node_id_);
        }
        idx_of_timestamps_for_node[timestamp] = timestamps_for_node.size();
        timestamps_for_node.emplace_back(timestamp);
        max_cluster_label_plus_one_for_next_timestamps_for_node[node_id_and_timestamp.node_id_].emplace_back(0);
        cluster_label_addition_for_next_timestamps_for_node[node_id_and_timestamp.node_id_].emplace_back(0);
        timestamps_for_nodes[node_id_and_timestamp.node_id_] = timestamps_for_node;
    }

    for (const semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo &semantic_points_with_timestamp : semantic_points_by_timestamp) {
        std::pair<uint32_t, uint32_t> timestamp = std::make_pair(semantic_points_with_timestamp.seconds,
                                                                 semantic_points_with_timestamp.nano_seconds);
        if (nodes_by_timestamp.find(timestamp) == nodes_by_timestamp.end()) {
            LOG(INFO) << "No node id found for timestamp " << timestamp.first << ", " << timestamp.second
                      << ". Skipping object detection";
            continue;
        }
        uint64_t node_id = nodes_by_timestamp[timestamp];
        if (idx_of_timestamps_for_node.find(timestamp) == idx_of_timestamps_for_node.end()) {
            LOG(ERROR) << "Could not find idx of timestamp in respective node " << timestamp.first << ", "
                       << timestamp.second;
            exit(1);
        }
        size_t idx_of_timestamp_for_node = idx_of_timestamps_for_node.at(timestamp);
        uint32_t max_cluster_label_plus_one_for_timestamp_for_node = max_cluster_label_plus_one_for_next_timestamps_for_node[node_id][idx_of_timestamp_for_node];
        max_cluster_label_plus_one_for_timestamp_for_node = std::max(max_cluster_label_plus_one_for_timestamp_for_node,
                                                                  semantic_points_with_timestamp.cluster_label + 1);
        max_cluster_label_plus_one_for_next_timestamps_for_node[node_id][idx_of_timestamp_for_node] = max_cluster_label_plus_one_for_timestamp_for_node;
    }

    for (const auto &node_id_and_timestamps_for_node : timestamps_for_nodes) {
        uint64_t node_num = node_id_and_timestamps_for_node.first;
        std::vector<std::pair<uint32_t, uint32_t>> timestamps_for_node = node_id_and_timestamps_for_node.second;
        for (size_t timestamp_for_node_num = 0;
             timestamp_for_node_num < timestamps_for_node.size(); timestamp_for_node_num++) {
            if (timestamp_for_node_num == 0) {
                cluster_label_addition_for_next_timestamps_for_node[node_num][timestamp_for_node_num] = 0;
            } else {
                uint32_t prev_cluster_min_stamp = cluster_label_addition_for_next_timestamps_for_node[node_num][timestamp_for_node_num - 1];
                uint32_t max_raw_cluster_id_for_prev_stamp_plus_one =  max_cluster_label_plus_one_for_next_timestamps_for_node[node_num][timestamp_for_node_num - 1];
                uint32_t min_cluster_id_curr_stamp = prev_cluster_min_stamp + max_raw_cluster_id_for_prev_stamp_plus_one;
                cluster_label_addition_for_next_timestamps_for_node[node_num][timestamp_for_node_num] = min_cluster_id_curr_stamp;
            }
        }
    }

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo> semantic_points_by_node_id;
    for (const semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo &semantic_points_with_timestamp : semantic_points_by_timestamp) {
        semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo semantic_points_with_node_id;
        std::pair<uint32_t, uint32_t> timestamp = std::make_pair(semantic_points_with_timestamp.seconds,
                                                                 semantic_points_with_timestamp.nano_seconds);
        if (nodes_by_timestamp.find(timestamp) == nodes_by_timestamp.end()) {
            LOG(INFO) << "No node id found for timestamp " << timestamp.first << ", " << timestamp.second
                      << ". Skipping object detection";
            continue;
        }
        uint64_t node_num = nodes_by_timestamp[timestamp];
        semantic_points_with_node_id.node_id = node_num;
        semantic_points_with_node_id.semantic_label = semantic_points_with_timestamp.semantic_label;
        semantic_points_with_node_id.orig_cluster_label = semantic_points_with_timestamp.cluster_label;
        semantic_points_with_node_id.point_x = semantic_points_with_timestamp.point_x;
        semantic_points_with_node_id.point_y = semantic_points_with_timestamp.point_y;
        semantic_points_with_node_id.point_z = semantic_points_with_timestamp.point_z;
        semantic_points_with_node_id.seconds = timestamp.first;
        semantic_points_with_node_id.nano_seconds = timestamp.second;

        if (idx_of_timestamps_for_node.find(timestamp) == idx_of_timestamps_for_node.end()) {
            LOG(ERROR) << "Could not find idx of timestamp in respective node " << timestamp.first << ", "
                       << timestamp.second;
            exit(1);
        }
        size_t timestamp_for_node_num = idx_of_timestamps_for_node.at(timestamp);
        uint32_t cluster_id_addition = cluster_label_addition_for_next_timestamps_for_node[node_num][timestamp_for_node_num];
        semantic_points_with_node_id.cluster_label = semantic_points_with_node_id.orig_cluster_label + cluster_id_addition;

        semantic_points_by_node_id.emplace_back(semantic_points_with_node_id);
    }

    semantic_segmentation::writeSemanticallyLabeledPointWithNodeIdInfosToFile(semantic_points_by_node_id_file_name,
                                                                              semantic_points_by_node_id);

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

    file_io::writeWaypointsAndNodeIdsToFile(waypoints_by_node_id_file_name, waypoints_by_node);

    return 0;
}