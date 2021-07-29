#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <ros/ros.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_node_id.h>
#include <visualization/ros_visualization.h>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kNumTrajectoriesParamName = "waypoint_consistency_num_trajectories";

const std::string kTrajectorySpecificPrefix = "trajectory_";

const std::string kWaypointToNodeIdParamNameSuffix = "waypoint_to_node_id_file";

const std::string kTrajectoryOutputSuffix = "trajectory_output_file";

std::string constructParamName(const std::string &param_prefix, const int &trajectory_num,
                               const std::string &param_suffix) {
    return param_prefix + kTrajectorySpecificPrefix + std::to_string(trajectory_num) + "/" + param_suffix;
}

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
              node_prefix + "plot_results");
    ros::NodeHandle n;

    // Known config params
    std::unordered_map<int, std::vector<pose::Pose2d>> trajectory_outputs_by_trajectory_num;

    // Outer key is the trajectory number
    // Inner key is the waypoint number
    // Inner value is the set of nodes that correspond to the waypoint
    std::unordered_map<int, std::unordered_map<uint64_t, std::unordered_set<uint64_t>>> waypoints_to_node_id_by_trajectory_num;
    int num_trajectories;

    uint64_t min_waypoint_id = std::numeric_limits<uint64_t>::max();
    uint64_t max_waypoint_id = 1;

    if (!n.getParam(param_prefix + kNumTrajectoriesParamName, num_trajectories)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kNumTrajectoriesParamName;
        exit(1);
    }

    if (num_trajectories <= 0) {
        LOG(INFO) << "Trajectory count must be a positive number";
        exit(1);
    }

    for (int i = 0; i < num_trajectories; i++) {
        std::string trajectory_file_name;
        std::string waypoints_to_nodes_file_name;

        std::string trajectory_file_param_name = constructParamName(param_prefix, i, kTrajectoryOutputSuffix);
        std::string waypoint_to_node_id_file_param_name = constructParamName(param_prefix, i,
                                                                             kWaypointToNodeIdParamNameSuffix);
        if (!n.getParam(trajectory_file_param_name, trajectory_file_name)) {
            LOG(INFO) << "No parameter value set for parameter with name " << trajectory_file_param_name;
            exit(1);
        }
        if (!n.getParam(waypoint_to_node_id_file_param_name, waypoints_to_nodes_file_name)) {
            LOG(INFO) << "No parameter value set for parameter with name " << waypoint_to_node_id_file_param_name;
            exit(1);
        }

        std::vector<pose::Pose2d> trajectory_estimate = file_io::readTrajFromFile(trajectory_file_name);
        trajectory_outputs_by_trajectory_num[i] = trajectory_estimate;

        std::vector<std::pair<uint64_t, uint64_t>> waypoints_and_node_ids_raw;
        file_io::readWaypointsAndNodeIdsFromFile(waypoints_to_nodes_file_name, waypoints_and_node_ids_raw);

        std::unordered_map<uint64_t, std::unordered_set<uint64_t>> waypoints_and_node_ids_for_traj;
        for (const std::pair<uint64_t, uint64_t> &waypoint_and_node_id : waypoints_and_node_ids_raw) {
            uint64_t waypoint = waypoint_and_node_id.first;
            uint64_t node_id = waypoint_and_node_id.second;

            std::unordered_set<uint64_t> nodes_for_waypoint;
            if (waypoints_and_node_ids_for_traj.find(waypoint) != waypoints_and_node_ids_for_traj.end()) {
                nodes_for_waypoint = waypoints_and_node_ids_for_traj[waypoint];
            }
            nodes_for_waypoint.insert(node_id);
            waypoints_and_node_ids_for_traj[waypoint] = nodes_for_waypoint;

            if (waypoint < min_waypoint_id) {
                min_waypoint_id = waypoint;
            }
            if (waypoint > max_waypoint_id) {
                max_waypoint_id = waypoint;
            }
        }

        waypoints_to_node_id_by_trajectory_num[i] = waypoints_and_node_ids_for_traj;
    }

    std::unordered_map<uint64_t, std::vector<pose::Pose2d>> poses_for_waypoints;
    for (uint64_t waypoint_id = min_waypoint_id; waypoint_id <= max_waypoint_id; waypoint_id++) {
        std::vector<pose::Pose2d> poses_for_waypoint;
        for (int i = 0; i < num_trajectories; i++) {
            std::unordered_set<uint64_t> nodes_for_waypoint_for_traj = waypoints_to_node_id_by_trajectory_num[i][waypoint_id];
            if (nodes_for_waypoint_for_traj.empty()) {
                continue;
            }
            std::vector<pose::Pose2d> trajectory = trajectory_outputs_by_trajectory_num[i];
            for (const uint64_t &node_id_for_waypoint : nodes_for_waypoint_for_traj) {
                poses_for_waypoint.emplace_back(trajectory[node_id_for_waypoint]);
            }
        }
        if (!poses_for_waypoint.empty()) {
            poses_for_waypoints[waypoint_id] = poses_for_waypoint;
        }
    }

    std::unordered_map<uint64_t, double> transl_deviation;
    for (const auto &waypoint_and_poses : poses_for_waypoints) {
        std::vector<pose::Pose2d> poses_for_waypoint = waypoint_and_poses.second;
        if (poses_for_waypoint.size() <= 1) {
            transl_deviation[waypoint_and_poses.first] = 0;
            continue;
        }
        Eigen::Vector2d transl_centroid(0, 0);
        for (const pose::Pose2d &pose_for_waypoint : poses_for_waypoint) {
            transl_centroid += pose_for_waypoint.first;
        }
        transl_centroid = transl_centroid / poses_for_waypoint.size();

        double average_deviation_from_centroid = 0;
        for (const pose::Pose2d &pose_for_waypoint : poses_for_waypoint) {
            average_deviation_from_centroid += (pose_for_waypoint.first - transl_centroid).norm();
        }
        average_deviation_from_centroid = average_deviation_from_centroid / (poses_for_waypoint.size() - 1);
        if (poses_for_waypoint.size() <= 1) {
            transl_deviation[waypoint_and_poses.first] = average_deviation_from_centroid;
        }
    }



// TODO should we visualize?

    return 0;
}