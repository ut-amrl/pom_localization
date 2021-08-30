#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <ros/ros.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoint_consistency_results_io.h>
#include <file_io/waypoints_and_node_id.h>
#include <visualization/ros_visualization.h>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kNumTrajectoriesParamName = "waypoint_consistency_num_trajectories";

const std::string kTrajectorySpecificPrefix = "trajectory_";

const std::string kWaypointToNodeIdParamNameSuffix = "waypoint_to_node_id_file";

const std::string kTrajectoryOutputSuffix = "trajectory_output_file";

const std::string kResultsFileParamName = "results_file";

std::string constructParamName(const std::string &param_prefix, const int &trajectory_num,
                               const std::string &param_suffix) {
    return param_prefix + kTrajectorySpecificPrefix + std::to_string(trajectory_num) + "/" + param_suffix;
}

double findMeanRotation(const std::vector<pose::Pose2d> &waypoint_poses) {
    Eigen::Vector2d mean_unit_vector(0, 0);
    for (const pose::Pose2d &pose : waypoint_poses) {
        mean_unit_vector += Eigen::Vector2d(cos(pose.second), sin(pose.second));
    }
    mean_unit_vector = mean_unit_vector / (waypoint_poses.size());
    return atan2(mean_unit_vector.y(), mean_unit_vector.x());
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

    std::string results_file_name;

    uint64_t min_waypoint_id = std::numeric_limits<uint64_t>::max();
    uint64_t max_waypoint_id = 1;

    if (!n.getParam(param_prefix + kNumTrajectoriesParamName, num_trajectories)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kNumTrajectoriesParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kResultsFileParamName, results_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kResultsFileParamName;
        exit(1);
    }

    if (num_trajectories <= 0) {
        LOG(INFO) << "Trajectory count must be a positive number";
        exit(1);
    }

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n, param_prefix);

    std::vector<std::vector<pose::Pose2d>> waypoints_list;
    std::vector<std::vector<pose::Pose2d>> trajectories_list;
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
        trajectories_list.emplace_back(trajectory_estimate);

        std::vector<std::pair<uint64_t, uint64_t>> waypoints_and_node_ids_raw;
        file_io::readWaypointsAndNodeIdsFromFile(waypoints_to_nodes_file_name, waypoints_and_node_ids_raw);

        std::unordered_map<uint64_t, std::unordered_set<uint64_t>> waypoints_and_node_ids_for_traj;
        std::vector<pose::Pose2d> waypoint_poses_for_traj;
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

            waypoint_poses_for_traj.emplace_back(trajectory_estimate[node_id]);
        }

        waypoints_to_node_id_by_trajectory_num[i] = waypoints_and_node_ids_for_traj;
        waypoints_list.emplace_back(waypoint_poses_for_traj);
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
    std::unordered_map<uint64_t, double> mean_rotation_deviations;
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
        double mean_rotation = findMeanRotation(poses_for_waypoint);

        double average_deviation_from_centroid = 0;
        std::vector<double> deviations_from_mean_rotation;
        double mean_rotation_deviation = 0;
        for (const pose::Pose2d &pose_for_waypoint : poses_for_waypoint) {
            average_deviation_from_centroid += (pose_for_waypoint.first - transl_centroid).norm();
            deviations_from_mean_rotation.emplace_back(math_util::AngleDiff(pose_for_waypoint.second, mean_rotation));
            mean_rotation_deviation += math_util::AngleDist(pose_for_waypoint.second, mean_rotation);
        }
        average_deviation_from_centroid = average_deviation_from_centroid / (poses_for_waypoint.size() - 1);
        transl_deviation[waypoint_and_poses.first] = average_deviation_from_centroid;
        mean_rotation_deviation = mean_rotation_deviation / (poses_for_waypoint.size() - 1);

        std::string rotation_string;
        for (const double &rotation : deviations_from_mean_rotation) {
            rotation_string += std::to_string(rotation);
            rotation_string += ", ";
        }
        mean_rotation_deviations[waypoint_and_poses.first] = mean_rotation_deviation;
    }

    std::vector<file_io::WaypointConsistencyResult> consistency_results;
    for (uint64_t i = min_waypoint_id; i <= max_waypoint_id; i++) {
        LOG(INFO) << "Waypoint: " << i << ", Transl Deviation: " << transl_deviation[i] << ", angle deviation " << mean_rotation_deviations[i];
        file_io::WaypointConsistencyResult consistency_result;
        consistency_result.waypoint = i;
        consistency_result.transl_deviation_ = transl_deviation[i];
        consistency_result.rot_deviation_ = mean_rotation_deviations[i];
        consistency_results.push_back(consistency_result);
    }

    file_io::writeWaypointConsistencyResultsToFile(results_file_name, consistency_results);

    manager->plotWaypoints(waypoints_list);
    manager->publishEstimatedTrajectories(trajectories_list);

    return 0;
}