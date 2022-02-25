#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <base_lib/pose_reps.h>

#include <file_io/semantic_point_with_timestamp_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_timestamp_io.h>
#include <base_lib/pose_utils.h>

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kRosbagFileNameParam = "rosbag_file_name";
const std::string kSemanticPointDetectionsFileParam = "semantic_point_detections_by_timestamp_file";
const std::string kOdomOutputFileParam = "odom_out_file_name";
const std::string kNodeIdAndTimestampOutputFileParam = "node_id_and_timestamp_file";
const std::string kWaypointsByTimestampsFile = "waypoints_by_timestamps_file";

const double kMaxPoseIncThresholdTransl = 1.0;
const double kMaxPoseIncThresholdRot = 0.25; // TODO?

const double kPoseEquivTolerance = 1e-3;

struct pair_hash {
    template<class T1, class T2>
    std::size_t operator()(std::pair<T1, T2> const &pair) const {
        std::size_t h1 = std::hash<T1>()(pair.first);
        std::size_t h2 = std::hash<T2>()(pair.second);

        return h1 ^ h2;
    }
};

struct timestamp_sort {
    inline bool
    operator()(const std::pair<uint32_t, uint32_t> &timestamp1, const std::pair<uint32_t, uint32_t> &timestamp2) {
        if (timestamp1.first != timestamp2.first) {
            return timestamp1.first < timestamp2.first;
        }
        return timestamp1.second <= timestamp2.second;
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
              node_prefix + "wheel_odom_rosbag_extractor_semantic_points");
    ros::NodeHandle n;

    std::string rosbag_file_name;
    std::string odom_out_file_name;
    std::string semantic_point_file_name;
    std::string waypoints_by_timestamp_file_name;
    std::string node_id_and_timestamp_file_name;

    bool include_semantic_point_times = false;
    bool include_waypoint_times = false;

    if (!n.getParam(param_prefix + kRosbagFileNameParam, rosbag_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kRosbagFileNameParam;
        exit(1);
    }

    if (!n.getParam(param_prefix + kOdomOutputFileParam, odom_out_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kOdomOutputFileParam;
        exit(1);
    }

    if (!n.getParam(param_prefix + kNodeIdAndTimestampOutputFileParam, node_id_and_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kNodeIdAndTimestampOutputFileParam;
        exit(1);
    }

    if (n.getParam(param_prefix + kSemanticPointDetectionsFileParam, semantic_point_file_name)) {
        include_semantic_point_times = true;
    }
    if (n.getParam(param_prefix + kWaypointsByTimestampsFile, waypoints_by_timestamp_file_name)) {
        include_waypoint_times = true;
    }

    rosbag::Bag bag;
    bag.open(rosbag_file_name, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/jackal_velocity_controller/odom"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<pose::Pose2d> full_odom_frame_poses;
    std::vector<std::pair<uint32_t, uint32_t>> full_timestamps;

    for (rosbag::MessageInstance const &m : view) {
        nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
        if (!full_timestamps.empty()) {
            std::pair<uint32_t, uint32_t> prev_stamp = full_timestamps.back();
            if ((prev_stamp.first > msg->header.stamp.sec) ||
                ((prev_stamp.first == msg->header.stamp.sec) && (prev_stamp.second > msg->header.stamp.nsec))) {
                LOG(INFO) << "Out of order messages!";
            }
        }
        full_timestamps.emplace_back(std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec));
        pose::Pose3d odom_3d = std::make_pair(
                Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                   msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

        pose::Pose2d pose = pose::toPose2d(odom_3d);
        full_odom_frame_poses.emplace_back(pose);
    }

    LOG(INFO) << "Min odom timestamp " << full_timestamps.front().first << ", " << full_timestamps.front().second;
    LOG(INFO) << "Max odom timestamp " << full_timestamps.back().first << ", " << full_timestamps.back().second;

    std::unordered_set<std::pair<uint32_t, uint32_t>, pair_hash> semantic_points_and_waypoint_timestamps_set;
    if (include_semantic_point_times) {
        LOG(INFO) << "Getting timestamps from file " << semantic_point_file_name;
        std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantic_points_by_timestamp;
        semantic_segmentation::readSemanticallyLabeledPointWithTimestampInfoFromFile(semantic_point_file_name,
                                                                                     semantic_points_by_timestamp);

        for (const semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo &semantic_points_with_timestamp : semantic_points_by_timestamp) {
            semantic_points_and_waypoint_timestamps_set.insert(
                    std::make_pair(semantic_points_with_timestamp.seconds,
                                   semantic_points_with_timestamp.nano_seconds));
        }
        // This assumes the file is read in time order
        std::pair<uint32_t, uint32_t> first_semantic_point_timestamp = std::make_pair(
                semantic_points_by_timestamp.front().seconds, semantic_points_by_timestamp.front().nano_seconds);
        std::pair<uint32_t, uint32_t> last_semantic_point_timestamp = std::make_pair(
                semantic_points_by_timestamp.back().seconds, semantic_points_by_timestamp.back().nano_seconds);

        if (!timestamp_sort()(full_timestamps.front(), first_semantic_point_timestamp)) {
            LOG(INFO) << "The first odom timestamp is greater than the first semantic point timestamp";
        }
        if (!timestamp_sort()(last_semantic_point_timestamp, full_timestamps.back())) {
            LOG(INFO) << "Last semantic point timestamp is not less than or equal to the odom timestamp";
        }
    }
    if (include_waypoint_times) {
        LOG(INFO) << "Getting waypoint times from file " << waypoints_by_timestamp_file_name;
        std::vector<file_io::WaypointAndTimestamp> waypoints_by_timestamp;
        file_io::readWaypointsAndTimestampsFromFile(waypoints_by_timestamp_file_name, waypoints_by_timestamp);

        for (const file_io::WaypointAndTimestamp &waypoint_with_timestamp : waypoints_by_timestamp) {
            semantic_points_and_waypoint_timestamps_set.insert(
                    std::make_pair(waypoint_with_timestamp.seconds_, waypoint_with_timestamp.nano_seconds_));
        }
        // This assumes the file is read in time order
        std::pair<uint32_t, uint32_t> first_waypoint_timestamp = std::make_pair(waypoints_by_timestamp.front().seconds_,
                                                                                waypoints_by_timestamp.front().nano_seconds_);
        std::pair<uint32_t, uint32_t> last_waypoint_timestamp = std::make_pair(waypoints_by_timestamp.back().seconds_,
                                                                               waypoints_by_timestamp.back().nano_seconds_);

        if (!timestamp_sort()(full_timestamps.front(), first_waypoint_timestamp)) {
            LOG(INFO) << "The first odom timestamp is greater than the first waypoint timestamp";
        }
        if (!timestamp_sort()(last_waypoint_timestamp, full_timestamps.back())) {
            LOG(INFO) << "Last waypoint timestamp is not less than or equal to the odom timestamp";
        }
    }

    std::vector<std::pair<uint32_t, uint32_t>> sorted_semantic_point_timestamps;
    sorted_semantic_point_timestamps.insert(sorted_semantic_point_timestamps.end(),
                                            semantic_points_and_waypoint_timestamps_set.begin(),
                                            semantic_points_and_waypoint_timestamps_set.end());
    std::sort(sorted_semantic_point_timestamps.begin(), sorted_semantic_point_timestamps.end(), timestamp_sort());

    LOG(INFO) << "Min semantic_point timestamp " << sorted_semantic_point_timestamps.front().first << ", "
              << sorted_semantic_point_timestamps.front().second;
    LOG(INFO) << "Max semantic_point timestamp " << sorted_semantic_point_timestamps.back().first << ", "
              << sorted_semantic_point_timestamps.back().second;
    std::vector<pose::Pose2d> poses_to_use;
    std::vector<std::pair<uint32_t, uint32_t>> timestamps_to_use;
    poses_to_use.emplace_back(full_odom_frame_poses[0]);
    timestamps_to_use.emplace_back(full_timestamps[0]);
    size_t index_next_semantic_point_timestamp_to_check = 0;
    if (!sorted_semantic_point_timestamps.empty()) {
        // If the first full timestamp is not less than or equal to the first sorted timestamp, need to start with the next detection/waypoint timestamp
        if (!timestamp_sort()(full_timestamps[0], sorted_semantic_point_timestamps[0])) {
            LOG(INFO) << "Skipping first sorted timestamp";
            index_next_semantic_point_timestamp_to_check = 1;
        }
    }

    for (size_t i = 1; i < full_timestamps.size(); i++) {
        bool added_pose = false;

        pose::Pose2d curr_pose = full_odom_frame_poses[i];
        std::pair<uint32_t, uint32_t> curr_timestamp = full_timestamps[i];
        if (index_next_semantic_point_timestamp_to_check < sorted_semantic_point_timestamps.size()) {
            std::pair<uint32_t, uint32_t> next_semantic_point_timestamp = sorted_semantic_point_timestamps[index_next_semantic_point_timestamp_to_check];
            if (timestamp_sort()(next_semantic_point_timestamp, curr_timestamp)) {
                if ((next_semantic_point_timestamp.first == curr_timestamp.first) &&
                    (next_semantic_point_timestamp.second == curr_timestamp.second)) {
                    timestamps_to_use.emplace_back(curr_timestamp);
                    poses_to_use.emplace_back(full_odom_frame_poses[i]);
                } else {
                    pose::Pose2d prev_pose = full_odom_frame_poses[i - 1];
                    std::pair<uint32_t, uint32_t> prev_timestamp = full_timestamps[i - 1];

                    pose::Pose2d rel_pose_interp_global = pose::interpolatePoses(
                            std::make_pair(prev_timestamp, prev_pose),
                            std::make_pair(curr_timestamp, curr_pose),
                            next_semantic_point_timestamp);
                    timestamps_to_use.emplace_back(next_semantic_point_timestamp);
                    poses_to_use.emplace_back(rel_pose_interp_global);
                }
                added_pose = true;
                index_next_semantic_point_timestamp_to_check++;
            }
        }
        if (!added_pose) {
            pose::Pose2d last_added_pose = poses_to_use.back();
            pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(curr_pose, last_added_pose);
            if ((rel_pose.first.norm() > kMaxPoseIncThresholdTransl) || (rel_pose.second > kMaxPoseIncThresholdRot)) {
                poses_to_use.emplace_back(curr_pose);
                timestamps_to_use.emplace_back(curr_timestamp);
            }
        }
    }
    pose::Pose2d last_pose_in_used_list = poses_to_use.back();
    pose::Pose2d last_pose = full_odom_frame_poses.back();
    if (!pose::posesAlmostSame(last_pose, last_pose_in_used_list, kPoseEquivTolerance, kPoseEquivTolerance)) {
        poses_to_use.emplace_back(last_pose);
    }

    std::vector<pose::Pose2d> poses_rel_to_origin;
    poses_rel_to_origin.emplace_back(pose::createPose2d(0, 0, 0));
    pose::Pose2d first_pose = poses_to_use[0];
    for (size_t i = 1; i < poses_to_use.size(); i++) {
        pose::Pose2d curr_pose = poses_to_use[i];
        poses_rel_to_origin.emplace_back(pose::getPoseOfObj1RelToObj2(curr_pose, first_pose));
    }

    std::vector<pose::Pose2d> deduped_poses_rel_to_origin;
    std::vector<file_io::NodeIdAndTimestamp> nodes_with_timestamps;
    file_io::NodeIdAndTimestamp first_node;
    first_node.node_id_ = 0;
    first_node.seconds_ = timestamps_to_use[0].first;
    first_node.nano_seconds_ = timestamps_to_use[0].second;
    nodes_with_timestamps.emplace_back(first_node);
    deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[0]);
    for (size_t i = 1; i < poses_rel_to_origin.size(); i++) {
//        if (!pose::posesSame(poses_rel_to_origin[i - 1], poses_rel_to_origin[i])) {
        if (!pose::posesAlmostSame(poses_rel_to_origin[i - 1], poses_rel_to_origin[i], kPoseEquivTolerance,
                                  kPoseEquivTolerance)) {
            deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[i]);
        }

        file_io::NodeIdAndTimestamp node_with_timestamp;
        node_with_timestamp.node_id_ = deduped_poses_rel_to_origin.size() - 1;
        node_with_timestamp.seconds_ = timestamps_to_use[i].first;
        node_with_timestamp.nano_seconds_ = timestamps_to_use[i].second;
        nodes_with_timestamps.emplace_back(node_with_timestamp);
    }

    bag.close();

    LOG(INFO) << "Trajectory nodes size " << poses_rel_to_origin.size();
    std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
    for (size_t node_num = 0; node_num < deduped_poses_rel_to_origin.size(); node_num++) {
        file_io::TrajectoryNode2d trajectory_node_2d;
        trajectory_node_2d.node_id_ = node_num;

        trajectory_node_2d.transl_x_ = deduped_poses_rel_to_origin[node_num].first.x();
        trajectory_node_2d.transl_y_ = deduped_poses_rel_to_origin[node_num].first.y();
        trajectory_node_2d.theta_ = deduped_poses_rel_to_origin[node_num].second;

        trajectory_nodes.emplace_back(trajectory_node_2d);
    }
    LOG(INFO) << "Num nodes " << trajectory_nodes.size();

    // Output poses
    LOG(INFO) << "Outputting trajectory nodes to " << odom_out_file_name;
    file_io::writeTrajectory2dToFile(odom_out_file_name, trajectory_nodes);

    // Output timestamps
    file_io::writeNodeIdsAndTimestampsToFile(node_id_and_timestamp_file_name, nodes_with_timestamps);

    if (nodes_with_timestamps.size() != poses_rel_to_origin.size()) {
        LOG(INFO) << "Number of nodes doesn't match! Num timestamps: " << nodes_with_timestamps.size() << ", num poses "
                  << poses_rel_to_origin.size();
    }

    return 0;
}