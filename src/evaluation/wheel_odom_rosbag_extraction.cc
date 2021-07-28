#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <base_lib/pose_reps.h>

#include <file_io/object_positions_by_timestamp_io.h>
#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_timestamp_io.h>

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kRosbagFileNameParam = "rosbag_file_name";
const std::string kObjectDetectionFileParam = "object_detection_file";
const std::string kOdomOutputFileParam = "odom_out_file_name";
const std::string kNodeIdAndTimestampOutputFileParam = "node_id_and_timestamp_out_file";
const std::string kWaypointsByTimestampsFile = "waypoints_by_timestamps_file";

const double kMaxPoseIncThresholdTransl = 1.0;
const double kMaxPoseIncThresholdRot = 0.25; // TODO?

const double kPoseEquivTolerance = 1e2;

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
        return timestamp1.second < timestamp2.second;
    }
};

uint64_t timestampToMillis(const std::pair<uint32_t, uint32_t> &timestamp) {
    return timestamp.first * 1000 + (timestamp.second / 1e6);
}

bool posesSame(const pose::Pose2d &p1, const pose::Pose2d &p2) {
    pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(p1, p2);
    LOG(INFO) << p1.first.x() << ", " << p1.first.y() << ", " << p1.second;
    LOG(INFO) << p2.first.x() << ", " << p2.first.y() << ", " << p2.second;
    LOG(INFO) << rel_pose.first.norm();
    LOG(INFO) << (rel_pose.first.norm() == 0);
    LOG(INFO) << (rel_pose.second == 0);
    return ((rel_pose.first.norm() == 0) && (rel_pose.second == 0));
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
              node_prefix + "wheel_odom_rosbag_extractor");
    ros::NodeHandle n;

    std::string rosbag_file_name;
    std::string odom_out_file_name;
    std::string obj_file_name;
    std::string waypoints_by_timestamp_file_name;
    std::string node_id_and_timestamp_file_name;

    bool include_obj_times = false;
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
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kNodeIdAndTimestampOutputFileParam;
        exit(1);
    }

    if (n.getParam(param_prefix + kObjectDetectionFileParam, obj_file_name)) {
        include_obj_times = true;
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

    std::unordered_set<std::pair<uint32_t, uint32_t>, pair_hash> obj_and_waypoint_timestamps_set;
    if (include_obj_times) {
        LOG(INFO) << "Getting timestamps from file " << obj_file_name;
        std::vector<file_io::ObjectPositionByTimestamp> objs_by_timestamp;
        file_io::readObjectPositionsByTimestampFromFile(obj_file_name, objs_by_timestamp);

        for (const file_io::ObjectPositionByTimestamp &obj_with_timestamp : objs_by_timestamp) {
            obj_and_waypoint_timestamps_set.insert(
                    std::make_pair(obj_with_timestamp.seconds_, obj_with_timestamp.nano_seconds_));
        }
    }
    if (include_waypoint_times) {
        LOG(INFO) << "Getting waypoint times from file " << waypoints_by_timestamp_file_name;
        std::vector<file_io::WaypointAndTimestamp> waypoints_by_timestamp;
        file_io::readWaypointsAndTimestampsFromFile(waypoints_by_timestamp_file_name, waypoints_by_timestamp);

        for (const file_io::WaypointAndTimestamp &waypoint_with_timestamp : waypoints_by_timestamp) {
            obj_and_waypoint_timestamps_set.insert(
                    std::make_pair(waypoint_with_timestamp.seconds_, waypoint_with_timestamp.nano_seconds_));
        }
    }

    std::vector<std::pair<uint32_t, uint32_t>> sorted_obj_timestamps;
    sorted_obj_timestamps.insert(sorted_obj_timestamps.end(), obj_and_waypoint_timestamps_set.begin(),
                                 obj_and_waypoint_timestamps_set.end());
    std::sort(sorted_obj_timestamps.begin(), sorted_obj_timestamps.end(), timestamp_sort());

    LOG(INFO) << "Min obj timestamp " << sorted_obj_timestamps.front().first << ", "
              << sorted_obj_timestamps.front().second;
    LOG(INFO) << "Max obj timestamp " << sorted_obj_timestamps.back().first << ", "
              << sorted_obj_timestamps.back().second;
    for (const auto &obj_timestamp : sorted_obj_timestamps) {
        LOG(INFO) << "Obj timestamp " << obj_timestamp.first << ", " << obj_timestamp.second;
    }
    std::vector<pose::Pose2d> poses_to_use;
    std::vector<std::pair<uint32_t, uint32_t>> timestamps_to_use;
    poses_to_use.emplace_back(full_odom_frame_poses[0]);
    timestamps_to_use.emplace_back(full_timestamps[0]);
    size_t index_next_obj_timestamp_to_check = 0;
    if (!sorted_obj_timestamps.empty()) {
        if (timestamp_sort()(sorted_obj_timestamps[0], full_timestamps[0])) {
            index_next_obj_timestamp_to_check = 1;
        }
    }

    for (size_t i = 1; i < full_timestamps.size(); i++) {
        bool added_pose = false;

        pose::Pose2d curr_pose = full_odom_frame_poses[i];
        std::pair<uint32_t, uint32_t> curr_timestamp = full_timestamps[i];
        if (index_next_obj_timestamp_to_check < sorted_obj_timestamps.size()) {
            std::pair<uint32_t, uint32_t> next_obj_timestamp = sorted_obj_timestamps[index_next_obj_timestamp_to_check];
            if (timestamp_sort()(next_obj_timestamp, curr_timestamp)) {
                if ((next_obj_timestamp.first == curr_timestamp.first) &&
                    (next_obj_timestamp.second == curr_timestamp.second)) {
                    timestamps_to_use.emplace_back(curr_timestamp);
                    poses_to_use.emplace_back(full_odom_frame_poses[i]);
                } else {
                    pose::Pose2d prev_pose = full_odom_frame_poses[i - 1];
                    std::pair<uint32_t, uint32_t> prev_timestamp = full_timestamps[i - 1];

                    uint64_t curr_timestamp_millis = timestampToMillis(curr_timestamp);
                    uint64_t prev_timestamp_millis = timestampToMillis(prev_timestamp);
                    uint64_t next_obj_millis = timestampToMillis(next_obj_timestamp);

                    double fraction = ((double) (next_obj_millis - prev_timestamp_millis)) /
                                      (curr_timestamp_millis - prev_timestamp_millis);

                    // Need to interpolate
                    // TODO should we do this along the arc instead of a straight line?
                    pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose);
                    pose::Pose2d rel_pose_interpolated;

                    rel_pose_interpolated = pose::createPose2d(rel_pose.first.x() * fraction,
                                                               rel_pose.first.y() * fraction,
                                                               rel_pose.second * fraction);
                    if (abs(rel_pose.second) > 1e-10) {
                        double radius = sqrt(rel_pose.first.squaredNorm() / (2 * (1 - cos(rel_pose.second))));
                        double x = radius * sin(abs(rel_pose.second) * fraction);
                        double y = radius - (radius * cos(rel_pose.second * fraction));
                        if (rel_pose.second < 0) {
                            y = -y;
                        }
                        if (rel_pose.first.x() < 0) {
                            y = -y;
                            x = -x;
                        }
                        rel_pose_interpolated = pose::createPose2d(x, y, fraction * rel_pose.second);
                    }
                    pose::Pose2d rel_pose_interp_global = pose::combinePoses(prev_pose, rel_pose_interpolated);
                    timestamps_to_use.emplace_back(next_obj_timestamp);
                    poses_to_use.emplace_back(rel_pose_interp_global);
                }
                added_pose = true;
                index_next_obj_timestamp_to_check++;
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
    pose::Pose2d last_pose_diff = pose::getPoseOfObj1RelToObj2(last_pose, last_pose_in_used_list);
    if ((last_pose_diff.first.norm() > kPoseEquivTolerance) || (last_pose_diff.second > kPoseEquivTolerance)) {
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
        if (!posesSame(poses_rel_to_origin[i - 1], poses_rel_to_origin[i])) {
            LOG(INFO) << "Pose i " << i << " not same as previous node";
            deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[i]);
        } else {
            LOG(INFO) << "Pose i " << i << " same as previous node";
        }

        file_io::NodeIdAndTimestamp node_with_timestamp;
        node_with_timestamp.node_id_ = deduped_poses_rel_to_origin.size() - 1;
        LOG(INFO) << "Node id " << node_with_timestamp.node_id_;
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