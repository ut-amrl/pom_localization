#include <iostream>
#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <base_lib/pose_reps.h>

#include <file_io/node_id_and_timestamp_io.h>
#include <file_io/trajectory_2d_io.h>
#include <file_io/waypoints_and_timestamp_io.h>
#include <file_io/trajectory_2d_by_timestamp.h>

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kTrajectoryEstimateParam = "trajectory_estimate_2d";
const std::string kOdomOutputFileParam = "odom_out_file_name";
const std::string kNodeIdAndTimestampOutputFileParam = "node_id_and_timestamp_file";
const std::string kWaypointsByTimestampsFile = "waypoints_by_timestamps_file";
const std::string kRosbagFileParamName = "bag_file_name";

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
              node_prefix + "interpolate_nodes");
    ros::NodeHandle n;

    std::string trajectory_estimate_file;
    std::string odom_out_file_name;
    std::string waypoints_by_timestamp_file_name;
    std::string node_id_and_timestamp_file_name;
    std::string rosbag_file_name;

    if (!n.getParam(param_prefix + kTrajectoryEstimateParam, trajectory_estimate_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kTrajectoryEstimateParam;
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

    if (!n.getParam(param_prefix + kWaypointsByTimestampsFile, waypoints_by_timestamp_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kWaypointsByTimestampsFile;
        exit(1);
    }

    if (!n.getParam(param_prefix + kRosbagFileParamName, rosbag_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kRosbagFileParamName;
        exit(1);
    }

    std::vector<pose::Pose2d> trajectory_est_poses;
    std::vector<std::pair<uint32_t, uint32_t>> trajectory_est_timestamps;

    std::vector<file_io::TrajectoryNode2dWithTimestamp> trajectory_with_timestamps;
    file_io::readRawTrajectory2dWithTimestampFromFile(trajectory_estimate_file, trajectory_with_timestamps);

    for (const file_io::TrajectoryNode2dWithTimestamp &trajectory_node : trajectory_with_timestamps) {
        trajectory_est_poses.emplace_back(
                pose::createPose2d(trajectory_node.transl_x_, trajectory_node.transl_y_, trajectory_node.theta_));
        trajectory_est_timestamps.emplace_back(std::make_pair(trajectory_node.seconds_, trajectory_node.nano_seconds_));
    }

    std::unordered_set<std::pair<uint32_t, uint32_t>, pair_hash> waypoint_timestamps_set;
    LOG(INFO) << "Getting waypoint times from file " << waypoints_by_timestamp_file_name;
    std::vector<file_io::WaypointAndTimestamp> waypoints_by_timestamp;
    LOG(INFO) << "Waypoint file " << waypoints_by_timestamp_file_name;
    file_io::readWaypointsAndTimestampsFromFile(waypoints_by_timestamp_file_name, waypoints_by_timestamp);
    LOG(INFO) << "Num waypoints " << waypoints_by_timestamp.size();

    for (const file_io::WaypointAndTimestamp &waypoint_with_timestamp : waypoints_by_timestamp) {
        waypoint_timestamps_set.insert(
                std::make_pair(waypoint_with_timestamp.seconds_, waypoint_with_timestamp.nano_seconds_));
    }

    std::vector<std::pair<uint32_t, uint32_t>> sorted_waypoint_timestamps;
    sorted_waypoint_timestamps.insert(sorted_waypoint_timestamps.end(), waypoint_timestamps_set.begin(),
                                      waypoint_timestamps_set.end());
    std::sort(sorted_waypoint_timestamps.begin(), sorted_waypoint_timestamps.end(), timestamp_sort());

    std::vector<pose::Pose2d> poses_to_use;
    std::vector<std::pair<uint32_t, uint32_t>> timestamps_to_use;

    size_t index_next_waypoint_timestamp_to_check = 0;
    if (!sorted_waypoint_timestamps.empty()) {
        // If the first waypoint timestamp is greater than the first trajectory node timestamp
        if (timestamp_sort()(sorted_waypoint_timestamps[0], trajectory_est_timestamps[0])) {
            index_next_waypoint_timestamp_to_check = 1;
            poses_to_use.emplace_back(trajectory_est_poses[0]);
            timestamps_to_use.emplace_back(sorted_waypoint_timestamps[0]);
        }
    }

    poses_to_use.emplace_back(trajectory_est_poses[0]);
    timestamps_to_use.emplace_back(trajectory_est_timestamps[0]);

    for (size_t i = 1; i < trajectory_est_timestamps.size(); i++) {
        pose::Pose2d curr_pose = trajectory_est_poses[i];

        std::pair<uint32_t, uint32_t> curr_timestamp = trajectory_est_timestamps[i];
        if (index_next_waypoint_timestamp_to_check < sorted_waypoint_timestamps.size()) {
            std::pair<uint32_t, uint32_t> next_waypoint_timestamp = sorted_waypoint_timestamps[index_next_waypoint_timestamp_to_check];
            while ((index_next_waypoint_timestamp_to_check < sorted_waypoint_timestamps.size()) &&
                   (timestamp_sort()(next_waypoint_timestamp, curr_timestamp))) {
                if ((next_waypoint_timestamp.first == curr_timestamp.first) &&
                    (next_waypoint_timestamp.second == curr_timestamp.second)) {

                    LOG(INFO) << "Adding pose for waypoint num " << index_next_waypoint_timestamp_to_check;
                    timestamps_to_use.emplace_back(next_waypoint_timestamp);
                    poses_to_use.emplace_back(trajectory_est_poses[i]);
                } else {
                    pose::Pose2d prev_pose = trajectory_est_poses[i - 1];
                    std::pair<uint32_t, uint32_t> prev_timestamp = trajectory_est_timestamps[i - 1];

                    uint64_t curr_timestamp_millis = timestampToMillis(curr_timestamp);
                    uint64_t prev_timestamp_millis = timestampToMillis(prev_timestamp);
                    uint64_t next_waypoint_millis = timestampToMillis(next_waypoint_timestamp);

                    double fraction = ((double) (next_waypoint_millis - prev_timestamp_millis)) /
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
                    LOG(INFO) << "Adding pose for waypoint num " << index_next_waypoint_timestamp_to_check;
                    pose::Pose2d rel_pose_interp_global = pose::combinePoses(prev_pose, rel_pose_interpolated);
                    timestamps_to_use.emplace_back(next_waypoint_timestamp);
                    poses_to_use.emplace_back(rel_pose_interp_global);
                }
                index_next_waypoint_timestamp_to_check++;
                next_waypoint_timestamp = sorted_waypoint_timestamps[index_next_waypoint_timestamp_to_check];
            }
        }
        poses_to_use.emplace_back(curr_pose);
        timestamps_to_use.emplace_back(curr_timestamp);
    }

    if (index_next_waypoint_timestamp_to_check < sorted_waypoint_timestamps.size()) {
        // TODO should we use wheel odom to extend the trajectory
        LOG(INFO) << "Could not interpolate waypoint timestamp " << index_next_waypoint_timestamp_to_check
                  << ", total waypoints " << sorted_waypoint_timestamps.size();
        LOG(INFO) << "Attempting to interpolate with odom";

        rosbag::Bag bag;
        bag.open(rosbag_file_name, rosbag::bagmode::Read);

        std::vector<std::string> topics = {"/jackal_velocity_controller/odom"};

        rosbag::View view(bag, rosbag::TopicQuery(topics));

        std::vector<pose::Pose2d> odom_poses;
        std::vector<std::pair<uint32_t, uint32_t>> odom_timestamps;

        for (rosbag::MessageInstance const &m : view) {
            nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
            if (!odom_timestamps.empty()) {
                std::pair<uint32_t, uint32_t> prev_stamp = odom_timestamps.back();
                if ((prev_stamp.first > msg->header.stamp.sec) ||
                    ((prev_stamp.first == msg->header.stamp.sec) && (prev_stamp.second > msg->header.stamp.nsec))) {
                    LOG(INFO) << "Out of order messages!";
                }
            }
            odom_timestamps.emplace_back(std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec));
            pose::Pose3d odom_3d = std::make_pair(
                    Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                    Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

            pose::Pose2d pose = pose::toPose2d(odom_3d);
            odom_poses.emplace_back(pose);
        }

        std::pair<uint32_t, uint32_t> last_timestamp = timestamps_to_use.back();
        pose::Pose2d accumulated_pose_since_last_node;
        bool found_odom_pose_after_last_timestamp = false;
        for (size_t i = 1; i < odom_poses.size(); i++) {
            std::pair<uint32_t, uint32_t> curr_timestamp = odom_timestamps[i];
            if (!found_odom_pose_after_last_timestamp) {
                if (timestamp_sort()(last_timestamp, curr_timestamp)) {
                    found_odom_pose_after_last_timestamp = true;
//                    LOG(INFO) << "Last timestamp " << last_timestamp.first << ", " << last_timestamp.second;
//                    LOG(INFO) << "Curr timestamp " << curr_timestamp.first << ", " << curr_timestamp.second;
                    if ((last_timestamp.first == curr_timestamp.first) &&
                        (last_timestamp.second == curr_timestamp.second)) {
                        accumulated_pose_since_last_node = pose::createPose2d(0, 0, 0);
                    } else {
                        pose::Pose2d prev_pose = odom_poses[i - 1];
                        std::pair<uint32_t, uint32_t> prev_timestamp = odom_timestamps[i - 1];

                        uint64_t curr_timestamp_millis = timestampToMillis(curr_timestamp);
                        uint64_t prev_timestamp_millis = timestampToMillis(prev_timestamp);
                        uint64_t next_waypoint_millis = timestampToMillis(last_timestamp);

                        double fraction = ((double) (curr_timestamp_millis - next_waypoint_millis)) /
                                          (curr_timestamp_millis - prev_timestamp_millis);
//                        LOG(INFO) << "Fraction " << fraction;

                        // Need to interpolate
                        // TODO should we do this along the arc instead of a straight line?
                        pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(odom_poses[i], prev_pose);
//                        LOG(INFO) << "Rel pose " << rel_pose.first.x() << ", " << rel_pose.first.y() << ", " << rel_pose.second;
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
                        accumulated_pose_since_last_node = rel_pose_interpolated;
//                        LOG(INFO) << "Accumulated pose since last node " << accumulated_pose_since_last_node.first.x() << ", " << accumulated_pose_since_last_node.first.y() << ", " << accumulated_pose_since_last_node.second;
                    }
                }
            } else {
                if (index_next_waypoint_timestamp_to_check >= sorted_waypoint_timestamps.size()) {
                    break;
                }
                std::pair<uint32_t, uint32_t> next_waypoint_timestamp = sorted_waypoint_timestamps[index_next_waypoint_timestamp_to_check];

                pose::Pose2d prev_pose = odom_poses[i - 1];
                pose::Pose2d curr_pose = odom_poses[i];
                pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose);

//                LOG(INFO) << "Rel pose " << rel_pose.first.x() << ", " << rel_pose.first.y() << ", " << rel_pose.second;
                bool made_new_node = false;
                while ((index_next_waypoint_timestamp_to_check < sorted_waypoint_timestamps.size()) &&
                       (timestamp_sort()(next_waypoint_timestamp, curr_timestamp))) {
                    if ((next_waypoint_timestamp.first == curr_timestamp.first) &&
                        (next_waypoint_timestamp.second == curr_timestamp.second)) {

//                        LOG(INFO) << "Adding pose for waypoint num " << index_next_waypoint_timestamp_to_check;
                        timestamps_to_use.emplace_back(next_waypoint_timestamp);
                        poses_to_use.emplace_back(pose::combinePoses(poses_to_use.back(), pose::combinePoses(
                                accumulated_pose_since_last_node, rel_pose)));
                        accumulated_pose_since_last_node = pose::createPose2d(0, 0, 0);
                    } else {
                        std::pair<uint32_t, uint32_t> prev_timestamp = odom_timestamps[i - 1];

                        uint64_t curr_timestamp_millis = timestampToMillis(curr_timestamp);
                        uint64_t prev_timestamp_millis = timestampToMillis(prev_timestamp);
                        uint64_t next_waypoint_millis = timestampToMillis(next_waypoint_timestamp);

                        double fraction = ((double) (next_waypoint_millis - prev_timestamp_millis)) /
                                          (curr_timestamp_millis - prev_timestamp_millis);
//                        LOG(INFO) << "Fraction " << fraction;

                        // Need to interpolate
                        // TODO should we do this along the arc instead of a straight line?
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

//                        LOG(INFO) << "Rel pose interpolated " << rel_pose.first.x() << ", " << rel_pose.first.y() << ", " << rel_pose.second;
                        accumulated_pose_since_last_node = pose::combinePoses(accumulated_pose_since_last_node,
                                                                              rel_pose_interpolated);
                        timestamps_to_use.emplace_back(next_waypoint_timestamp);

//                        LOG(INFO) << "Accumulated pose since last node " << accumulated_pose_since_last_node.first.x() << ", " << accumulated_pose_since_last_node.first.y() << ", " << accumulated_pose_since_last_node.second;
                        poses_to_use.emplace_back(
                                pose::combinePoses(poses_to_use.back(), accumulated_pose_since_last_node));
                        double remaining_fraction = ((double) (curr_timestamp_millis - next_waypoint_millis)) /
                                                    (curr_timestamp_millis - prev_timestamp_millis);

                        accumulated_pose_since_last_node = pose::createPose2d(rel_pose.first.x() * remaining_fraction,
                                                                              rel_pose.first.y() * remaining_fraction,
                                                                              rel_pose.second * remaining_fraction);
                        if (abs(rel_pose.second) > 1e-10) {
                            double radius = sqrt(rel_pose.first.squaredNorm() / (2 * (1 - cos(rel_pose.second))));
                            double x = radius * sin(abs(rel_pose.second) * remaining_fraction);
                            double y = radius - (radius * cos(rel_pose.second * remaining_fraction));
                            if (rel_pose.second < 0) {
                                y = -y;
                            }
                            if (rel_pose.first.x() < 0) {
                                y = -y;
                                x = -x;
                            }
                            accumulated_pose_since_last_node = pose::createPose2d(x, y,
                                                                                  remaining_fraction * rel_pose.second);
                        }
                    }
                    made_new_node = true;
                    index_next_waypoint_timestamp_to_check++;
                    next_waypoint_timestamp = sorted_waypoint_timestamps[index_next_waypoint_timestamp_to_check];
                }
                if (!made_new_node) {
                    accumulated_pose_since_last_node = pose::combinePoses(accumulated_pose_since_last_node, rel_pose);
                }
            }
        }
    }

    if (index_next_waypoint_timestamp_to_check < sorted_waypoint_timestamps.size()) {
        LOG(INFO) << "Couldn't interpolate using odom even, next waypoint index " << index_next_waypoint_timestamp_to_check << ", total waypoint timestamps " << sorted_waypoint_timestamps.size();
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
            deduped_poses_rel_to_origin.emplace_back(poses_rel_to_origin[i]);
        }

        file_io::NodeIdAndTimestamp node_with_timestamp;
        node_with_timestamp.node_id_ = deduped_poses_rel_to_origin.size() - 1;
        node_with_timestamp.seconds_ = timestamps_to_use[i].first;
        node_with_timestamp.nano_seconds_ = timestamps_to_use[i].second;
        nodes_with_timestamps.emplace_back(node_with_timestamp);
    }

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