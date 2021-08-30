#include <glog/logging.h>

#include <file_io/lidar_odom.h>
#include <file_io/trajectory_2d_by_timestamp.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sensor_msgs/PointCloud2.h>

#include <base_lib/pose_reps.h>

using namespace pose;

const std::string kLidarOdomEstFileParamName = "lidar_odom_estimate_file";
const std::string kTrajectory2dOutputFileParamName = "trajectory_estimate_2d";
const std::string kBagFileParamName = "bag_file_name";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "lidar_odom_to_2d_trajectory");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string lidar_odom_trajectory_file_name;
    std::string trajectory_2d_output_file_name;
    std::string rosbag_file_name;

    if (!n.getParam(kLidarOdomEstFileParamName, lidar_odom_trajectory_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kLidarOdomEstFileParamName;
        exit(1);
    }

    if (!n.getParam(kTrajectory2dOutputFileParamName, trajectory_2d_output_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kTrajectory2dOutputFileParamName;
        exit(1);
    }

    if (!n.getParam(kBagFileParamName, rosbag_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kBagFileParamName;
        exit(1);
    }

    rosbag::Bag bag;
    bag.open(rosbag_file_name, rosbag::bagmode::Read);

    std::vector<std::string> topics = {"/velodyne_points"};

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    std::vector<pose::Pose2d> full_odom_frame_poses;
    std::vector<std::pair<uint32_t, uint32_t>> full_timestamps;

    std::pair<uint32_t, uint32_t> first_timestamp;
    bool first = true;
    for (rosbag::MessageInstance const &m : view) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (first) {
            first_timestamp = std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
            first = false;
        } else if ((first_timestamp.first > msg->header.stamp.sec) ||
            ((first_timestamp.first == msg->header.stamp.sec) && (first_timestamp.second > msg->header.stamp.nsec))) {
            first_timestamp = std::make_pair(msg->header.stamp.sec, msg->header.stamp.nsec);
        }
    }

    std::vector<file_io::RawAbsoluteLidarOdomNode3D> lidar_odom_entries;
    file_io::readRawLidarOdomFromFile(lidar_odom_trajectory_file_name, lidar_odom_entries);

    std::vector<pose::Pose3d> read_poses;
    std::vector<std::pair<uint32_t, uint32_t>> timestamps;
    timestamps.emplace_back(first_timestamp);
    for (const file_io::RawAbsoluteLidarOdomNode3D &lidar_odom : lidar_odom_entries) {
        ros::Time node_time(lidar_odom.timestamp_);
        timestamps.emplace_back(node_time.sec, node_time.nsec);
        read_poses.emplace_back(std::make_pair(Eigen::Vector3d(lidar_odom.transl_x_, lidar_odom.transl_y_, lidar_odom.transl_z_),
                                               Eigen::Quaterniond(lidar_odom.quat_w_, lidar_odom.quat_x_, lidar_odom.quat_y_, lidar_odom.quat_z_)));
    }

    std::vector<pose::Pose3d> relative_poses_3d;
    relative_poses_3d.emplace_back(std::make_pair(Eigen::Vector3d(), Eigen::Quaterniond(1, 0, 0, 0)));

    pose::Pose3d prev_pose = std::make_pair(Eigen::Vector3d(), Eigen::Quaterniond(1, 0, 0, 0));

    for (size_t i = 0; i < read_poses.size(); i++) {
        pose::Pose3d curr_pose = read_poses[i];
        pose::Pose3d relative_motion = pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose);

        relative_poses_3d.emplace_back(relative_motion);
        prev_pose = curr_pose;
    }

    std::vector<pose::Pose2d> relative_poses_2d;
    for (const pose::Pose3d &relative_pose_3d : relative_poses_3d) {
        relative_poses_2d.emplace_back(pose::toPose2d(relative_pose_3d));
    }

    LOG(INFO) << "Trajectory nodes size " << relative_poses_2d.size();
    LOG(INFO) << "Timestamps size " << timestamps.size();

    std::vector<file_io::TrajectoryNode2dWithTimestamp> trajectory_nodes;
    pose::Pose2d prev_pose_global_frame = pose::createPose2d(0, 0, 0);
    for (size_t node_num = 0; node_num < relative_poses_2d.size(); node_num++) {
        file_io::TrajectoryNode2dWithTimestamp trajectory_node_2d;
        std::pair<uint32_t, uint32_t> timestamp = timestamps[node_num];
        trajectory_node_2d.seconds_ = timestamp.first;
        trajectory_node_2d.nano_seconds_ = timestamp.second;
        pose::Pose2d rel_motion_2d = relative_poses_2d[node_num];
        pose::Pose2d global_est = pose::combinePoses(prev_pose_global_frame, rel_motion_2d);
        prev_pose_global_frame = global_est;

        trajectory_node_2d.transl_x_ = global_est.first.x();
        trajectory_node_2d.transl_y_ = global_est.first.y();
        trajectory_node_2d.theta_ = global_est.second;

        trajectory_nodes.emplace_back(trajectory_node_2d);
    }

    file_io::writeTrajectory2dWithTimestampsToFile(trajectory_2d_output_file_name, trajectory_nodes);

    return 0;
}