#include <glog/logging.h>

#include <file_io/lidar_odom.h>
#include <file_io/trajectory_2d_io.h>

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <ctime>

#include <base_lib/pose_reps.h>

using namespace pose;

const std::string kLidarOdomEstFileParamName = "lidar_odom_estimate_file";
const std::string kTrajectory2dOutputFileParamName = "output_file_name";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "lidar_odom_to_2d_trajectory");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string lidar_odom_trajectory_file_name;
    std::string trajectory_2d_output_file_name;

    if (!n.getParam(kLidarOdomEstFileParamName, lidar_odom_trajectory_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kLidarOdomEstFileParamName;
        exit(1);
    }

    if (!n.getParam(kTrajectory2dOutputFileParamName, trajectory_2d_output_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kTrajectory2dOutputFileParamName;
        exit(1);
    }

    std::vector<file_io::RawAbsoluteLidarOdomNode3D> lidar_odom_entries;
    file_io::readRawLidarOdomFromFile(lidar_odom_trajectory_file_name, lidar_odom_entries);

    std::vector<pose::Pose3d> read_poses;
    for (const file_io::RawAbsoluteLidarOdomNode3D &lidar_odom : lidar_odom_entries) {
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

    std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
    pose::Pose2d prev_pose_global_frame = pose::createPose2d(0, 0, 0);
    for (size_t node_num = 0; node_num < relative_poses_2d.size(); node_num++) {
        file_io::TrajectoryNode2d trajectory_node_2d;
        trajectory_node_2d.node_id_ = node_num;
        pose::Pose2d rel_motion_2d = relative_poses_2d[node_num];
        pose::Pose2d global_est = pose::combinePoses(prev_pose_global_frame, rel_motion_2d);
        prev_pose_global_frame = global_est;

        trajectory_node_2d.transl_x_ = global_est.first.x();
        trajectory_node_2d.transl_y_ = global_est.first.y();
        trajectory_node_2d.theta_ = global_est.second;

        trajectory_nodes.emplace_back(trajectory_node_2d);
    }

    file_io::writeTrajectory2dToFile(trajectory_2d_output_file_name, trajectory_nodes);

    return 0;
}