#include <glog/logging.h>

#include <file_io/kitti_poses_io.h>
#include <file_io/trajectory_2d_io.h>

#include <ros/ros.h>

#include <iostream>
#include <ctime>

#include <base_lib/pose_reps.h>
#include <file_io/kitti_calib_io.h>

using namespace pose;

const std::string kKittiFormattedEstFileParamName = "kitti_formatted_estimate";
const std::string kTrajectory2dOutputFileParamName = "output_file_name";
const std::string kKittiCalibFile = "kitti_calib_file";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "kitti_to_2d_trajectory");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string parking_spot_file_name;

    std::string kitti_formatted_trajectory_file_name;
    std::string trajectory_2d_output_file_name;
    std::string kitti_calib_file;

    if (!n.getParam(kKittiFormattedEstFileParamName, kitti_formatted_trajectory_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kKittiFormattedEstFileParamName;
        exit(1);
    }

    if (!n.getParam(kTrajectory2dOutputFileParamName, trajectory_2d_output_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kTrajectory2dOutputFileParamName;
        exit(1);
    }

    if(!n.getParam(kKittiCalibFile, kitti_calib_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kKittiCalibFile;
        exit(1);
    }

    std::vector<file_io::KittiFormattedPose> kitti_poses;
    file_io::readRawKittiPosesFromFile(kitti_formatted_trajectory_file_name, kitti_poses);

    Eigen::Affine3d velodyne_rel_camera_tf = file_io::getVelodyneToCameraTf(kitti_calib_file);

    std::vector<pose::Pose3d> velodyne_frame_poses;
    for (const file_io::KittiFormattedPose &kitti_pose : kitti_poses) {
        Eigen::Affine3d velodyne_frame_at_time_x_rel_map = kitti_pose.relative_pose * velodyne_rel_camera_tf;
        Eigen::Quaterniond quaternion(velodyne_frame_at_time_x_rel_map.linear());
        velodyne_frame_poses.emplace_back(std::make_pair(velodyne_frame_at_time_x_rel_map.translation(), quaternion));
    }

    std::vector<pose::Pose3d> relative_poses_3d;
    relative_poses_3d.emplace_back(std::make_pair(Eigen::Vector3d(), Eigen::Quaterniond(1, 0, 0, 0)));

    pose::Pose3d prev_pose;
    prev_pose = velodyne_frame_poses[0];

    for (size_t i = 1; i < velodyne_frame_poses.size(); i++) {
        pose::Pose3d curr_pose = velodyne_frame_poses[i];
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