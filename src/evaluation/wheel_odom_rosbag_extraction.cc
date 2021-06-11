#include <glog/logging.h>


#include <ros/ros.h>

#include <iostream>

#include <unordered_map>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <base_lib/pose_reps.h>

const std::string kRosbagFileNameParam = "rosbag_file_name";
const std::string kOdomOutputFileParam = "odom_out_file_name";
const std::string kTimestampFileParam = "timestamp_file_name";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "movable_obj_trajectory_estimation");
    ros::NodeHandle n;

    std::string rosbag_file_name;
    std::string odom_out_file_name;
    std::string timestamp_file_name;

    bool use_specified_timestamps = false;

    if (!n.getParam(kRosbagFileNameParam, rosbag_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kRosbagFileNameParam;
        exit(1);
    }

    if (!n.getParam(kOdomOutputFileParam, odom_out_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kOdomOutputFileParam;
        exit(1);
    }

    if (n.getParam(kTimestampFileParam, timestamp_file_name)) {
        use_specified_timestamps = true;
    }

    google::InitGoogleLogging(argv[0]);

    FLAGS_logtostderr = true;


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
    std::vector<pose::Pose2d> poses_to_use;
    std::vector<std::pair<uint32_t, uint32_t>> timestamps_to_use;
    if (use_specified_timestamps) {
        LOG(INFO) << "Getting timestamps from file " << timestamp_file_name;
    } else {
        poses_to_use = full_odom_frame_poses;
        timestamps_to_use = full_timestamps;
    }

    std::vector<pose::Pose2d> relative_poses;
    relative_poses.emplace_back(pose::createPose2d(0, 0, 0));
    pose::Pose2d prev_pose = poses_to_use[0];
    for (size_t i = 1; i < poses_to_use.size(); i++) {
        pose::Pose2d curr_pose = poses_to_use[i];
        relative_poses.emplace_back(pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose));
        prev_pose = curr_pose;
    }

    bag.close();


//    auto t = std::time(nullptr);
//    auto tm = *std::localtime(&t);
//    std::ostringstream oss;;
//    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
//    std::string time_str = oss.str();
//    std::string csv_file_name = "results/noise_eval_" + time_str + ".csv";

    return 0;
}