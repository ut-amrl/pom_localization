#include <glog/logging.h>


#include <ros/ros.h>

#include <iostream>

#include <unordered_map>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <base_lib/pose_reps.h>

#include <file_io/object_positions_by_timestamp_io.h>
#include <unordered_set>

const std::string kRosbagFileNameParam = "rosbag_file_name";
const std::string kOdomOutputFileParam = "odom_out_file_name";
const std::string kObjectDetectionFileParam = "object_detection_file";

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

int main(int argc, char **argv) {

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    ros::init(argc, argv,
              "wheel_odom_rosbag_extractor");
    ros::NodeHandle n;

    std::string rosbag_file_name;
    std::string odom_out_file_name;
    std::string obj_file_name;

    bool include_obj_times = false;

    if (!n.getParam(kRosbagFileNameParam, rosbag_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kRosbagFileNameParam;
        exit(1);
    }

    if (!n.getParam(kOdomOutputFileParam, odom_out_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kOdomOutputFileParam;
        exit(1);
    }

    if (n.getParam(kObjectDetectionFileParam, obj_file_name)) {
        include_obj_times = true;
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
    std::unordered_set<std::pair<uint32_t, uint32_t>, pair_hash> obj_timestamps_set;
    if (include_obj_times) {
        LOG(INFO) << "Getting timestamps from file " << obj_file_name;
        std::vector<file_io::ObjectPositionByTimestamp> objs_by_timestamp;
        file_io::readObjectPositionsByTimestampFromFile(obj_file_name, objs_by_timestamp);

        for (const file_io::ObjectPositionByTimestamp &obj_with_timestamp : objs_by_timestamp) {
            obj_timestamps_set.insert(std::make_pair(obj_with_timestamp.seconds_, obj_with_timestamp.nano_seconds_));
        }
    }

    std::vector<std::pair<uint32_t, uint32_t>> sorted_obj_timestamps;
    sorted_obj_timestamps.insert(sorted_obj_timestamps.end(), obj_timestamps_set.begin(), obj_timestamps_set.end());
    std::sort(sorted_obj_timestamps.begin(), sorted_obj_timestamps.end(), timestamp_sort());

    std::vector<pose::Pose2d> poses_to_use;
    poses_to_use.emplace_back(full_odom_frame_poses[0]);
    size_t index_next_obj_timestamp_to_check = 0;
    if (!sorted_obj_timestamps.empty()) {
        if (timestamp_sort()(sorted_obj_timestamps[0], full_timestamps[0])) {
            index_next_obj_timestamp_to_check = 1;
        }
    }

    std::vector<std::pair<uint32_t, uint32_t>> timestamps_to_use;
    for (size_t i = 1; i < full_timestamps.size(); i++) {
        bool added_pose = false;

        pose::Pose2d curr_pose = full_odom_frame_poses[i];
        if (index_next_obj_timestamp_to_check < sorted_obj_timestamps.size()) {
            std::pair<uint32_t, uint32_t> curr_timestamp = full_timestamps[i];
            std::pair<uint32_t, uint32_t> next_obj_timestamp = sorted_obj_timestamps[index_next_obj_timestamp_to_check];
            if (!timestamp_sort()(next_obj_timestamp, curr_timestamp)) {
                if ((next_obj_timestamp.first == curr_timestamp.first) &&
                    (next_obj_timestamp.second == curr_timestamp.second)) {
                    poses_to_use.emplace_back(full_odom_frame_poses[i]);
                } else {
                    pose::Pose2d prev_pose = full_odom_frame_poses[i - 1];
                    std::pair<uint32_t, uint32_t> prev_timestamp = full_timestamps[i - 1];

                    uint64_t curr_timestamp_millis = curr_timestamp.first * 1000;
                    curr_timestamp_millis += (curr_timestamp.second / 1e6);

                    uint64_t prev_timestamp_millis = prev_timestamp.first * 1000;
                    prev_timestamp_millis += (prev_timestamp.second / 1e6);

                    uint64_t next_obj_millis = next_obj_timestamp.first * 1000;
                    next_obj_millis += (next_obj_timestamp.second / 1e6);

                    double fraction = ((double) (next_obj_millis - prev_timestamp_millis)) /
                                      (curr_timestamp_millis - prev_timestamp_millis);

                    // Need to interpolate
                    pose::Pose2d rel_pose = pose::getPoseOfObj1RelToObj2(curr_pose, prev_pose);
                    pose::Pose2d rel_pose_interpolated = pose::createPose2d(rel_pose.first.x() * fraction,
                                                                            rel_pose.first.y() * fraction,
                                                                            rel_pose.second * fraction);
                    poses_to_use.emplace_back(pose::combinePoses(prev_pose, rel_pose_interpolated));
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

            }
        }
    }
    pose::Pose2d last_pose_in_used_list = poses_to_use.back();
    pose::Pose2d last_pose = full_odom_frame_poses.back();
    pose::Pose2d last_pose_diff = pose::getPoseOfObj1RelToObj2(last_pose, last_pose_in_used_list);
    if ((last_pose_diff.first.norm() > kPoseEquivTolerance) || (last_pose_diff.second > kPoseEquivTolerance)) {
        poses_to_use.emplace_back(last_pose);
    }

    std::vector<pose::Pose2d> relative_poses;
    relative_poses.emplace_back(pose::createPose2d(0, 0, 0));
    pose::Pose2d first_pose = poses_to_use[0];
    for (size_t i = 1; i < poses_to_use.size(); i++) {
        pose::Pose2d curr_pose = poses_to_use[i];
        relative_poses.emplace_back(pose::getPoseOfObj1RelToObj2(curr_pose, first_pose));
    }

    bag.close();


    // Output poses
    // Output timestamps



//    auto t = std::time(nullptr);
//    auto tm = *std::localtime(&t);
//    std::ostringstream oss;;
//    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
//    std::string time_str = oss.str();
//    std::string csv_file_name = "results/noise_eval_" + time_str + ".csv";

    return 0;
}