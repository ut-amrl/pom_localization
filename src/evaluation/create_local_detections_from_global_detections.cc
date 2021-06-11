#include <glog/logging.h>

#include <file_io/trajectory_2d_io.h>

#include <file_io/object_instances_observed_at_poses_io.h>
#include <ros/ros.h>
#include <iostream>
#include <base_lib/pose_reps.h>
#include <file_io/object_positions_by_pose_io.h>

using namespace pose;

const std::string kTrajectory2dFileParamName = "trajectory_2d_file";
const std::string kParkingSpotsFileParamName = "parking_spots_file";
const std::string kDetectionsObservedAtFile = "detections_observations_file";
const std::string kLocalDetectionsOutputFile = "local_detections_output_file";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "create_local_detections_from_global_detections");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string trajectory_2d_file;
    std::string global_detections_file;
    std::string detections_observed_at_file;
    std::string local_detections_output_file;

    if (!n.getParam(kLocalDetectionsOutputFile, local_detections_output_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kLocalDetectionsOutputFile;
        exit(1);
    }

    if (!n.getParam(kDetectionsObservedAtFile, detections_observed_at_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kDetectionsObservedAtFile;
        exit(1);
    }

    if (!n.getParam(kTrajectory2dFileParamName, trajectory_2d_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kTrajectory2dFileParamName;
        exit(1);
    }

    if (!n.getParam(kParkingSpotsFileParamName, global_detections_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kParkingSpotsFileParamName;
        exit(1);
    }

    std::vector<file_io::TrajectoryNode2d> trajectory_2d;
    file_io::readRawTrajectory2dFromFile(trajectory_2d_file, trajectory_2d);

    std::vector<pose::Pose2d> base_trajectory;
    for (const file_io::TrajectoryNode2d &trajectory_entry : trajectory_2d) {
        // Assumes that we don't need the node numbers because they are in order
        base_trajectory.emplace_back(
                pose::createPose2d(trajectory_entry.transl_x_, trajectory_entry.transl_y_, trajectory_entry.theta_));
    }

    std::vector<file_io::ObjectPositionByPose> object_positions_by_pose;
    file_io::readObjectPositionsByPoseFromFile(global_detections_file, object_positions_by_pose);

    std::unordered_map<uint64_t, pose::Pose2d> parking_spots_global_frame;
    for (const file_io::ObjectPositionByPose &obj_pos_rel_scan : object_positions_by_pose) {
        pose::Pose2d scan_pose = base_trajectory[obj_pos_rel_scan.pose_number_];
        pose::Pose2d global_pose_for_inst = pose::combinePoses(scan_pose, pose::createPose2d(obj_pos_rel_scan.transl_x_,
                                                                                             obj_pos_rel_scan.transl_y_,
                                                                                             obj_pos_rel_scan.theta_));

        if (parking_spots_global_frame.find(obj_pos_rel_scan.identifier_) != parking_spots_global_frame.end()) {
            LOG(WARNING) << "Found multiple instances of object with identifier " << obj_pos_rel_scan.identifier_;
        }
        parking_spots_global_frame[obj_pos_rel_scan.identifier_] = global_pose_for_inst;
    }

    std::unordered_map<uint64_t, std::vector<pose_graph::NodeId>> observed_at_poses = file_io::readInstancesObservedAtPosesFromFile(
            detections_observed_at_file);

    std::vector<file_io::ObjectPositionByPose> local_detections;
    for (const auto &instance_id_and_pose : parking_spots_global_frame) {
        for (const pose_graph::NodeId &observed_at_node : observed_at_poses[instance_id_and_pose.first]) {
            pose::Pose2d node_pose = base_trajectory[observed_at_node];
            pose::Pose2d relative_obj_pose = pose::getPoseOfObj1RelToObj2(instance_id_and_pose.second, node_pose);
            file_io::ObjectPositionByPose object_position_by_pose;
            object_position_by_pose.pose_number_ = observed_at_node;
            object_position_by_pose.identifier_ = instance_id_and_pose.first;
            object_position_by_pose.transl_x_ = relative_obj_pose.first.x();
            object_position_by_pose.transl_y_ = relative_obj_pose.first.y();
            object_position_by_pose.theta_ = relative_obj_pose.second;

            local_detections.emplace_back(object_position_by_pose);
        }
    }

    file_io::writeObjectPositionsByPoseToFile(local_detections_output_file, local_detections);

    return 0;
}