#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <ctime>

#include <file_io/past_sample_io.h>
#include <pose_optimization/offline/offline_problem_runner.h>
#include <pose_optimization/utils/pose_graph_creation_utils.h>
#include <pose_optimization/offline/ceres_visualization_callback_2d.h>

#include <file_io/trajectory_2d_io.h>

#include <unordered_map>
#include <file_io/object_positions_by_pose_io.h>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kPastSamplesFilesParamName = "past_samples_files";

const std::string kOdomTrajectoryEstimatesFileParamName = "odom_traj_est_file";

const std::string kObjDetectionsCurrTrajectoryFileParamName = "obj_det_curr_traj_file";

const std::string kEstTrajFileParamName = "est_traj_file";

const std::string kGtTrajectoryFile = "gt_trajectory_file";

std::vector<std::vector<pose::Pose2d>> getObjDetections(
        const std::string &object_detections_file_name,
        const pose_graph::NodeId &num_nodes) {

    std::vector<file_io::ObjectPositionByPose> raw_obj_detections;
    file_io::readObjectPositionsByPoseFromFile(object_detections_file_name, raw_obj_detections);

    std::vector<std::vector<pose::Pose2d>> noisy_observations_for_class;
    for (size_t node_num = 0; node_num < num_nodes; node_num++) {
        noisy_observations_for_class.push_back({});
    }

    for (const file_io::ObjectPositionByPose &obs : raw_obj_detections) {
        pose_graph::NodeId observed_at_node = obs.pose_number_;

        noisy_observations_for_class[observed_at_node].emplace_back(
                pose::createPose2d(obs.transl_x_, obs.transl_y_, obs.theta_));
    }

    return noisy_observations_for_class;
}

std::vector<pose::Pose2d> readTrajFromFile(const std::string &file_name) {
    std::vector<file_io::TrajectoryNode2d> trajectory_nodes;
    file_io::readRawTrajectory2dFromFile(file_name, trajectory_nodes);

    // Assuming trajectory nodes are in order
    std::vector<pose::Pose2d> init_traj_poses;
    for (const file_io::TrajectoryNode2d traj_node : trajectory_nodes) {
        init_traj_poses.emplace_back(pose::createPose2d(traj_node.transl_x_, traj_node.transl_y_, traj_node.theta_));
    }
    return init_traj_poses;
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
    std::string car_semantic_class = "car";

    std::vector<std::string> past_sample_files;
    std::string odom_estimates_file_name;
    std::string object_detections_file_name;
    std::string estimated_trajectory_file;
    std::string gt_traj_file;

    if (!n.getParam(param_prefix + kPastSamplesFilesParamName, past_sample_files)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kPastSamplesFilesParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kOdomTrajectoryEstimatesFileParamName, odom_estimates_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kOdomTrajectoryEstimatesFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kObjDetectionsCurrTrajectoryFileParamName, object_detections_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kObjDetectionsCurrTrajectoryFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kEstTrajFileParamName, estimated_trajectory_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kEstTrajFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kGtTrajectoryFile, gt_traj_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kGtTrajectoryFile;
        exit(1);
    }

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n);

    // Read initial 2d initial estimates
    std::vector<pose::Pose2d> odom_trajectory = readTrajFromFile(odom_estimates_file_name);
    std::vector<pose::Pose2d> estimated_trajectory = readTrajFromFile(estimated_trajectory_file);
    std::vector<pose::Pose2d> gt_trajectory = readTrajFromFile(gt_traj_file);

    // Read samples
    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories;
    for (const std::string &past_sample_file : past_sample_files) {
        std::vector<file_io::PastSample2d> past_samples;
        file_io::readPastSample2dsFromFile(past_sample_file, past_samples);
        for (const file_io::PastSample2d &past_sample : past_samples) {
            samples_for_prev_trajectories[past_sample.semantic_class_].emplace_back(
                    std::make_pair(
                            std::make_pair(Eigen::Vector2d(past_sample.transl_x_, past_sample.transl_y_),
                                           past_sample.theta_),
                            past_sample.value_));
        }
    }

    std::vector<std::vector<pose::Pose2d>> obj_detections = getObjDetections(object_detections_file_name,
                                                                             odom_trajectory.size());

    manager->displayPastSampleValues(car_semantic_class, samples_for_prev_trajectories[car_semantic_class]);
    manager->displayTrueTrajectory(gt_trajectory);
    manager->displayOdomTrajectory(odom_trajectory);
    manager->displayEstTrajectory(estimated_trajectory);
    manager->displayObjObservationsFromEstTrajectory(estimated_trajectory, obj_detections, car_semantic_class);
    manager->displayObjObservationsFromOdomTrajectory(odom_trajectory, obj_detections, car_semantic_class);
    manager->displayObjObservationsFromGtTrajectory(gt_trajectory, obj_detections, car_semantic_class);

    return 0;
}