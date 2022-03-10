#include <unordered_map>

#include <glog/logging.h>
#include <ros/ros.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/pose_3d_io.h>
#include <file_io/semantic_point_with_node_id_io.h>
#include <visualization/ros_visualization.h>
#include <file_io/semantic_index_to_string_map_io.h>
#include <pose_optimization/pose_graph_generic.h>
#include <algorithm>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kOdomTrajectoryEstimatesFileParamName = "odom_traj_est_file";

const std::string kSemanticPointDetectionsCurrTrajectoryFileParamName = "semantic_point_det_curr_traj_file";

const std::string kTrajectoryOutputFileParamName = "traj_est_output_file";

const std::string kGtTrajectoryFile = "gt_trajectory_file";

const std::string kDetectionSensorRelBaseLinkFileParamName = "detection_sensor_rel_baselink_file";

const std::string kSemanticIndexToStringClassFileParamName = "semantic_index_to_string_file";

typedef pose_graph::MovableObservationSemanticPoints<2> MovableObservationSemanticPoints2d;
typedef pose_graph::MovableObservationFactor<MovableObservationSemanticPoints2d> MovableObservationSemanticPointsFactor2d;

void plotTrajectoryAndSemanticPoints(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                                     const std::vector<pose::Pose2d> &node_poses_list,
                                     const std::vector<pose::Pose2d> &unoptimized_trajectory,
                                     const std::vector<pose::Pose2d> &ground_truth_trajectory,
                                     const std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>>
                                     &noisy_obj_observations_by_class) {

    vis_manager->displayOdomTrajectory(unoptimized_trajectory);
    vis_manager->displayEstTrajectory(node_poses_list);
    vis_manager->displayTrueTrajectory(ground_truth_trajectory);

    for (const auto &obs_with_class : noisy_obj_observations_by_class) {
        std::string semantic_class = obs_with_class.first;
        std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> noisy_obj_observations = obs_with_class.second;
        std::unordered_map<uint64_t, std::unordered_map<size_t, std::vector<pose::Pose2d>>> rectangle_samples_for_class;
        vis_manager->displaySemanticPointObsFromEstTrajectory(node_poses_list, noisy_obj_observations,
                                                              semantic_class);
        if (!ground_truth_trajectory.empty()) {
            vis_manager->displaySemanticPointObsFromGtTrajectory(ground_truth_trajectory,
                                                                 noisy_obj_observations,
                                                                 semantic_class);

        }
        vis_manager->displaySemanticPointObsFromOdomTrajectory(unoptimized_trajectory,
                                                               noisy_obj_observations, semantic_class);
    }
}

pose::Pose2d getDetectionsSensorRelBaselinkPoseFromFile(const std::string &detection_sensor_rel_baselink_file) {
    std::vector<pose::Pose3d> sensor_poses_vec;
    file_io::readPose3dsFromFile(detection_sensor_rel_baselink_file, sensor_poses_vec);
    if (sensor_poses_vec.size() == 0) {
        LOG(ERROR) << "No poses specifying the detection sensor's pose relative to the baselink";
        exit(1);
    }
    if (sensor_poses_vec.size() != 1) {
        LOG(WARNING) << "Multiple poses in file specifying the detection sensor's pose relative to the base link. Using"
                        " the first entry";
    }
    pose::Pose3d sensor_pose_3d = sensor_poses_vec.front();
    pose::Pose2d sensor_pose_2d = pose::toPose2d(sensor_pose_3d);
    return sensor_pose_2d;
}

std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>>
getNoisyObjObservationsByClass(const std::string &semantic_point_detections_file_name,
                               const std::string &detection_sensor_file_name,
                               const std::string &semantic_index_to_string_map_file,
                               const size_t &trajectory_size) {


    pose::Pose2d detections_sensor_rel_baselink = getDetectionsSensorRelBaselinkPoseFromFile(
            detection_sensor_file_name);

    std::unordered_map<unsigned short, std::string> semantic_index_to_string_map;
    file_io::readSemanticIndexToStringMapFromFile(semantic_index_to_string_map_file, semantic_index_to_string_map);

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo> raw_obj_detections;
    LOG(INFO) << "Reading from file " << semantic_point_detections_file_name;
    semantic_segmentation::readSemanticallyLabeledPointWithNodeIdInfoFromFile(semantic_point_detections_file_name,
                                                                              raw_obj_detections);

    std::vector<MovableObservationSemanticPointsFactor2d> observation_factors;
    // Node id is outer key, cluster id is inner key
    std::unordered_map<uint64_t, std::unordered_map<uint32_t,
            std::vector<semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo>>> points_by_node_and_cluster;
    for (const semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo &semantic_points : raw_obj_detections) {
        points_by_node_and_cluster[semantic_points.node_id][semantic_points.cluster_label].emplace_back(
                semantic_points);
    }

    for (const auto &points_for_node : points_by_node_and_cluster) {
        uint64_t node_id = points_for_node.first;
        for (const auto &points_for_cluster : points_for_node.second) {
            if (points_for_cluster.second.size() <= 5) { // TODO make configurable
                continue;
            }
            unsigned short semantic_index = points_for_cluster.second.front().semantic_label;
            if (semantic_index_to_string_map.find(semantic_index) == semantic_index_to_string_map.end()) {
                LOG(WARNING) << "Skipping observations with cluster id " << points_for_cluster.first << " and node id "
                             << node_id << " because the semantic index was not found in the index to class string map";
                continue;
            }
            std::string semantic_string = semantic_index_to_string_map.at(semantic_index);
            MovableObservationSemanticPoints2d observation;
            observation.semantic_class_ = semantic_string;
            observation.cluster_num_ = points_for_cluster.first;
            for (const semantic_segmentation::SemanticallyLabeledPointWithNodeIdInfo &semantic_point : points_for_cluster.second) {
                observation.object_points_.emplace_back(
                        Eigen::Vector2d(semantic_point.point_x, semantic_point.point_y));
            }
            observation_factors.emplace_back(MovableObservationSemanticPointsFactor2d(node_id, observation));
        }
    }

    std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>> noisy_observations_by_class;
    for (const MovableObservationSemanticPointsFactor2d &movable_factor : observation_factors) {
        pose_graph::NodeId observed_at_node = movable_factor.observed_at_node_;
        std::string semantic_class = movable_factor.observation_.semantic_class_;

        std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>> observations_for_class;
        if (noisy_observations_by_class.find(semantic_class) == noisy_observations_by_class.end()) {
            observations_for_class.resize(trajectory_size);
        } else {
            observations_for_class = noisy_observations_by_class[semantic_class];
        }
        std::vector<Eigen::Vector2d> baselink_detection_points;
        for (const Eigen::Vector2d &sensor_detection_point : movable_factor.observation_.object_points_) {
            baselink_detection_points.emplace_back(
                    pose::transformPoint(detections_sensor_rel_baselink, sensor_detection_point));
        }
        observations_for_class[observed_at_node][movable_factor.observation_.cluster_num_] = baselink_detection_points;
        noisy_observations_by_class[semantic_class] = observations_for_class;
    }

    return noisy_observations_by_class;
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
    node_prefix.erase(std::remove(node_prefix.begin(), node_prefix.end(), '/'), node_prefix.end());
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "plot_results");
    ros::NodeHandle n;

    std::shared_ptr<visualization::VisualizationManager> manager =
            std::make_shared<visualization::VisualizationManager>(n, param_prefix);

    std::string odom_estimates_file_name;
    std::string semantic_point_detections_file_name;
    std::string traj_est_output_file;
    std::string gt_traj_file;
    std::string detection_sensor_rel_baselink_file;
    std::string semantic_index_to_string_file;

    if (!n.getParam(param_prefix + kOdomTrajectoryEstimatesFileParamName, odom_estimates_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kOdomTrajectoryEstimatesFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kSemanticPointDetectionsCurrTrajectoryFileParamName,
                    semantic_point_detections_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kSemanticPointDetectionsCurrTrajectoryFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kTrajectoryOutputFileParamName, traj_est_output_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kTrajectoryOutputFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kDetectionSensorRelBaseLinkFileParamName, detection_sensor_rel_baselink_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kDetectionSensorRelBaseLinkFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kSemanticIndexToStringClassFileParamName, semantic_index_to_string_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kSemanticIndexToStringClassFileParamName;
        exit(1);
    }

    // Read initial 2d initial estimates
    std::vector<pose::Pose2d> initial_trajectory_estimates = readTrajFromFile(odom_estimates_file_name);
    std::vector<pose::Pose2d> optimized_trajectory = readTrajFromFile(traj_est_output_file);

    std::vector<pose::Pose2d> gt_trajectory;
    if (n.getParam(param_prefix + kGtTrajectoryFile, gt_traj_file)) {
        gt_trajectory = readTrajFromFile(gt_traj_file);
    }

    std::unordered_map<std::string, std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>>
            noisy_obj_observations_by_class = getNoisyObjObservationsByClass(semantic_point_detections_file_name, detection_sensor_rel_baselink_file, semantic_index_to_string_file, optimized_trajectory.size());

    plotTrajectoryAndSemanticPoints(manager,
                                    optimized_trajectory, initial_trajectory_estimates,
                                    gt_trajectory, noisy_obj_observations_by_class);
    return 0;
}