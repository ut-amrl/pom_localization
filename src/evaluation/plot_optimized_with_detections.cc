#include <unordered_map>
#include <unordered_set>

#include <glog/logging.h>
#include <ros/ros.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/object_positions_by_pose_io.h>
#include <visualization/ros_visualization.h>
#include <algorithm>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");

const std::string kNumTrajectoriesParamName = "num_trajectories";

const std::string kTrajectorySpecificPrefix = "trajectory_";

const std::string kTrajectoryOutputSuffix = "trajectory_output_file";

const std::string kObjectDetectionsFile = "object_detections";

const std::string kSemanticPointsFile = "semantic_points";

std::string constructParamName(const std::string &param_prefix, const int &trajectory_num,
                               const std::string &param_suffix) {
    return param_prefix + kTrajectorySpecificPrefix + std::to_string(trajectory_num) + "/" + param_suffix;
}

std::vector<std::vector<pose::Pose2d>>
getMovableObjectDetectionsFromFile(const std::string &object_detections_file_name, const size_t &traj_len) {

    std::vector<file_io::ObjectPositionByPose> raw_obj_detections;
    LOG(INFO) << "Reading from file " << object_detections_file_name;
    LOG(INFO) << "Trajectory length " << traj_len;
    file_io::readObjectPositionsByPoseFromFile(object_detections_file_name, raw_obj_detections);

    std::vector<std::vector<pose::Pose2d>> relative_detections;
    relative_detections.resize(traj_len);
    for (const file_io::ObjectPositionByPose &raw_detection : raw_obj_detections) {
        pose::Pose2d obj_pose = pose::createPose2d(raw_detection.transl_x_, raw_detection.transl_y_,
                                                   raw_detection.theta_);
        relative_detections[raw_detection.pose_number_].emplace_back(obj_pose);

    }
    LOG(INFO) << "Returning relative detections";
    return relative_detections;
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

    int num_trajectories;

    if (!n.getParam(param_prefix + kNumTrajectoriesParamName, num_trajectories)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kNumTrajectoriesParamName;
        exit(1);
    }

    if (num_trajectories <= 0) {
        LOG(INFO) << "Trajectory count must be a positive number";
        exit(1);
    }

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n, param_prefix);

    std::vector<std::vector<pose::Pose2d>> trajectories_list;
    std::vector<std::vector<std::vector<pose::Pose2d>>> relative_car_poses;
    std::vector<std::vector<std::unordered_map<size_t, std::vector<Eigen::Vector2d>>>> relative_semantic_points;

    for (int i = 0; i < num_trajectories; i++) {
        LOG(INFO) << "Trajectory " << i;
        std::string trajectory_file_name;
        std::string obj_det_file;

        std::string trajectory_file_param_name = constructParamName(param_prefix, i, kTrajectoryOutputSuffix);
        std::string object_det_file_param_name = constructParamName(param_prefix, i, kObjectDetectionsFile);

        if (!n.getParam(trajectory_file_param_name, trajectory_file_name)) {
            LOG(INFO) << "No parameter value set for parameter with name " << trajectory_file_param_name;
            exit(1);
        }
        if (!n.getParam(object_det_file_param_name, obj_det_file)) {
            LOG(INFO) << "No parameter value set for parameter with name " << object_det_file_param_name;
            exit(1);
        }

        std::vector<pose::Pose2d> trajectory_estimate = file_io::readTrajFromFile(trajectory_file_name);
        LOG(INFO) << "Trajectory length for file " << trajectory_file_name << ": " << trajectory_estimate.size();
        trajectories_list.emplace_back(trajectory_estimate);

        std::vector<std::vector<pose::Pose2d>> objs_for_traj = getMovableObjectDetectionsFromFile(obj_det_file, trajectory_estimate.size());
        relative_car_poses.emplace_back(objs_for_traj);
    }

    manager->publishEstimatedTrajectories(trajectories_list);
    manager->displayObjObservationsFromEstTrajectories(trajectories_list,
                                                       relative_car_poses,
                                                       "car");

    return 0;
}