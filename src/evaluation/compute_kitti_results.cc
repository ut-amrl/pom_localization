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
DEFINE_bool(run_viz, false, "Run viz");
DEFINE_string(param_prefix, "", "param_prefix");

const std::string kOdomTrajectoryEstimatesFileParamName = "odom_traj_est_file";

const std::string kAlignedEstTrajFileParamName = "aligned_est_traj_file";

const std::string kMisalignedEstTrajFileParamName = "misaligned_est_traj_file";

const std::string kTwentyMisalignedEstTrajParamName = "twenty_percent_misaligned_est_traj_file";

const std::string kFiftyMisalignedEstTrajParamName = "fifty_percent_misaligned_est_traj_file";
const std::string kEightyMisalignedEstTrajParamName = "eighty_percent_misaligned_est_traj_file";


const std::string kGtTrajectoryFile = "gt_trajectory_file";

double computeATE(const std::vector<pose::Pose2d> &ground_truth_trajectory,
                  const std::vector<pose::Pose2d> &optimized_trajectory) {
    double squared_error_sum = 0;
    for (size_t i = 0; i < ground_truth_trajectory.size(); i++) {
        pose::Pose2d optimized = optimized_trajectory[i];
        pose::Pose2d ground_truth = ground_truth_trajectory[i];
        squared_error_sum += (optimized.first - ground_truth.first).squaredNorm();
    }
    if (ground_truth_trajectory.size() == 0) {
        return 0;
    }
    return squared_error_sum / ground_truth_trajectory.size();
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
    std::string odom_estimates_file_name;
    std::string aligned_estimated_trajectory_file;
    std::string misaligned_estimated_trajectory_file;
    std::string gt_traj_file;
    std::string twenty_misaligned_traj_file;
    bool include_twenty_misaligned = false;
    std::string fifty_misaligned_traj_file;
    bool include_fifty_misaligned = false;
    std::string eighty_misaligned_traj_file;
    bool include_eighty_misaligned = false;

    if (!n.getParam(param_prefix + kOdomTrajectoryEstimatesFileParamName, odom_estimates_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kOdomTrajectoryEstimatesFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kAlignedEstTrajFileParamName, aligned_estimated_trajectory_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kAlignedEstTrajFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kMisalignedEstTrajFileParamName, misaligned_estimated_trajectory_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kMisalignedEstTrajFileParamName;
        exit(1);
    }

    if (!n.getParam(param_prefix + kGtTrajectoryFile, gt_traj_file)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kGtTrajectoryFile;
        exit(1);
    }

    if (n.getParam(param_prefix + kTwentyMisalignedEstTrajParamName, twenty_misaligned_traj_file)) {
        include_twenty_misaligned = true;
    }
    if (n.getParam(param_prefix + kFiftyMisalignedEstTrajParamName, fifty_misaligned_traj_file)) {
        include_fifty_misaligned = true;
    }
    if (n.getParam(param_prefix + kEightyMisalignedEstTrajParamName, eighty_misaligned_traj_file)) {
        include_eighty_misaligned = true;
    }

    // Read odometry, ground truth, and estimated trajectories
    std::vector<pose::Pose2d> odom_trajectory = file_io::readTrajFromFile(odom_estimates_file_name);
    std::vector<pose::Pose2d> aligned_estimated_trajectory = file_io::readTrajFromFile(aligned_estimated_trajectory_file);
    std::vector<pose::Pose2d> misaligned_estimated_trajectory = file_io::readTrajFromFile(misaligned_estimated_trajectory_file);
    std::vector<pose::Pose2d> gt_trajectory = file_io::readTrajFromFile(gt_traj_file);
    std::vector<pose::Pose2d> twenty_misaligned_trajectory;
    std::vector<pose::Pose2d> fifty_misaligned_trajectory;
    std::vector<pose::Pose2d> eighty_misaligned_trajectory;
    if (include_twenty_misaligned) {
        twenty_misaligned_trajectory = file_io::readTrajFromFile(twenty_misaligned_traj_file);
    }
    if (include_fifty_misaligned) {
        fifty_misaligned_trajectory = file_io::readTrajFromFile(fifty_misaligned_traj_file);
    }
    if (include_eighty_misaligned) {
        eighty_misaligned_trajectory = file_io::readTrajFromFile(eighty_misaligned_traj_file);
    }

    double odom_ate = computeATE(gt_trajectory, odom_trajectory);
    double aligned_est_ate = computeATE(gt_trajectory, aligned_estimated_trajectory);
    double misaligned_est_ate = computeATE(gt_trajectory, misaligned_estimated_trajectory);
    double twenty_misaligned_ate;
    double fifty_misaligned_ate;
    double eighty_misaligned_ate;
    std::vector<std::vector<pose::Pose2d>> trajectories_to_plot = {odom_trajectory, gt_trajectory, aligned_estimated_trajectory, misaligned_estimated_trajectory};
    if (include_twenty_misaligned) {
        twenty_misaligned_ate = computeATE(gt_trajectory, twenty_misaligned_trajectory);
        trajectories_to_plot.emplace_back(twenty_misaligned_trajectory);
    }
    if (include_fifty_misaligned) {
        fifty_misaligned_ate = computeATE(gt_trajectory, fifty_misaligned_trajectory);
        trajectories_to_plot.emplace_back(fifty_misaligned_trajectory);
    }
    if (include_eighty_misaligned) {
        eighty_misaligned_ate= computeATE(gt_trajectory, eighty_misaligned_trajectory);
        trajectories_to_plot.emplace_back(eighty_misaligned_trajectory);
    }

    if (FLAGS_run_viz) {
        std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
                n, param_prefix);

        manager->publishEstimatedTrajectories(trajectories_to_plot);
    }

    LOG(INFO) << "Odom ate " << odom_ate;
    LOG(INFO) << "Misaligned est ate " << misaligned_est_ate;
    if (include_eighty_misaligned) {
        LOG(INFO) << "80% misaligned est ate " << eighty_misaligned_ate;
    }
    if (include_fifty_misaligned) {
        LOG(INFO) << "50% misaligned est ate " << fifty_misaligned_ate;
    }
    if (include_twenty_misaligned) {
        LOG(INFO) << "20% misaligned est ate " << twenty_misaligned_ate;
    }
    LOG(INFO) << "Aligned est ate " << aligned_est_ate;

    return 0;
}