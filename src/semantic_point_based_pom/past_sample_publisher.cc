#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <ctime>

#include <pose_optimization/utils/pose_graph_creation_utils.h>

#include <file_io/trajectory_2d_io.h>
#include <file_io/past_sample_io.h>
#include <file_io/runtime_params_config_io.h>
#include <file_io/shape_dimensions_2d_by_semantic_class_io.h>
#include <unordered_map>

using namespace pose;

DEFINE_string(param_prefix, "", "param_prefix");
DEFINE_bool(run_gpc_viz, false, "Run GPC viz");

const std::string kPastSamplesFilesParamName = "past_samples_files";

const std::string kRuntimeParamsConfigParamName = "runtime_params_config_file";

pose_optimization::PoseOptimizationParameters
setupPoseOptimizationParams(const file_io::RuntimeParamsConfig &config, const bool &kitti) {
    pose_optimization::PoseOptimizationParameters pose_opt_params;
    pose_optimization::CostFunctionParameters cost_function_params;

    cost_function_params.full_optimization_interval_ = config.full_optimization_interval_;
    cost_function_params.num_nodes_in_optimization_window_ = config.num_nodes_in_optimization_window_;
    cost_function_params.max_gpc_samples_ = config.max_gpc_samples_;

    cost_function_params.mean_position_kernel_len_ = config.mean_position_kernel_len_;
    cost_function_params.mean_orientation_kernel_len_ = config.mean_orientation_kernel_len_;

    cost_function_params.mean_position_kernel_var_ = config.mean_position_kernel_var_;
    cost_function_params.mean_orientation_kernel_var_ = config.mean_orientation_kernel_var_;

    cost_function_params.default_obj_probability_input_variance_for_mean_ =
            config.default_obj_probability_input_variance_for_mean_;

    cost_function_params.var_position_kernel_len_ = config.var_position_kernel_len_;
    cost_function_params.var_orientation_kernel_len_ = config.var_orientation_kernel_len_;

    cost_function_params.var_position_kernel_var_ = config.var_position_kernel_var_;
    cost_function_params.var_orientation_kernel_var_ = config.var_orientation_kernel_var_;

    cost_function_params.default_obj_probability_input_variance_for_var_ =
            config.default_obj_probability_input_variance_for_var_;

    pose_opt_params.cost_function_params_ = cost_function_params;

    return pose_opt_params;
}

std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> createGpc(
        const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                pose_graph::MovableObservationSemanticPoints2d>> &pose_graph,
        const std::string &semantic_class,
        const uint64_t &max_samples,
        const double &radius,
        const Eigen::Vector2d &search_point) {
    return pose_graph->getMovableObjGpcWithinRadius(semantic_class, radius, search_point, max_samples);
}

void displayGpc(const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                const std::vector<Eigen::Vector2d> &map_frame_obj_points,
                const size_t &max_samples,
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3,
                        2, double, 3, pose_graph::MovableObservationSemanticPoints2d>> &pose_graph) {

    std::pair<Eigen::Vector2d, Eigen::Vector2d> min_max_points_to_display =
            visualization::VisualizationManager::getMinMaxCornersForDistributionVisualization(
                    map_frame_obj_points);
    std::function<std::shared_ptr<gp_regression::GaussianProcessClassifier<3,
            gp_kernel::Pose2dKernel>>(const Eigen::Vector2d &)> gpc_creator =
            std::bind(createGpc, pose_graph, "car",
                      max_samples,
                      8.5,
                      std::placeholders::_1);
    vis_manager->displayMaxGpRegressorOutput(
            gpc_creator,
            0.5,
            min_max_points_to_display.first.x(),
            min_max_points_to_display.second.x(),
            min_max_points_to_display.first.y(),
            min_max_points_to_display.second.y()
    );
}

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;


    util_random::Random random_generator;
//
//    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(
//            std::chrono::system_clock::now().time_since_epoch()).count());

    std::string param_prefix = FLAGS_param_prefix;
    std::string node_prefix = FLAGS_param_prefix;
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "past_sample_publisher");
    ros::NodeHandle n;

    // Known config params
    std::string car_semantic_class = "car";

    std::vector<std::string> past_sample_files;

    if (!n.getParam(param_prefix + kPastSamplesFilesParamName, past_sample_files)) {
        LOG(INFO) << "No parameter value set for parameter with name " << param_prefix + kPastSamplesFilesParamName;
        exit(1);
    }

    std::shared_ptr<visualization::VisualizationManager> manager =
            std::make_shared<visualization::VisualizationManager>(n, param_prefix);

    // Read samples
    std::vector<pose_graph::MapObjectObservation2d> samples;
    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories;
    std::vector<Eigen::Vector2d> past_sample_positions;
    for (const std::string &past_sample_file : past_sample_files) {
        std::vector<file_io::PastSample2d> past_samples;
        file_io::readPastSample2dsFromFile(past_sample_file, past_samples);
        for (const file_io::PastSample2d &past_sample : past_samples) {
            pose_graph::MapObjectObservation<2, double> map_observation;
            map_observation.semantic_class_ = past_sample.semantic_class_;
            map_observation.transl_ = Eigen::Vector2d(past_sample.transl_x_, past_sample.transl_y_);

            map_observation.orientation_ = past_sample.theta_;
            map_observation.obs_value_ = past_sample.value_;
            samples_for_prev_trajectories[past_sample.semantic_class_].emplace_back(
                    std::make_pair(std::make_pair(map_observation.transl_, map_observation.orientation_),
                                   map_observation.obs_value_));
            samples.emplace_back(map_observation);

            past_sample_positions.emplace_back(map_observation.transl_);
        }
    }

    for (const auto &samples_for_prev_trajectories_with_class : samples_for_prev_trajectories) {
        manager->displayPastSampleValues(samples_for_prev_trajectories_with_class.first,
                                         samples_for_prev_trajectories_with_class.second);
    }

    if (!FLAGS_run_gpc_viz) {
        return 0;
    }

    std::string runtime_params_config_file_name;

    if (!n.getParam(param_prefix + kRuntimeParamsConfigParamName, runtime_params_config_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name "
                  << param_prefix + kRuntimeParamsConfigParamName;
        exit(1);
    }

    LOG(INFO) << "Reading runtime params config from file " << runtime_params_config_file_name;
    file_io::RuntimeParamsConfig runtime_params_config;
    file_io::readRuntimeParamsConfigFromFile(runtime_params_config_file_name, runtime_params_config);

    pose_optimization::PoseOptimizationParameters pose_opt_params = setupPoseOptimizationParams(runtime_params_config,
                                                                                                false);

    std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
            pose_graph::MovableObservationSemanticPoints<2>>> pose_graph = pose_graph::utils::createFully2dPoseGraphSemanticPointDetectionsFromParams(
            pose_opt_params.cost_function_params_,
            pose::createPose2d(0, 0, 0), nullptr);

    // Add the map observations
    std::unordered_map<std::string, std::vector<pose_graph::MapObjectObservation2d>> observations_by_class;
    for (const pose_graph::MapObjectObservation2d &map_observation : samples) {
        std::vector<pose_graph::MapObjectObservation2d> observations_for_class;
        auto obs_for_class_iter = observations_by_class.find(map_observation.semantic_class_);
        if (obs_for_class_iter == observations_by_class.end()) {
            observations_for_class = {};
        } else {
            observations_for_class = observations_by_class.at(map_observation.semantic_class_);
        }

        observations_for_class.emplace_back(map_observation);
        observations_by_class[map_observation.semantic_class_] = observations_for_class;
    }
    pose_graph->addMapFrameObservations(observations_by_class);
    displayGpc(manager,
               past_sample_positions,
               runtime_params_config.max_gpc_samples_,
               pose_graph);

    return 0;
}