#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <ctime>
#include <pose_optimization/pose_3d_factor_graph.h>
#include <chrono>
#include <pose_optimization/utils/pose_graph_creation_utils.h>

#include <synthetic_problem/uncertainty_aware/uncertainty_aware_synthetic_problem.h>

#include <file_io/parking_spot_io.h>
#include <file_io/synthetic_distribution_generation_config_io.h>

using namespace pose;

std::vector<std::pair<Pose2d, unsigned int>> createParkedCarPosesWithFrequency() {
    std::vector<std::pair<Pose2d, unsigned int>> poses;
    poses.emplace_back(std::make_pair(createPose2d(0, 0, 0), 2));
    poses.emplace_back(std::make_pair(createPose2d(1, 0, 0), 3));
    poses.emplace_back(std::make_pair(createPose2d(2, 0, 0), 1));
    poses.emplace_back(std::make_pair(createPose2d(3, 0, 0), 1));
    poses.emplace_back(std::make_pair(createPose2d(4, 0, 0), 2));
    poses.emplace_back(std::make_pair(createPose2d(5, 0, 0), 2));
    poses.emplace_back(std::make_pair(createPose2d(6, 0, 0), 3));

    return poses;
}

std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>>
generateObjConfigurations(const std::string &semantic_class) {
    std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>> cfgs_with_class;
    std::vector<std::vector<pose::Pose2d>> obj_configs = {
            {
                    createPose2d(1, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(6, 0, 0)
            },
            {
                    createPose2d(1, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(6, 0, 0)
            },
            {
                    createPose2d(1, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(6, 0, 0)
            },
            {
                    createPose2d(1, 0, 0),
                    createPose2d(2, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(6, 0, 0)
            },
            {
                    createPose2d(1, 0, 0),
                    createPose2d(2, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(5, 0, 0),
                    createPose2d(6, 0, 0)
            },             {
                    createPose2d(1, 0, 0),
                    createPose2d(2, 0, 0),
                    createPose2d(4, 0, 0),
                    createPose2d(5, 0, 0),
                    createPose2d(6, 0, 0)
            }
    };
    for (const std::vector<pose::Pose2d> &config : obj_configs) {
        std::unordered_map<std::string, std::vector<pose::Pose2d>> cfg_with_class;
        cfg_with_class[semantic_class] = config;
        cfgs_with_class.emplace_back(cfg_with_class);
    }

    return cfgs_with_class;
}

std::vector<Pose2d> createTrajectory() {
    std::vector<Pose2d> poses;

    poses.emplace_back(createPose2d(0, 1, 0));
    poses.emplace_back(createPose2d(1, 1, 0));
    poses.emplace_back(createPose2d(2, 1, 0));
    poses.emplace_back(createPose2d(3, 1, 0));
    poses.emplace_back(createPose2d(4, 1, 0));
    poses.emplace_back(createPose2d(5, 1, 0));
    poses.emplace_back(createPose2d(6, 1, 0));
    poses.emplace_back(createPose2d(0, -1, 0));
    poses.emplace_back(createPose2d(1, -1, 0));
    poses.emplace_back(createPose2d(2, -1, 0));
    poses.emplace_back(createPose2d(3, -1, 0));
    poses.emplace_back(createPose2d(4, -1, 0));
    poses.emplace_back(createPose2d(5, -1, 0));
    poses.emplace_back(createPose2d(6, -1, 0));

    return poses;
}

file_io::SyntheticDistributionGenerationConfig createDistributionConfig() {

    file_io::SyntheticDistributionGenerationConfig distribution_config;
    distribution_config.num_trajectories_ = 5;
    distribution_config.num_samples_per_beam_ = 1;
    distribution_config.scan_num_beams_ = 100;

    distribution_config.parking_spot_generation_std_dev_x_ = 0.25;
    distribution_config.parking_spot_generation_std_dev_y_ = 0.25;
    distribution_config.parking_spot_generation_std_dev_theta_ = 0.15;

    distribution_config.percent_parking_spots_filled_ = 1;

    distribution_config.trajectory_variation_std_dev_x_ = 5;
    distribution_config.trajectory_variation_std_dev_y_ = 5;
    distribution_config.trajectory_variation_std_dev_theta_ = 1.0;

    // TODO revisit these
    distribution_config.object_detection_variance_per_len_x_ = 0.02;
    distribution_config.object_detection_variance_per_len_y_ = 0.02;
    distribution_config.object_detection_variance_per_len_theta_ = 0.05;

    // TODO revisit
    distribution_config.max_object_detection_distance_ = 40;

    distribution_config.scan_min_angle_ = -M_PI;
    distribution_config.scan_max_angle_ = M_PI;
    distribution_config.scan_min_range_ = 0.2;
    distribution_config.scan_max_range_ = 7;

    distribution_config.percent_beams_per_scan_ = 1;
    distribution_config.percent_poses_to_sample_ = 1;
    return distribution_config;
}

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "parking_spot_3d_plotter");
    ros::NodeHandle n;
    std::string semantic_class = "car";

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    util_random::Random random_generator(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());


    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n);

    synthetic_problem::ScanGenerationParams2d scan_gen_params;
    file_io::SyntheticDistributionGenerationConfig distribution_config = createDistributionConfig();
    scan_gen_params.min_range_ = distribution_config.scan_min_range_;
    scan_gen_params.max_range_ = distribution_config.scan_max_range_;
    scan_gen_params.num_beams_ = distribution_config.scan_num_beams_;
    scan_gen_params.min_angle_ = distribution_config.scan_min_angle_;
    scan_gen_params.max_angle_ = distribution_config.scan_max_angle_;

    std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>> obj_configurations = generateObjConfigurations(
            semantic_class);

    std::vector<std::vector<pose::Pose2d>> trajectories;
    for (size_t i = 0; i < obj_configurations.size(); i++) {
        trajectories.emplace_back(createTrajectory());
    }

    std::vector<std::vector<std::pair<pose::Pose2d, SensorInfo<pose::Pose2d, 3, sensor_msgs::LaserScan>>>> sensor_info_for_past_trajectories =
            synthetic_problem::generateSensorInfoForTrajectories<pose::Pose2d, 3, sensor_msgs::LaserScan, synthetic_problem::ScanGenerationParams2d>(
                    trajectories, obj_configurations,
                    Eigen::Vector3d(pow(distribution_config.object_detection_variance_per_len_x_, 2),
                                    pow(distribution_config.object_detection_variance_per_len_y_, 2),
                                    pow(distribution_config.object_detection_variance_per_len_theta_, 2)),
                    distribution_config.max_object_detection_distance_, scan_gen_params, random_generator);

    std::function<double(
            const double &)> pdf_squashing_function = pose_optimization::squashPdfValueToZeroToOneRangeExponential;
    std::function<double(const pose::Pose2d &,
                         const std::vector<ObjectDetectionRelRobot<pose::Pose2d, 3>> &)> sample_value_generator =
            std::bind(&pose_optimization::computeValueForSample<pose::Pose2d, 3>, std::placeholders::_1,
                      std::placeholders::_2, pdf_squashing_function);

    std::unordered_map<std::string, pose_optimization::SampleGeneratorParams2d> sample_gen_params_by_class;
    sample_gen_params_by_class[semantic_class].num_samples_per_beam_ = distribution_config.num_samples_per_beam_;
    sample_gen_params_by_class[semantic_class].percent_beams_per_scan_ = distribution_config.percent_beams_per_scan_;
    sample_gen_params_by_class[semantic_class].percent_poses_to_include_ = distribution_config.percent_poses_to_sample_;

    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories = generateSamplesForPastTrajectories(
            sensor_info_for_past_trajectories, sample_value_generator,
            sample_gen_params_by_class,
            random_generator);

    LOG(INFO) << "Generated " << samples_for_prev_trajectories[semantic_class].size() << " samples";

    std::vector<pose_graph::MapObjectObservation2d> samples;
    for (const std::pair<pose::Pose2d, double> &past_sample : samples_for_prev_trajectories[semantic_class]) {
        pose_graph::MapObjectObservation<2, double> map_observation;
        map_observation.semantic_class_ = semantic_class;
        map_observation.transl_ = Eigen::Vector2d(past_sample.first.first.x(), past_sample.first.first.y());
        map_observation.orientation_ = past_sample.first.second;
        map_observation.obs_value_ = past_sample.second;
        samples.emplace_back(map_observation);
    }

    pose_optimization::PoseOptimizationParameters params;
    params.cost_function_params_.mean_position_kernel_len_ = 0.45;
    params.cost_function_params_.mean_orientation_kernel_len_ = 100;

    std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>> pose_graph = pose_graph::utils::createFully2dPoseGraphFromParams(
            params.cost_function_params_);
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
    LOG(INFO) << "Adding map frame observations";

    manager->displayPastSampleValues(semantic_class, samples_for_prev_trajectories[semantic_class]);
    pose_graph->addMapFrameObservations(observations_by_class);


    std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> gp = pose_graph->getMovableObjGpc(
            semantic_class, 1, random_generator);

    std::function<std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
            const Eigen::Vector2d &)> gpc_creator = [gp](const Eigen::Vector2d &loc) {
        return gp;
    };
    manager->displayMaxGpRegressorOutput(
            gp,
            0.05,
//            0.2,
            -2, 8, -4, 1
//            -3, 9, 3,-3
    );

    std::vector<pose::Pose2d> trajectory_to_plot = {
            createPose2d(0, -1, 0),
            createPose2d(1.5, -1.1, -0.03),
            createPose2d(2.5, -1.02, -0.1),
            createPose2d(4, -0.95, -0.05),
            createPose2d(5, -1.03, 0.1),
    };

    std::vector<pose::Pose2d> true_odometry;
    for (size_t i = 1; i < trajectory_to_plot.size(); i++) {
        true_odometry.emplace_back(pose::getPoseOfObj1RelToObj2(trajectory_to_plot[i],
                                                                trajectory_to_plot[i - 1]));
    }

//            util_random::Random random_generator;
    std::vector<pose::Pose2d> noisy_odometry;
    for (const pose::Pose2d &true_odom : true_odometry) {
        pose::Pose2d noisy_odom = true_odom;
        noisy_odom.second = -0.17;
        noisy_odometry.emplace_back(noisy_odom);
    }

    std::vector<pose::Pose2d> initial_node_positions;
    pose::Pose2d prev_pose = trajectory_to_plot[0];
    initial_node_positions.emplace_back(prev_pose);
    prev_pose.second = -0.15;
    for (const pose::Pose2d &odom_to_next_pos : noisy_odometry) {
        pose::Pose2d new_pose = pose::combinePoses(prev_pose, odom_to_next_pos);
        initial_node_positions.emplace_back(new_pose);
        prev_pose = new_pose;
    }

    manager->displayEstTrajectory(trajectory_to_plot);
    manager->displayOdomTrajectory(initial_node_positions);

    Eigen::Vector3d object_detection_variance_per_detection_len;
    object_detection_variance_per_detection_len(0) = 0.0025;
    object_detection_variance_per_detection_len(1) = 0.0025;
    object_detection_variance_per_detection_len(2) = 0.0025;
    std::vector<pose::Pose2d> obj_poses = {
            createPose2d(1, 0, 0),
            createPose2d(2, 0, 0),
            createPose2d(6, 0, 0)
    };
    std::unordered_map<std::string, std::vector<pose::Pose2d>> obj_poses_by_class = {{semantic_class, obj_poses}};
    std::unordered_map<std::string, std::vector<std::vector<ObjectDetectionRelRobot<pose::Pose2d, 3 >>>> obj_dets_rel_traj = synthetic_problem::generateObjectDetectionsForTrajectory(
            trajectory_to_plot,
            4.0,
            obj_poses_by_class,
            object_detection_variance_per_detection_len,
            random_generator);

    LOG(INFO) << "Constructing relative detections from detailed detections";
    std::vector<std::vector<pose::Pose2d>> relative_detections;
    for (const std::vector<ObjectDetectionRelRobot<pose::Pose2d, 3>> &obj_dets_at_node : obj_dets_rel_traj[semantic_class]) {
        std::vector<pose::Pose2d> detections_at_node;
        for (const ObjectDetectionRelRobot<pose::Pose2d, 3> &obj_det : obj_dets_at_node) {
            detections_at_node.emplace_back(obj_det.pose_);
        }
        relative_detections.emplace_back(detections_at_node);
    }
    LOG(INFO) << "Plotting relative detections from detailed detections";

    manager->displayObjObservationsFromOdomTrajectory(initial_node_positions, relative_detections, semantic_class);
    manager->displayObjObservationsFromEstTrajectory(trajectory_to_plot, relative_detections, semantic_class);

    return 0;
}