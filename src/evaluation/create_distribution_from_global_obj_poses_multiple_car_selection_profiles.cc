#include <glog/logging.h>

#include <file_io/kitti_poses_io.h>
#include <file_io/trajectory_2d_io.h>

#include <pose_optimization/pose_graph_generic.h>
#include <ros/ros.h>
#include <iostream>
#include <base_lib/pose_reps.h>
#include <file_io/past_sample_io.h>
#include <synthetic_problem/uncertainty_aware/uncertainty_aware_synthetic_problem.h>
#include <file_io/synthetic_distribution_generation_config_multiple_car_selection_profiles.h>
#include <file_io/object_positions_by_pose_io.h>
#include <visualization/ros_visualization.h>

using namespace pose;

const std::string kDistributionGenConfigFileParamName = "distribution_gen_config_file";
const std::string kDistributionSamplesOutputFilePrefixParamName = "distribution_samples_output_file_prefix";
const std::string kTrajectory2dFileParamName = "trajectory_2d_file";
const std::string kParkingSpotsFileParamName = "parking_spots_file";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "create_distribution_from_global_obj_poses");
    ros::NodeHandle n;

    std::shared_ptr<visualization::VisualizationManager> vis_manager = std::make_shared<visualization::VisualizationManager>(n);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    // Get config ----------------------------------------------------------------------------------------------------
    std::string distribution_gen_config_file;
    std::string sample_output_file_prefix;
    std::string sample_output_file;
    std::string trajectory_file;
    std::string global_spots_file;

    if (!n.getParam(kDistributionGenConfigFileParamName, distribution_gen_config_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kDistributionGenConfigFileParamName;
        exit(1);
    }

    if (!n.getParam(kDistributionSamplesOutputFilePrefixParamName, sample_output_file_prefix)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kDistributionSamplesOutputFilePrefixParamName;
        exit(1);
    }

    if (!n.getParam(kTrajectory2dFileParamName, trajectory_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kTrajectory2dFileParamName;
        exit(1);
    }

    if (!n.getParam(kParkingSpotsFileParamName, global_spots_file)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kParkingSpotsFileParamName;
        exit(1);
    }

    std::string distribution_config_file_base_name = distribution_gen_config_file.substr(
            distribution_gen_config_file.find_last_of("/\\") + 1);
    size_t lastindex = distribution_config_file_base_name.find_last_of(".");
    distribution_config_file_base_name = distribution_config_file_base_name.substr(0, lastindex);
    sample_output_file = sample_output_file_prefix + distribution_config_file_base_name + ".txt";

    // Read data from files  -----------------------------------------------------------------------------------------
    file_io::SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles distribution_config;
    file_io::readSyntheticDistributionConfigFromFile(distribution_gen_config_file, distribution_config);

    std::vector<file_io::ObjectPositionByPose> object_positions_by_pose;
    file_io::readObjectPositionsByPoseFromFile(global_spots_file, object_positions_by_pose);

    std::vector<std::pair<pose_graph::NodeId, pose::Pose2d>> parking_spots_rel_pose;
    for (const file_io::ObjectPositionByPose &obj_pos_by_pose : object_positions_by_pose) {
        parking_spots_rel_pose.emplace_back(std::make_pair(obj_pos_by_pose.pose_number_,
                                                           pose::createPose2d(obj_pos_by_pose.transl_x_,
                                                                              obj_pos_by_pose.transl_y_,
                                                                              obj_pos_by_pose.theta_)));
    }

    std::vector<file_io::TrajectoryNode2d> trajectory_2d;
    file_io::readRawTrajectory2dFromFile(trajectory_file, trajectory_2d);

    std::vector<pose::Pose2d> base_trajectory;
    for (const file_io::TrajectoryNode2d &trajectory_entry : trajectory_2d) {
        // Assumes that we don't need the node numbers because they are in order
        base_trajectory.emplace_back(pose::createPose2d(trajectory_entry.transl_x_, trajectory_entry.transl_y_, trajectory_entry.theta_));
    }

    util_random::Random random_generator;

    // Create past trajectories and spots  ---------------------------------------------------------------------------
    synthetic_problem::ObjectPlacementConfiguration<pose::Pose2d, 3> car_placement_config;
    std::vector<pose::Pose2d> parking_spots_global_frame;
    for (const std::pair<pose_graph::NodeId, pose::Pose2d> &parking_spot_rel_pose : parking_spots_rel_pose) {
        pose::Pose2d observed_at_pose = base_trajectory[parking_spot_rel_pose.first];
        synthetic_problem::ObjectOccurrenceParams<pose::Pose2d, 3> object_occurrence;
        object_occurrence.relative_frequency_ = 1; // Uniform frequency
        object_occurrence.object_canonical_pose_ = pose::combinePoses(observed_at_pose, parking_spot_rel_pose.second);
        parking_spots_global_frame.emplace_back(object_occurrence.object_canonical_pose_);
        double profile_selection_param = random_generator.UniformRandom();
        file_io::CarPlacementNoiseProfile car_placement_profile = distribution_config.car_placement_profile.back().second;
        double prev_accumulated_boundary = 0;
        for (size_t car_placement_prof_num = 0; car_placement_prof_num < distribution_config.car_placement_profile.size(); car_placement_prof_num++) {
            double next_boundary = prev_accumulated_boundary + distribution_config.car_placement_profile[car_placement_prof_num].first;
            if (profile_selection_param <= next_boundary) {
                car_placement_profile = distribution_config.car_placement_profile[car_placement_prof_num].second;
                break;
            }
            prev_accumulated_boundary = next_boundary;
        }
        object_occurrence.object_pose_variance_ = Eigen::Vector3d(
                pow(car_placement_profile.parking_spot_generation_std_dev_x_, 2),
                pow(car_placement_profile.parking_spot_generation_std_dev_y_, 2),
                pow(car_placement_profile.parking_spot_generation_std_dev_theta_, 2));
        car_placement_config.obj_placement_config_.emplace_back(object_occurrence);
    }

    std::string car_class = "car";
    synthetic_problem::ObjectPlacementConfigurationAllClasses<pose::Pose2d, 3> object_config_all_classes;
    object_config_all_classes.obj_placement_configs_by_class_[car_class] = car_placement_config;

    std::unordered_map<std::string, std::pair<double, double>> valid_percent_filled_range;
    valid_percent_filled_range[car_class] = std::make_pair(distribution_config.percent_parking_spots_filled_,
                                                           distribution_config.percent_parking_spots_filled_);

    std::vector<std::unordered_map<std::string, std::vector<pose::Pose2d>>> gt_object_placements =
            synthetic_problem::getObjectInstantiationsFromConfiguration(
                    object_config_all_classes, distribution_config.num_trajectories_, valid_percent_filled_range,
                    random_generator);

    std::vector<std::vector<pose::Pose2d>> similar_trajectories = synthetic_problem::generateSimilarTrajectories(
            base_trajectory,
            Eigen::Vector3d(pow(distribution_config.trajectory_variation_std_dev_x_, 2),
                            pow(distribution_config.trajectory_variation_std_dev_y_, 2),
                            pow(distribution_config.trajectory_variation_std_dev_theta_, 2)),
            distribution_config.num_trajectories_,
            random_generator);
    LOG(INFO) << "Similar trajectories count " << similar_trajectories.size();

    synthetic_problem::ScanGenerationParams2d scan_gen_params;
    scan_gen_params.min_range_ = distribution_config.scan_min_range_;
    scan_gen_params.max_range_ = distribution_config.scan_max_range_;
    scan_gen_params.num_beams_ = distribution_config.scan_num_beams_;
    scan_gen_params.min_angle_ = distribution_config.scan_min_angle_;
    scan_gen_params.max_angle_ = distribution_config.scan_max_angle_;

    std::vector<std::vector<std::pair<pose::Pose2d, SensorInfo<pose::Pose2d, 3, sensor_msgs::LaserScan>>>> sensor_info_for_past_trajectories =
            synthetic_problem::generateSensorInfoForTrajectories<pose::Pose2d, 3, sensor_msgs::LaserScan, synthetic_problem::ScanGenerationParams2d>(
                    similar_trajectories, gt_object_placements,
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
    sample_gen_params_by_class[car_class].num_samples_per_beam_ = distribution_config.num_samples_per_beam_;
    sample_gen_params_by_class[car_class].percent_beams_per_scan_ = distribution_config.percent_beams_per_scan_;
    sample_gen_params_by_class[car_class].percent_poses_to_include_ = distribution_config.percent_poses_to_sample_;

    std::unordered_map<std::string, std::vector<std::pair<pose::Pose2d, double>>> samples_for_prev_trajectories = generateSamplesForPastTrajectories(
            sensor_info_for_past_trajectories, sample_value_generator,
            sample_gen_params_by_class,
            random_generator);

    std::vector<file_io::PastSample2d> past_samples_to_write;
    for (const auto &samples_for_class : samples_for_prev_trajectories) {
        for (const std::pair<pose::Pose2d, double> &past_sample_for_class : samples_for_class.second) {
            file_io::PastSample2d sample;
            sample.semantic_class_ = samples_for_class.first;
            sample.transl_x_ = past_sample_for_class.first.first.x();
            sample.transl_y_ = past_sample_for_class.first.first.y();
            sample.theta_ = past_sample_for_class.first.second;
            sample.value_ = past_sample_for_class.second;
            past_samples_to_write.emplace_back(sample);
        }
    };

    // TODO visualize?
    LOG(INFO) << "Visualizing " << samples_for_prev_trajectories[car_class].size() << " samples ";
    vis_manager->displayTrueObjPoses(parking_spots_global_frame, car_class);
    vis_manager->displayPastSampleValues(car_class, samples_for_prev_trajectories[car_class]);

    writePastSamples2dToFile(sample_output_file, past_samples_to_write);

    return 0;
}