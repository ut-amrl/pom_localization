#include <glog/logging.h>

#include <ros/ros.h>
#include <iostream>
#include <file_io/synthetic_distribution_generation_config_multiple_car_selection_profiles.h>

DEFINE_string(cfg_file_name, "xxx", "Config file name");

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);
    ros::init(argc, argv,
              "write_distribution");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    // Get config ----------------------------------------------------------------------------------------------------
    std::string distribution_gen_config_file = FLAGS_cfg_file_name;
    file_io::SyntheticDistributionGenerationConfigMultipleSelectionNoiseProfiles distribution_config;
    distribution_config.num_trajectories_ = 1;
    distribution_config.num_samples_per_beam_ = 1;
    distribution_config.scan_num_beams_ = 100;

    distribution_config.percent_parking_spots_filled_ = 1;

    distribution_config.trajectory_variation_std_dev_x_ = 0;
    distribution_config.trajectory_variation_std_dev_y_ = 0;
    distribution_config.trajectory_variation_std_dev_theta_ = 0;

    // TODO revisit these
    distribution_config.object_detection_variance_per_len_x_ = 0.02;
    distribution_config.object_detection_variance_per_len_y_ = 0.02;
    distribution_config.object_detection_variance_per_len_theta_ = 0.05;

    // TODO revisit
    distribution_config.max_object_detection_distance_ = 40;

    distribution_config.scan_min_angle_ = -M_PI;
    distribution_config.scan_max_angle_ = M_PI;
    distribution_config.scan_min_range_ = 0.2;
    distribution_config.scan_max_range_ = 40;

    distribution_config.percent_beams_per_scan_ = 0.1;
    distribution_config.percent_poses_to_sample_ = 0.2;

    file_io::CarPlacementNoiseProfile noise_profile_1;
    noise_profile_1.parking_spot_generation_std_dev_x_ = 0.4;
    noise_profile_1.parking_spot_generation_std_dev_y_ = 0.4;
    noise_profile_1.parking_spot_generation_std_dev_theta_ = 0.2;

    distribution_config.car_placement_profile.emplace_back(std::make_pair(0.2, noise_profile_1));

    file_io::CarPlacementNoiseProfile noise_profile_2;
    noise_profile_2.parking_spot_generation_std_dev_x_ = 20;
    noise_profile_2.parking_spot_generation_std_dev_y_ = 20;
    noise_profile_2.parking_spot_generation_std_dev_theta_ = 20;

    distribution_config.car_placement_profile.emplace_back(std::make_pair(0.8, noise_profile_2));

    file_io::writeSyntheticDistributionConfigToFile(distribution_gen_config_file, distribution_config);

    return 0;
}