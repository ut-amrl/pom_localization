#include <glog/logging.h>
#include <pose_optimization/odometry_3d_cost_functor.h>
#include <pose_optimization/pose_3d_factor_graph.h>
#include <pose_optimization/pose_graph_optimizer.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <util/random.h>

#include <gaussian_process/kernel_density_estimator.h>

#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_2d.h>
#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_3d.h>
#include <synthetic_problem/synthetic_problem_runner_2d.h>

#include <base_lib/pose_reps.h>
#include <visualization/ros_visualization.h>
#include <pose_optimization/offline/offline_problem_runner.h>

#include <ros/ros.h>

#include <synthetic_problem/parking_lot_simulator.h>

using namespace pose;



pose::Pose2d addGaussianNoise(const pose::Pose2d &original_pose_2d, const double &x_std_dev,
                              const double &y_std_dev, const double &theta_std_dev,
                              util_random::Random &rand_gen) {
    return std::make_pair(Eigen::Vector2d(rand_gen.Gaussian(original_pose_2d.first.x(), x_std_dev),
                                          rand_gen.Gaussian(original_pose_2d.first.y(), y_std_dev)),
                          rand_gen.Gaussian(original_pose_2d.second, theta_std_dev));
}
// TODO sample angle from Gaussian in SO(3)
Pose3d addGaussianNoise(const Pose3d &original_pose_3d, const float &x_std_dev, const float &y_std_dev,
                        const float &z_std_dev, const float &yaw_std_dev, util_random::Random &rand_gen) {
    Eigen::Vector3d transl(rand_gen.Gaussian(original_pose_3d.first.x(), x_std_dev),
                           rand_gen.Gaussian(original_pose_3d.first.y(), y_std_dev),
                           rand_gen.Gaussian(original_pose_3d.first.z(), z_std_dev));
    Eigen::Quaterniond noise_rot;
    noise_rot = Eigen::AngleAxisd(rand_gen.Gaussian(0, yaw_std_dev), Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond rot = original_pose_3d.second * noise_rot;
    return std::make_pair(transl, rot);
}


std::vector<Pose2d> createGroundTruthPoses() {
    std::vector<Pose2d> poses;
    poses.emplace_back(createPose2d(0, 0, 0)); // Should test how this deals with non-zero-origins?
    poses.emplace_back(createPose2d(0.1, 1, 0));
    poses.emplace_back(createPose2d(0, 4, M_PI_2));
    poses.emplace_back(createPose2d(-.04, 7, M_PI_2));
    poses.emplace_back(createPose2d(0, 10, M_PI_2));
    poses.emplace_back(createPose2d(0.3, 13, M_PI_2));
    poses.emplace_back(createPose2d(0.7, 15, M_PI_4));
    poses.emplace_back(createPose2d(2, 17, 0));
    poses.emplace_back(createPose2d(4, 18, 0));
    poses.emplace_back(createPose2d(7, 18, 0));
    poses.emplace_back(createPose2d(10, 17.5, 0));
    poses.emplace_back(createPose2d(12, 15, -M_PI_4));
    poses.emplace_back(createPose2d(12, 12, -M_PI_2));
    poses.emplace_back(createPose2d(11.5, 9, -M_PI_2));
    poses.emplace_back(createPose2d(11.7, 6, -M_PI_2));
    poses.emplace_back(createPose2d(11.3, 3, -M_PI_2));
    poses.emplace_back(createPose2d(11, -1, -(M_PI_2 + M_PI_4)));
    poses.emplace_back(createPose2d(9, -0.0, M_PI));
    poses.emplace_back(createPose2d(6, -0.9, M_PI));
    poses.emplace_back(createPose2d(3, -0.7, M_PI));

//    poses.emplace_back(createPose2d(0, 0, 0)); // Should test how this deals with non-zero-origins?
//    poses.emplace_back(createPose2d(0.1, 1, 0));
//    poses.emplace_back(createPose2d(0, 4, 0));
//    poses.emplace_back(createPose2d(-.04, 7, 0));
//    poses.emplace_back(createPose2d(0, 10, 0));
//    poses.emplace_back(createPose2d(0.3, 13, 0));
//    poses.emplace_back(createPose2d(0.7, 15, 0));
//    poses.emplace_back(createPose2d(2, 17, 0));
//    poses.emplace_back(createPose2d(4, 18, 0));
//    poses.emplace_back(createPose2d(7, 18, 0));
//    poses.emplace_back(createPose2d(10, 17.5, 0));
//    poses.emplace_back(createPose2d(12, 15, 0));
//    poses.emplace_back(createPose2d(12, 12, 0));
//    poses.emplace_back(createPose2d(11.5, 9, 0));
//    poses.emplace_back(createPose2d(11.7, 6, 0));
//    poses.emplace_back(createPose2d(11.3, 3, 0));
//    poses.emplace_back(createPose2d(11, -1, 0));
//    poses.emplace_back(createPose2d(9, -0.0, 0));
//    poses.emplace_back(createPose2d(6, -0.9, 0));
//    poses.emplace_back(createPose2d(3, -0.7, 0));
    return poses;
}

std::vector<Pose2d> createParkedCarPoses() {
    std::vector<Pose2d> poses;
    poses.emplace_back(createPose2d(-3, 2, -(M_PI * 5.0/6.0)));
    poses.emplace_back(createPose2d(-3, 4, -(M_PI * 5.0/6.0)));
    poses.emplace_back(createPose2d(-7, 6, -(M_PI / 6.0)));

    poses.emplace_back(createPose2d(3, 2, -(M_PI /6.0)));
    poses.emplace_back(createPose2d(3, 6, -(M_PI /6.0)));
    poses.emplace_back(createPose2d(3, 8, -(M_PI /6.0)));
    poses.emplace_back(createPose2d(3, 14, -(M_PI /6.0)));

    poses.emplace_back(createPose2d(7, 12, -(M_PI * 5.0 /6.0)));
    poses.emplace_back(createPose2d(7, 10, -(M_PI * 5.0/6.0)));
    poses.emplace_back(createPose2d(7, 6, -(M_PI * 5.0 /6.0)));
    poses.emplace_back(createPose2d(7, 2, -(M_PI * 5.0 /6.0)));
    return poses;
}

std::vector<std::pair<Pose2d, unsigned int>> createParkedCarPosesWithFrequency() {
    std::vector<std::pair<Pose2d, unsigned int>> poses;
    poses.emplace_back(std::make_pair(createPose2d(-3, 2, -(M_PI * 5.0/6.0)), 10));
    poses.emplace_back(std::make_pair(createPose2d(-3, 4, -(M_PI * 5.0/6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(-7, 6, -(M_PI / 6.0)), 2));

    poses.emplace_back(std::make_pair(createPose2d(3, 2, -(M_PI /6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(3, 6, -(M_PI /6.0)), 3));
    poses.emplace_back(std::make_pair(createPose2d(3, 8, -(M_PI /6.0)), 2));
    poses.emplace_back(std::make_pair(createPose2d(3, 14, -(M_PI /6.0)), 1));

    poses.emplace_back(std::make_pair(createPose2d(7, 12, -(M_PI * 5.0 /6.0)), 3));
    poses.emplace_back(std::make_pair(createPose2d(7, 10, -(M_PI * 5.0/6.0)), 1));
    poses.emplace_back(std::make_pair(createPose2d(7, 6, -(M_PI * 5.0 /6.0)), 2));
    poses.emplace_back(std::make_pair(createPose2d(7, 2, -(M_PI * 5.0 /6.0)), 2));
    return poses;
}

std::vector<Eigen::Vector2d> createNegativeObservations() {
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> exclude_regions;
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2d(-9, 0), Eigen::Vector2d(1, 10)));
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2d(1, 0), Eigen::Vector2d(9, 16)));

    std::vector<Eigen::Vector2d> negative_observations;
    for (int x = -15; x <= 20; x++) {
        for (int y = -5; y <= 25; y++) {
            bool in_exclude_region = false;
            for (const std::pair<Eigen::Vector2d, Eigen::Vector2d> &exclude_region : exclude_regions) {
                double x_min = exclude_region.first.x();
                double x_max = exclude_region.second.x();
                double y_min = exclude_region.first.y();
                double y_max = exclude_region.second.y();

                if ((y >= y_min) && (y <= y_max) && (x >= x_min) && (x <= x_max)) {
                    in_exclude_region = true;
                    break;
                }
            }

            if (!in_exclude_region) {
                negative_observations.emplace_back(Eigen::Vector2d(x, y));
            }
        }
    }
    return negative_observations;
}

void outputToCsv(const std::string &file_name, const std::vector<Pose2d> &poses) {
// TODO
}

void callSyntheticProblem(const std::shared_ptr<visualization::VisualizationManager> &vis_manager) {

    LOG(INFO) << "Setting up synthetic problem";
    synthetic_problem::SyntheticProblemRunner2d synthetic_prob_runner(vis_manager);


    std::vector<pose::Pose2d> ground_truth_trajectory = createGroundTruthPoses();
//    std::vector<pose::Pose2d> car_poses = createParkedCarPoses();
    std::vector<pose::Pose2d> past_car_poses;
    std::vector<pose::Pose2d> current_car_poses;
    double parking_lot_std_dev_x = 0.2;
    double parking_lot_std_dev_y = 0.2;
    double parking_lot_std_dev_yaw = 0.075;
    int32_t num_samples_multiplier = 4;
    double percent_parking_lot_filled = 0.6;
//    const double &parking_lot_std_dev_x, const double &parking_lot_std_dev_y,
//    const double &parking_lot_std_dev_yaw,
//    const double &percent_filled,
//    std::vector<pose::Pose2d> &past_observations,
//    std::vector<pose::Pose2d> &current_placements
    synthetic_problem::createPastAndPresentObservations(createParkedCarPosesWithFrequency(), parking_lot_std_dev_x,
                                                        parking_lot_std_dev_y, parking_lot_std_dev_yaw, num_samples_multiplier,
                                                        percent_parking_lot_filled, past_car_poses, current_car_poses);
    std::string car_class = "car_class";
//    std::unordered_map<std::string, std::vector<pose::Pose2d>> mov_obj_positions_by_class = {{car_class, car_poses}};
    std::unordered_map<std::string, std::vector<pose::Pose2d>> past_mov_obj_positions_by_class = {{car_class, past_car_poses}};
    std::unordered_map<std::string, std::vector<pose::Pose2d>> curr_mov_obj_positions_by_class = {{car_class, current_car_poses}};

    synthetic_problem::SyntheticProblemNoiseConfig2d noise_config;
    noise_config.add_additional_initial_noise_ = false;
    noise_config.odometry_x_std_dev_ = 0.6;
    noise_config.odometry_y_std_dev_ = 0.6;
    noise_config.odometry_yaw_std_dev_ = 0.2;
    noise_config.max_observable_moving_obj_distance_ = 8.0;
    noise_config.movable_observation_x_std_dev_ = 0.00005;
    noise_config.movable_observation_y_std_dev_ = 0.00005;
    noise_config.movable_observation_yaw_std_dev_ = 0.00005;

    pose_optimization::CostFunctionParameters cost_function_params;
    pose_optimization::PoseOptimizationParameters optimization_params;
    LOG(INFO) << "Calling run synthetic problem";
    synthetic_prob_runner.runSyntheticProblem(
            ground_truth_trajectory,
            curr_mov_obj_positions_by_class, //mov_obj_positions_by_class, // TODO make this different
            past_mov_obj_positions_by_class, //mov_obj_positions_by_class,
            noise_config,
            cost_function_params,
            optimization_params);
}

int main(int argc, char** argv) {
    ros::init(argc, argv,
              "momo_demo");
    ros::NodeHandle n;
    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(n);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    callSyntheticProblem(manager);

    return 0;
}