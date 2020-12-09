#include <glog/logging.h>
#include <pose_optimization/odometry_cost_functor.h>
#include <pose_optimization/pose_3d_factor_graph.h>
#include <pose_optimization/pose_graph_optimizer.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <util/random.h>

typedef std::pair<Eigen::Vector2f, float> Pose2d;
typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> Pose3d;

Pose2d createPose2d(const float &x, const float &y, const float &theta) {
    return std::make_pair(Eigen::Vector2f(x, y), theta);
}

Pose3d toPose3d(Pose2d pose_2d) {
    Eigen::Vector3d transl(pose_2d.first.x(), pose_2d.first.y(), 0);
    Eigen::Quaterniond rot;
    rot = Eigen::AngleAxisd(pose_2d.second, Eigen::Vector3d::UnitZ());
    return std::make_pair(transl, rot);
}

Pose2d addGaussianNoise(const Pose2d &original_pose_2d, const float &x_std_dev, const float &y_std_dev, const float &theta_std_dev, util_random::Random &rand_gen) {
    return std::make_pair(Eigen::Vector2f(rand_gen.Gaussian(original_pose_2d.first.x(), x_std_dev), rand_gen.Gaussian(original_pose_2d.first.y(), y_std_dev)), rand_gen.Gaussian(original_pose_2d.second, theta_std_dev));
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

Eigen::Affine3d convertPoseToAffine(Pose3d &pose) {
    Eigen::Affine3d mat;
    mat.translation() = pose.first;
    mat.linear() = pose.second.toRotationMatrix();
    return mat;
}

Pose3d convertAffineToPose(Eigen::Affine3d &mat) {
    return std::make_pair(mat.translation(), Eigen::Quaterniond(mat.linear()));
}

Pose3d getPoseOfObj1RelToObj2(Pose3d obj1, Pose3d obj2) {

    Eigen::Affine3d affine_1 = convertPoseToAffine(obj1);
    Eigen::Affine3d affine_2 = convertPoseToAffine(obj2);
    Eigen::Affine3d combined_affine = affine_2.inverse() * affine_1;
    return convertAffineToPose(combined_affine);
}

/**
 * Get the pose of object 2 in the frame that pose 1 is relative to.
 *
 * Ex. if pose_1 is in the map frame and pose_2 is relative to pose_1, this returns the position of the coordinate
 * frame for pose 2 in the map frame.
 *
 * @param pose_1
 * @param pose_2
 * @return
 */
Pose3d combinePoses(Pose3d pose_1, Pose3d pose_2) {
    Eigen::Affine3d affine_1 = convertPoseToAffine(pose_1);
    Eigen::Affine3d affine_2 = convertPoseToAffine(pose_2);
    Eigen::Affine3d combined_affine = affine_1 * affine_2;
    return convertAffineToPose(combined_affine);
}

std::vector<Pose2d> createGroundTruthPoses() {
    std::vector<Pose2d> poses;
    poses.emplace_back(createPose2d(0, 0, 0)); // Should test how this deals with non-zero-origins?
    poses.emplace_back(createPose2d(1, 1, M_PI_4));
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

std::vector<Eigen::Vector2f> createNegativeObservations() {
    std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> exclude_regions;
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2f(-9, 0), Eigen::Vector2f(1, 10)));
    exclude_regions.emplace_back(std::make_pair(Eigen::Vector2f(1, 0), Eigen::Vector2f(9, 16)));

    std::vector<Eigen::Vector2f> negative_observations;
    for (int x = -15; x <= 20; x++) {
        for (int y = -5; y <= 25; y++) {
            bool in_exclude_region = false;
            for (const std::pair<Eigen::Vector2f, Eigen::Vector2f> &exclude_region : exclude_regions) {
                float x_min = exclude_region.first.x();
                float x_max = exclude_region.second.x();
                float y_min = exclude_region.first.y();
                float y_max = exclude_region.second.y();

                if ((y >= y_min) && (y <= y_max) && (x >= x_min) && (x <= x_max)) {
                    in_exclude_region = true;
                    break;
                }
            }

            if (!in_exclude_region) {
                negative_observations.emplace_back(Eigen::Vector2f(x, y));
            }
        }
    }
    return negative_observations;
}

void outputToCsv(const std::string &file_name, const std::vector<Pose2d> &poses) {
// TODO
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string car_class = "car";
    // TODO the current values are based on the trained hyperparams for the toy observations I used in the python
    //  demoscript
    // We should find better params for a broader class of examples, or figure out how to hook this up to GPflow to get
    // param values at the beginning of a run
    float position_kernel_len = 0.669073; // tODO get this from config
    float position_kernel_var = 0.539952; // TODO get this from config
    float orientation_kernel_len = 0.539952; // tODO get this from config
    float orientation_kernel_var = 15.7752; // TODO get this from config

    float movable_observation_x_std_dev = 0.6;
    float movable_observation_y_std_dev = 0.6;
    float movable_observation_z_std_dev = 0.6;
    float movable_observation_yaw_std_dev = 0.3;

    float odometry_x_std_dev = 0.4;
    float odometry_y_std_dev = 0.4;
    float odometry_z_std_dev = 0.4;
    float odometry_yaw_std_dev = 0.3;

    std::vector<Pose2d> robot_gt_poses_2d = createGroundTruthPoses();
    std::vector<Pose2d> filled_parking_spots = createParkedCarPoses();

    std::vector<Pose2d> previous_car_locations = filled_parking_spots; // tODO at some point, we should replace this with observatiosn at all parking spots
    std::vector<Eigen::Vector2f> negative_car_observations = createNegativeObservations();

    // Create 3d robot poses from 2d
    std::vector<Pose3d> robot_gt_poses_3d;
    for (const Pose2d &robot_pose_gt : robot_gt_poses_2d) {
        robot_gt_poses_3d.emplace_back(toPose3d(robot_pose_gt));
    }

    // TODO Perturb/duplicate parking spots in future iterations
    std::vector<Pose2d> car_poses_in_parking_spots_2d = filled_parking_spots;

    // Create the car locations in 3d
    std::vector<Pose3d> car_poses_in_parking_spots_3d;
    for (const Pose2d &car_pose_gt : car_poses_in_parking_spots_2d) {
        car_poses_in_parking_spots_3d.emplace_back(toPose3d(car_pose_gt));
    }

    double distance_limit = 8; // Robot can only see cars 8 m away

    // For each pose in the trajectory, get the true position of the objects relative to the robot (for objects within
    // some radius)
    std::vector<std::vector<Pose3d>> true_transforms_at_bot_poses;
    for (size_t i = 0; i < robot_gt_poses_3d.size(); i++) {
        std::vector<Pose3d> true_obj_transforms;
        for (const Pose3d &car_pose : car_poses_in_parking_spots_3d) {
            if ((robot_gt_poses_3d[i].first - car_pose.first).norm() <= distance_limit) {
                true_obj_transforms.emplace_back(getPoseOfObj1RelToObj2(car_pose, robot_gt_poses_3d[i]));
            }
        }
        true_transforms_at_bot_poses.emplace_back(true_obj_transforms);
    }

    util_random::Random random_generator;

    // Add noise to true_transforms_at_bot_poses
    std::vector<std::vector<Pose3d>> noisy_observations;
    for (const std::vector<Pose3d> &true_observations : true_transforms_at_bot_poses) {
        std::vector<Pose3d> noisy_at_pose;
        for (const Pose3d &true_obs : true_observations) {
            noisy_at_pose.emplace_back(addGaussianNoise(true_obs, movable_observation_x_std_dev,
                                                        movable_observation_y_std_dev, movable_observation_z_std_dev,
                                                        movable_observation_yaw_std_dev, random_generator));
        }
        noisy_observations.emplace_back(noisy_at_pose);
    }


    std::vector<Pose3d> true_odometry;
    for (size_t i = 1; i < robot_gt_poses_3d.size(); i++) {
        true_odometry.emplace_back(getPoseOfObj1RelToObj2(robot_gt_poses_3d[i], robot_gt_poses_3d[i-1]));
    }

    std::vector<Pose3d> noisy_odometry; // TODO add noise to true odometry
    for (const Pose3d &true_odom: true_odometry) {
        noisy_odometry.emplace_back(addGaussianNoise(true_odom, odometry_x_std_dev,
                                                     odometry_y_std_dev, odometry_z_std_dev,
                                                     odometry_yaw_std_dev, random_generator));
    }

    gp_kernel::GaussianKernel<2> position_kernel(position_kernel_len, position_kernel_var);
    gp_kernel::PeriodicGaussianKernel<1> orientation_kernel(M_PI * 2, orientation_kernel_var, orientation_kernel_len);
    gp_kernel::Pose2dKernel pose_2d_kernel(position_kernel, orientation_kernel);

    pose_graph::PoseGraph pose_graph(pose_2d_kernel);

    std::vector<Pose3d> initial_node_positions;
    Pose3d prev_pose = toPose3d(createPose2d(0, 0, 0));
    initial_node_positions.emplace_back(prev_pose);
    for (const Pose3d &odom_to_next_pos : noisy_odometry) {
        Pose3d new_pose = combinePoses(prev_pose, odom_to_next_pos);
        initial_node_positions.emplace_back(new_pose);
        prev_pose = new_pose;
    }

    // Create nodes with initial positions based on odometry
    for (size_t i = 0; i < initial_node_positions.size(); i++) {
        pose_graph::Node node;
        node.id_ = i;
        node.est_position_ = std::make_shared<Eigen::Vector3d>(initial_node_positions[i].first);
        node.est_orientation_ = std::make_shared<Eigen::Quaterniond>(initial_node_positions[i].second);
        pose_graph.addNode(node);
    }

    // Create the movable observations at each node from the noisy observations
    for (uint64_t i = 0; i < noisy_observations.size(); i++) {
        std::vector<pose_graph::MovableObservation3D> movable_observations;
        for (const Pose3d &noisy_obs_at_node : noisy_observations[i]) {
            pose_graph::MovableObservation3D obs;
            obs.semantic_class_ = car_class;
            obs.observation_transl_ = noisy_obs_at_node.first.cast<float>();
            obs.observation_orientation_ = noisy_obs_at_node.second.cast<float>();
            movable_observations.emplace_back(obs);
        }
        pose_graph.addMovableObservationFactors(i, movable_observations);
    }

    for (uint64_t i = 0; i < noisy_odometry.size(); i++) {
        pose_graph::NodeId prev_node = i;
        pose_graph::NodeId to_node = i + 1;
        pose_graph::GaussianBinaryFactor factor;
        factor.to_node_ = to_node;
        factor.from_node_ = prev_node;
        factor.translation_change_ = noisy_odometry[i].first;
        factor.orientation_change_ = noisy_odometry[i].second;
        Eigen::Matrix<double, 6, 6> cov_mat = Eigen::Matrix<double, 6, 6>::Zero();
        cov_mat(0, 0) = pow(odometry_x_std_dev, 2);
        cov_mat(1, 1) = pow(odometry_y_std_dev, 2);
        cov_mat(2, 2) = pow(odometry_z_std_dev, 2);

        // Fix these - probably not right...
        cov_mat(3, 3) = pow(odometry_yaw_std_dev, 2);
        cov_mat(4, 4) = pow(odometry_yaw_std_dev, 2);
        cov_mat(5, 5) = pow(odometry_yaw_std_dev, 2);

        Eigen::Matrix<double, 6, 6> information_mat = cov_mat.inverse();
        factor.sqrt_information_ = information_mat.sqrt();

        // TODO sqrt information matrix
        pose_graph.addGaussianBinaryFactor(factor);
    }


    std::unordered_map<std::string, std::pair<std::vector<pose_graph::NegativeMovableObservation2D>, std::vector<pose_graph::MapObservation2D>>> obs_by_class;
    std::vector<pose_graph::NegativeMovableObservation2D> neg_obs;
    for (const Eigen::Vector2f &neg_obs_pos : negative_car_observations) {
        pose_graph::NegativeMovableObservation2D negative_observation;
        negative_observation.semantic_class_ = car_class;
        negative_observation.transl_ = neg_obs_pos;
        neg_obs.emplace_back(negative_observation);
    }

    std::vector<pose_graph::MapObservation2D> pos_observations;
    for (const Pose2d &obs_pos : previous_car_locations) {
        pose_graph::MapObservation2D observation;
        observation.semantic_class_ = car_class;
        observation.transl_ = obs_pos.first;
        observation.orientation_ = obs_pos.second;
        pos_observations.emplace_back(observation);
    }
    obs_by_class[car_class] = std::make_pair(neg_obs, pos_observations);
    LOG(INFO) << "Adding map frame observations to regressor";
    pose_graph.addMapFrameObservations(obs_by_class);
    LOG(INFO) << "Done adding map frame observations to regressor";

    pose_optimization::PoseGraphOptimizer optimizer;
    ceres::Problem problem;
    LOG(INFO) << "Building pose graph optimization problem";
    optimizer.buildPoseGraphOptimizationProblem(pose_graph, &problem);
    LOG(INFO) << "Solving optimization problem";
    optimizer.SolveOptimizationProblem(&problem);
    LOG(INFO)<<"Done solving optimization problem";

    std::unordered_map<pose_graph::NodeId, Pose3d> node_poses;
    pose_graph.getNodePoses(node_poses);

    for (size_t i = 0; i < initial_node_positions.size(); i++) {
        Pose3d init_pose = initial_node_positions[i];
        Pose3d optimized_pose = node_poses[i];
        Pose2d gt_2d = robot_gt_poses_2d[i];
        LOG(INFO) << "True pose " << gt_2d.first.x() << ", " << gt_2d.first.y() << ", " << gt_2d.second;
        LOG(INFO) << "Init est " << init_pose.first.x() << ", " << init_pose.first.y() << ", " << init_pose.first.z() << ", " << init_pose.second.z() << ", " << init_pose.second.w() << ", " << init_pose.second.x() << ", " << init_pose.second.y();
        LOG(INFO) << "Opt est " << optimized_pose.first.x() << ", " << optimized_pose.first.y() << ", " << optimized_pose.first.z() << ", " << optimized_pose.second.z() << ", " << optimized_pose.second.w() << ", " << optimized_pose.second.x() << ", " << optimized_pose.second.y();
    }


    // TODO output updated poses

    return 0;
}