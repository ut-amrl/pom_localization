//
// Created by amanda on 3/18/21.
//
#include <glog/logging.h>
#include <ros/ros.h>
#include <visualization/ros_visualization.h>
#include <synthetic_problem/uncertainty_aware/uncertainty_aware_synthetic_problem.h>

double runSingleSyntheticProblem(const std::shared_ptr<visualization::VisualizationManager> &vis_manager) {

    // Subtask: Generate canonical trajectory
    // Input: None
    // Output: Sequence of poses
    // Scope: Particular run of synthetic problem

    // Subtask: Generate object placement configuration
    // Input: None
    // Output: map of object class to object placement config
    //      Object placement config: list of {object mean pose, object relative frequency (positive integer), object pose variance (3 dimensional)}
    // Scope: Particular run of synthetic problem

    // Subtask: Generate actual trajectories (including current trajectory on which to localize)
    // Input: Canonical trajectory, variance in each dimension for how much poses can spread
    // Output: Vector of vector of poses
    // Question: How do we want to apply noise? Noise per edge or noise around each pose in trajectory?
    // Scope: Any synthetic problem

    // Subtask: Generate object configuration per trajectory
    // Input: Number of trajectories (add 1 for the current trajectory), map of object class to object placement config, map of object class to valid range of % object placements filled
    // Output: list of map of object class to object poses (item 1 in list corresponds to trajectory 1)
    // Scope: Any synthetic problem
    // Question: Should this output objects by class or just a list of objects?
    // Sub-subtask: Generate object poses given the % filled
    // Input: object placement config, % of spots filled
    // Output: list of object poses

    // Subtask: Generate sensor info for each pose in each previous trajectory
    // Input: Previous trajectories (vector of vector of poses),  list of map of object class to object poses (item 1 in list corresponds to trajectory 1), object detection noise (variance in x, y, theta), maximum object detection distance, laser max range, laser max angle, laser min angle, laser angle resolution, robot poses,
    // Output: Vector of vector of sensor info -- each sensor info has laser scan (will just be max range for now), map of object class to list of detections (relative pose to respective pose in trajectory + noise)
    // Scope: Any synthetic problem
    // Subsubtask: Generate fake lidar scan
    // Input: Max range, min angle, max angle, beam count
    // Output: Scan data (need to determine format for this)
    // Subsubtask: Get detections relative to robot
    // Input: Maximum detection range, object positions for trajectory, robot pose
    // Output: Object detections relative to robot + noise

    // Subtask: Generate samples from past trajectory info
    // Input: past trajectories, where each trajectory contains a list of {robot pose, laser scan, map <object class, list of object detections relative to robot>, map of object class to percentage of poses to include, map of object class to beams to use per lidar scan, map of object class to  number of samples per beam, sample value generator
    // Output: map<class, list of samples <pose, value>> (both on and off detection). Value should be in range 0-1.
    // Scope: Any problems (real and synthetic)
    // Subsubtask: Generate sample locations for a particular pose
    // Input: Robot pose, detections relative to robot pose, lidar scan, beam fraction to use, number of samples per beam
    // Output: map<class, list of samples <pose, value>> (both on and off detection). Value should be in range 0-1.
    // Subsubtask: Generate sample value for a sample location
    // Input: sample location (global frame), object detections for the class (global frame) with variance (pair<pose, variance in x, y, yaw>)
    // Output: value at the sample location

    // Subtask: Generate current detections
    // Input: Current trajectory poses, current trajectory object pose configuration (map of class to list of poses), detection noise parameters
    // Output: vector of map of object class to list of detections (relative pose to respective pose in trajectory + noise)
    // Scope: Synthetic
    // Similar to generating for past detections

    // Subtask: add noise to true trajectory
    // Input: Ground truth poses, odometry variance (x, y, yaw)
    // Output: Poses (with noise added)
    // Scope: Synthetic

    // Subtask: map samples onto real line
    // Input: Samples, mapping function
    // Output: mapped sample values

    // Subtask: Add prior samples to pose graph
    // TODO

    // Subtask: Add current movable observation factors to pose graph
    // TODO

    // Subtask: Add odom factors to pose graph
    // TODO

    // Run optimization
    // TODO



    // Subtask: Generate

    // Also todo: Add new cost functor math
    // Also todo: Make cost functor use GP

    return 0;

}

int main(int argc, char** argv) {
    ros::init(argc, argv,
              "uncertainty_aware_synthetic_demo");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(n);


    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
    std::string time_str = oss.str();
    std::string csv_file_name = "results/noise_eval_" + time_str + ".csv";

    LOG(INFO) << runSingleSyntheticProblem(manager);
//    runSyntheticProblemWithConfigVariations(manager, createParkedCarPosesWithFrequency(), createGroundTruthPoses(),
//                                            csv_file_name);

    return 0;
}
