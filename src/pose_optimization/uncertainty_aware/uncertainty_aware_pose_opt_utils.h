//
// Created by amanda on 3/19/21.
//

#ifndef AUTODIFF_GP_UNCERTAINTY_AWARE_POSE_OPT_UTILS_H
#define AUTODIFF_GP_UNCERTAINTY_AWARE_POSE_OPT_UTILS_H

#include <sensor_msgs/LaserScan.h>

#include <shared/util/random.h>

#include <base_lib/pose_reps.h>
#include <math/math_util.h>
#include <math/statistics.h>

namespace pose_optimization {

    double kMinProbRange = 1e-2;
    double kMaxProbRange = 1.0 - kMinProbRange;
    double kLimitedProbRange = kMaxProbRange - kMinProbRange;

    template<typename PoseType, int PoseParamCount>
    struct ObjectDetectionRelRobot {
        PoseType pose_;

        // TODO should this be square matrix or just variances
        /**
         * Contains the variance of the object's pose (consistency in how it is placed).
         *
         * Will be XYZ, RPY for 3D and X, Y, Yaw for 2D.
         */
        Eigen::Matrix<double, PoseParamCount, 1> object_pose_variance_;
    };

    template<typename PoseType, int PoseParamCount, typename ScanType>
    struct SensorInfo {
        std::unordered_map<std::string, std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>>> object_detections_;
        ScanType scan_info_;
    };

    struct SampleGeneratorParams2d {
        double percent_beams_per_scan_;
        double num_samples_per_beam_;

        // TODO actually use this
        double percent_poses_to_include_;
    };

    std::vector<pose::Pose2d> getSamplePosesFromScanData(const sensor_msgs::LaserScan &laser_scan,
                                                         const SampleGeneratorParams2d &sample_gen_params,
                                                         util_random::Random &rand_gen) {

        // Pick a random beam to start
        size_t start_beam = rand_gen.RandomInt(0, (int) laser_scan.ranges.size() - 1);
        bool first_beam = true;
        size_t next_beam = start_beam;
        size_t iterated_beam_count = 0;
        size_t included_beam_count = 0;
        std::vector<pose::Pose2d> samples;
        while (first_beam || (next_beam != start_beam)) {
            if ((first_beam) || (((double) included_beam_count / iterated_beam_count) < sample_gen_params.percent_beams_per_scan_)) {
                included_beam_count++;
                double angle_to_point = laser_scan.angle_min + (next_beam * laser_scan.angle_increment);
                double range_end = laser_scan.ranges[next_beam];
                double range_min = laser_scan.range_min;
                for (unsigned int i = 0; i < sample_gen_params.num_samples_per_beam_; i++) {
                    double sample_range = rand_gen.UniformRandom(range_min, range_end);
                    double random_angle = rand_gen.UniformRandom(0, 2 * M_PI);
                    pose::Pose2d sample_pose = pose::createPose2d(sample_range * cos(angle_to_point),
                                                                  sample_range * sin(angle_to_point),
                                                                  random_angle);
                    samples.emplace_back(sample_pose);
                }
            }
            next_beam++;
            if (next_beam >= laser_scan.ranges.size()) {
                next_beam = 0;
            }
            first_beam = false;
            iterated_beam_count++;
        }
        return samples;
    }


    // Subsubtask: Generate sample locations for a particular pose
    // Input: Robot pose, detections relative to robot pose, lidar scan, beam fraction to use, number of samples per beam
    // Output: map<class, list of samples <pose, value>> (both on and off detection). Value should be in range 0-1.


    template<typename PoseType, int PoseParamCount, typename ScanType, typename SampleGenParamsType>
    std::vector<PoseType> generateSamplePointsForPose(const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &detected_objects,
                                                      const ScanType &scan,
                                                      const SampleGenParamsType &sample_gen_params,
                                                      util_random::Random &rand_gen) {
        std::vector<pose::Pose2d> sample_poses = getSamplePosesFromScanData(scan, sample_gen_params, rand_gen);
//        std::vector<pose::Pose2d> sample_poses;
        for (const ObjectDetectionRelRobot<PoseType, PoseParamCount> &object_detection : detected_objects) {
            sample_poses.emplace_back(object_detection.pose_);
        }
        return sample_poses;
    }

    double squashPdfValueToZeroToOneRangeExponential(const double &pdf_value) {
        // TODO consider having additional scaling to the pdf value
        // TODO need less hacky way to keep this from going to infinity when scaled to real value range
        // Using this for now so we can debug other nan/inf issues and remove this as the source of the problem
        double pre_range_limited = (1 - exp(-1 * (8.0 * pdf_value)));
        double post_range_limited = kMinProbRange + pre_range_limited * kLimitedProbRange;
        return post_range_limited;
    }

    double squashPdfValueToZeroToOneRangeATan(const double &pdf_value) {
        // TODO consider having additional scaling to the pdf value
        return atan(pdf_value) / (M_PI_2);
    }

    double computeGaussianValue(const ObjectDetectionRelRobot<pose::Pose2d, 3> &obj_pose, const pose::Pose2d &sample_pose) {
        // TODO should we instead have a translation component that is normally distributed and a rotation component and just use the distance for each?
        // Assuming since each dimension is independent, that we can just evaluated the gaussian for each dim, and then multiply
//        LOG(INFO) << "Obj pose " << obj_pose.pose_.first.x() << ", " << obj_pose.pose_.first.y() << ", " << obj_pose.pose_.second;
//        LOG(INFO) << "Sample pose " << sample_pose.first.x() << ", " << sample_pose.first.y() << ", " << sample_pose.second;
        LOG(INFO) << "Angle Variance " << obj_pose.object_pose_variance_(2);
        double prob = statistics::ProbabilityDensityGaussian(sample_pose.first.x(), obj_pose.pose_.first.x(),
                                                               2 * sqrt(obj_pose.object_pose_variance_(0)));
//
//        double prob = statistics::ProbabilityDensityGaussian(sample_pose.first.x(), obj_pose.pose_.first.x(),
//                                                             sqrt(obj_pose.object_pose_variance_(0)));
//        LOG(INFO) << "X prob: " << prob;
        double y_prob = statistics::ProbabilityDensityGaussian(sample_pose.first.y(), obj_pose.pose_.first.y(),
                                                               2 * sqrt(obj_pose.object_pose_variance_(1)));

//        double y_prob = statistics::ProbabilityDensityGaussian(sample_pose.first.y(), obj_pose.pose_.first.y(),
//                                                               sqrt(obj_pose.object_pose_variance_(1)));
//        LOG(INFO) << "Y prob: " << y_prob;
        prob *= y_prob;
        // Have to handle this one more carefully since it is periodic... TODO not sure if doing this right (maybe look at von Mises distribution?)
        double yaw_diff = math_util::AngleDiff(obj_pose.pose_.second, sample_pose.second);
//        LOG(INFO) << "Yaw diff " << yaw_diff;
        double yaw_prob = statistics::ProbabilityDensityGaussian(yaw_diff, (double) 0, 2 * sqrt(obj_pose.object_pose_variance_(2)));
//        double yaw_prob = statistics::ProbabilityDensityGaussian(yaw_diff, (double) 0, sqrt(obj_pose.object_pose_variance_(2)));
//        LOG(INFO) << "Yaw prob: " << yaw_prob;
        prob *= yaw_prob;
        LOG(INFO) << "sample value " << prob;
        return prob;
    }

    // Subsubtask: Generate sample value for a sample location
    // Input: sample location (global frame), object detections for the class (global frame) with variance (pair<pose, variance in x, y, yaw>)
    // Output: value at the sample location
    /**
     * Compute the value of the local distribution at the sample pose.
     *
     * All poses are relative to the robot at the pose at which the objects were detected.
     *
     * @tparam PoseType
     * @tparam PoseParamCount
     * @param sample_pose
     * @return
     */
    template<typename PoseType, int PoseParamCount>
    double computeValueForSample(const PoseType &sample_pose,
                                 const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &local_detections,
                                 const std::function<double(const double&)> &pdf_squashing_function) {
        double max_of_gaussians = 0;
        LOG(INFO) << "Computing val for sample " << sample_pose.first.x() << ", " << sample_pose.first.y() << ", " << sample_pose.second;
        for (const ObjectDetectionRelRobot<PoseType, PoseParamCount> &object_detection : local_detections) {
            max_of_gaussians = std::max(max_of_gaussians, pdf_squashing_function(computeGaussianValue(object_detection, sample_pose)));
//            LOG(INFO) << "Max val " << max_of_gaussians;
        }
        LOG(INFO) << "Pose norm: " << sample_pose.first.norm()  << ", Value for sample "<< max_of_gaussians;
        return max_of_gaussians;
    }


    template<typename PoseType, int PoseParamCount, typename ScanType, typename SampleGenParamsType>
    std::vector<std::pair<PoseType, double>> generateSamplesForRobotPoseSingleClass(
            const PoseType &robot_pose,
            const ScanType &scan,
            const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &object_detections,
            const std::function<double (const PoseType &, const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &)> &sample_value_generator,
            const SampleGenParamsType &sample_gen_params,
            util_random::Random &rand_gen) {
        std::vector<PoseType> sample_poses_rel_robot = generateSamplePointsForPose(
                object_detections, scan, sample_gen_params, rand_gen);

        std::vector<std::pair<PoseType, double>> samples_and_value_rel_map;
        for (const PoseType &sample_pose : sample_poses_rel_robot) {
            std::pair<PoseType, double> sample_val = std::make_pair(pose::combinePoses(robot_pose, sample_pose), sample_value_generator(sample_pose, object_detections));
//            for (int i = 0; i < 8; i++) {
                samples_and_value_rel_map.emplace_back(sample_val);
//            }
//            samples_and_value_rel_map.emplace_back(std::make_pair(pose::combinePoses(robot_pose, sample_pose),
//                    sample_value_generator(sample_pose, object_detections)));
//            std::pair<pose::Pose2d, double> last_value = samples_and_value_rel_map.back();
//            if (last_value.second> 0.5) {
//                LOG(INFO) << "Object detection global frame " << last_value.first.first.x() << ", " << last_value.first.first.y() << ", " << last_value.first.second;
//            }
        }
        return samples_and_value_rel_map;
    }

    template<typename PoseType, int PoseParamCount, typename ScanType, typename SampleGenParamsType>
    std::unordered_map<std::string, std::vector<std::pair<PoseType, double>>> generateSamplesForRobotPose(
            const PoseType &robot_pose,
            const SensorInfo<PoseType, PoseParamCount, ScanType> &sensor_info_at_pose,
            const std::function<double (const PoseType &, const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &)> sample_value_generator,
            const std::unordered_map<std::string, SampleGenParamsType> &sample_gen_params_by_class,
            util_random::Random &rand_gen) {
        std::unordered_map<std::string, std::vector<std::pair<PoseType, double>>> samples_by_class;
        for (const auto &obj_detections_by_class : sensor_info_at_pose.object_detections_) {
            if (sample_gen_params_by_class.find(obj_detections_by_class.first) == sample_gen_params_by_class.end()) {
                continue;
            }

            samples_by_class[obj_detections_by_class.first] = generateSamplesForRobotPoseSingleClass(
                    robot_pose,
                    sensor_info_at_pose.scan_info_,
                    obj_detections_by_class.second,
                    sample_value_generator,
                    sample_gen_params_by_class.at(obj_detections_by_class.first),rand_gen);
        }
        return samples_by_class;
    }

    // Subtask: Generate samples from past trajectory info
    // Input: past trajectories, where each trajectory contains a list of {robot pose, laser scan, map <object class, list of object detections relative to robot>, map of object class to percentage of poses to include, map of object class to beams to use per lidar scan, map of object class to  number of samples per beam, sample value generator
    // Output: map<class, list of samples <pose, value>> (both on and off detection). Value should be in range 0-1.
    // Scope: Any problems (real and synthetic)
    template<typename PoseType, int PoseParamCount, typename ScanType, typename SampleGenParamsType>
    std::unordered_map<std::string, std::vector<std::pair<PoseType, double>>> generateSamplesForPastTrajectories(
            const std::vector<std::vector<std::pair<PoseType, SensorInfo<PoseType, PoseParamCount, ScanType>>>> &past_trajectories,
            const std::function<double (const PoseType &, const std::vector<ObjectDetectionRelRobot<PoseType, PoseParamCount>> &)> sample_value_generator,
            const std::unordered_map<std::string, SampleGenParamsType> &sample_gen_params_by_class,
            util_random::Random &rand_gen) {

        std::unordered_map<std::string, std::vector<std::pair<PoseType, double>>> samples;
        for (size_t traj_num = 0; traj_num < past_trajectories.size(); traj_num++) {
            std::vector<std::pair<PoseType, SensorInfo<PoseType, PoseParamCount, ScanType>>> trajectory = past_trajectories[traj_num];
            for (size_t pose_num = 0; pose_num < trajectory.size(); pose_num++) {
                std::unordered_map<std::string, std::vector<std::pair<PoseType, double>>> samples_for_pose =
                        generateSamplesForRobotPose(
                        trajectory[pose_num].first,
                        trajectory[pose_num].second,
                        sample_value_generator,
                        sample_gen_params_by_class,
                        rand_gen);
                for (const auto &samples_for_pose_for_class : samples_for_pose) {
                    std::vector<std::pair<PoseType, double>> samples_for_class;
                    if (samples.find(samples_for_pose_for_class.first) != samples.end()) {
                        samples_for_class = samples[samples_for_pose_for_class.first];
                    }
                    samples_for_class.insert(samples_for_class.begin(), samples_for_pose_for_class.second.begin(),
                                             samples_for_pose_for_class.second.end());

                    samples[samples_for_pose_for_class.first] = samples_for_class;
                }
            }
        }

        return samples;

    }
}

#endif //AUTODIFF_GP_UNCERTAINTY_AWARE_POSE_OPT_UTILS_H
