//
// Created by amanda on 12/21/20.
//

#ifndef AUTODIFF_GP_POSE_REPS_H
#define AUTODIFF_GP_POSE_REPS_H

#include <eigen3/Eigen/Dense>
#include <shared/util/random.h>

namespace pose {
    typedef std::pair<Eigen::Vector2d, double> Pose2d;
    typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> Pose3d;

    Pose2d createPose2d(const double &x, const double &y, const double &theta) {
        return std::make_pair(Eigen::Vector2d(x, y), theta);
    }

    Eigen::Affine3d convertPoseToAffine(Pose3d &pose) {
        Eigen::Affine3d mat;
        mat.translation() = pose.first;
        mat.linear() = pose.second.toRotationMatrix();
        return mat;
    }

    Eigen::Affine2d convertPoseToAffine(Pose2d &pose) {
        Eigen::Affine2d mat;
        mat.translation() = pose.first;
        mat.linear() = Eigen::Rotation2Dd(pose.second).toRotationMatrix();
        return mat;
    }

    Pose3d convertAffineToPose(Eigen::Affine3d &mat) {
        return std::make_pair(mat.translation(), Eigen::Quaterniond(mat.linear()));
    }

    Pose2d convertAffineToPose(Eigen::Affine2d &mat) {
        Eigen::Rotation2Dd rotation(mat.linear());
        return std::make_pair(mat.translation(), rotation.angle());
    }

    Pose3d getPoseOfObj1RelToObj2(Pose3d obj1, Pose3d obj2) {

        Eigen::Affine3d affine_1 = convertPoseToAffine(obj1);
        Eigen::Affine3d affine_2 = convertPoseToAffine(obj2);
        Eigen::Affine3d combined_affine = affine_2.inverse() * affine_1;
        return convertAffineToPose(combined_affine);
    }

    Pose2d getPoseOfObj1RelToObj2(Pose2d obj1, Pose2d obj2) {

        Eigen::Affine2d affine_1 = convertPoseToAffine(obj1);
        Eigen::Affine2d affine_2 = convertPoseToAffine(obj2);
        Eigen::Affine2d combined_affine = affine_2.inverse() * affine_1;
        return convertAffineToPose(combined_affine);
    }

    Pose3d toPose3d(Pose2d pose_2d) {
        Eigen::Vector3d transl(pose_2d.first.x(), pose_2d.first.y(), 0);
        Eigen::Quaterniond rot;
        rot = Eigen::AngleAxisd(pose_2d.second, Eigen::Vector3d::UnitZ());
        return std::make_pair(transl, rot);
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

    Pose2d combinePoses(Pose2d pose_1, Pose2d pose_2) {
        Eigen::Affine2d affine_1 = convertPoseToAffine(pose_1);
        Eigen::Affine2d affine_2 = convertPoseToAffine(pose_2);
        Eigen::Affine2d combined_affine = affine_1 * affine_2;
        return convertAffineToPose(combined_affine);
    }

    pose::Pose2d addGaussianNoise(const pose::Pose2d &original_pose_2d, const double &x_std_dev,
                                  const double &y_std_dev, const double &theta_std_dev,
                                  util_random::Random &rand_gen) {
        return std::make_pair(Eigen::Vector2d(rand_gen.Gaussian(original_pose_2d.first.x(), x_std_dev),
                                              rand_gen.Gaussian(original_pose_2d.first.y(), y_std_dev)),
                              rand_gen.Gaussian(original_pose_2d.second, theta_std_dev));
    }


    pose::Pose2d addRelativeGaussianNoise(const pose::Pose2d &original_pose_2d, const double &x_std_dev,
                                          const double &y_std_dev, const double &theta_std_dev,
                                          util_random::Random &rand_gen) {
        double scaled_x_std_dev = original_pose_2d.first.x() * x_std_dev;
        double scaled_y_std_dev = original_pose_2d.first.y() * y_std_dev;
        double scaled_yaw_std_dev = original_pose_2d.second * theta_std_dev;

        return addGaussianNoise(original_pose_2d, scaled_x_std_dev, scaled_y_std_dev, scaled_yaw_std_dev,
                                rand_gen);
    }
}
#endif //AUTODIFF_GP_POSE_REPS_H
