//
// Created by amanda on 12/21/20.
//

#ifndef AUTODIFF_GP_POSE_REPS_H
#define AUTODIFF_GP_POSE_REPS_H

#include <eigen3/Eigen/Dense>

namespace pose {
    typedef std::pair<Eigen::Vector2d, double> Pose2d;
    typedef std::pair<Eigen::Vector3d, Eigen::Quaterniond> Pose3d;


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
}
#endif //AUTODIFF_GP_POSE_REPS_H
