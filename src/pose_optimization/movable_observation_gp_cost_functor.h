//
// Created by amanda on 11/30/20.
//

#ifndef AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H
#define AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

#include <gaussian_process/gp_regression.h>
#include <gaussian_process/kernel/periodic_gaussian_kernel.h>
#include <gaussian_process/kernel/pose_2d_kernel.h>

namespace pose_optimization {

    struct GPObservationWithOrientationResidual {

        /**
         * Constructor for a single observation residual.
         *
         * @param gp            Gaussian process regressor that has been trained with previous observations.
         * @param observation   Observation relative to the robot.
         */
        GPObservationWithOrientationResidual(
                gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel> *gp,
                Eigen::Matrix<float, 3, 1> &observation) :
                gp_(gp), observation_(observation) {
            float obs_x = observation(0, 0);
            float obs_y = observation(1, 0);
            float observation_angle = observation(2, 0);
            observation_transform_.translation() = Eigen::Vector3f(obs_x, obs_y, 0.0);
            Eigen::Matrix3f rotation_mat = Eigen::Matrix3f::Identity();
            rotation_mat.block<2, 2>(0, 0) = Eigen::Rotation2Df(observation_angle).toRotationMatrix();
            observation_transform_.linear() = rotation_mat;
        }

        template<typename T>
        bool operator()(T *robot_position_ptr, T* robot_orientation_ptr, T *residuals) {

            // Assuming robot_position_ptr is
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> robot_position(robot_orientation_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> robot_orientation(robot_orientation_ptr);

            Eigen::Transform<T,3,Eigen::Affine> robot_tf = Eigen::Transform<T, 3, Eigen::Affine>::Identity();
            robot_tf.translation() = robot_position;
            robot_tf.linear() = robot_orientation.toRotationMatrix();

            Eigen::Transform<T, 3, Eigen::Affine> observation_3d = robot_tf * observation_transform_;
            // TODO verify that yaw extraction is correct
            T yaw = observation_3d.rotation().eulerAngles(0, 1, 2)[2];

            Eigen::Matrix<T, 3, 1> object_pose_2d;
            object_pose_2d << observation_transform_.translation().x(), observation_transform_.translation().y(), yaw;
            gp_->Inference<T>(object_pose_2d);

            // TODO is this the correct form for the cost?
            residuals[0] = -log(gp_->Inference<T>(object_pose_2d));
            return true;
        }

        gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel> *gp_;
        Eigen::Matrix<float, 3, 1> &observation_;
        Eigen::Affine3f observation_transform_;
    };
}

#endif //AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H
