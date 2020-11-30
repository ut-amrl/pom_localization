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
            observation_transform_ << cos(observation_angle), (-sin(observation_angle)), obs_x,
                    sin(observation_angle), cos(observation_angle), obs_y,
                    0, 0, 1;
        }

        template<typename T>
        bool operator()(T *robot_pose_ptr, T *residuals) {

            // Assuming robot pose is a 3-entry vector with x, y, theta.
            Eigen::Matrix<T, 3, 3> robot_transform;
            T robot_angle = robot_pose_ptr[2];
            robot_transform << cos(robot_angle), (-sin(robot_angle)), robot_pose_ptr[0],
                    sin(robot_angle), cos(robot_angle), robot_pose_ptr[1],
                    0, 0, 1;

            Eigen::Matrix<T, 3, 3> transformed_object = robot_transform * observation_transform_;
            Eigen::Matrix<T, 3, 1> transformed_object_pose;
            Eigen::Rotation2D <T> rotation(transformed_object.template block<2, 2>());
            transformed_object_pose << transformed_object(0, 2), transformed_object(1, 2), rotation.angle();
            gp_->Inference<T>(transformed_object_pose);

            // TODO is this the correct form for the cost?
            residuals[0] = -log(gp_->Inference<T>(transformed_object_pose));
            return true;
        }

        gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel> *gp_;
        Eigen::Matrix<float, 3, 1> &observation_;
        Eigen::Matrix<float, 3, 3> observation_transform_;
    };
}

#endif //AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H
