//
// Created by amanda on 11/30/20.
//

#ifndef AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H
#define AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <memory>

#include <gaussian_process/gp_regression.h>
#include <gaussian_process/kernel/periodic_gaussian_kernel.h>
#include <gaussian_process/kernel/pose_2d_kernel.h>

namespace pose_optimization {

    /**
     * Ceres cost functor used for evaluating the cost of a robot pose given the likelihood of a movable observation
     * that was made.
     *
     * The observation is given in 3D, but compared to past data using only the 2D projection of the resulting object
     * pose.
     *
     * TODO: I'm not sure this will handle detections that have significant pitch or roll (right now, assuming we end
     * up with pretty much only yaw). Might need to revisit the projection logic.
     */
    struct MovableObservationCostFunctor {

        /**
         * Constructor for a single observation residual.
         *
         * @param gp                        Gaussian process regressor that has been trained with previous observations.
         * @param observation_transl        Translation component of the observation (in 3D).
         * @param observation_orientation   Orientation component of the observation (in 3D).
         */
        MovableObservationCostFunctor(
                std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp,
                Eigen::Vector3f &observation_transl, Eigen::Quaternionf &observation_orientation) :gp_(gp),
                observation_translation_(observation_transl), observation_orientation_(observation_orientation) {

            observation_transform_.translation() = observation_translation_;
            observation_transform_.linear() = observation_orientation_.toRotationMatrix();
        }

        template <typename T>
        bool operator()(const T* const robot_position_ptr, const T* const robot_orientation_ptr, T* residuals) const {

            // Assuming robot_position_ptr is
//            LOG(INFO) << "Observation transform transl " << observation_transform_.translation();
//            LOG(INFO) << "Observation transform rot " << observation_transform_.rotation();
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> robot_position(robot_position_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> robot_orientation(robot_orientation_ptr);
//            T entry_1 = *robot_orientation_ptr;
//            LOG(INFO) << "robot orientation quat " << entry_1;
//            Eigen::Quaternion<T> robot_quat_copy;
//            robot_quat_copy = robot_orientation;
//            LOG(INFO) << robot_quat_copy.x() << ", " << robot_quat_copy.y() << ", " << robot_quat_copy.z() << ", " << robot_quat_copy.w();
//            LOG(INFO) << " Rot mat: " << robot_quat_copy.toRotationMatrix();
//            LOG(INFO) << "Orig rot mat " << robot_orientation.toRotationMatrix();

            Eigen::Transform<T,3,Eigen::Affine> robot_tf =   Eigen::Transform<T, 3, Eigen::Affine>::Identity();
            robot_tf.translation() = robot_position;
//            Eigen::Matrix<T, 3, 3> rotation = robot_orientation.toRotationMatrix();
//            LOG(INFO) << " robot tf rot " << robot_tf.linear();
//            robot_tf.linear() = rotation;

            robot_tf.linear() = robot_orientation.toRotationMatrix();
//            LOG(INFO) << " Robot tf " << robot_tf.translation();
//            LOG(INFO) << " robot tf rot " << robot_tf.linear();

            Eigen::Transform<T, 3, Eigen::Affine> observation_3d = robot_tf * observation_transform_.cast<T>();
//            LOG(INFO) << " observation 3d transl " << observation_3d.translation();
//            LOG(INFO) << " observation 3d rot " << observation_3d.rotation();

//            LOG(INFO) << " observation 3d rot linear " << observation_3d.linear();
            // TODO verify that yaw extraction is correct
//            T yaw = observation_3d.rotation().eulerAngles(0, 1, 2)[2];

            T yaw = observation_3d.linear().eulerAngles(0, 1, 2)[2];

//            LOG(INFO) << "Yaw: " << yaw;

            Eigen::Matrix<T, 3, 1> object_pose_2d;
            object_pose_2d << observation_3d.translation().x(), observation_3d.translation().y(), yaw;

            // TODO is this the correct form for the cost?
            // Should we take square root, since the other one is squared later
            T inference_val = gp_->Inference<T>(object_pose_2d)(0, 0);
//            LOG(INFO) << "Inference output " << inference_val;
//            residuals[0] = -inference_val;

//            residuals[0] = -log(inference_val);
//            residuals[0] = log(inference_val + 0.0000000000001);

//            residuals[0] = -log(0.9 * inference_val + 0.0000000000001);
            residuals[0] = T(-20) * (T(2.0) - inference_val);
            LOG(INFO) << "Residual " << residuals[0];

            return true;
        }

        /**
         * Gaussian process regressor for evaluating the likelihood of the 2D projection of the object detection in
         * the map frame.
         */
        std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_;

        /**
         * Translation component of the observation relative to the robot.
         */
        Eigen::Vector3f observation_translation_;

        /**
         * Orientation component of the observation relative to the robot's frame.
         */
        Eigen::Quaternionf observation_orientation_;

        /**
         * Affine transform that provides the object's coordinate frame relative to the robot's coordinate frame.
         */
        Eigen::Affine3f observation_transform_;
    };
} // end pose_optimization

#endif //AUTODIFF_GP_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_H
