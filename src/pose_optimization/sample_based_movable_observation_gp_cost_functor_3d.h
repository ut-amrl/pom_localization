//
// Created by amanda on 11/30/20.
//

#ifndef AUTODIFF_GP_SAMPLE_BASED_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_3D_H
#define AUTODIFF_GP_SAMPLE_BASED_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_3D_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <memory>

#include <gaussian_process/gp_regression.h>
#include <gaussian_process/kernel/periodic_gaussian_kernel.h>
#include <gaussian_process/kernel/pose_2d_kernel.h>
#include <gaussian_process/kernel_density_estimator.h>

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
    template<typename ProbabilityEvaluator>
    struct SampleBasedMovableObservationCostFunctor3D {

        SampleBasedMovableObservationCostFunctor3D(
                const std::shared_ptr<ProbabilityEvaluator> probability_evaluator,
                const std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &observation_samples) :probability_evaluator_(probability_evaluator) {
            num_samples_ = observation_samples.size();
            CHECK_GT(num_samples_, 0);
            for (const std::pair<Eigen::Vector3d, Eigen::Quaterniond> &observation_sample : observation_samples) {
                observation_transforms_.emplace_back(convertTranslationAndRotationToMatrix(observation_sample.first,
                                                                                           observation_sample.second));
            }
        }

        template<typename T>
        Eigen::Transform<T, 3, Eigen::Affine> convertTranslationAndRotationToMatrix(
                const Eigen::Matrix<T, 3, 1> &translation, const Eigen::Quaternion<T> &rotation) const {
            Eigen::Transform<T, 3, Eigen::Affine> transform = Eigen::Transform<T, 3, Eigen::Affine>::Identity();
            transform.translate(translation);
            transform.rotate(rotation);
            return transform;
        }
//
//        template <typename T>
//        bool operator()(const T* const robot_position_ptr, const T* const robot_orientation_ptr, T* residuals) const {
//
//            Eigen::Map<const Eigen::Matrix<T, 3, 1>> robot_position(robot_position_ptr);
//            Eigen::Map<const Eigen::Quaternion<T>> robot_orientation(robot_orientation_ptr);
//
//            Eigen::Transform<T, 3, Eigen::Affine> robot_tf =
//                    convertTranslationAndRotationToMatrix<T>(robot_position, robot_orientation);
//
//            T cumulative_probability = T(0.0);
//
//            for (const Eigen::Affine3f &obs_sample_tf : observation_transforms_) {
//                Eigen::Transform<T, 3, Eigen::Affine> world_frame_obj_tf = robot_tf * obs_sample_tf.cast<T>();
//                Eigen::Matrix<T, 3, 1> obj_pose_vector;
//
//                T yaw = world_frame_obj_tf.linear().eulerAngles(0, 1, 2)[2];
//
//                obj_pose_vector << world_frame_obj_tf.translation().x(), world_frame_obj_tf.translation().y(), yaw;
//                cumulative_probability += probability_evaluator_->template Inference<T>(obj_pose_vector)(0, 0);
//            }
//
//            // TODO Do we need to divide by length scale or are we relying on KDE to do that?
//            // TODO Do we need to multiply by 1/<number of samples> - shouldn't change overall result, but perhaps would allow probability to exceed 1 (BAD)
//            cumulative_probability = cumulative_probability / T(num_samples_);
//            // Dividing by number of observations in KDE is already done by KDE
//
//            residuals[0] = sqrt(T(-2.0) * log(cumulative_probability));
//
//            return true;
//        }


        bool operator()(const double* const robot_position_ptr, const double* const robot_orientation_ptr, double* residuals) const {

            Eigen::Map<const Eigen::Matrix<double, 3, 1>> robot_position(robot_position_ptr);
            Eigen::Map<const Eigen::Quaternion<double>> robot_orientation(robot_orientation_ptr);

            Eigen::Transform<double, 3, Eigen::Affine> robot_tf =
                    convertTranslationAndRotationToMatrix<double>(robot_position, robot_orientation);

            double cumulative_probability = double(0.0);

            for (const Eigen::Affine3f &obs_sample_tf : observation_transforms_) {
                Eigen::Transform<double, 3, Eigen::Affine> world_frame_obj_tf = robot_tf * obs_sample_tf.cast<double>();
                Eigen::Matrix<double, 3, 1> obj_pose_vector;

                double yaw = world_frame_obj_tf.linear().eulerAngles(0, 1, 2)[2];

                obj_pose_vector << world_frame_obj_tf.translation().x(), world_frame_obj_tf.translation().y(), yaw;
                cumulative_probability += probability_evaluator_->Inference(obj_pose_vector)(0, 0);
            }

            // TODO Do we need to divide by length scale or are we relying on KDE to do that?
            // TODO Do we need to multiply by 1/<number of samples> - shouldn't change overall result, but perhaps would allow probability to exceed 1 (BAD)
            cumulative_probability = cumulative_probability / double(num_samples_);
            // Dividing by number of observations in KDE is already done by KDE

            residuals[0] = sqrt(double(-2.0) * log(cumulative_probability));

            return true;
        }


        template <int JetDim>
        bool operator()(const ceres::Jet<double, JetDim>* const robot_position_ptr, const ceres::Jet<double, JetDim>* const robot_orientation_ptr, ceres::Jet<double, JetDim>* residuals) const {

            Eigen::Map<const Eigen::Matrix<ceres::Jet<double, JetDim>, 3, 1>> robot_position(robot_position_ptr);
            Eigen::Map<const Eigen::Quaternion<ceres::Jet<double, JetDim>>> robot_orientation(robot_orientation_ptr);

            Eigen::Transform<ceres::Jet<double, JetDim>, 3, Eigen::Affine> robot_tf =
                    convertTranslationAndRotationToMatrix<ceres::Jet<double, JetDim>>(robot_position, robot_orientation);

            ceres::Jet<double, JetDim> cumulative_probability = ceres::Jet<double, JetDim>(0.0);

            for (const Eigen::Affine3f &obs_sample_tf : observation_transforms_) {
                Eigen::Transform<ceres::Jet<double, JetDim>, 3, Eigen::Affine> world_frame_obj_tf = robot_tf * obs_sample_tf.cast<ceres::Jet<double, JetDim>>();
                Eigen::Matrix<ceres::Jet<double, JetDim>, 3, 1> obj_pose_vector;

                ceres::Jet<double, JetDim> yaw = world_frame_obj_tf.linear().eulerAngles(0, 1, 2)[2];

                obj_pose_vector << world_frame_obj_tf.translation().x(), world_frame_obj_tf.translation().y(), yaw;
                cumulative_probability += probability_evaluator_->template Inference<JetDim>(obj_pose_vector)(0, 0);
            }

            // TODO Do we need to divide by length scale or are we relying on KDE to do that?
            // TODO Do we need to multiply by 1/<number of samples> - shouldn't change overall result, but perhaps would allow probability to exceed 1 (BAD)
            cumulative_probability = cumulative_probability / ceres::Jet<double, JetDim>(num_samples_);
            // Dividing by number of observations in KDE is already done by KDE

            residuals[0] = sqrt(ceres::Jet<double, JetDim>(-2.0) * log(cumulative_probability));

            return true;
        }

//
//        /**
//         * Gaussian process regressor for evaluating the likelihood of the 2D projection of the object detection in
//         * the map frame.
//         */
//        std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_;

        /**
         * Gaussian process regressor for evaluating the likelihood of the 2D projection of the object detection in
         * the map frame.
         */
        std::shared_ptr<ProbabilityEvaluator> probability_evaluator_;

        /**
         * Affine transforms that provides the sampled object's coordinate frame relative to the robot's coordinate frame.
         *
         * Contains one for each observation sample to estimate.
         */
        std::vector<Eigen::Affine3f> observation_transforms_;

        int num_samples_;
    };
} // end pose_optimization

#endif //AUTODIFF_GP_SAMPLE_BASED_MOVABLE_OBSERVATION_GP_COST_FUNCTOR_3D_H
