//
// Created by amanda on 12/3/20.
// This file borrows heavily from https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_3d/pose_graph_3d_error_term.h
//

#ifndef AUTODIFF_GP_ODOMETRY_3D_COST_FUNCTOR_H
#define AUTODIFF_GP_ODOMETRY_3D_COST_FUNCTOR_H

#include <eigen3/Eigen/Dense>

namespace pose_optimization {

    /**
     * Cost functor for an odometry (wheel odometry or lidar/visual odometry) factor in a pose graph.
     */
    class Odometry3dCostFunctor {
    public:

        /**
         * Create the cost functor with the observed odometry data.
         *
         * @param translation_change    Observed translation change.
         * @param orientation_change    Observed orientation change.
         * @param sqrt_information      Information matrix (inverse of covariance matrix).
         */
        Odometry3dCostFunctor(const Eigen::Vector3d &translation_change, const Eigen::Quaterniond &orientation_change,
                              const Eigen::Matrix<double, 6, 6>& sqrt_information) : translation_change_(translation_change),
                            orientation_change_(orientation_change), sqrt_information_(sqrt_information) {

        }

        template <typename T>
        bool operator()(const T* const p_a_ptr,
                        const T* const q_a_ptr,
                        const T* const p_b_ptr,
                        const T* const q_b_ptr,
                        T* residuals_ptr) const {
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(p_a_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_a(q_a_ptr);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(p_b_ptr);
            Eigen::Map<const Eigen::Quaternion<T>> q_b(q_b_ptr);

            // Compute the relative transformation between the two frames.
            Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
            Eigen::Quaternion<T> q_ab_estimated = q_a_inverse * q_b;

            // Represent the displacement between the two frames in the A frame.
            Eigen::Matrix<T, 3, 1> p_ab_estimated = q_a_inverse * (p_b - p_a);

            // Compute the error between the two orientation estimates.
            Eigen::Quaternion<T> delta_q =
                    orientation_change_.template cast<T>() * q_ab_estimated.conjugate();

            // Compute the residuals.
            // [ position         ]   [ delta_p          ]
            // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
            Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
            residuals.template block<3, 1>(0, 0) =
                    p_ab_estimated - translation_change_.template cast<T>();
            residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

            // Scale the residuals by the measurement uncertainty.
            residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

//            LOG(INFO) << "Residuals " << residuals;

            return true;
        }

    private:

        /**
         * Observed translation change.
         */
        Eigen::Vector3d translation_change_;

        /**
         * Observed orientation change.
         */
        Eigen::Quaterniond orientation_change_;

        /**
         * Information matrix (inverse of covariance matrix).
         */
        Eigen::Matrix<double, 6, 6> sqrt_information_;
    };

} // end pose_optimization

#endif //AUTODIFF_GP_ODOMETRY_3D_COST_FUNCTOR_H
