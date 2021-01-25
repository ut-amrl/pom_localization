//
// Created by amanda on 1/24/21.
//

#ifndef AUTODIFF_GP_ODOMETRY_2D_COST_FUNCTOR_H
#define AUTODIFF_GP_ODOMETRY_2D_COST_FUNCTOR_H

#include <eigen3/Eigen/Dense>
#include <pose_optimization/ceres_math_utils.h>

namespace pose_optimization {

    /**
     * Cost functor for an odometry (wheel odometry or lidar/visual odometry) factor in a pose graph.
     */
    class Odometry2dCostFunctor {
    public:
        /**
         * Create the cost functor with the observed odometry data.
         *
         * @param translation_change    Observed translation change.
         * @param orientation_change    Observed orientation change.
         * @param sqrt_information      Information matrix (inverse of covariance matrix).
         */
        Odometry2dCostFunctor(const Eigen::Vector2d &translation_change, const double &orientation_change,
                              const Eigen::Matrix3d& sqrt_information) : translation_change_(translation_change),
                              orientation_change_(orientation_change), sqrt_information_(sqrt_information) {

        }

        template <typename T>
        bool operator()(const T* const p_a_ptr,
                        const T* const q_a_ptr,
                        const T* const p_b_ptr,
                        const T* const q_b_ptr,
                        T* residuals_ptr) const {

                Eigen::Map<const Eigen::Matrix<T, 2, 1>> p_a(p_a_ptr);
                Eigen::Map<const Eigen::Matrix<T, 2, 1>> p_b(p_b_ptr);

                Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals_map(residuals_ptr);

                residuals_map.template head<2>() =
                        Eigen::Rotation2D<T>(*q_a_ptr).inverse() * (p_b - p_a) - translation_change_.cast<T>();
                residuals_map(2) = ceres_math_utils::NormalizeAngle(
                        (*q_b_ptr - *q_a_ptr) - static_cast<T>(orientation_change_));
                // Scale the residuals by the square root information matrix to account for
                // the measurement uncertainty.
                residuals_map = sqrt_information_.template cast<T>() * residuals_map;

            return true;
        }

    private:

        /**
         * Observed translation change.
         */
        Eigen::Vector2d translation_change_;

        /**
         * Observed orientation change.
         */
        double orientation_change_;

        /**
         * Information matrix (inverse of covariance matrix).
         */
        Eigen::Matrix3d sqrt_information_;
    };

} // end pose_optimization

#endif //AUTODIFF_GP_ODOMETRY_2D_COST_FUNCTOR_H
