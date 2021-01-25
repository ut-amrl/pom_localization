//
// Created by amanda on 1/24/21.
// This file borrows heavily from
// https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_2d/normalize_angle.h
// and https://ceres-solver.googlesource.com/ceres-solver/+/master/examples/slam/pose_graph_2d/angle_local_parameterization.h
//

#ifndef AUTODIFF_GP_ANGLE_LOCAL_PARAMETERIZATION_H
#define AUTODIFF_GP_ANGLE_LOCAL_PARAMETERIZATION_H

#include <ceres/autodiff_local_parameterization.h>
#include <pose_optimization/ceres_math_utils.h>

namespace pose_optimization {


    // [-pi to pi).
    class AngleLocalParameterization {
    public:
        template <typename T>
        bool operator()(const T* theta_radians,
                        const T* delta_theta_radians,
                        T* theta_radians_plus_delta) const {
            *theta_radians_plus_delta =
                    ceres_math_utils::NormalizeAngle(*theta_radians + *delta_theta_radians);
            return true;
        }

        static ceres::LocalParameterization* create() {
            return new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>();
        }
    };
}

#endif //AUTODIFF_GP_ANGLE_LOCAL_PARAMETERIZATION_H
