//
// Created by amanda on 1/24/21.
//

#ifndef AUTODIFF_GP_CERES_MATH_UTILS_H
#define AUTODIFF_GP_CERES_MATH_UTILS_H

// TODO includes?

namespace ceres_math_utils {
    // Normalizes the angle in radians between [-pi and pi).
    template <typename T>
    inline T NormalizeAngle(const T& angle_radians) {
        // Use ceres::floor because it is specialized for double and Jet types.
        T two_pi(2.0 * M_PI);
        return angle_radians -
               two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
    }

} // end ceres_math_utils

#endif //AUTODIFF_GP_CERES_MATH_UTILS_H
