//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_PERIODIC_GAUSSIAN_KERNEL_H
#define AUTODIFF_GP_PERIODIC_GAUSSIAN_KERNEL_H

#include <eigen3/Eigen/Dense>

#include <shared/math/math_util.h>

#include <gaussian_process/kernel/kernel.h>

namespace gp_kernel {

    /**
     * Similar to the gaussian kernel, but assumes the inputs are periodic with the given periodicity.
     *
     * See GPFlow docs for similar (but more general) description of periodic kernel.
     *
     * @tparam N Number of values in input.
     */
    template<int N>
    class PeriodicGaussianKernel : Kernel<N> {
    public:

        template<typename T>
        using InputType = typename Kernel<N>::template KernelInputType<T>;

        /**
         * Constructor.
         * @param period    Periodicity of data. Ex. for orientation in 2D, this is 2*pi.
         * @param variance  Variance for the kernel (see GaussianKernel).
         * @param length    Length scale for the kernel (see GaussianKernel).
         */
        explicit PeriodicGaussianKernel(const float &period, const float &variance,
                                        const float &length) : Kernel<N>(),
                                                               inside_sin_multiplier_(M_PI /period),
                                                               variance_(variance),
                                                               exponential_multiplier_(-0.5 /math_util::Sq(length)) {}

        template<typename T>
        T evaluateKernel(const InputType<T> &x1, const InputType<T> &x2) {

//
//            T intermediate_dist = (x1 - x2).norm();
//            T period_invariant_dist = M_PI * intermediate_dist / static_cast<T>(period_);

//            T norm_val = (x1 - x2).norm();
//
//            if (ceres::IsNaN(norm_val)) {
//                LOG(INFO) << "x1: " << x1;
//                LOG(INFO) << "x2: " << x2;
//                LOG(INFO) << x1 - x2;
//                LOG(INFO) << "norm is NaN";
//                exit(1);
//            } else {
//                T inside_sin_val = norm_val * T(inside_sin_multiplier_);
//                if (ceres::IsNaN(inside_sin_val)) {
//                    LOG(INFO) << "Inside sin val is NaN";
//                    exit(1);
//                } else {
//                    T sin_val = sin(inside_sin_val);
//                    if (ceres::IsNaN(sin_val)) {
//                        LOG(INFO) << "Sin val is NaN";
//                        exit(1);
//                    } else {
//                        T squared_sin_val = math_util::Sq(sin_val);
//                        if (ceres::IsNaN(squared_sin_val)) {
//                            LOG(INFO) << "Squared sin val is NaN";
//                            exit(1);
//                        } else {
//                            T inside_exponent = T(exponential_multiplier_) * squared_sin_val;
//                            if (ceres::IsNaN(inside_exponent)) {
//                                LOG(INFO) << "Inside exponent is NaN";
//                                exit(1);
//                            } else {
//                                T exponent_val = exp(inside_exponent);
//                                if (ceres::IsNaN(exponent_val)) {
//                                    LOG(INFO) << "Exponent val is NaN";
//                                    exit(1);
//                                } else {
//                                    T return_val = static_cast<T>(variance_) * exponent_val;
//                                    if (ceres::IsNaN(return_val)) {
//                                        LOG(INFO) << "Return val is NaN";
//                                        exit(1);
//                                    }
//                                }
//                            }
//                        }
//                    }
//                }
//            }

            return static_cast<T>(variance_) *
                   exp(T(exponential_multiplier_) * math_util::Sq(sin((x1 - x2).norm() * T(inside_sin_multiplier_))));
//            T intermediate_dist = (x1 - x2).norm();
//            T period_invariant_dist = M_PI * intermediate_dist / static_cast<T>(period_);
//
//            return static_cast<T>(variance_) * exp(-0.5 * math_util::Sq(sin(period_invariant_dist)) / math_util::Sq(static_cast<T>(length_)));
        }

        double getKernelSelfValue() {
            return variance_;
        }

    private:

//        /**
//         * Period of the data. Ex. for angles, this will be 2*pi.
//         */
//        float period_;

        double inside_sin_multiplier_;

        /**
         * Variance for the kernel.
         */
        float variance_;
//
//        /**
//         * Length for the kernel.
//         */
//        float length_;

        double exponential_multiplier_;
    };
} // end gp_kernel

#endif //AUTODIFF_GP_PERIODIC_GAUSSIAN_KERNEL_H
