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
