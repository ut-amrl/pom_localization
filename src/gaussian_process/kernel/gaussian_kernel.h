//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_GAUSSIAN_KERNEL_H
#define AUTODIFF_GP_GAUSSIAN_KERNEL_H


#include <eigen3/Eigen/Dense>
#include <gaussian_process/kernel/kernel.h>
#include <math/math_util.h>

namespace gp_kernel {

    /**
     * Gaussian/RBF kernel with length and variance.
     *
     * Takes the form variance *  exp(-0.5 * l2norm(x1 - x2) / sq(length)).
     *
     * @tparam N Number of parameters in input.
     */
    template <int N>
    class GaussianKernel : Kernel<N> {
    public:

        template <typename T>
        using InputType = typename Kernel<N>::template KernelInputType<T>;

        /**
         * Constructor.
         *
         * @param length    Length scale parameter.
         * @param variance  Variance parameter.
         */
        explicit GaussianKernel(const float &length, const float &variance) : Kernel<N>(),
                norm_factor_(-0.5 / math_util::Sq(length)), variance_(variance) {}

        template<typename T>
        T evaluateKernel(const InputType<T>& x1, const InputType<T>& x2) {
            return static_cast<T>(variance_) * exp(T(norm_factor_) * (x1 - x2).squaredNorm());
        }

        double getKernelSelfValue() {
            return variance_;
        }

    private:

        /**
         * Length parameter.
         *
         * Distance between points is divided by the square of this value.
         */
        double norm_factor_;

        /**
         * Variance parameter.
         */
        float variance_;
    };
}

#endif //AUTODIFF_GP_GAUSSIAN_KERNEL_H
