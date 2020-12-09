//
// Created by Amanda Adkins on 12/3/20.
//

#ifndef AUTODIFF_GP_KERNEL_H
#define AUTODIFF_GP_KERNEL_H

#include <eigen3/Eigen/Dense>

namespace gp_kernel {

    template <int N>
    class Kernel {
    public:

        template<typename T>
        using KernelInputType = Eigen::Matrix<T, N, 1>;

        /**
         * Constructor.
         */
        Kernel() = default;

        /**
         * Destructor.
         */
        virtual ~Kernel() = default;

        /**
         * If C++ could handle inheritance+function templatization well, this would be the a virtual function defining
         * the interface for interacting with the kernel and specific kernel implementations would override this.
         *
         * This is left here for documentation purposes. Subclasses should be written as if this were a virtual function
         * (excluding the override operator, which would make no sense).
         *
         * @tparam T    Type to evaluate the kernel for (needed for Ceres autodifferentiation).
         * @param x1    First entry to use in difference computation.
         * @param x2    Second entry to use in difference computation.
         *
         * @return Kernel evaluated with the two samples (gives metric of difference between inputs).
         */
//        template<typename T>
//        virtual T evaluateKernel(const KernelInputType<T>& x1, const KernelInputType<T>& x2) = 0;
    };
}

#endif //AUTODIFF_GP_KERNEL_H
