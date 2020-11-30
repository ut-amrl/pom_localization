//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_POSE_2D_KERNEL_H
#define AUTODIFF_GP_POSE_2D_KERNEL_H

#include <eigen3/Eigen/Dense>

#include <gaussian_process/kernel/gaussian_kernel.h>
#include <gaussian_process/kernel/kernel.h>
#include <gaussian_process/kernel/periodic_gaussian_kernel.h>

namespace gp_kernel {

    namespace pose_2d {

        /**
         * Number of values in the position component of a 2d pose.
         */
        static const int kPositionDims = 2;

        /**
         * Number of values in the orientation component of a 2d pose.
         */
        static const int kOrientation2dDims = 1;
    }

    /**
     * Kernel for comparing similarity between 2D poses.
     */
    class Pose2dKernel : Kernel<3> {
    public:

        template <typename T>
        using InputType = typename Kernel<3>::template KernelInputType<T>;

        /**
         * Create the 2D pose kernel. Composed of a kernel for comparing the position (x,y) and another for
         * comparing the orientation (theta).
         *
         * @param position_kernel       Kernel for computing the similarity between the positions.
         * @param orientation_kernel    Kernel for computing the similarity between orientations.
         */
        explicit Pose2dKernel(const GaussianKernel<pose_2d::kPositionDims> &position_kernel,
                              const PeriodicGaussianKernel<pose_2d::kOrientation2dDims> &orientation_kernel) :
                              position_kernel_(position_kernel),
                              orientation_kernel_(orientation_kernel) {
            // TODO should we add a constant kernel for noise?
        }

        template<typename T>
        T evaluateKernel(const InputType<T>& x1, const InputType<T>& x2) {
            Eigen::Matrix<T, pose_2d::kPositionDims, Eigen::Dynamic> position_data_1 =
                    x1.topRows(pose_2d::kPositionDims);
            Eigen::Matrix<T, pose_2d::kPositionDims, Eigen::Dynamic> position_data_2 =
                    x2.topRows(pose_2d::kPositionDims);
            Eigen::Matrix<T, pose_2d::kOrientation2dDims, Eigen::Dynamic> orientation_data_1 =
                    x1.bottomRows(pose_2d::kOrientation2dDims);
            Eigen::Matrix<T, pose_2d::kOrientation2dDims, Eigen::Dynamic> orientation_data_2 =
                    x2.bottomRows(pose_2d::kOrientation2dDims);
            return position_kernel_.evaluateKernel<T>(position_data_1, position_data_2)
                    * orientation_kernel_.evaluateKernel<T>(orientation_data_1, orientation_data_2);
        }

    private:

        /**
         * Kernel for comparing the position similarity.
         */
        GaussianKernel<pose_2d::kPositionDims> position_kernel_;

        /**
         * Kernel for comparing the orientation similarity.
         */
        PeriodicGaussianKernel<pose_2d::kOrientation2dDims> orientation_kernel_;
    };
}

#endif //AUTODIFF_GP_POSE_2D_KERNEL_H
