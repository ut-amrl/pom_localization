//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_GP_REGRESSION_H
#define AUTODIFF_GP_GP_REGRESSION_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

namespace gp_regression {
    // Implement a Gaussian Process Regressor for N input dimensions, and M output
    // dimensions, with the kernel type Kernel (which should be a subclass of Kernel)
    // That is, the GP approximates f: R^N -> R^M
    template <int N, int M, typename Kernel>
    class GaussianProcessRegression {

        // Constructor to initialize the regressor with the provided input / output
        // pairs.
        // inputs must be N x D, outputs M x D, for a total of D input / output pairs.
        explicit GaussianProcessRegression(
                const Eigen::MatrixXf& inputs,
                const Eigen::MatrixXf& outputs,
                Kernel* kernel) : num_datapoints_(inputs.cols()), inputs_(inputs), kernel_(kernel) {
            CHECK_EQ(inputs.rows(), N);
            CHECK_EQ(outputs.rows(), M);
            CHECK_EQ(inputs.cols(), outputs.cols());
            outputs_transp_ = outputs.transpose();

            // Create the K matrix.
            Eigen::MatrixXf gram_matrix(num_datapoints_, num_datapoints_);

            // Invert it once for fast reuse later.
            for (int row = 0; row < num_datapoints_; row++) {
                Eigen::Matrix<float, N, 1> input_sample_i = inputs.col(row);
                for (int col = 0; col < num_datapoints_; col++) {
                    Eigen::Matrix<float, N, 1> input_sample_j = inputs.col(col);
                    gram_matrix(row, col) = kernel_->evaluateKernel(input_sample_i, input_sample_j);
                }
            }
            inv_gram_matrix_ = gram_matrix.inverse();
        }

        // Templated inference.
        template<typename T>
        Eigen::Matrix<T, M, 1> Inference(const Eigen::Matrix<T, N, 1>& x) {

            Eigen::Matrix<T, 1, Eigen::Dynamic> k_x_transp(1, num_datapoints_);
            for (int i = 0; i < num_datapoints_; i++) {
                // TODO Is it a problem here that inputs_.col(i) is of type float instead of T? Need to investigate
                k_x_transp(0, i) = kernel_->evaluateKernel(inputs_.col(i), x);
            }
            Eigen::Matrix<T, 1, M> mu_star_transp = k_x_transp * inv_gram_matrix_ * outputs_transp_;

            return mu_star_transp.transpose();
        }

    private:

        /**
         * Input data (points where we have observations).
         */
        Eigen::MatrixXf inputs_;

        /**
         * Transpose of the output data (observations at the input points).
         */
        Eigen::MatrixXf outputs_transp_;

        /**
         * Kernel for evaluating the difference between two points.
         */
        Kernel* kernel_;

        /**
         * Number of input/output pairs that the GP was trained on.
         */
        int num_datapoints_;

        /**
         * Inverse of the gram matrix (kernel values for each pair of inputs).
         */
        Eigen::MatrixXf inv_gram_matrix_;
    };
} // end gp_regression
#endif //AUTODIFF_GP_GP_REGRESSION_H
