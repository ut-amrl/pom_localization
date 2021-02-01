//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_KERNEL_DENSITY_ESTIMATOR_H
#define AUTODIFF_GP_KERNEL_DENSITY_ESTIMATOR_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

namespace gp_regression {
    // Implement a Gaussian Process Regressor for N input dimensions, and M output
    // dimensions, with the kernel type Kernel (which should be a subclass of Kernel and should have the same
    // N template parameter)
    // That is, the GP approximates f: R^N -> R^M
    template <int N, typename Kernel>
    class KernelDensityEstimator {
    public:

        // Constructor to initialize the regressor with the provided input / output
        // pairs.
        // inputs must be N x D, outputs M x D, for a total of D input / output pairs.

        /**
         * Constructor to initialize the regressor with the provided input / output pairs.
         *
         * @param inputs    Input data. Must be NxD.
         * @param outputs   Output data. Must be MxD (same number of columns (samples) as inputs).
         * @param kernel    Kernel for comparing similarity of two input values. Must be a subclass of Kernel and that
         *                  kernel must be parameterized with N.
         */
        explicit KernelDensityEstimator(
                const Eigen::MatrixXf& inputs,
                Kernel* kernel) : num_datapoints_(inputs.cols()), inputs_(inputs), kernel_(kernel) {
            CHECK_EQ(inputs.rows(), N);
//            CHECK_EQ(outputs.rows(), M);
//            CHECK_EQ(inputs.cols(), outputs.cols());
//            outputs_transp_ = outputs.transpose();
//
//            refreshInvGramMatrix();
        }

        void appendData(const Eigen::MatrixXf &positive_observations) {

            // Only using positive observations here.

            CHECK_EQ(positive_observations.rows(), N);

            auto prev_size = inputs_.cols();
            auto additional_count = positive_observations.cols();
            auto new_size = additional_count + prev_size;

            inputs_.conservativeResize(Eigen::NoChange, new_size);
            inputs_.rightCols(additional_count) = positive_observations;
//            inputs_.block(0, prev_size + 1, N, new_size) = new_inputs;
            num_datapoints_ = new_size;
        }

        /**
         * Infer the value of the given input point.
         *
         * @tparam T type. Needed for Ceres optimization.
         *
         * @param x Estimate the output value for this input.
         *
         * @return Estimated output value for the given input.
         */
        template<typename T>
        Eigen::Matrix<T, 1, Eigen::Dynamic> Inference(const Eigen::Matrix<T, N, Eigen::Dynamic>& x) {
            // TODO need to figure out how to deal with 0 data points
//            LOG(INFO) << "O " << O;

            int input_size = x.cols();

            Eigen::Matrix<T, 1, Eigen::Dynamic> output = Eigen::Matrix<T, 1, Eigen::Dynamic>::Zero(1, input_size);
            for (int i = 0; i < num_datapoints_; i++) {
                Eigen::Matrix<T, N, 1> input_i = inputs_.col(i).cast<T>();
                for (int j = 0; j < x.cols(); j++) {
                    Eigen::Matrix<T, N, 1> eval_input = x.col(j);
                    output(0, j) += kernel_->evaluateKernel(input_i, eval_input);
                }
//                LOG(INFO) << "Output for iteration i=" << i << ": " << output;
            }
            T num_obs(num_datapoints_);
//            LOG(INFO) << "Undivided output " << output;
            return output / num_obs;
//
//            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> k_x_transp(x.cols(), num_datapoints_);
//            for (int i = 0; i < num_datapoints_; i++) {
//                // TODO Is it a problem here that inputs_.col(i) is of type float instead of T? Need to investigate
//                Eigen::Matrix<T, N, 1> input_i = inputs_.col(i).cast<T>();
//                for (int j = 0; j < x.cols(); j++) {
////                    LOG(INFO) << " j  " << j;
//                    Eigen::Matrix<T, N, 1> eval_input = x.col(j);
//                    k_x_transp(j, i) = kernel_->evaluateKernel(input_i, eval_input);
//                }
//            }
//
////            return k_x_transp;
//            Eigen::Matrix<T, Eigen::Dynamic, M> mu_star_transp = k_x_transp * inv_gram_matrix_.cast<T>() * outputs_transp_.cast<T>();
//
//            return mu_star_transp.transpose();
        }

    private:

        /**
         * Number of input/output pairs that the GP was trained on.
         */
        int num_datapoints_;

        /**
         * Input data (points where we have observations).
         */
        Eigen::MatrixXf inputs_;

//        /**
//         * Transpose of the output data (observations at the input points).
//         */
//        Eigen::MatrixXf outputs_transp_;

        /**
         * Kernel for evaluating the difference between two points.
         */
        Kernel* kernel_;
//
//        /**
//         * Inverse of the gram matrix (kernel values for each pair of inputs).
//         */
//        Eigen::MatrixXf inv_gram_matrix_;
    };
} // end gp_regression
#endif //AUTODIFF_GP_KERNEL_DENSITY_ESTIMATOR_H
