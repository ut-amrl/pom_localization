//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_GP_REGRESSION_H
#define AUTODIFF_GP_GP_REGRESSION_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>

namespace gp_regression {
    // Implement a Gaussian Process Regressor for N input dimensions, and M output
    // dimensions, with the kernel type Kernel (which should be a subclass of Kernel and should have the same
    // N template parameter)
    // That is, the GP approximates f: R^N -> R^M
    template <int N, int M, typename Kernel>
    class GaussianProcessRegression {
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
        explicit GaussianProcessRegression(
                const Eigen::MatrixXf& inputs,
                const Eigen::MatrixXf& outputs,
                const double &prior_mean,
                const double &identity_noise,
                Kernel* kernel) : num_datapoints_(inputs.cols()), inputs_(inputs), kernel_(kernel), prior_mean_(prior_mean), identity_noise_(identity_noise) {
            CHECK_EQ(inputs.rows(), N);
            CHECK_EQ(outputs.rows(), M);
            CHECK_EQ(inputs.cols(), outputs.cols());
            outputs_transp_ = outputs.transpose();

            prior_mean_mat_ = Eigen::MatrixXf(1, 1);
            prior_mean_mat_ << prior_mean;

            mean_adjusted_outputs_transp_ = (outputs_transp_.array() - prior_mean).matrix();

            refreshInvGramMatrix();
        }

        void appendData(const Eigen::MatrixXf &new_inputs, const Eigen::MatrixXf &new_outputs) {
            CHECK_EQ(new_inputs.rows(), N);
            CHECK_EQ(new_outputs.rows(), M);
            CHECK_EQ(new_inputs.cols(), new_outputs.cols());

            Eigen::MatrixXf new_outputs_transp = new_outputs.transpose();

            auto prev_size = outputs_transp_.rows();
            auto additional_count = new_outputs_transp.rows();
            auto new_size = additional_count + prev_size;
            outputs_transp_.conservativeResize(new_size, Eigen::NoChange);
            outputs_transp_.bottomRows(additional_count) = new_outputs_transp;
//            outputs_transp_.block(prev_size + 1, 0, new_size, M) = new_outputs_transp;

            inputs_.conservativeResize(Eigen::NoChange, new_size);
            inputs_.rightCols(additional_count) = new_inputs;
//            inputs_.block(0, prev_size + 1, N, new_size) = new_inputs;
            num_datapoints_ = new_size;

            mean_adjusted_outputs_transp_ = (outputs_transp_.array() - prior_mean_).matrix();
            refreshInvGramMatrix();
        }

        void refreshInvGramMatrix() {

            LOG(INFO) << "Creating gram matrix";
            // Create the K matrix.
            Eigen::MatrixXf gram_matrix(num_datapoints_, num_datapoints_);

            // TODO is it faster to recompute the kernel for old points or get the previous gram matrix by inverting
            // the previous inverse gram matrix?
            // Invert it once for fast reuse later.
            for (int row = 0; row < num_datapoints_; row++) {
                Eigen::Matrix<float, N, 1> input_sample_i = inputs_.col(row);
                for (int col = 0; col < num_datapoints_; col++) {
                    Eigen::Matrix<float, N, 1> input_sample_j = inputs_.col(col);
                    gram_matrix(row, col) = kernel_->evaluateKernel(input_sample_i, input_sample_j);
                }
            }
            Eigen::MatrixXf self_var_mat = identity_noise_ * Eigen::MatrixXf::Identity(gram_matrix.rows(), gram_matrix.cols());
            LOG(INFO) << "Adding mat " << self_var_mat;

            gram_matrix = gram_matrix + self_var_mat;

            LOG(INFO) << "Inputs " << inputs_;
            LOG(INFO) << "Outputs " << mean_adjusted_outputs_transp_;
            LOG(INFO) << "Gram matrix (not inverted)";
            LOG(INFO) << gram_matrix;
            LOG(INFO) << "Inverting matrix ";
            inv_gram_matrix_ = gram_matrix.inverse();
            LOG(INFO) << "Done inverting matrix";

            LOG(INFO) << inv_gram_matrix_;
            LOG(INFO) << "Testing inversion";
            Eigen::MatrixXf matrix_times_inv = gram_matrix * inv_gram_matrix_;
            LOG(INFO) << "Gram matrix determinant " << gram_matrix.determinant();
            LOG(INFO) << matrix_times_inv;
            LOG(INFO) << "Diag entry " << matrix_times_inv(matrix_times_inv.rows() - 1, matrix_times_inv.cols() - 1);
        }

        /**
         * Infer the value of the given input point.
         *
         * @tparam T type. Needed for Ceres optimization.
         *
         * @param x Estimate the output value for this input.
         *
         * @return Pair of estimated output value for the given input and variance for the estimate.
         */
        template<typename T>
        std::pair<Eigen::Matrix<T, M, Eigen::Dynamic>, Eigen::Matrix<T, 1, Eigen::Dynamic>> Inference(const Eigen::Matrix<T, N, Eigen::Dynamic>& x) {
//            LOG(INFO) << "O " << O;

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> k_x_transp(x.cols(), num_datapoints_);
            for (int i = 0; i < num_datapoints_; i++) {
                Eigen::Matrix<T, N, 1> input_i = inputs_.col(i).cast<T>();
                for (int j = 0; j < x.cols(); j++) {
//                    LOG(INFO) << " j  " << j;
                    Eigen::Matrix<T, N, 1> eval_input = x.col(j);
                    k_x_transp(j, i) = kernel_->evaluateKernel(input_i, eval_input);
                }
            }

            Eigen::Matrix<T, Eigen::Dynamic, 1> input_variance = Eigen::Matrix<T, Eigen::Dynamic, 1>(x.cols(), 1);
            input_variance.setConstant(T(identity_noise_));
//            for (int j = 0; j < x.cols(); j++) {
//                Eigen::Matrix<T, N, 1> eval_input = x.col(j);
//                input_variance(j, 1) = kernel_->evaluateKernel(eval_input, eval_input);
//            }

//            LOG(INFO) << "Input variance " << input_variance;

            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram = k_x_transp * inv_gram_matrix_.cast<T>();

            Eigen::Matrix<T, Eigen::Dynamic, M> mu_star_transp = prior_mean_mat_.cast<T>() + kernel_times_inv_gram * mean_adjusted_outputs_transp_.cast<T>();

            // Compute variance: k_xx + K_x^T * K_d^-1 * K_x
            Eigen::Matrix<T, Eigen::Dynamic, 1> variance_column_vec = input_variance + ((kernel_times_inv_gram.cwiseProduct(k_x_transp)).rowwise().sum());
            // TODO!!!! Need to add self-variance back in -- currently causing NaNs
//            Eigen::Matrix<T, Eigen::Dynamic, 1> variance_column_vec =  ((kernel_times_inv_gram.cwiseProduct(k_x_transp)).rowwise().sum());
//            LOG(INFO) << "Non-self-variance " << ((kernel_times_inv_gram.cwiseProduct(k_x_transp)).rowwise().sum());
//            LOG(INFO) << "Variance col vec " << variance_column_vec;

            return std::make_pair(mu_star_transp.transpose(), variance_column_vec.transpose());
        }

    private:


        Eigen::MatrixXf prior_mean_mat_;

        /**
         * Number of input/output pairs that the GP was trained on.
         */
        int num_datapoints_;

        /**
         * Input data (points where we have observations).
         */
        Eigen::MatrixXf inputs_;

        /**
         * Transpose of the output data (observations at the input points).
         */
        Eigen::MatrixXf outputs_transp_;

        /**
         * Transpose of the output data (observations at the input points).
         */
        Eigen::MatrixXf mean_adjusted_outputs_transp_;

        /**
         * Kernel for evaluating the difference between two points.
         */
        Kernel* kernel_;

        /**
         * Prior mean.
         */
        double prior_mean_;

        double identity_noise_;

        /**
         * Inverse of the gram matrix (kernel values for each pair of inputs).
         */
        Eigen::MatrixXf inv_gram_matrix_;
    };
} // end gp_regression
#endif //AUTODIFF_GP_GP_REGRESSION_H
