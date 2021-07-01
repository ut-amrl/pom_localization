//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_GP_REGRESSION_H
#define AUTODIFF_GP_GP_REGRESSION_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <ceres/jet.h>

#include <util/timer.h>

namespace gp_regression {
    // Implement a Gaussian Process Regressor for N input dimensions, and M output
    // dimensions, with the kernel type Kernel (which should be a subclass of Kernel and should have the same
    // N template parameter)
    // That is, the GP approximates f: R^N -> R^M
    template<int N, int M, typename Kernel>
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
                const Eigen::MatrixXd &inputs,
                const Eigen::MatrixXd &outputs,
                const double &prior_mean,
                const double &identity_noise_mean,
                const double &identity_noise_var,
                const std::shared_ptr<Kernel> &mean_kernel,
                const std::shared_ptr<Kernel> &variance_kernel,
                std::shared_ptr<CumulativeFunctionTimer> jet_mat_mult_timer,
                std::shared_ptr<CumulativeFunctionTimer> jet_kernel_eval_timer,
                std::shared_ptr<CumulativeFunctionTimer> jet_overall_timer,
                std::shared_ptr<CumulativeFunctionTimer> double_mat_mult_timer,
                std::shared_ptr<CumulativeFunctionTimer> double_kernel_eval_timer,
                std::shared_ptr<CumulativeFunctionTimer> double_overall_timer) : num_datapoints_(inputs.cols()), inputs_(inputs),
                                                                  mean_kernel_(mean_kernel), prior_mean_(prior_mean),
                                                                  identity_noise_mean_(identity_noise_mean),
                                                                  identity_noise_var_(identity_noise_var),
                                                                  jet_mat_mult_timer_(jet_mat_mult_timer),
                                                                                 jet_kernel_eval_timer_(jet_kernel_eval_timer),
                                                                                 jet_overall_timer_(jet_overall_timer),
                                                                                 double_mat_mult_timer_(double_mat_mult_timer),
                                                                                 double_kernel_eval_timer_(double_kernel_eval_timer),
                                                                                 double_overall_timer_(double_overall_timer) {
            CHECK_EQ(inputs.rows(), N);
            CHECK_EQ(outputs.rows(), M);
            CHECK_EQ(inputs.cols(), outputs.cols());
            outputs_transp_ = outputs.transpose();

            if (variance_kernel == nullptr) {
                // TODO is there a way to handle this in the cost functor so we don't need to redo the K_D and K_X calculations?
                variance_kernel_ = mean_kernel;
            } else {
                variance_kernel_ = variance_kernel;
            }

            mean_adjusted_outputs_transp_ = (outputs_transp_.array() - prior_mean).matrix();

            kernel_self_value_ = variance_kernel_->getKernelSelfValue();

            refreshInvGramMatrix(mean_kernel_, inv_gram_matrix_mean_, identity_noise_mean_);
            inv_mat_times_outputs_ = inv_gram_matrix_mean_ * mean_adjusted_outputs_transp_;
//            if (variance_kernel_ != mean_kernel_) {
//                refreshInvGramMatrix(variance_kernel_, inv_gram_matrix_variance_, identity_noise_var_);
//            } else {
//                inv_gram_matrix_variance_ = inv_gram_matrix_mean_;
//            }
        }

        void appendData(const Eigen::MatrixXd &new_inputs, const Eigen::MatrixXd &new_outputs) {
            CHECK_EQ(new_inputs.rows(), N);
            CHECK_EQ(new_outputs.rows(), M);
            CHECK_EQ(new_inputs.cols(), new_outputs.cols());

            Eigen::MatrixXd new_outputs_transp = new_outputs.transpose();

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
            refreshInvGramMatrix(mean_kernel_, inv_gram_matrix_mean_, identity_noise_mean_);
            inv_mat_times_outputs_ = inv_gram_matrix_mean_ * mean_adjusted_outputs_transp_;
//            if (variance_kernel_ != mean_kernel_) {
//                refreshInvGramMatrix(variance_kernel_, inv_gram_matrix_variance_, identity_noise_var_);
//            } else {
//                inv_gram_matrix_variance_ = inv_gram_matrix_mean_;
//            }
        }

        void
        refreshInvGramMatrix(std::shared_ptr<Kernel> kernel, Eigen::MatrixXd &inv_gram_mat, double identity_noise) {

//            LOG(INFO) << "Creating gram matrix";
            // Create the K matrix.
            Eigen::MatrixXd gram_matrix(num_datapoints_, num_datapoints_);

            // TODO is it faster to recompute the kernel for old points or get the previous gram matrix by inverting
            // the previous inverse gram matrix?
            // Invert it once for fast reuse later.
            for (int row = 0; row < num_datapoints_; row++) {
                Eigen::Matrix<double, N, 1> input_sample_i = inputs_.col(row);
                for (int col = 0; col < num_datapoints_; col++) {
                    Eigen::Matrix<double, N, 1> input_sample_j = inputs_.col(col);
                    gram_matrix(row, col) = kernel->evaluateKernel(input_sample_i, input_sample_j);
                }
            }
            Eigen::MatrixXd self_var_mat =
                    identity_noise * Eigen::MatrixXd::Identity(gram_matrix.rows(), gram_matrix.cols());
//            LOG(INFO) << "Adding mat " << self_var_mat;

            gram_matrix = gram_matrix + self_var_mat;
//            LOG(INFO) << "Diagonal: " << gram_matrix.diagonal();
//            LOG(INFO) << "Expected diag val " << (identity_noise + kernel->getKernelSelfValue());
//            LOG(INFO) << "Gram mat " << gram_matrix;

//            LOG(INFO) << "Inputs " << inputs_;
//            LOG(INFO) << "Outputs " << mean_adjusted_outputs_transp_;
//            LOG(INFO) << "Gram matrix (not inverted)";
//            LOG(INFO) << gram_matrix;
//            LOG(INFO) << "Inverting matrix ";
//            inv_gram_matrix_ = gram_matrix.inverse();
            inv_gram_mat = gram_matrix.inverse();
//            LOG(INFO) << "Done inverting matrix";

//            LOG(INFO) << inv_gram_mat;
//            LOG(INFO) << "Testing inversion";
//            Eigen::MatrixXd matrix_times_inv = gram_matrix * inv_gram_mat;
//            LOG(INFO) << "Gram matrix determinant " << gram_matrix.determinant();
//            LOG(INFO) << matrix_times_inv;
//            LOG(INFO) << "Diag entry " << matrix_times_inv(matrix_times_inv.rows() - 1, matrix_times_inv.cols() - 1);
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
//        template<typename T>
//        std::pair<Eigen::Matrix<T, M, Eigen::Dynamic>, Eigen::Matrix<T, 1, Eigen::Dynamic>> Inference(const Eigen::Matrix<T, N, Eigen::Dynamic>& x) {
//
//            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel(x.cols(), num_datapoints_);
//
//            Eigen::Matrix<T, 1, Eigen::Dynamic> kde_based_var = Eigen::Matrix<T, 1, Eigen::Dynamic>::Zero(1, x.cols());
//            for (int i = 0; i < num_datapoints_; i++) {
//                Eigen::Matrix<T, N, 1> input_i = inputs_.col(i).cast<T>();
//                for (int j = 0; j < x.cols(); j++) {
//                    Eigen::Matrix<T, N, 1> eval_input = x.col(j);
//                    k_x_transp_mean_kernel(j, i) = mean_kernel_->evaluateKernel(input_i, eval_input);
//                    if (ceres::IsNaN(k_x_transp_mean_kernel(j, i))) {
//                        LOG(INFO) << "Kernel entry " << j << ", " << i << " is nan";
//                    }
//                    kde_based_var(0, j) += variance_kernel_->evaluateKernel(input_i, eval_input);
//                }
//            }
//            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram_mean = k_x_transp_mean_kernel * inv_gram_matrix_mean_.cast<T>();
////            Eigen::Matrix<T, Eigen::Dynamic, M> mu_star_transp = prior_mean_mat_.cast<T>() + kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<T>();
////
//            Eigen::Matrix<T, Eigen::Dynamic, M> mu_star_transp = kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<T>().array().matrix();
//            mu_star_transp = (T(prior_mean_) + mu_star_transp.array()).matrix();
//
//            return std::make_pair(mu_star_transp.transpose(), kde_based_var.array().inverse().matrix());
//        }

        std::pair<Eigen::Matrix<double, M, Eigen::Dynamic>, Eigen::Matrix<double, 1, Eigen::Dynamic>>
        Inference(const Eigen::Matrix<double, N, Eigen::Dynamic> &x) {

            CumulativeFunctionTimer::Invocation invoc(double_overall_timer_.get());

//            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel(x.cols(), num_datapoints_);
//
//            Eigen::Matrix<double, 1, Eigen::Dynamic> kde_based_var = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1,
//                                                                                                                    x.cols());
//            for (int i = 0; i < num_datapoints_; i++) {
//                Eigen::Matrix<double, N, 1> input_i = inputs_.col(i).cast<double>();
//                for (int j = 0; j < x.cols(); j++) {
//                    Eigen::Matrix<double, N, 1> eval_input = x.col(j);
//                    k_x_transp_mean_kernel(j, i) = mean_kernel_->evaluateKernel(input_i, eval_input);
//                    if (ceres::IsNaN(k_x_transp_mean_kernel(j, i))) {
//                        LOG(INFO) << "Kernel entry " << j << ", " << i << " is nan";
//                    }
//                    kde_based_var(0, j) += variance_kernel_->evaluateKernel(input_i, eval_input);
//                }
//            }

            std::pair< Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<double, 1, Eigen::Dynamic>> kernel_out = ComputeKernel(x);
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel = kernel_out.first;

            Eigen::Matrix<double, 1, Eigen::Dynamic> kde_based_var = kernel_out.second;

//            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram_mean =
//                    k_x_transp_mean_kernel * inv_gram_matrix_mean_.cast<double>();
//            Eigen::Matrix<double, Eigen::Dynamic, M> mu_star_transp = prior_mean_mat_.cast<double>() +
//                                                                      kernel_times_inv_gram_mean *
//                                                                      mean_adjusted_outputs_transp_.cast<double>();
            Eigen::Matrix<double, Eigen::Dynamic, M> mu_star_transp = MatrixMult(k_x_transp_mean_kernel);

            return std::make_pair(mu_star_transp.transpose(), kde_based_var.array().inverse().matrix());
        }

        std::pair<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<double, 1, Eigen::Dynamic>>
        ComputeKernel(const Eigen::Matrix<double, N, Eigen::Dynamic> &x) {

            CumulativeFunctionTimer::Invocation invoc(double_kernel_eval_timer_.get());

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel(x.cols(), num_datapoints_);

            Eigen::Matrix<double, 1, Eigen::Dynamic> kde_based_var = Eigen::Matrix<double, 1, Eigen::Dynamic>::Zero(1,
                                                                                                                    x.cols());
            for (int i = 0; i < num_datapoints_; i++) {
                Eigen::Matrix<double, N, 1> input_i = inputs_.col(i).cast<double>();
                for (int j = 0; j < x.cols(); j++) {
                    Eigen::Matrix<double, N, 1> eval_input = x.col(j);
                    k_x_transp_mean_kernel(j, i) = mean_kernel_->evaluateKernel(input_i, eval_input);
                    if (ceres::IsNaN(k_x_transp_mean_kernel(j, i))) {
                        LOG(INFO) << "Kernel entry " << j << ", " << i << " is nan";
                    }
                    kde_based_var(0, j) += variance_kernel_->evaluateKernel(input_i, eval_input);
                }
            }

            return std::make_pair(k_x_transp_mean_kernel, kde_based_var);
        }

        Eigen::Matrix<double, Eigen::Dynamic, M> MatrixMult(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &k_x_transp_mean_kernel) {

            CumulativeFunctionTimer::Invocation invoc(double_mat_mult_timer_.get());
//            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram_mean =
//                    k_x_transp_mean_kernel * inv_gram_matrix_mean_.cast<double>();
//            Eigen::Matrix<double, Eigen::Dynamic, M> mu_star_transp = kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<double>().array().matrix();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mu_star_transp = k_x_transp_mean_kernel * inv_mat_times_outputs_.cast<double>();
            mu_star_transp = (prior_mean_ + mu_star_transp.array()).matrix();
            return mu_star_transp;
//            return prior_mean_mat_.cast<double>() +
//                   kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<double>();
        }

        template<int JetDim>
        std::pair<Eigen::Matrix<ceres::Jet<double, JetDim>, M, Eigen::Dynamic>, Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic>>
        Inference(const Eigen::Matrix<ceres::Jet<double, JetDim>, N, Eigen::Dynamic> &x) {

            CumulativeFunctionTimer::Invocation invoc(jet_overall_timer_.get());

//            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel(x.cols(),
//                                                                                                             num_datapoints_);
//
//            Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic> kde_based_var = Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic>::Zero(
//                    1, x.cols());
//            for (int i = 0; i < num_datapoints_; i++) {
//                Eigen::Matrix<ceres::Jet<double, JetDim>, N, 1> input_i = inputs_.col(
//                        i).cast<ceres::Jet<double, JetDim>>();
//                for (int j = 0; j < x.cols(); j++) {
//                    Eigen::Matrix<ceres::Jet<double, JetDim>, N, 1> eval_input = x.col(j);
//                    k_x_transp_mean_kernel(j, i) = mean_kernel_->evaluateKernel(input_i, eval_input);
//                    if (ceres::IsNaN(k_x_transp_mean_kernel(j, i))) {
//                        LOG(INFO) << "Kernel entry " << j << ", " << i << " is nan";
//                    }
//                    kde_based_var(0, j) += variance_kernel_->evaluateKernel(input_i, eval_input);
//                }
//            }

            std::pair< Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic>> kernel_out = ComputeKernel(x);
            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel = kernel_out.first;

            Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic> kde_based_var = kernel_out.second;



//            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram_mean =
//                    k_x_transp_mean_kernel * inv_gram_matrix_mean_.cast<ceres::Jet<double, JetDim>>();
//            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, M> mu_star_transp =
//                    prior_mean_mat_.cast<ceres::Jet<double, JetDim>>() +
//                    kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<ceres::Jet<double, JetDim>>();
            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, M> mu_star_transp = MatrixMult(k_x_transp_mean_kernel);

            return std::make_pair(mu_star_transp.transpose(), kde_based_var.array().inverse().matrix());
        }

        template<int JetDim>
        std::pair< Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic>>
        ComputeKernel(const Eigen::Matrix<ceres::Jet<double, JetDim>, N, Eigen::Dynamic> &x) {

            CumulativeFunctionTimer::Invocation invoc(jet_kernel_eval_timer_.get());

            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> k_x_transp_mean_kernel(x.cols(),
                                                                                                             num_datapoints_);

            Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic> kde_based_var = Eigen::Matrix<ceres::Jet<double, JetDim>, 1, Eigen::Dynamic>::Zero(
                    1, x.cols());
            for (int i = 0; i < num_datapoints_; i++) {
                Eigen::Matrix<ceres::Jet<double, JetDim>, N, 1> input_i = inputs_.col(
                        i).cast<ceres::Jet<double, JetDim>>();
                for (int j = 0; j < x.cols(); j++) {
                    Eigen::Matrix<ceres::Jet<double, JetDim>, N, 1> eval_input = x.col(j);
                    k_x_transp_mean_kernel(j, i) = mean_kernel_->evaluateKernel(input_i, eval_input);
                    if (ceres::IsNaN(k_x_transp_mean_kernel(j, i))) {
                        LOG(INFO) << "Kernel entry " << j << ", " << i << " is nan";
                    }
                    kde_based_var(0, j) += variance_kernel_->evaluateKernel(input_i, eval_input);
                }
            }

            return std::make_pair(k_x_transp_mean_kernel, kde_based_var);
        }

        template<int JetDim>
        Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, M> MatrixMult(const Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> &k_x_transp_mean_kernel) {

            CumulativeFunctionTimer::Invocation invoc(jet_mat_mult_timer_.get());
//            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> kernel_times_inv_gram_mean =
//                    k_x_transp_mean_kernel * inv_gram_matrix_mean_.cast<ceres::Jet<double, JetDim>>();
//            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, M> mu_star_transp = kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<ceres::Jet<double, JetDim>>();
            Eigen::Matrix<ceres::Jet<double, JetDim>, Eigen::Dynamic, Eigen::Dynamic> mu_star_transp = k_x_transp_mean_kernel * inv_mat_times_outputs_.cast<ceres::Jet<double, JetDim>>();
            mu_star_transp = (ceres::Jet<double, JetDim>(prior_mean_) + mu_star_transp.array()).matrix();
            return mu_star_transp;
//            return prior_mean_mat_.cast<ceres::Jet<double, JetDim>>() +
//                    kernel_times_inv_gram_mean * mean_adjusted_outputs_transp_.cast<ceres::Jet<double, JetDim>>();
        }

    private:

//        Eigen::MatrixXd prior_mean_mat_;

        /**
         * Number of input/output pairs that the GP was trained on.
         */
        int num_datapoints_;

        /**
         * Input data (points where we have observations).
         */
        Eigen::MatrixXd inputs_;

        /**
         * Transpose of the output data (observations at the input points).
         */
        Eigen::MatrixXd outputs_transp_;

        /**
         * Transpose of the output data (observations at the input points).
         */
        Eigen::MatrixXd mean_adjusted_outputs_transp_;

        Eigen::MatrixXd inv_mat_times_outputs_;

        /**
         * Kernel for evaluating the difference between two points.
         */
        std::shared_ptr<Kernel> mean_kernel_;

        std::shared_ptr<Kernel> variance_kernel_;

        /**
         * Prior mean.
         */
        double prior_mean_;

        double identity_noise_mean_;
        double identity_noise_var_;

        double kernel_self_value_;

        /**
         * Inverse of the gram matrix (kernel values for each pair of inputs).
         */
        Eigen::MatrixXd inv_gram_matrix_mean_;

        /**
         * Inverse of the gram matrix (kernel values for each pair of inputs).
         */
        Eigen::MatrixXd inv_gram_matrix_variance_;


        std::shared_ptr<CumulativeFunctionTimer> jet_mat_mult_timer_;
        std::shared_ptr<CumulativeFunctionTimer> jet_kernel_eval_timer_;
        std::shared_ptr<CumulativeFunctionTimer> jet_overall_timer_;

        std::shared_ptr<CumulativeFunctionTimer> double_mat_mult_timer_;
        std::shared_ptr<CumulativeFunctionTimer> double_kernel_eval_timer_;
        std::shared_ptr<CumulativeFunctionTimer> double_overall_timer_;

    };
} // end gp_regression
#endif //AUTODIFF_GP_GP_REGRESSION_H
