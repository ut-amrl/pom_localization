//
// Created by amanda on 11/29/20.
//

#ifndef AUTODIFF_GP_GP_CLASSIFIER_H
#define AUTODIFF_GP_GP_CLASSIFIER_H

#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
#include <gaussian_process/gp_regression.h>

namespace gp_regression {
    // Implement a Gaussian Process Regressor for N input dimensions, and M output
    // dimensions, with the kernel type Kernel (which should be a subclass of Kernel and should have the same
    // N template parameter)
    // That is, the GP approximates f: R^N -> R^M
    template <int N, typename Kernel>
    class GaussianProcessClassifier {
    public:

        // Constructor to initialize the regressor with the provided input / output
        // pairs.
        // inputs must be N x D, outputs 1 x D, for a total of D input / output pairs.

        /**
         * Constructor to initialize the regressor with the provided input / output pairs.
         *
         * @param inputs    Input data. Must be NxD.
         * @param outputs   Output data. Must be MxD (same number of columns (samples) as inputs).
         * @param kernel    Kernel for comparing similarity of two input values. Must be a subclass of Kernel and that
         *                  kernel must be parameterized with N.
         */
        explicit GaussianProcessClassifier(
                const Eigen::MatrixXf& inputs,
                const Eigen::MatrixXf& outputs,
                Kernel* kernel) : num_datapoints_(inputs.cols()), inputs_(inputs), outputs_(outputs) {
            CHECK_EQ(inputs.rows(), N);
            CHECK_EQ(outputs.rows(), 1);
            CHECK_EQ(inputs.cols(), outputs.cols());
            output_data_transformed_ = Eigen::MatrixXf(outputs.rows(), outputs.cols());
//            LOG(INFO) << "Outputs " << outputs;
            transformZeroToOneOutputsToRealRange(outputs, output_data_transformed_);
//            LOG(INFO) << "Transformed outputs " << output_data_transformed_;
            for (int i = 0; i < outputs.cols(); i++) {
                LOG(INFO) << "Original, transformed " << outputs(0, i) << ", " << output_data_transformed_(0, i);
            }

            LOG(INFO) << "Creating regressor";
            gp_regressor_  = std::make_shared<GaussianProcessRegression<N, 1, Kernel>>(inputs, output_data_transformed_, kernel);
            LOG(INFO) << "Done creating regressor";
        }

        void appendData(const Eigen::MatrixXf &new_inputs, const Eigen::MatrixXf &new_outputs) {
            CHECK_EQ(new_inputs.rows(), N);
            CHECK_EQ(new_outputs.rows(), 1);
            CHECK_EQ(new_inputs.cols(), new_outputs.cols());

            Eigen::MatrixXf new_outputs_transformed = Eigen::MatrixXf(new_outputs.rows(), new_outputs.cols());
            transformZeroToOneOutputsToRealRange(new_outputs, new_outputs_transformed);

            auto prev_size = outputs_.cols();
            auto additional_count = new_outputs.cols();
            auto new_size = additional_count + prev_size;
            outputs_.conservativeResize(Eigen::NoChange, new_size);
            output_data_transformed_.conservativeResize(Eigen::NoChange, new_size);
            outputs_.rightCols(additional_count) = new_outputs;
            output_data_transformed_.rightCols(additional_count) = new_outputs_transformed;

            inputs_.conservativeResize(Eigen::NoChange, new_size);
            inputs_.rightCols(additional_count) = new_inputs;
            num_datapoints_ = new_size;

            gp_regressor_->appendData(new_inputs, output_data_transformed_);
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

            std::pair<Eigen::Matrix<T, 1, Eigen::Dynamic>, Eigen::Matrix<T, 1, Eigen::Dynamic>> regressor_out = gp_regressor_->Inference(x);
//            LOG(INFO) << "Regressor out mean " << regressor_out.first;
//            LOG(INFO) << "Regressor out variance " << regressor_out.second;

            Eigen::Matrix<T, 1, Eigen::Dynamic> classification_output = Eigen::Matrix<T, 1, Eigen::Dynamic>(1, x.cols());
            sigmoidNormalConvolution(regressor_out.first, regressor_out.second, classification_output);
//            LOG(INFO) << "Classification output " << classification_output;

            return classification_output;
        }

    private:

        std::shared_ptr<GaussianProcessRegression<N, 1, Kernel>> gp_regressor_;

        /**
         * Number of input/output pairs that the GP was trained on.
         */
        int num_datapoints_;

        /**
         * Input data (points where we have observations).
         *
         * TODO do we need to maintain this or should we just pass it off to the regressor.
         */
        Eigen::MatrixXf inputs_;

        /**
         * Output data (observations at the input points).
         *
         * Values are in the [0, 1] range.
         *
         * TODO do we need to maintain this at all?
         */
        Eigen::MatrixXf outputs_;

        /**
         * Values are spread out across the entire real line.
         *
         * TODO do we need to maintain this or should we just transform and pass it off to the regressor.
         */
        Eigen::MatrixXf output_data_transformed_;

        // TODO need to figure out how to have non-zero mean

        void transformZeroToOneOutputsToRealRange(const Eigen::MatrixXf &zero_to_one_output_vals,
                                                  Eigen::MatrixXf &real_valued_output_vals) {
            // Using the logit function (inverse of logistic function)
            // f(x) = - ln((1/x) - 1)
            real_valued_output_vals = (-1 * (zero_to_one_output_vals.array().inverse() - 1).log()).matrix();
        }

        template<typename T>
        void sigmoidNormalConvolution(const Eigen::Matrix<T, 1, Eigen::Dynamic> &mean,
                                      const Eigen::Matrix<T, 1, Eigen::Dynamic> &variance,
                                      Eigen::Matrix<T, 1, Eigen::Dynamic> &sigmoid_approx) {

            // mean / sqrt(1 + (pi/8) * variance)
            Eigen::Array<T, 1, Eigen::Dynamic> sigmoid_input = (mean.array()) / ((((variance.array()) * T(M_PI / 8)) + T(1)).sqrt());

            // 1 / (1 + e^(-x))
            sigmoid_approx = ((T(1) + ((T(-1) * sigmoid_input).exp())).inverse()).matrix();
        }
    };
} // end gp_regression
#endif //AUTODIFF_GP_GP_CLASSIFIER_H
