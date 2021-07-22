

#ifndef AUTODIFF_GP_POSE_OPTIMIZATION_PARAMETERS_H
#define AUTODIFF_GP_POSE_OPTIMIZATION_PARAMETERS_H

//#include <eigen3/Eigen/Dense>

namespace pose_optimization {

    /**
     * This is for parameters used in running the optimizer
     */
    struct OptimizerParameters {

    };

    /**
     * This is for parameters used in generating/evaluating cost functors and setting up the optimization problem
     * (weights, num samples, etc).
     */
    struct CostFunctionParameters {

        CostFunctionParameters() : num_samples_per_movable_obj_observation_(kDefaultNumSamplesPerMovableObjObservation),
        odometry_enabled_(kDefaultOdometryEnabled),
        movable_obj_observations_enabled_(kDefaultMovableObjObservationsEnabled) {

        }

        static const int kDefaultNumSamplesPerMovableObjObservation = 25;
        static const bool kDefaultOdometryEnabled = true;
        static const bool kDefaultMovableObjObservationsEnabled = true;
        int num_samples_per_movable_obj_observation_;
        bool odometry_enabled_;
        bool movable_obj_observations_enabled_;

        float mean_position_kernel_len_ = 0.7;
        float mean_position_kernel_var_ = 1;
        float mean_orientation_kernel_len_ = 0.1;
        float mean_orientation_kernel_var_ = 1;

        float var_position_kernel_len_ = 0.7;
        float var_position_kernel_var_ = 1;
        float var_orientation_kernel_len_ = 0.1;
        float var_orientation_kernel_var_ = 1;

        std::unordered_map<std::string, double> obj_probability_prior_mean_by_class_;
        double default_obj_probability_prior_mean_ = 0.05;

        std::unordered_map<std::string, double> obj_probability_input_variance_by_class_for_mean_;
        double default_obj_probability_input_variance_for_mean_ = 1.0;

        std::unordered_map<std::string, double> obj_probability_input_variance_by_class_for_var_;
        double default_obj_probability_input_variance_for_var_ = 1.0;

        double gp_radius_change_tolerance_ = 0.5;
        double gp_position_change_tolerance_ = 0.5;

        uint64_t num_nodes_in_optimization_window_ = 60;
        uint64_t full_optimization_interval_ = 60;

        uint64_t max_gpc_samples_ = std::numeric_limits<uint64_t>::max();
    };

    /**
     * All parameters used in building and solving the optimization problem.
     */
    struct PoseOptimizationParameters {
        OptimizerParameters optimizer_params_;
        CostFunctionParameters cost_function_params_;
    };

} // end pose_optimization

#endif //AUTODIFF_GP_ODOMETRY_COST_FUNCTOR_H
