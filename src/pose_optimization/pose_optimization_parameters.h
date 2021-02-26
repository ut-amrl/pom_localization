

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

        static const int kDefaultNumSamplesPerMovableObjObservation = 10;
        static const bool kDefaultOdometryEnabled = true;
        static const bool kDefaultMovableObjObservationsEnabled = true;
        int num_samples_per_movable_obj_observation_;
        bool odometry_enabled_;
        bool movable_obj_observations_enabled_;

        float position_kernel_len_ = 0.7;
        float position_kernel_var_ = 1;
        float orientation_kernel_len_ = 0.1;
        float orientation_kernel_var_ = 1;
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
