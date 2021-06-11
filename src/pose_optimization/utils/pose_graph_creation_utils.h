//
// Created by amanda on 5/31/21.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_CREATION_UTILS_H
#define AUTODIFF_GP_POSE_GRAPH_CREATION_UTILS_H

#include <pose_optimization/pose_3d_factor_graph.h>

namespace pose_graph {
    namespace utils {
        static std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3>>
        createFully2dPoseGraphFromParams(const pose_optimization::CostFunctionParameters &cost_function_params) {

            gp_kernel::GaussianKernel<2> mean_position_kernel(cost_function_params.mean_position_kernel_len_,
                                                              cost_function_params.mean_position_kernel_var_);
            gp_kernel::PeriodicGaussianKernel<1> mean_orientation_kernel(M_PI * 2,
                                                                         cost_function_params.mean_orientation_kernel_var_,
                                                                         cost_function_params.mean_orientation_kernel_len_);
            std::shared_ptr<gp_kernel::Pose2dKernel> mean_pose_2d_kernel = std::make_shared<gp_kernel::Pose2dKernel>(
                    mean_position_kernel, mean_orientation_kernel);

            std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(
                    const double &)> kernel_creator = [cost_function_params](const double &subsampling_ratio) {
                gp_kernel::GaussianKernel<2> var_position_kernel(cost_function_params.var_position_kernel_len_,
                                                                 (1.0 / subsampling_ratio) *
                                                                 cost_function_params.var_position_kernel_var_);
                gp_kernel::PeriodicGaussianKernel<1> var_orientation_kernel(M_PI * 2,
                                                                            cost_function_params.var_orientation_kernel_var_,
                                                                            cost_function_params.var_orientation_kernel_len_);

                return std::make_shared<gp_kernel::Pose2dKernel>(var_position_kernel, var_orientation_kernel);
            };

            return std::make_shared<pose_graph::PoseGraph2dMovObjDistribution2d>(
                    cost_function_params.obj_probability_prior_mean_by_class_,
                    cost_function_params.default_obj_probability_prior_mean_,
                    cost_function_params.obj_probability_input_variance_by_class_for_mean_,
                    cost_function_params.default_obj_probability_input_variance_for_mean_,
                    cost_function_params.obj_probability_input_variance_by_class_for_var_,
                    cost_function_params.default_obj_probability_input_variance_for_var_,
                    mean_pose_2d_kernel,
                    kernel_creator);
        }
    }
}

#endif //AUTODIFF_GP_POSE_GRAPH_CREATION_UTILS_H
