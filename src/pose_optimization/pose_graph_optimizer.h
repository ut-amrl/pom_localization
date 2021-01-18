//
// Created by amanda on 12/4/20.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H
#define AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <pose_optimization/pose_3d_factor_graph.h>
#include <pose_optimization/movable_observation_gp_cost_functor.h>

namespace pose_optimization {

    class PoseGraphOptimizer {
    public:
        PoseGraphOptimizer() = default;

        void buildPoseGraphOptimizationProblem(pose_graph::PoseGraph &pose_graph, const std::unordered_set<pose_graph::NodeId> &nodes_to_optimize, ceres::Problem *problem) {

            ceres::LocalParameterization* quaternion_local_parameterization =
                    new ceres::EigenQuaternionParameterization;

            // Add residuals from movable object observations
            for (pose_graph::MovableObservationFactor &factor : pose_graph.getMovableObservationFactors()) {
                std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.observed_at_node_, pose_vars_)) {
                    LOG(ERROR) << "Node " << factor.observed_at_node_ << " did not exist in the pose graph. Skipping movable object observation";
                    continue;
                }

                if (nodes_to_optimize.find(factor.observed_at_node_) == nodes_to_optimize.end()) {
                    continue;
                }

//                std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> movable_object_regressor = pose_graph.getMovableObjGpRegressor(factor.observation_.semantic_class_);
                std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> movable_object_kde = pose_graph.getMovableObjKde(factor.observation_.semantic_class_);
                if (movable_object_kde) {

                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<pose_optimization::MovableObservationCostFunctor, 1, 3, 4>(
                            new pose_optimization::MovableObservationCostFunctor(
//                                    pose_graph.getMovableObjGpRegressor(factor.observation_.semantic_class_),
                                    movable_object_kde,
                                    factor.observation_.observation_transl_,
                                    factor.observation_.observation_orientation_));
//
//                    ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<pose_optimization::MovableObservationCostFunctor, ceres::CENTRAL, 1, 3, 4>(
//                            new pose_optimization::MovableObservationCostFunctor(
//                                    pose_graph.getMovableObjGpRegressor(factor.observation_.semantic_class_),
//                                    factor.observation_.observation_transl_,
//                                    factor.observation_.observation_orientation_));


                    problem->AddResidualBlock(cost_function, nullptr, pose_vars_.first->data(),
                                              pose_vars_.second->coeffs().data());

                    problem->SetParameterization(pose_vars_.second->coeffs().data(),
                                                 quaternion_local_parameterization);
                } else {
                    LOG(WARNING) << "No gp regressor for semantic class " << factor.observation_.semantic_class_;
                }
            }


            // Add residuals from odometry (visual, lidar, or wheel) factors
            for (pose_graph::GaussianBinaryFactor &factor : pose_graph.getBinaryFactors()) {

                if (nodes_to_optimize.find(factor.to_node_) == nodes_to_optimize.end()) {
                    continue;
                }

                if (nodes_to_optimize.find(factor.from_node_) == nodes_to_optimize.end()) {
                    continue;
                }

                std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> from_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.from_node_, from_pose_vars_)) {
                    LOG(ERROR) << "From node " << factor.from_node_ << " did not exist in the pose graph. Skipping odometry observation";
                    continue;
                }

                std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> to_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.to_node_, to_pose_vars_)) {
                    LOG(ERROR) << "To node " << factor.to_node_ << " did not exist in the pose graph. Skipping odometry observation";
                    continue;
                }
//                LOG(INFO) << "Odom factor " << factor.from_node_ << " to " << factor.to_node_ << ": " <<
//                factor.translation_change_ << ", " << factor.orientation_change_.w() << ", " <<
//                factor.orientation_change_.x() << ", " << factor.orientation_change_.y() << ", " <<
//                factor.orientation_change_.z();
                ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<pose_optimization::OdometryCostFunctor, 6, 3, 4, 3, 4>(
                        new pose_optimization::OdometryCostFunctor(
                                factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));

                problem->AddResidualBlock(cost_function, nullptr, from_pose_vars_.first->data(), from_pose_vars_.second->coeffs().data(),
                                         to_pose_vars_.first->data(), to_pose_vars_.second->coeffs().data());

                problem->SetParameterization(from_pose_vars_.second->coeffs().data(),
                                             quaternion_local_parameterization);
                problem->SetParameterization(to_pose_vars_.second->coeffs().data(),
                                             quaternion_local_parameterization);
            }
            std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> start_pose_vars_;
            pose_graph.getNodePosePointers(0, start_pose_vars_);
            problem->SetParameterBlockConstant(start_pose_vars_.first->data());

            problem->SetParameterBlockConstant(start_pose_vars_.second->coeffs().data());
        }

        bool SolveOptimizationProblem(ceres::Problem* problem, std::vector<ceres::IterationCallback*> callbacks) {
            CHECK(problem != NULL);
            ceres::Solver::Options options;
//            options.max_num_iterations = 100000;

            options.max_num_iterations = 10000;
            options.minimizer_progress_to_stdout = true;
//            options.function_tolerance = 1e-10;
            options.function_tolerance = 1e-20;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            options.gradient_tolerance = 1e-15;
            options.parameter_tolerance = 1e-20;
            options.callbacks = callbacks;
            if (!callbacks.empty()) {
                options.update_state_every_iteration = true;
            }
//            options.minimizer_type = ceres::LINE_SEARCH;
//            options.line_search_direction_type = ceres::STEEPEST_DESCENT;
//            options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
            ceres::Solver::Summary summary;
            ceres::Solve(options, problem, &summary);
            std::cout << summary.FullReport() << '\n';

            return summary.IsSolutionUsable();
        }
    };

}


#endif //AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H
