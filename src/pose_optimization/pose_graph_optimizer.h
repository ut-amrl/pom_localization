//
// Created by amanda on 12/4/20.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H
#define AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H

#include <ceres/ceres.h>
#include <glog/logging.h>

#include <pose_optimization/pose_3d_factor_graph.h>
//#include <pose_optimization/movable_observation_gp_cost_functor.h>
#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_3d.h>
#include <pose_optimization/pose_optimization_parameters.h>

namespace pose_optimization {

    class PoseGraphOptimizer {
    public:
        PoseGraphOptimizer() = default;

        /**
         *
         * @tparam MovObjKernelType
         * @tparam MeasurementTranslationDim
         * @tparam MeasurementRotationType
         * @tparam CovDim
         * @tparam MovObjDistributionTranslationDim
         * @tparam MovObjDistributionRotationType
         * @tparam KernelDim
         * @param pose_graph
         * @param nodes_to_optimize                 Set of all nodes to optimize. New nodes to optimize should be a
         *                                          subset of this. Nodes in this set that are not in new nodes to
         *                                          optimize have already had their relevant constraints added in
         *                                          previous iterations.
         * @param new_nodes_to_optimize             Set of new nodes to optimize. Tse have not had any constraints with
         *                                          them added in a previous iteration.
         * @param cost_function_params
         * @param problem
         */
        template<typename MovObjKernelType, int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
                int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType, int KernelDim>
        void buildPoseGraphOptimizationProblem(
                pose_graph::PoseGraph<MovObjKernelType, MeasurementTranslationDim, MeasurementRotationType, CovDim,
                MovObjDistributionTranslationDim, MovObjDistributionRotationType, KernelDim> &pose_graph,
                const std::unordered_set<pose_graph::NodeId> &nodes_to_optimize,
                const std::unordered_set<pose_graph::NodeId> &new_nodes_to_optimize,
                const CostFunctionParameters &cost_function_params, ceres::Problem *problem) {

            ceres::LocalParameterization *rotation_parameterization = pose_graph.getRotationParameterization();

            std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                    std::shared_ptr<MeasurementRotationType>> start_pose_vars_;
            pose_graph.getNodePosePointers(0, start_pose_vars_);
            std::pair<double *, double *> raw_pointers_for_start_node_data = pose_graph.getPointersToUnderlyingData(
                    start_pose_vars_);

            // Add residuals from movable object observations
            for (pose_graph::MovableObservationFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim>
                    &factor : pose_graph.getMovableObservationFactors()) {

                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.observed_at_node_, pose_vars_)) {
                    LOG(ERROR) << "Node " << factor.observed_at_node_ << " did not exist in the pose graph. Skipping movable object observation";
                    continue;
                }

                if (new_nodes_to_optimize.find(factor.observed_at_node_) == new_nodes_to_optimize.end()) {
                    continue;
                }

                std::shared_ptr<gp_regression::GaussianProcessClassifier<KernelDim, MovObjKernelType>> movable_object_gpc =
                        pose_graph.getMovableObjGpc(factor.observation_.semantic_class_);
                if (movable_object_gpc) {

                    ceres::CostFunction *cost_function =
                            pose_graph.createMovableObjectCostFunctor(movable_object_gpc, factor, cost_function_params);

                    std::pair<double*, double*> raw_pointers_for_node_data = pose_graph.getPointersToUnderlyingData(pose_vars_);

                    problem->AddResidualBlock(cost_function, nullptr, raw_pointers_for_node_data.first,
                                              raw_pointers_for_node_data.second);

                    if (rotation_parameterization != nullptr) {
                        problem->SetParameterization(raw_pointers_for_node_data.second,
                                                     rotation_parameterization);
                    }
                } else {
                    LOG(WARNING) << "No gp regressor for semantic class " << factor.observation_.semantic_class_;
                }
            }


            // Add residuals from odometry (visual, lidar, or wheel) factors
            for (pose_graph::GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> &factor
            : pose_graph.getBinaryFactors()) {

                bool from_node_new = (new_nodes_to_optimize.find(factor.from_node_) != new_nodes_to_optimize.end());
                bool to_node_new = (new_nodes_to_optimize.find(factor.to_node_) != new_nodes_to_optimize.end());

                // Either the to or from node has to be a new node
                // The other node must be a node in nodes to optimize (new or not)
                if (!(from_node_new || to_node_new)) {
                    continue;
                }

                if (nodes_to_optimize.find(factor.to_node_) == nodes_to_optimize.end()) {
                    continue;
                }

                if (nodes_to_optimize.find(factor.from_node_) == nodes_to_optimize.end()) {
                    continue;
                }
                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> from_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.from_node_, from_pose_vars_)) {
                    LOG(ERROR) << "From node " << factor.from_node_ << " did not exist in the pose graph. Skipping odometry observation";
                    continue;
                }

                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> to_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.to_node_, to_pose_vars_)) {
                    LOG(ERROR) << "To node " << factor.to_node_ << " did not exist in the pose graph. Skipping odometry observation";
                    continue;
                }

                ceres::CostFunction *cost_function = pose_graph.createGaussianBinaryCostFunctor(factor);

                std::pair<double*, double*> raw_pointers_for_from_node_data = pose_graph.getPointersToUnderlyingData(from_pose_vars_);
                std::pair<double*, double*> raw_pointers_for_to_node_data = pose_graph.getPointersToUnderlyingData(to_pose_vars_);

                problem->AddResidualBlock(cost_function, nullptr, raw_pointers_for_from_node_data.first,
                                          raw_pointers_for_from_node_data.second,
                                          raw_pointers_for_to_node_data.first,
                                          raw_pointers_for_to_node_data.second);

                if (rotation_parameterization != nullptr) {
                    if (from_node_new) {
                        problem->SetParameterization(raw_pointers_for_from_node_data.second,
                                                     rotation_parameterization);
                    }
                    if (to_node_new) {
                        problem->SetParameterization(raw_pointers_for_to_node_data.second,
                                                     rotation_parameterization);
                    }
                }
            }


            if (new_nodes_to_optimize.find(0) != new_nodes_to_optimize.end()) {

                problem->AddParameterBlock(raw_pointers_for_start_node_data.first, MeasurementTranslationDim);
                problem->AddParameterBlock(raw_pointers_for_start_node_data.second, (CovDim - MeasurementTranslationDim));

                if (rotation_parameterization != nullptr) {
                    problem->SetParameterization(raw_pointers_for_start_node_data.second, rotation_parameterization);
                }
            }

            problem->SetParameterBlockConstant(raw_pointers_for_start_node_data.first);
            problem->SetParameterBlockConstant(raw_pointers_for_start_node_data.second);
        }

        bool SolveOptimizationProblem(ceres::Problem* problem, std::vector<ceres::IterationCallback*> callbacks) {
            CHECK(problem != NULL);
            ceres::Solver::Options options;
//            options.max_num_iterations = 100000;

            options.max_num_iterations = 1000;
            options.minimizer_progress_to_stdout = true;
//            options.function_tolerance = 1e-10;
//            options.function_tolerance = 1e-20;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//            options.gradient_tolerance = 1e-15;
//            options.parameter_tolerance = 1e-20;
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
