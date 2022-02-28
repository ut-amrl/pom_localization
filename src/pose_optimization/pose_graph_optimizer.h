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
#include <util/timer.h>

namespace pose_optimization {

    template<typename MovObjKernelType, int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
            int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType, int KernelDim, typename MovableObservationType>
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
        void buildPoseGraphOptimizationProblem(
                pose_graph::PoseGraph<MovObjKernelType, MeasurementTranslationDim, MeasurementRotationType, CovDim,
                        MovObjDistributionTranslationDim, MovObjDistributionRotationType, KernelDim, MovableObservationType> &pose_graph,
                const pose_graph::NodeId &min_node_id,
                const std::unordered_set<pose_graph::NodeId> &nodes_to_optimize,
                const CostFunctionParameters &cost_function_params, ceres::Problem *problem) {

            FunctionTimer ft(__PRETTY_FUNCTION__);

            ceres::LocalParameterization *rotation_parameterization = pose_graph.getRotationParameterization();

            std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                    std::shared_ptr<MeasurementRotationType>> start_pose_vars_;


//            std::unordered_set<pose_graph::NodeId> nodes_to_remove;
//            for (const pose_graph::NodeId &old_node_id : nodes_with_obs_) {
            for (const pose_graph::NodeId &old_node_id : last_optimized_nodes_) {
                if (nodes_to_optimize.find(old_node_id) == nodes_to_optimize.end()) {
                    LOG(INFO) << "Removing node " << old_node_id;
                    std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                            std::shared_ptr<MeasurementRotationType>> pose_vars;
                    pose_graph.getNodePosePointers(old_node_id, pose_vars);
                    pose_graph.getPointersToUnderlyingData(pose_vars);
                    std::pair<double *, double *> raw_pointers_for_node = pose_graph.getPointersToUnderlyingData(
                            pose_vars);
                    problem->RemoveParameterBlock(raw_pointers_for_node.first);
                    problem->RemoveParameterBlock(raw_pointers_for_node.second);
//                    nodes_to_remove.insert(old_node_id);
                }
            }

//            for (const pose_graph::NodeId node_to_remove : nodes_to_remove) {
//                nodes_with_obs_.erase(node_to_remove);
//            }

            pose_graph.getNodePosePointers(min_node_id, start_pose_vars_);
            std::pair<double *, double *> raw_pointers_for_start_node_data = pose_graph.getPointersToUnderlyingData(
                    start_pose_vars_);

            // Add residuals from movable object observations
            for (auto &factor : pose_graph.getMovableObservationFactors()) {

                bool include_factor = true;

                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.second.observed_at_node_, pose_vars_)) {
                    LOG(ERROR) << "Node " << factor.second.observed_at_node_
                               << " did not exist in the pose graph. Skipping movable object observation";
                    include_factor = false;
                }

//                if (new_nodes_to_optimize.find(factor.observed_at_node_) == new_nodes_to_optimize.end()) {
                if (nodes_to_optimize.find(factor.second.observed_at_node_) == nodes_to_optimize.end()) {
                    include_factor = false;
                }

                std::pair<double, Eigen::Matrix<double, MovObjDistributionTranslationDim, 1>> search_criteria =
                        pose_graph.getSampleSearchCriteria(factor.second);

                bool refresh_gpc = false;
                if (include_factor) {
                    if (factor_id_to_search_criteria_.find(factor.first) == factor_id_to_search_criteria_.end()) {
                        refresh_gpc = true;
                    } else {
                        std::pair<double, Eigen::Matrix<double, MovObjDistributionTranslationDim, 1>> prev_search_criteria = factor_id_to_search_criteria_.at(
                                factor.first);
                        if ((abs(search_criteria.first - prev_search_criteria.first) >
                             cost_function_params.gp_radius_change_tolerance_)
                            || ((search_criteria.second - prev_search_criteria.second).norm() >
                                cost_function_params.gp_position_change_tolerance_)) {
                            refresh_gpc = true;
                        }
                    }
                }

                bool remove_old_residual = false;
                bool old_residual_present =
                        factor_id_to_residual_block_.find(factor.first) != factor_id_to_residual_block_.end();
                if (!include_factor || refresh_gpc) {
                    if (old_residual_present) {
                        remove_old_residual = true;
                    }
                }

                if (remove_old_residual) {
                    LOG(INFO) << "Removing residual block for node " << factor.second.observed_at_node_ << " factor id "
                              << factor.first;
                    if (nodes_to_optimize.find(factor.second.observed_at_node_) != nodes_to_optimize.end()) {
                        problem->RemoveResidualBlock(factor_id_to_residual_block_.at(factor.first));
                    }
                    factor_id_to_residual_block_.erase(factor.first);
                }

                if (!include_factor) {
                    continue;
                }

                std::shared_ptr<gp_regression::GaussianProcessClassifier<KernelDim, MovObjKernelType>> movable_object_gpc;
                if (!refresh_gpc && !old_residual_present) {
                    movable_object_gpc = factor_id_to_gpc_[factor.first];
                } else if (!refresh_gpc) {
                    LOG(INFO) << "Using old GP for node " << factor.second.observed_at_node_;
                    continue;
                } else {
                    LOG(INFO) << "Refreshing GP for observation from node " << factor.second.observed_at_node_;
                    movable_object_gpc = pose_graph.getMovableObjGpcWithinRadius(
                            factor.second.observation_.semantic_class_, search_criteria.first, search_criteria.second,
                            cost_function_params.max_gpc_samples_);

                    factor_id_to_gpc_[factor.first] = movable_object_gpc;
                    factor_id_to_search_criteria_[factor.first] = search_criteria;
                }
                if (movable_object_gpc) {

                    ceres::CostFunction *cost_function =
                            pose_graph.createMovableObjectCostFunctor(movable_object_gpc, factor.second,
                                                                      cost_function_params);
                    if (cost_function == nullptr) {
                        // TODO maybe we should just let this cause a failure?
                        LOG(WARNING) << "Could not create cost functor for factor at node "
                                     << factor.second.observed_at_node_;
                        continue;
                    }

                    std::pair<double *, double *> raw_pointers_for_node_data = pose_graph.getPointersToUnderlyingData(
                            pose_vars_);

                    factor_id_to_residual_block_[factor.first] = problem->AddResidualBlock(
                            cost_function, nullptr, raw_pointers_for_node_data.first,
                            raw_pointers_for_node_data.second);
//                    nodes_with_obs_.insert(factor.second.observed_at_node_);

                    if (rotation_parameterization != nullptr) {
                        if (last_optimized_nodes_.find(factor.second.observed_at_node_) ==
                            last_optimized_nodes_.end()) {
                            problem->SetParameterization(raw_pointers_for_node_data.second,
                                                         rotation_parameterization);
                        }
                    }
                } else {
                    LOG(WARNING) << "No gp regressor for semantic class " << factor.second.observation_.semantic_class_;
                }
            }

            // Add residuals from odometry (visual, lidar, or wheel) factors
            for (auto &factor : pose_graph.getBinaryFactors()) {
//
//                bool from_node_new = (new_nodes_to_optimize.find(factor.from_node_) != new_nodes_to_optimize.end());
//                bool to_node_new = (new_nodes_to_optimize.find(factor.to_node_) != new_nodes_to_optimize.end());

                bool include_factor = true;

                bool from_node_new = (nodes_to_optimize.find(factor.second.from_node_) != nodes_to_optimize.end());
                bool to_node_new = (nodes_to_optimize.find(factor.second.to_node_) != nodes_to_optimize.end());

                // Either the to or from node has to be a new node
                // The other node must be a node in nodes to optimize (new or not)
                if (!(from_node_new || to_node_new)) {
                    include_factor = false;
                }

                if (nodes_to_optimize.find(factor.second.to_node_) == nodes_to_optimize.end()) {
                    include_factor = false;
                }

                if (nodes_to_optimize.find(factor.second.from_node_) == nodes_to_optimize.end()) {
                    include_factor = false;
                }
                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> from_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.second.from_node_, from_pose_vars_)) {
                    LOG(ERROR) << "From node " << factor.second.from_node_
                               << " did not exist in the pose graph. Skipping odometry observation";
                    include_factor = false;
                }

                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> to_pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.second.to_node_, to_pose_vars_)) {
                    LOG(ERROR) << "To node " << factor.second.to_node_
                               << " did not exist in the pose graph. Skipping odometry observation";
                    include_factor = false;
                }

                bool remove_old_residual = false;
                if (!include_factor) {
                    if (factor_id_to_residual_block_.find(factor.first) != factor_id_to_residual_block_.end()) {
                        remove_old_residual = true;
                    }
                }

                if (remove_old_residual) {

                    LOG(INFO) << "Removing residual block for nodes " << factor.second.from_node_ << ", "
                              << factor.second.to_node_;
                    if ((nodes_to_optimize.find(factor.second.from_node_) != nodes_to_optimize.end()) &&
                        (nodes_to_optimize.find(factor.second.to_node_) != nodes_to_optimize.end())) {
                        problem->RemoveResidualBlock(factor_id_to_residual_block_.at(factor.first));
                    }
                    factor_id_to_residual_block_.erase(factor.first);
                }

                if (!include_factor) {
                    continue;
                }

                ceres::CostFunction *cost_function = pose_graph.createGaussianBinaryCostFunctor(factor.second);

                std::pair<double *, double *> raw_pointers_for_from_node_data = pose_graph.getPointersToUnderlyingData(
                        from_pose_vars_);
                std::pair<double *, double *> raw_pointers_for_to_node_data = pose_graph.getPointersToUnderlyingData(
                        to_pose_vars_);

                factor_id_to_residual_block_[factor.first] = problem->AddResidualBlock(cost_function, nullptr,
                                                                                       raw_pointers_for_from_node_data.first,
                                                                                       raw_pointers_for_from_node_data.second,
                                                                                       raw_pointers_for_to_node_data.first,
                                                                                       raw_pointers_for_to_node_data.second);

                if (rotation_parameterization != nullptr) {
                    if (last_optimized_nodes_.find(factor.second.from_node_) == last_optimized_nodes_.end()) {
                        problem->SetParameterization(raw_pointers_for_from_node_data.second,
                                                     rotation_parameterization);
                    }
                    if (last_optimized_nodes_.find(factor.second.to_node_) == last_optimized_nodes_.end()) {
                        problem->SetParameterization(raw_pointers_for_to_node_data.second,
                                                     rotation_parameterization);
                    }
                }
            }


//            if (new_nodes_to_optimize.find(0) != new_nodes_to_optimize.end()) {
            // TODO adjust to first node to optimize
            if (last_optimized_nodes_.find(min_node_id) == last_optimized_nodes_.end()) {
                problem->AddParameterBlock(raw_pointers_for_start_node_data.first, MeasurementTranslationDim);
                problem->AddParameterBlock(raw_pointers_for_start_node_data.second,
                                           (CovDim - MeasurementTranslationDim));

                if (rotation_parameterization != nullptr) {
                    problem->SetParameterization(raw_pointers_for_start_node_data.second, rotation_parameterization);
                }
            }

//            if (!last_optimized_nodes_.empty()) {
//                LOG(INFO) << "Last optimized nodes empty";
//                if (last_min_pose_id_ != min_node_id) {}
            if (((!last_optimized_nodes_.empty()) && (last_min_pose_id_ != min_node_id) &&
                 (nodes_to_optimize.find(last_min_pose_id_) != nodes_to_optimize.end())) && (last_min_pose_id_ != 0)) {

                std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                        std::shared_ptr<MeasurementRotationType>> pose_vars;
                pose_graph.getNodePosePointers(last_min_pose_id_, pose_vars);
                pose_graph.getPointersToUnderlyingData(pose_vars);
                std::pair<double *, double *> raw_pointers_for_node = pose_graph.getPointersToUnderlyingData(
                        pose_vars);

                LOG(INFO) << "Setting parameter block variable for node " << last_min_pose_id_;
                problem->SetParameterBlockVariable(raw_pointers_for_node.first);
                problem->SetParameterBlockVariable(raw_pointers_for_node.second);
            }

//            if (min_node_id == 0) {
//            if (nodes_with_obs_.find(min_node_id) != nodes_with_obs_.end()) {
            LOG(INFO) << "Setting parameter block " << min_node_id << " constant";
            problem->SetParameterBlockConstant(raw_pointers_for_start_node_data.first);
            problem->SetParameterBlockConstant(raw_pointers_for_start_node_data.second);
//            }
//            }
            last_min_pose_id_ = min_node_id;
            last_optimized_nodes_ = nodes_to_optimize;

        }

        bool SolveOptimizationProblem(ceres::Problem *problem, std::vector<ceres::IterationCallback *> callbacks,
                                      const pose_optimization::OptimizerParameters &opt_params,
                                      const bool &extra_refinement, const bool &extra_extra_refinement) {
            CHECK(problem != NULL);
            ceres::Solver::Options options;
//            options.max_num_iterations = 100000;

//            options.max_num_iterations = 1000;
            if (extra_extra_refinement) {
                options.max_num_iterations = opt_params.extra_extra_refinement_max_iter;
            } else if (extra_refinement) {
                options.max_num_iterations = opt_params.extra_refinement_max_iter;
            } else {
                options.max_num_iterations = opt_params.normal_max_iter;
            }

            LOG(INFO) << "Max iter " << options.max_num_iterations;

//            options.max_num_iterations = 0;
            options.minimizer_progress_to_stdout = true;
            if (extra_extra_refinement) {
                options.function_tolerance = opt_params.extra_extra_refinement_function_tolerance;
            } else if (extra_refinement) {
                options.function_tolerance = opt_params.extra_refinement_function_tolerance;
            } else {
                options.function_tolerance = opt_params.normal_function_tolerance;
            }
            options.use_nonmonotonic_steps = opt_params.allow_non_monotonic_steps;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
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

            if ((summary.termination_type == ceres::TerminationType::FAILURE) ||
                (summary.termination_type == ceres::TerminationType::USER_FAILURE)) {
                LOG(ERROR) << "Ceres optimization failed";
//                exit(1);
            }

            return summary.IsSolutionUsable();
        }

    private:
        std::unordered_set<pose_graph::NodeId> last_optimized_nodes_;

        pose_graph::NodeId last_min_pose_id_ = 0;

        std::unordered_map<uint64_t, ceres::ResidualBlockId> factor_id_to_residual_block_;

        std::unordered_map<uint64_t, std::shared_ptr<gp_regression::GaussianProcessClassifier<KernelDim, MovObjKernelType>>> factor_id_to_gpc_;

        std::unordered_map<uint64_t, std::pair<double, Eigen::Matrix<double, MovObjDistributionTranslationDim, 1>>> factor_id_to_search_criteria_;

//        std::unordered_set<pose_graph::NodeId> nodes_with_obs_;
    };

}


#endif //AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H
