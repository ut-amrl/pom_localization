//
// Created by amanda on 1/28/21.
//

#ifndef AUTODIFF_GP_OFFLINE_PROBLEM_RUNNER_H
#define AUTODIFF_GP_OFFLINE_PROBLEM_RUNNER_H

#include <ceres/iteration_callback.h>

#include <pose_optimization/offline/offline_problem_data.h>
#include <pose_optimization/pose_graph_optimizer.h>
#include <ros/ros.h>

namespace offline_optimization {

    enum VisualizationTypeEnum {
        BEFORE_ANY_OPTIMIZATION,
        BEFORE_EACH_OPTIMIZATION,
        AFTER_EACH_OPTIMIZATION,
        AFTER_ALL_OPTIMIZATION
    };

    template<typename MovObjKernelType, int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
            int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType, int KernelDim, typename MovableObservationType>
    class OfflinePoseOptimizer {
    public:

        typedef pose_graph::PoseGraph<MovObjKernelType, MeasurementTranslationDim, MeasurementRotationType, CovDim,
                MovObjDistributionTranslationDim, MovObjDistributionRotationType, KernelDim,
                MovableObservationType> PoseGraphType;
        typedef OfflineProblemData<MeasurementTranslationDim, MeasurementRotationType, CovDim,
                MovObjDistributionTranslationDim, MovObjDistributionRotationType,
                MovableObservationType> OfflineProblemDataType;
        typedef pose_graph::Node<MeasurementTranslationDim, MeasurementRotationType> NodeType;
        typedef pose_graph::GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> GaussianBinaryFactorType;
        typedef pose_graph::MovableObservationFactor<MovableObservationType> MovableObservationFactorType;
        typedef pose_graph::MapObjectObservation<MovObjDistributionTranslationDim, MovObjDistributionRotationType> MapObjectObservationType;
        typedef Eigen::Matrix<double, MeasurementTranslationDim, 1> TranslType;
        typedef std::pair<TranslType, MeasurementRotationType> PoseType;


        OfflinePoseOptimizer() {
        }

        ~OfflinePoseOptimizer() = default;

        std::unordered_map<pose_graph::NodeId, PoseType>
        runOfflineOptimization(const OfflineProblemDataType &problem_data,
                               const pose_optimization::PoseOptimizationParameters &problem_params,
                               const std::function<std::shared_ptr<PoseGraphType>(
                                       const pose_optimization::CostFunctionParameters &)> pose_graph_creator,
                               const std::function<std::shared_ptr<ceres::IterationCallback>(const pose_graph::NodeId &,
                                                                                             const std::shared_ptr<PoseGraphType> &)> callback_creator,
                               const std::function<void(const pose_graph::NodeId &,
                                                        const std::shared_ptr<PoseGraphType> &,
                                                        const VisualizationTypeEnum &)> visualization_callback) {

            // Create pose graph
            LOG(INFO) << "Creating pose graph";
            std::shared_ptr<PoseGraphType> pose_graph = pose_graph_creator(problem_params.cost_function_params_);

            // Add data to pose graph ---------------------------------------------------------------------------------
            // Add nodes
            pose_graph::NodeId max_node_id = 0;
            for (const NodeType &node : problem_data.initial_node_positions_) {
                pose_graph->addNode(node);
                max_node_id = std::max(max_node_id, node.id_);
            }

            // Add Gaussian factors (odometry)
            if (problem_params.cost_function_params_.odometry_enabled_) {

                LOG(INFO) << "Adding odom factors";
                for (const GaussianBinaryFactorType &odom_factor : problem_data.odometry_factors_) {
                    pose_graph->addGaussianBinaryFactor(odom_factor);
                }
            }

            if (problem_params.cost_function_params_.movable_obj_observations_enabled_) {

                LOG(INFO) << "Adding observation factors";
                // Add the map observations
                std::unordered_map<std::string, std::vector<MapObjectObservationType>> observations_by_class;
                for (const MapObjectObservationType &map_observation : problem_data.map_object_observations_) {
                    std::vector<MapObjectObservationType> observations_for_class;
                    auto obs_for_class_iter = observations_by_class.find(map_observation.semantic_class_);
                    if (obs_for_class_iter == observations_by_class.end()) {
                        observations_for_class = {};
                    } else {
                        observations_for_class = observations_by_class.at(map_observation.semantic_class_);
                    }

                    observations_for_class.emplace_back(map_observation);
                    observations_by_class[map_observation.semantic_class_] = observations_for_class;
                }
                LOG(INFO) << "Adding map frame observations";
                pose_graph->addMapFrameObservations(observations_by_class);

                // Add the movable object observations as seen from nodes in the trajectory that we're optimizing
                LOG(INFO) << "Adding movable object observation factors";
                pose_graph->addMovableObservationFactors(problem_data.movable_observation_factors_);
            }

            // Initial visualization -------------------------------------------------------------------------
            LOG(INFO) << "Initial visualization";
            visualization_callback(max_node_id, pose_graph, VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);

            // Run optimization, adding 1 node at a time
            // Assumes that we have all nodes in the range 0 to N, where 0 is immovable and N is the highest numbered
            // node
            // TODO consider just adding prior to 0 instead of making it non-optimizable

            pose_optimization::PoseGraphOptimizer<MovObjKernelType, MeasurementTranslationDim, MeasurementRotationType,
                    CovDim, MovObjDistributionTranslationDim, MovObjDistributionRotationType, KernelDim,
                    MovableObservationType> optimizer;
            ceres::Problem problem;

//            std::unordered_set<pose_graph::NodeId> nodes_to_optimize;
//            nodes_to_optimize.insert(0);
            for (pose_graph::NodeId next_pose_to_optimize = 1;
                 next_pose_to_optimize <= max_node_id; next_pose_to_optimize++) {
                if (!ros::ok()) {
                    exit(1);
                }


                std::unordered_set<pose_graph::NodeId> nodes_to_optimize;
                pose_graph::NodeId start_node_to_optimize;
                bool refine_full = (
                        (next_pose_to_optimize % problem_params.cost_function_params_.full_optimization_interval_) ==
                        0);
                if (refine_full || (next_pose_to_optimize == max_node_id)) {
                    start_node_to_optimize = 0;
                } else {
                    if ((next_pose_to_optimize) >=
                        (problem_params.cost_function_params_.num_nodes_in_optimization_window_ - 1)) {
                        start_node_to_optimize = next_pose_to_optimize -
                                                 (problem_params.cost_function_params_.num_nodes_in_optimization_window_ -
                                                  1);
                    } else {
                        start_node_to_optimize = 0;
                    }
                }
                LOG(INFO) << "Starting node " << start_node_to_optimize;
                for (pose_graph::NodeId node_to_include = start_node_to_optimize;
                     node_to_include <= next_pose_to_optimize; node_to_include++) {
//                    LOG(INFO) << "Adding node " << node_to_include;
                    nodes_to_optimize.insert(node_to_include);
                }

                // Recompute the initial pose for this node based on the
                std::pair<std::shared_ptr<TranslType>, std::shared_ptr<MeasurementRotationType>> pose_vars;
                if (!pose_graph->getNodePosePointers(next_pose_to_optimize, pose_vars)) {
                    LOG(ERROR) << "Node " << next_pose_to_optimize
                               << " did not exist in the pose graph.This shouldn't happen";
                    continue;
                }
                LOG(INFO) << "Initial initial pose " << *(pose_vars.first) << ", " << *(pose_vars.second);
                for (auto &factor_with_id : pose_graph->getBinaryFactors()) {
                    auto factor = factor_with_id.second;
                    if (factor.to_node_ == next_pose_to_optimize) {
                        std::pair<std::shared_ptr<TranslType>, std::shared_ptr<MeasurementRotationType>> from_pose_vars;
                        if ((pose_graph->getNodePosePointers(factor.from_node_, from_pose_vars)) &&
                            (nodes_to_optimize.find(factor.from_node_) != nodes_to_optimize.end())) {
                            PoseType updated_init_est = pose::combinePoses(
                                    std::make_pair(*(from_pose_vars.first), *(from_pose_vars.second)),
                                    std::make_pair(factor.translation_change_, factor.orientation_change_));
                            *(pose_vars.first) = updated_init_est.first;
                            *(pose_vars.second) = updated_init_est.second;
                            break;
                        }
                    }
                }
                std::pair<std::shared_ptr<TranslType>, std::shared_ptr<MeasurementRotationType>> updated_pose_vars;
                if (!pose_graph->getNodePosePointers(next_pose_to_optimize, updated_pose_vars)) {
                    LOG(ERROR) << "Node " << next_pose_to_optimize
                               << " did not exist in the pose graph.This shouldn't happen";
                    continue;
                } else {
                    LOG(INFO) << "Revised initial pose " << *(updated_pose_vars.first) << ", "
                              << *(updated_pose_vars.second);
                }

                nodes_to_optimize.insert(next_pose_to_optimize);


                // TODO Modify the pose graph optimizer to incrementally add data instead of needing to rebuild
                //  the ceres problem every time

//                std::unordered_set<pose_graph::NodeId> new_nodes_to_optimize = {next_pose_to_optimize};
//                if (next_pose_to_optimize == 1) {
//                    new_nodes_to_optimize.insert(0);
//                }
                optimizer.buildPoseGraphOptimizationProblem(*(pose_graph.get()), start_node_to_optimize,
                                                            nodes_to_optimize,
                                                            problem_params.cost_function_params_, &problem);

                // Run any per-pose pre-solving visualization
                visualization_callback(next_pose_to_optimize, pose_graph,
                                       VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);

                std::vector<ceres::IterationCallback *> ceres_callbacks;
                std::shared_ptr<ceres::IterationCallback> callback = callback_creator(next_pose_to_optimize,
                                                                                      pose_graph);
                if (callback.get() != nullptr) {
                    ceres_callbacks.emplace_back(callback.get());
                }

                // Set up callback for visualization
                optimizer.SolveOptimizationProblem(&problem, ceres_callbacks, problem_params.optimizer_params_,
                                                   refine_full, next_pose_to_optimize == max_node_id);

                // Run any per-pose post-solving visualization
                visualization_callback(next_pose_to_optimize, pose_graph,
                                       VisualizationTypeEnum::AFTER_EACH_OPTIMIZATION);
            }

            // Run any final visualization
            visualization_callback(max_node_id, pose_graph, VisualizationTypeEnum::AFTER_ALL_OPTIMIZATION);

            std::unordered_map<pose_graph::NodeId, PoseType> optimized_trajectory;
            pose_graph->getNodePoses(optimized_trajectory);
            return optimized_trajectory;
        }
    };
}

#endif //AUTODIFF_GP_OFFLINE_PROBLEM_RUNNER_H
