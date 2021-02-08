//
// Created by amanda on 1/28/21.
//

#ifndef AUTODIFF_GP_OFFLINE_PROBLEM_RUNNER_H
#define AUTODIFF_GP_OFFLINE_PROBLEM_RUNNER_H

#include <pose_optimization/offline/offline_problem_data.h>

namespace offline_optimization {

    enum VisualizationTypeEnum {
        BEFORE_ANY_OPTIMIZATION,
        BEFORE_EACH_OPTIMIZATION,
        AFTER_EACH_OPTIMIZATION,
        AFTER_ALL_OPTIMIZATION
    };

    template<typename MovObjKernelType, int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
            int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType, int KernelDim>
    class OfflinePoseOptimizer {
    public:

        typedef pose_graph::PoseGraph<MovObjKernelType,
                MeasurementTranslationDim, MeasurementRotationType, CovDim,
                MovObjDistributionTranslationDim, MovObjDistributionRotationType, KernelDim> PoseGraphType;
        typedef OfflineProblemData<MeasurementTranslationDim, MeasurementRotationType, CovDim,
        MovObjDistributionTranslationDim, MovObjDistributionRotationType> OfflineProblemDataType;
        typedef pose_graph::Node<MeasurementTranslationDim, MeasurementRotationType> NodeType;
        typedef pose_graph::GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> GaussianBinaryFactorType;
        typedef pose_graph::MovableObservationFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> MovableObservationFactorType;
        typedef pose_graph::MapObjectObservation<MovObjDistributionTranslationDim, MovObjDistributionRotationType> MapObjectObservationType;
        typedef Eigen::Matrix<double, MeasurementTranslationDim, 1> TranslType;
        typedef std::pair<TranslType, MeasurementRotationType> PoseType;


        OfflinePoseOptimizer() {
        }

        std::unordered_map<pose_graph::NodeId, PoseType> runOfflineOptimization(const OfflineProblemDataType &problem_data,
                                    const pose_optimization::CostFunctionParameters &cost_func_params,
                                    const pose_optimization::PoseOptimizationParameters &optimization_params,
                                    const std::function<std::shared_ptr<PoseGraphType> (const pose_optimization::CostFunctionParameters &)> pose_graph_creator,
                                    const std::function<ceres::IterationCallback*(const pose_graph::NodeId&, const std::shared_ptr<PoseGraphType>&)> callback_creator,
                                    const std::function<void(const pose_graph::NodeId&, const std::shared_ptr<PoseGraphType>&, const VisualizationTypeEnum&)> visualization_callback) {

            // Create pose graph
            std::shared_ptr<PoseGraphType> pose_graph = pose_graph_creator(cost_func_params);

            // Add data to pose graph ---------------------------------------------------------------------------------
            // Add nodes
            pose_graph::NodeId max_node_id = 0;
            for (const NodeType &node : problem_data.initial_node_positions_) {
                pose_graph->addNode(node);
                max_node_id = std::max(max_node_id, node.id_);
            }

            // Add Gaussian factors (odometry)
            if (cost_func_params.odometry_enabled_) {
                for (const GaussianBinaryFactorType &odom_factor : problem_data.odometry_factors_) {
                    pose_graph->addGaussianBinaryFactor(odom_factor);
                }
            }

            if (cost_func_params.movable_obj_observations_enabled_) {

                // Add the map observations
                std::unordered_map<std::string, std::pair<
                        std::vector<pose_graph::NegativeMapObjectObservation<MovObjDistributionTranslationDim>>,
                        std::vector<MapObjectObservationType>>> observations_by_class;
                for (const MapObjectObservationType &map_observation : problem_data.map_object_observations_) {
                    std::pair<std::vector<pose_graph::NegativeMapObjectObservation<MovObjDistributionTranslationDim>>,
                            std::vector<MapObjectObservationType>> observations_for_class;
                    auto obs_for_class_iter = observations_by_class.find(map_observation.semantic_class_);
                    if (obs_for_class_iter == observations_by_class.end()) {
                        observations_for_class = std::make_pair(
                                (std::vector<pose_graph::NegativeMapObjectObservation<MovObjDistributionTranslationDim>>) {},
                                (std::vector<MapObjectObservationType>) {});
                    } else {
                        observations_for_class = observations_by_class.at(map_observation.semantic_class_);
                    }

                    observations_for_class.second.emplace_back(map_observation);
                    observations_by_class[map_observation.semantic_class_] = observations_for_class;
                }
                pose_graph->addMapFrameObservations(observations_by_class);

                // Add the movable object observations as seen from nodes in the trajectory that we're optimizing
                pose_graph->addMovableObservationFactors(problem_data.movable_observation_factors_);
            }

            // Initial visualization -------------------------------------------------------------------------
            visualization_callback(max_node_id, pose_graph, VisualizationTypeEnum::BEFORE_ANY_OPTIMIZATION);

            // Run optimization, adding 1 node at a time
            // Assumes that we have all nodes in the range 0 to N, where 0 is immovable and N is the highest numbered
            // node
            // TODO consider just adding prior to 0 instead of making it non-optimizable

            pose_optimization::PoseGraphOptimizer optimizer;

            std::unordered_set<pose_graph::NodeId> nodes_to_optimize;
            nodes_to_optimize.insert(0);
            for (pose_graph::NodeId next_pose_to_optimize = 1;
                 next_pose_to_optimize <= max_node_id; next_pose_to_optimize++) {
                nodes_to_optimize.insert(next_pose_to_optimize);

                // Run any per-pose pre-solving visualization
                visualization_callback(next_pose_to_optimize, pose_graph,
                                       VisualizationTypeEnum::BEFORE_EACH_OPTIMIZATION);

                // TODO Modify the pose graph optimizer to incrementally add data instead of needing to rebuild
                //  the ceres problem every time

                ceres::Problem problem;
                optimizer.buildPoseGraphOptimizationProblem(*(pose_graph.get()), nodes_to_optimize, cost_func_params, &problem);

                std::vector<ceres::IterationCallback *> ceres_callbacks;
                ceres::IterationCallback *callback = callback_creator(next_pose_to_optimize, pose_graph);
                if (callback != nullptr) {
                    ceres_callbacks.emplace_back(callback);
                }

                // Set up callback for visualization
                optimizer.SolveOptimizationProblem(&problem, ceres_callbacks);

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