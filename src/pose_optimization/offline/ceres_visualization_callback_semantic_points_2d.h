//
// Created by amanda on 1/30/21.
//

#ifndef AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_SEMANTIC_POINTS_2D_H
#define AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_SEMANTIC_POINTS_2D_H

#include <visualization/ros_visualization.h>
#include <ceres/iteration_callback.h>
#include <pose_optimization/pose_graph_generic.h>

namespace offline_optimization {

    class CeresVisualizationCallbackSemanticPoints2d : public ceres::IterationCallback {
    public:
        explicit CeresVisualizationCallbackSemanticPoints2d(
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                        pose_graph::MovableObservationSemanticPoints<2>>> &pose_graph,
                const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                const int32_t &num_poses,
                const std::unordered_map<std::string, std::vector<std::vector<std::vector<Eigen::Vector2d>>>> &observed_semantic_points_by_class,
                const std::unordered_map<std::string, std::unordered_map<pose_graph::NodeId, std::unordered_map<size_t, std::vector<pose::Pose2d>>>> &rectangle_samples,
                const std::unordered_map<std::string, Eigen::Vector2d> &shape_dimensions_by_class) :
                pose_graph_(pose_graph),
                vis_manager_(vis_manager),
                num_poses_(num_poses),
                observed_semantic_points_by_class_(observed_semantic_points_by_class),
                rectangle_samples_(rectangle_samples),
                shape_dimensions_by_class_(shape_dimensions_by_class) {
        }

        ~CeresVisualizationCallbackSemanticPoints2d() override = default;

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {

            std::unordered_map<pose_graph::NodeId, pose::Pose2d> node_poses;
            pose_graph_->getNodePoses(node_poses);
            std::vector<pose::Pose2d> node_poses_list;

            for (int32_t i = 0; i <= num_poses_; i++) {
                pose::Pose2d optimized_pose = node_poses[i];
                node_poses_list.emplace_back(optimized_pose);
            }
            vis_manager_->displayEstTrajectory(node_poses_list);

            for (const auto &objs_and_class : observed_semantic_points_by_class_) {
                vis_manager_->displaySemanticPointObsFromEstTrajectory(node_poses_list, objs_and_class.second,
                                                                       objs_and_class.first,
                                                                       rectangle_samples_[objs_and_class.first],
                                                                       shape_dimensions_by_class_[objs_and_class.first]);
            }

            return ceres::SOLVER_CONTINUE;
        }

    private:
        std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                pose_graph::MovableObservationSemanticPoints<2>>> pose_graph_;

        std::shared_ptr<visualization::VisualizationManager> vis_manager_;

        int32_t num_poses_;

        std::unordered_map<std::string, std::vector<std::vector<std::vector<Eigen::Vector2d>>>> observed_semantic_points_by_class_;

        std::unordered_map<std::string, std::unordered_map<pose_graph::NodeId, std::unordered_map<size_t, std::vector<pose::Pose2d>>>> rectangle_samples_;

        std::unordered_map<std::string, Eigen::Vector2d> shape_dimensions_by_class_;
    };
}

#endif //AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_SEMANTIC_POINTS_2D_H
