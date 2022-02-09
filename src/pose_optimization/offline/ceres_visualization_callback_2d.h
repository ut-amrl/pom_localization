//
// Created by amanda on 1/30/21.
//

#ifndef AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_2D_H
#define AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_2D_H

#include <visualization/ros_visualization.h>
#include <ceres/iteration_callback.h>
#include <pose_optimization/pose_graph_generic.h>

namespace offline_optimization {

    class CeresVisualizationCallback2d : public ceres::IterationCallback {
    public:
        explicit CeresVisualizationCallback2d(
                const std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                        pose_graph::MovableObservationObjectPose<2, double, 3>>> &pose_graph,
                const std::shared_ptr<visualization::VisualizationManager> &vis_manager,
                const int32_t &num_poses,
                const std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> &observed_objs_by_class) :
                pose_graph_(pose_graph),
                vis_manager_(vis_manager), num_poses_(num_poses),
                observed_objs_by_class_(observed_objs_by_class) {

        }

        ~CeresVisualizationCallback2d() override = default;

        ceres::CallbackReturnType operator()(const ceres::IterationSummary &summary) override {

            std::unordered_map<pose_graph::NodeId, pose::Pose2d> node_poses;
            pose_graph_->getNodePoses(node_poses);
            std::vector<pose::Pose2d> node_poses_list;

            for (int32_t i = 0; i <= num_poses_; i++) {
                pose::Pose2d optimized_pose = node_poses[i];
                node_poses_list.emplace_back(optimized_pose);
            }
            vis_manager_->displayEstTrajectory(node_poses_list);

            for (const auto &objs_and_class : observed_objs_by_class_) {
                vis_manager_->displayObjObservationsFromEstTrajectory(node_poses_list, objs_and_class.second,
                                                                      objs_and_class.first);
            }

//         ros::Duration(0.1).sleep();

            return ceres::SOLVER_CONTINUE;
        }

    private:
        std::shared_ptr<pose_graph::PoseGraph<gp_kernel::Pose2dKernel, 2, double, 3, 2, double, 3,
                pose_graph::MovableObservationObjectPose<2, double, 3>>> pose_graph_;

        std::shared_ptr<visualization::VisualizationManager> vis_manager_;

        int32_t num_poses_;

        std::unordered_map<std::string, std::vector<std::vector<pose::Pose2d>>> observed_objs_by_class_;
    };
}

#endif //AUTODIFF_GP_CERES_VISUALIZATION_CALLBACK_2D_H
