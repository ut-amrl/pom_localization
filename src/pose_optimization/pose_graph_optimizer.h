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

        void buildPoseGraphOptimizationProblem(pose_graph::PoseGraph &pose_graph, ceres::Problem *problem) {

            ceres::LocalParameterization* quaternion_local_parameterization =
                    new ceres::EigenQuaternionParameterization;

            // Add residuals from movable object observations
            for (pose_graph::MovableObservationFactor &factor : pose_graph.getMovableObservationFactors()) {
                std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> pose_vars_;
                if (!pose_graph.getNodePosePointers(factor.observed_at_node_, pose_vars_)) {
                    LOG(ERROR) << "Node " << factor.observed_at_node_ << " did not exist in the pose graph. Skipping movable object observation";
                    continue;
                }

                std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> movable_object_regressor = pose_graph.getMovableObjGpRegressor(factor.observation_.semantic_class_);
                if (movable_object_regressor) {

                    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<pose_optimization::MovableObservationCostFunctor, 1, 3, 4>(
                            new pose_optimization::MovableObservationCostFunctor(
                                    pose_graph.getMovableObjGpRegressor(factor.observation_.semantic_class_),
                                    factor.observation_.observation_transl_,
                                    factor.observation_.observation_orientation_));

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


        }

        bool SolveOptimizationProblem(ceres::Problem* problem) {
            CHECK(problem != NULL);
            ceres::Solver::Options options;
            options.max_num_iterations = 200;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            ceres::Solver::Summary summary;
            ceres::Solve(options, problem, &summary);
            std::cout << summary.FullReport() << '\n';
            return summary.IsSolutionUsable();
        }
    };

}


#endif //AUTODIFF_GP_POSE_GRAPH_OPTIMIZER_H
