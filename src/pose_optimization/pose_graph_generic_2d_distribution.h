//
// Created by amanda on 12/4/20.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_GENERIC_2D_DISTRIBUTION_H
#define AUTODIFF_GP_POSE_GRAPH_GENERIC_2D_DISTRIBUTION_H

#include <base_lib/pose_reps.h>
#include <chrono>
#include <gaussian_process/kernel/pose_2d_kernel.h>
#include <pose_optimization/pose_graph_generic.h>
#include <util/random.h>

namespace pose_graph {

    typedef MapObjectObservation<2, double> MapObjectObservation2d;

    typedef Node<2, double> Node2d;
    typedef GaussianBinaryFactor<2, double, 3> GaussianBinaryFactor2d;

    typedef Node<3, Eigen::Quaterniond> Node3d;
    typedef GaussianBinaryFactor<3, Eigen::Quaterniond, 6> GaussianBinaryFactor3d;

    /**
     * Pose Graph containing all factors that constrain the robot's pose and the nodes representing the robot's pose.
     */
    template<int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim, typename MovableObservationType>
    class PoseGraphXdMovObjDistribution2d
            : public PoseGraph<gp_kernel::Pose2dKernel, MeasurementTranslationDim, MeasurementRotationType, CovDim, 2, double, 3,
                    MovableObservationType> {
    public:
        PoseGraphXdMovObjDistribution2d(
                const std::function<ceres::LocalParameterization *()> &rotation_local_parameterization_creator,
                const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                const double &default_obj_probability_prior_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                const double &default_obj_probability_input_variance_for_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                const double &default_obj_probability_input_variance_for_var,
                const std::shared_ptr<gp_kernel::Pose2dKernel> &mean_kernel,
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(const double &)>
                &var_kernel_creator) : PoseGraph<gp_kernel::Pose2dKernel, MeasurementTranslationDim,
                MeasurementRotationType, CovDim, 2, double, 3, MovableObservationType>(
                rotation_local_parameterization_creator, obj_probability_prior_mean_by_class,
                default_obj_probability_prior_mean, obj_probability_input_variance_by_class_for_mean,
                default_obj_probability_input_variance_for_mean, obj_probability_input_variance_by_class_for_var,
                default_obj_probability_input_variance_for_var, mean_kernel, var_kernel_creator) {}

        ~PoseGraphXdMovObjDistribution2d() override = default;

        bool getMatrixRepresentationOfDetections(const std::vector<MapObjectObservation2d> &pos_observations,
                                                 Eigen::MatrixXd &input_matrix) const override {
            size_t pos_obs_count = pos_observations.size();
            if (pos_obs_count == 0) {
                return false;
            }

            // Inputs should have 3 rows and as many columns as examples
            input_matrix = Eigen::MatrixXd(3, pos_obs_count);
            for (size_t i = 0; i < pos_obs_count; i++) {
                input_matrix(0, i) = pos_observations[i].transl_.x();
                input_matrix(1, i) = pos_observations[i].transl_.y();
                input_matrix(2, i) = pos_observations[i].orientation_;
            }
            return true;
        }

        util_kdtree::KDNodeValue<double, 2> getKdRepForObs(const MapObjectObservation2d &pos_observation,
                                                           const int &index) const override {
            return util_kdtree::KDNodeValue<double, 2>(
                    Eigen::Vector2d(pos_observation.transl_.x(), pos_observation.transl_.y()),
                    Eigen::Vector2d(0, 0), index);
        }
    };

}
#endif //AUTODIFF_GP_POSE_GRAPH_GENERIC_2D_DISTRIBUTION_H
