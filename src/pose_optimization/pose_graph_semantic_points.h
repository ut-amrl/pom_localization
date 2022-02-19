//
// Created by amanda on 12/4/20.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_SEMANTIC_POINTS_H
#define AUTODIFF_GP_POSE_GRAPH_SEMANTIC_POINTS_H

#include <pose_optimization/pose_graph_generic_2d_distribution.h>
#include <pose_optimization/angle_local_parameterization.h>
#include <util/random.h>
#include <chrono>
#include <base_lib/pose_reps.h>

namespace pose_graph {

    typedef MapObjectObservation<2, double> MapObjectObservation2d;
    typedef MovableObservationSemanticPoints<3> MovableObservationSemanticPoints3d;
    typedef MovableObservationFactor<MovableObservationSemanticPoints3d> MovableObservationSemanticPointsFactor3d;

    typedef MovableObservationSemanticPoints<2> MovableObservationSemanticPoints2d;
    typedef MovableObservationFactor<MovableObservationSemanticPoints2d> MovableObservationSemanticPointsFactor2d;

    /**
     * Pose Graph containing all factors that constrain the robot's pose and the nodes representing the robot's pose.
     */
    template<int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim>
    class PoseGraphXdMovObjDistribution2dSemanticPoints
            : public PoseGraphXdMovObjDistribution2d<MeasurementTranslationDim, MeasurementRotationType, CovDim,
                    MovableObservationSemanticPoints<MeasurementTranslationDim>> {
    public:
        PoseGraphXdMovObjDistribution2dSemanticPoints(
                const std::function<ceres::LocalParameterization *()> &rotation_local_parameterization_creator,
                const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                const double &default_obj_probability_prior_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                const double &default_obj_probability_input_variance_for_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                const double &default_obj_probability_input_variance_for_var,
                const std::shared_ptr<gp_kernel::Pose2dKernel> &mean_kernel,
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(const double &)>
                        &var_kernel_creator) : PoseGraphXdMovObjDistribution2d<MeasurementTranslationDim,
                        MeasurementRotationType, CovDim, MovableObservationSemanticPoints<MeasurementTranslationDim>>(
                                rotation_local_parameterization_creator, obj_probability_prior_mean_by_class,
                                default_obj_probability_prior_mean, obj_probability_input_variance_by_class_for_mean,
                                default_obj_probability_input_variance_for_mean,
                                obj_probability_input_variance_by_class_for_var,
                                default_obj_probability_input_variance_for_var, mean_kernel, var_kernel_creator) {}

        ~PoseGraphXdMovObjDistribution2dSemanticPoints() override = default;
    };

    class PoseGraph2dMovObjDistribution2d : public PoseGraphXdMovObjDistribution2dSemanticPoints<2, double, 3> {
    public:
        PoseGraph2dMovObjDistribution2d(
                const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                const double &default_obj_probability_prior_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                const double &default_obj_probability_input_variance_for_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                const double &default_obj_probability_input_variance_for_var,
                const std::shared_ptr<gp_kernel::Pose2dKernel> &mean_kernel,
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(const double &)> &var_kernel_creator,
                const pose::Pose2d &object_detection_sensor_pose_rel_baselink,
                const std::function<std::vector<pose::Pose2d>(
                        const MovableObservationSemanticPointsFactor2d &, const pose::Pose2d)>
                        &sample_object_pose_generator): PoseGraphXdMovObjDistribution2dSemanticPoints<2, double, 3>(
                                pose_optimization::AngleLocalParameterization::create,
                                obj_probability_prior_mean_by_class,
                                default_obj_probability_prior_mean,
                                obj_probability_input_variance_by_class_for_mean,
                                default_obj_probability_input_variance_for_mean,
                                obj_probability_input_variance_by_class_for_var,
                                default_obj_probability_input_variance_for_var,
                                mean_kernel,
                                var_kernel_creator),
                                object_detection_sensor_pose_rel_baselink_(object_detection_sensor_pose_rel_baselink),
                                sample_object_pose_generator_(sample_object_pose_generator) {}

        ~PoseGraph2dMovObjDistribution2d() override = default;

        ceres::CostFunction *createMovableObjectCostFunctor(
                const std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>
                &movable_object_gpc,
                const MovableObservationSemanticPointsFactor2d &factor,
                const pose_optimization::CostFunctionParameters &cost_function_params) const override {
            std::vector<pose::Pose2d> samples = sample_object_pose_generator_(factor,
                                                                              object_detection_sensor_pose_rel_baselink_);

            if (samples.empty()) {
                return nullptr;
            }

            return new ceres::AutoDiffCostFunction<pose_optimization::SampleBasedMovableObservationCostFunctor2D<
                    gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>, 1, 2, 1>(
                            new pose_optimization::SampleBasedMovableObservationCostFunctor2D<
                                    gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                                            movable_object_gpc, samples,
                                            jet_cost_functor_timer_, double_cost_functor_timer_));
        }

        ceres::CostFunction *createGaussianBinaryCostFunctor(const GaussianBinaryFactorType &factor) const override {
            return new ceres::AutoDiffCostFunction<pose_optimization::Odometry2dCostFunctor, 3, 2, 1, 2, 1>(
                    new pose_optimization::Odometry2dCostFunctor(
                            factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));
        }

        std::pair<double *, double *> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, 2, 1>>,
                        std::shared_ptr<double>> node_pose_pointers) const override {
            return {node_pose_pointers.first->data(), node_pose_pointers.second.get()};
        }

        std::pair<double, Eigen::Matrix<double, 2, 1>> getSampleSearchCriteria(
                MovableObservationFactorType mov_obj_factor) const override {
            if (nodes_.find(mov_obj_factor.observed_at_node_) == nodes_.end()) {
                return std::make_pair(0, Eigen::Vector2d(0, 0));
            }

            NodeType node = nodes_.at(mov_obj_factor.observed_at_node_);
            pose::Pose2d node_pos = std::make_pair(*(node.est_position_), *(node.est_orientation_));
            Eigen::Vector2d mean_detected_point;
            std::vector<Eigen::Vector2d> observed_points = mov_obj_factor.observation_.object_points_;
            for (const Eigen::Vector2d &observed_point : observed_points) {
                mean_detected_point += observed_point;
            }
            mean_detected_point = mean_detected_point / observed_points.size();
            Eigen::Vector2d mean_detected_point_rel_map = pose::transformPoint(
                    pose::combinePoses(node_pos, object_detection_sensor_pose_rel_baselink_), mean_detected_point);

            return std::make_pair(10, mean_detected_point_rel_map);
//            return std::make_pair(mean_detected_point.norm(), mean_detected_point_rel_map);
        }

    private:

        /**
         * Pose of the sensor that detected the observed points relative to the baselink frame of the robot.
         */
        pose::Pose2d object_detection_sensor_pose_rel_baselink_;

        std::function<std::vector<pose::Pose2d>(const MovableObservationSemanticPointsFactor2d &,
                                                const pose::Pose2d)> sample_object_pose_generator_;
    };
}

#endif //AUTODIFF_GP_POSE_GRAPH_SEMANTIC_POINTS_H
