//
// Created by amanda on 12/4/20.
//

#ifndef AUTODIFF_GP_POSE_3D_FACTOR_GRAPH_H
#define AUTODIFF_GP_POSE_3D_FACTOR_GRAPH_H

#include <pose_optimization/pose_graph_generic.h>
#include <gaussian_process/kernel/pose_2d_kernel.h>
#include <pose_optimization/angle_local_parameterization.h>
#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_2d.h>
#include <pose_optimization/sample_based_movable_observation_gp_cost_functor_3d.h>
#include <pose_optimization/odometry_2d_cost_functor.h>
#include <pose_optimization/odometry_3d_cost_functor.h>
#include <util/random.h>
#include <chrono>
#include <base_lib/pose_reps.h>

namespace pose_graph {

    typedef Node<3, Eigen::Quaterniond> Node3d;
    typedef GaussianBinaryFactor<3, Eigen::Quaterniond, 6> GaussianBinaryFactor3d;
    typedef MapObjectObservation<2, double> MapObjectObservation2d;
    typedef MovableObservation<3, Eigen::Quaterniond, 6> MovableObservation3d;
    typedef MovableObservationFactor<3, Eigen::Quaterniond, 6> MovableObservationFactor3d;

    typedef Node<2, double> Node2d;
    typedef GaussianBinaryFactor<2, double, 3> GaussianBinaryFactor2d;
    typedef MovableObservation<2, double, 3> MovableObservation2d;
    typedef MovableObservationFactor<2, double, 3> MovableObservationFactor2d;

    /**
     * Pose Graph containing all factors that constrain the robot's pose and the nodes representing the robot's pose.
     */
    template<int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim>
    class PoseGraphXdMovObjDistribution2d
            : public PoseGraph<gp_kernel::Pose2dKernel, MeasurementTranslationDim, MeasurementRotationType, CovDim, 2, double, 3> {
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
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(const double &)> &var_kernel_creator) : PoseGraph<
                gp_kernel::Pose2dKernel, MeasurementTranslationDim, MeasurementRotationType,
                CovDim, 2, double, 3>(rotation_local_parameterization_creator,
                                      obj_probability_prior_mean_by_class,
                                      default_obj_probability_prior_mean,
                                      obj_probability_input_variance_by_class_for_mean,
                                      default_obj_probability_input_variance_for_mean,
                                      obj_probability_input_variance_by_class_for_var,
                                      default_obj_probability_input_variance_for_var,
                                      mean_kernel,
                                      var_kernel_creator) {}

        ~PoseGraphXdMovObjDistribution2d() override {}

        bool getMatrixRepresentationOfDetections(
                const std::vector<MapObjectObservation2d> &pos_observations,
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

        util_kdtree::KDNodeValue<double, 2> getKdRepForObs(
                const MapObjectObservation2d &pos_observation,
                const int &index) const override {
            return util_kdtree::KDNodeValue<double, 2>(
                    Eigen::Vector2d(pos_observation.transl_.x(), pos_observation.transl_.y()), Eigen::Vector2d(0, 0),
                    index);
        }
    };

    class PoseGraph3dMovObjDistribution2d : public PoseGraphXdMovObjDistribution2d<3, Eigen::Quaterniond, 6> {
    public:
        PoseGraph3dMovObjDistribution2d(
                const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                const double &default_obj_probability_prior_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                const double &default_obj_probability_input_variance_for_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                const double &default_obj_probability_input_variance_for_var,
                const std::shared_ptr<gp_kernel::Pose2dKernel> &mean_kernel,
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel> (const double &)> &var_kernel_creator) : PoseGraphXdMovObjDistribution2d<3, Eigen::Quaterniond, 6>(
                createQuaternionParameterization,
                obj_probability_prior_mean_by_class,
                default_obj_probability_prior_mean,
                obj_probability_input_variance_by_class_for_mean,
                default_obj_probability_input_variance_for_mean,
                obj_probability_input_variance_by_class_for_var,
                default_obj_probability_input_variance_for_var,
                mean_kernel,
                var_kernel_creator) {
        }

        static ceres::LocalParameterization *createQuaternionParameterization() {
            return new ceres::EigenQuaternionParameterization();
        }

        ceres::CostFunction *createMovableObjectCostFunctor(
                const std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> &movable_object_gpc,
                const MovableObservationFactor<3, Eigen::Quaterniond, 6> &factor,
                const pose_optimization::CostFunctionParameters &cost_function_params) const override {
            std::vector<std::pair<Eigen::Vector3d, Eigen::Quaterniond>> observation_samples;
//            util_random::Random random_generator;
//            Eigen::Matrix<double, 6, 6> obs_cov = factor.observation_.observation_covariance_;
            for (int i = 0; i < cost_function_params.num_samples_per_movable_obj_observation_; i++) {
                // TODO need to sample here instead of just adding multiple MLEs
                // Not exactly sure how to sample the orientation component though or how to sample without just
                // sampling each component independently
                std::pair<Eigen::Vector3d, Eigen::Quaterniond> sample = {
                        factor.observation_.observation_transl_,
                        factor.observation_.observation_orientation_};
                observation_samples.emplace_back(sample);
            }

            return new ceres::AutoDiffCostFunction<pose_optimization::SampleBasedMovableObservationCostFunctor3D<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>, 1, 3, 4>(
                    new pose_optimization::SampleBasedMovableObservationCostFunctor3D<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                            movable_object_gpc,
                            observation_samples));
        }

        ceres::CostFunction *createGaussianBinaryCostFunctor(
                const GaussianBinaryFactor<3, Eigen::Quaterniond, 6> &factor) const override {
            return new ceres::AutoDiffCostFunction<pose_optimization::Odometry3dCostFunctor, 6, 3, 4, 3, 4>(
                    new pose_optimization::Odometry3dCostFunctor(
                            factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));
        };

        std::pair<double *, double *> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, 3, 1>>,
                        std::shared_ptr<Eigen::Quaterniond>> node_pose_pointers) const override {
            return {node_pose_pointers.first->data(), node_pose_pointers.second->coeffs().data()};
        };


        std::pair<double, Eigen::Matrix<double, 2, 1>> getSampleSearchCriteria(MovableObservationFactorType mov_obj_factor) const override {
            if (nodes_.find(mov_obj_factor.observed_at_node_) == nodes_.end()) {
                return std::make_pair(0, Eigen::Vector2d(0, 0));
            }
            NodeType node = nodes_.at(mov_obj_factor.observed_at_node_);
            pose::Pose3d node_pos = std::make_pair(*(node.est_position_), *(node.est_orientation_));
            pose::Pose3d obs_pos = std::make_pair(mov_obj_factor.observation_.observation_transl_,
                                                  mov_obj_factor.observation_.observation_orientation_);
            pose::Pose3d obs_pos_map_frame = pose::combinePoses(node_pos, obs_pos);
            return std::make_pair(mov_obj_factor.observation_.observation_transl_.norm() * 0.75,
                                  Eigen::Vector2d(obs_pos_map_frame.first.x(), obs_pos_map_frame.first.y()));
        }
    };

    class PoseGraph2dMovObjDistribution2d : public PoseGraphXdMovObjDistribution2d<2, double, 3> {
    public:
        PoseGraph2dMovObjDistribution2d(
                const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                const double &default_obj_probability_prior_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                const double &default_obj_probability_input_variance_for_mean,
                const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                const double &default_obj_probability_input_variance_for_var,
                const std::shared_ptr<gp_kernel::Pose2dKernel> &mean_kernel,
                const std::function<std::shared_ptr<gp_kernel::Pose2dKernel>(const double &)> &var_kernel_creator) : PoseGraphXdMovObjDistribution2d<2, double, 3>(
                pose_optimization::AngleLocalParameterization::create,
                obj_probability_prior_mean_by_class,
                default_obj_probability_prior_mean,
                obj_probability_input_variance_by_class_for_mean,
                default_obj_probability_input_variance_for_mean,
                obj_probability_input_variance_by_class_for_var,
                default_obj_probability_input_variance_for_var,
                mean_kernel,
                var_kernel_creator) {
        }

        ceres::CostFunction *createMovableObjectCostFunctor(
                const std::shared_ptr<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>> &movable_object_gpc,
                const MovableObservationFactor<2, double, 3> &factor,
                const pose_optimization::CostFunctionParameters &cost_function_params) const override {
            std::vector<std::pair<Eigen::Vector2d, double>> observation_samples;
            util_random::Random rand_gen(std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
            Eigen::Matrix<double, 3, 3> obs_cov = factor.observation_.observation_covariance_;
            for (int i = 0; i < cost_function_params.num_samples_per_movable_obj_observation_; i++) {
                // TODO can this instead sample from the whole covariance matrix instead of sampling each dimension independently.
                double x_std_dev = sqrt(obs_cov(0, 0));
                double y_std_dev = sqrt(obs_cov(1, 1));
                double yaw_std_dev = sqrt(obs_cov(2, 2));
                std::pair<Eigen::Vector2d, double> sample = {
                        Eigen::Vector2d(rand_gen.Gaussian(factor.observation_.observation_transl_.x(), x_std_dev),
                                        rand_gen.Gaussian(factor.observation_.observation_transl_.y(), y_std_dev)),
                        rand_gen.Gaussian(factor.observation_.observation_orientation_, yaw_std_dev)};
                observation_samples.emplace_back(sample);
            }

            return new ceres::AutoDiffCostFunction<pose_optimization::SampleBasedMovableObservationCostFunctor2D<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>, 1, 2, 1>(
                    new pose_optimization::SampleBasedMovableObservationCostFunctor2D<gp_regression::GaussianProcessClassifier<3, gp_kernel::Pose2dKernel>>(
                            movable_object_gpc,
                            observation_samples));
        }

        ceres::CostFunction *createGaussianBinaryCostFunctor(
                const GaussianBinaryFactorType &factor) const override {
            return new ceres::AutoDiffCostFunction<pose_optimization::Odometry2dCostFunctor, 3, 2, 1, 2, 1>(
                    new pose_optimization::Odometry2dCostFunctor(
                            factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));
        };

        std::pair<double *, double *> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, 2, 1>>,
                        std::shared_ptr<double>> node_pose_pointers) const override {
            return {node_pose_pointers.first->data(), node_pose_pointers.second.get()};
        }

        std::pair<double, Eigen::Matrix<double, 2, 1>> getSampleSearchCriteria(MovableObservationFactorType mov_obj_factor) const override {
            if (nodes_.find(mov_obj_factor.observed_at_node_) == nodes_.end()) {
                return std::make_pair(0, Eigen::Vector2d(0, 0));
            }
            NodeType node = nodes_.at(mov_obj_factor.observed_at_node_);
            pose::Pose2d node_pos = std::make_pair(*(node.est_position_), *(node.est_orientation_));
            pose::Pose2d obs_pos = std::make_pair(mov_obj_factor.observation_.observation_transl_,
                                                  mov_obj_factor.observation_.observation_orientation_);
            pose::Pose2d obs_pos_map_frame = pose::combinePoses(node_pos, obs_pos);
            return std::make_pair(mov_obj_factor.observation_.observation_transl_.norm(), obs_pos_map_frame.first);
        }
    };
}

#endif //AUTODIFF_GP_POSE_3D_FACTOR_GRAPH_H
