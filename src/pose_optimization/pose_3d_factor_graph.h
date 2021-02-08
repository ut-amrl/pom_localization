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

namespace pose_graph {

    typedef Node<3, Eigen::Quaterniond> Node3d;
    typedef GaussianBinaryFactor<3, Eigen::Quaterniond, 6> GaussianBinaryFactor3d;
    typedef MapObjectObservation<2, double> MapObjectObservation2d;
    typedef NegativeMapObjectObservation<2> NegativeMapObjectObservation2d;
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
    class PoseGraphXdMovObjDistribution2d : public PoseGraph<gp_kernel::Pose2dKernel, MeasurementTranslationDim, MeasurementRotationType, CovDim, 2, double, 3> {
    public:

        PoseGraphXdMovObjDistribution2d(const std::function<ceres::LocalParameterization*()> &rotation_local_parameterization_creator,
                             gp_kernel::Pose2dKernel &kernel) : PoseGraph<
                                     gp_kernel::Pose2dKernel, MeasurementTranslationDim, MeasurementRotationType,
                                     CovDim, 2, double, 3>(rotation_local_parameterization_creator, kernel) {}

        ~PoseGraphXdMovObjDistribution2d() override {}

//        /**
//         * Create the pose graph.
//         *
//         * @param kernel Kernel for comparing 2d movable object poses. // TODO, this doesn't seem like it necessarily
//         * belongs to this class intuitively, any way to restructure?
//         */
//        PoseGraph3d(const gp_kernel::Pose2dKernel &kernel) : pose_2d_kernel_(kernel) {
//
//        }
//
//        /**
//         * Add a node.
//         *
//         * @param node Node in the trajectory.
//         */
//        void addNode(const Node3d &node) {
//            nodes_[node.id_] = node;
//        }
//
//        /**
//         * Add factors from movable object detections.
//         *
//         * @param node_id               Node at which the object detections occurred.
//         * @param observations_at_node  Observations that were made at the node.
//         */
//        void addMovableObservationFactors(const NodeId &node_id, const std::vector<MovableObservation3d> &observations_at_node) {
//            for (const MovableObservation3d &observation : observations_at_node) {
//                observation_factors_.emplace_back(node_id, observation);
//            }
//        }
//
//        /**
//         * Add a factor based on the estimated location of one node relative to another, assuming Gaussian noise.
//         *
//         * @param binary_factor Binary factor to add
//         */
//        void addGaussianBinaryFactor(const GaussianBinaryFactor3d &binary_factor) {
//            binary_factors_.push_back(binary_factor);
//        }
//
//        /**
//         * Get the movable observation factors.
//         *
//         * @return movable observation factors.
//         */
//        std::vector<MovableObservationFactor3d> getMovableObservationFactors() {
//            return observation_factors_;
//        }
//
//        /**
//         * Get the binary factors with gaussian noise.
//         *
//         * @return binary factors with gaussian noise.
//         */
//        std::vector<GaussianBinaryFactor3d> getBinaryFactors() {
//            return binary_factors_;
//        }
//
//        /**
//         * Add 2d positive and negative observations to the GP regressors per class, or make new ones if this class has
//         * not yet been observed.
//         *
//         * @param observations_by_class Observations by their class.
//         */
////        void addMapFrameObservations(const std::unordered_map<std::string, std::pair<std::vector<NegativeMovableObservation2D>, std::vector<MapObservation2D>>> &observations_by_class) {
////            for (const auto &obs_by_class : observations_by_class) {
////                Eigen::MatrixXf inputs;
////                Eigen::MatrixXf outputs;
////                if (getMatrixRepresentationOfDetections(obs_by_class.second, inputs, outputs)) {
////                    auto gp_iter = movable_object_2d_gp_regressors_by_class_.find(obs_by_class.first);
////                    std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_regressor;
////                    if (gp_iter != movable_object_2d_gp_regressors_by_class_.end()) {
////                        gp_regressor = gp_iter->second;
////                        gp_regressor->appendData(inputs, outputs);
////                    } else {
////                        gp_regressor = std::make_shared<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>>(inputs, outputs, &pose_2d_kernel_);
////                    }
////                    movable_object_2d_gp_regressors_by_class_[obs_by_class.first] = gp_regressor;
////                }
////            }
////        }
//
//        void addMapFrameObservations(const std::unordered_map<std::string, std::pair<std::vector<NegativeMapObjectObservation2d>, std::vector<MapObjectObservation2d>>> &observations_by_class) {
//            for (const auto &obs_by_class : observations_by_class) {
//                Eigen::MatrixXf inputs;
//                if (getMatrixRepresentationOfDetections(obs_by_class.second.second, inputs)) {
//                    auto kde_iter = movable_object_2d_kdes_by_class_.find(obs_by_class.first);
//                    std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> kde;
//                    if (kde_iter != movable_object_2d_kdes_by_class_.end()) {
//                        kde = kde_iter->second;
//                        kde->appendData(inputs);
//                    } else {
//                        kde = std::make_shared<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>>(inputs, &pose_2d_kernel_);
//                    }
//                    movable_object_2d_kdes_by_class_[obs_by_class.first] = kde;
//                }
//            }
//        }
////
////        /**
////         * Get the movable object gp regressor for the given semantic class.
////         *
////         * @param class_label Class label to get the regressor for.
////         *
////         * @return Regressor (or null pointer if it doesn't exist).
////         */
////        std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> getMovableObjGpRegressor(const std::string &class_label) {
////            auto regressor_iter = movable_object_2d_gp_regressors_by_class_.find(class_label);
////            if (regressor_iter != movable_object_2d_gp_regressors_by_class_.end()) {
////                return regressor_iter->second;
////            }
////            return nullptr;
////        }
//
//        std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> getMovableObjKde(const std::string &class_label) {
//            auto regressor_iter = movable_object_2d_kdes_by_class_.find(class_label);
//            if (regressor_iter != movable_object_2d_kdes_by_class_.end()) {
//                return regressor_iter->second;
//            }
//            return nullptr;
//        }
//
//        /**
//         * Get the pointers to the position components of the given node.
//         *
//         * @param node_id[in]           Node id to get pointers to the position variables for.
//         * @param pointer_results[out]  Pointers to the position and orientation variables for the node with the given
//         *                              id.
//         * @return True if the pointers were populated, false if not.
//         */
//        bool getNodePosePointers(const NodeId &node_id, std::pair<std::shared_ptr<Eigen::Vector3d>, std::shared_ptr<Eigen::Quaterniond>> &pointer_results) {
//            if (nodes_.find(node_id) != nodes_.end()) {
//                Node3d node = nodes_.at(node_id);
//                pointer_results = std::make_pair(node.est_position_, node.est_orientation_);
//                return true;
//            }
//            return false;
//        }
//
//        void getNodePoses(std::unordered_map<NodeId, std::pair<Eigen::Vector3d, Eigen::Quaterniond>> &node_positions) {
//            for (const auto &node : nodes_) {
//                node_positions[node.first] = std::make_pair(Eigen::Vector3d(*(node.second.est_position_)),
//                                                            Eigen::Quaterniond(*(node.second.est_orientation_)));
////                LOG(INFO) << node.first;
////                LOG(INFO) << node_positions[node.first].first;
////                LOG(INFO) << node.second.est_position_->x() << ", " << node.second.est_position_->y() << ", " << node.second.est_position_->z();
//            }
//        }
//
//    private:
//
//        // TODO there has to be a better way than just inserting samples at a variety of orientations
//        const uint8_t kNumDiscreteOrientationsNegObservations = 5;
//
//        /**
//         * Kernel for comparing 2d movable object pose similarity.
//         */
//        gp_kernel::Pose2dKernel pose_2d_kernel_;
//
//        /**
//         * Map of node id to the nodes.
//         */
//        std::unordered_map<NodeId, Node3d> nodes_;
//
//        /**
//         * Movable observation factors.
//         *
//         * TODO should we store this in some more intelligent data structure? (Unordered map by viewing node id?).
//         */
//        std::vector<MovableObservationFactor3d> observation_factors_;
//
//        /**
//         * Factors relating two nodes that assume gaussian noise.
//         */
//        std::vector<GaussianBinaryFactor3d> binary_factors_;
//
//        // TODO convert class labels to enum?
////        /**
////         * Movable object GP regressors, by their semantic class.
////         */
////        std::unordered_map<std::string, std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>>> movable_object_2d_gp_regressors_by_class_;
//
//        /**
//         * Movable object KDEs, by their semantic class.
//         */
//        std::unordered_map<std::string, std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>>> movable_object_2d_kdes_by_class_;
//
//        /**
//         * Convert the negative and positive observations to a matrix of inputs and outputs representing the observations.
//         *
//         * @param pos_and_neg_observations[in]  Positive and negative observations.
//         * @param input_matrix[out]             Matrix to fill with input data (positions). Each observation will
//         *                                      generate a column (or more for negative since we create them for a few
//         *                                      angles).
//         * @param output_matrix[out]            Matrix to fill with output data (1s for positive detections, 0s for
//         *                                      negative detections). Each observation will generate a column (or more
//         *                                      for negative since we create them for a few angles).
//         * @return True if the matrices were populated, false if not (if the observations were empty).
//         */
//        bool getMatrixRepresentationOfDetections(
//                const std::pair<std::vector<NegativeMapObjectObservation2d>,
//                        std::vector<MapObjectObservation2d>> &pos_and_neg_observations,
//                        Eigen::MatrixXf &input_matrix, Eigen::MatrixXf &output_matrix) const {
//            std::vector<MapObjectObservation2d> observations = pos_and_neg_observations.second;
//            std::vector<NegativeMapObjectObservation2d> neg_obs = pos_and_neg_observations.first;
//
//            size_t neg_obs_count = neg_obs.size();
//            size_t pos_obs_count = observations.size();
//            size_t total_entries_count = (pos_obs_count + (kNumDiscreteOrientationsNegObservations * neg_obs_count));
//
//            if (total_entries_count == 0) {
//                return false;
//            }
//
//            // Inputs should have 3 rows and as many columns as examples
//            input_matrix = Eigen::MatrixXf(3, total_entries_count);
//            output_matrix = Eigen::MatrixXf::Zero(1, total_entries_count);
//            for (size_t i = 0; i < pos_obs_count; i++) {
//                input_matrix(0, i) = observations[i].transl_.x();
//                input_matrix(1, i) = observations[i].transl_.y();
//                input_matrix(2, i) = observations[i].orientation_;
//            }
//            output_matrix.leftCols(pos_obs_count) = Eigen::MatrixXf::Ones(1, pos_obs_count);
//
//            for (size_t i = 0; i < neg_obs_count; i++) {
//                // TODO figure out a better way to handle orientation in negative observations
//
//                float angle_inc = (M_PI * 2) / kNumDiscreteOrientationsNegObservations;
//                // Base the initial angle offset from the 0 on the number of the negative observation so that it is
//                // deterministic, but we don't end up with all negative observations at the same angle
//                float angle_start = ((float) i) * angle_inc / neg_obs_count;
//                for (uint8_t angle_index = 0; angle_index < kNumDiscreteOrientationsNegObservations; angle_index++) {
//                    float angle = angle_start + (angle_index * angle_inc);
//                    size_t index = pos_obs_count + (i * kNumDiscreteOrientationsNegObservations) + angle_index;
//                    input_matrix(0, index) = neg_obs[i].transl_.x();
//                    input_matrix(1, index) = neg_obs[i].transl_.y();
//                    input_matrix(2, index) = angle;
//                }
//            }
//            return true;
//        }
//
        bool getMatrixRepresentationOfDetections(
                const std::vector<MapObjectObservation2d> &pos_observations,
                Eigen::MatrixXf &input_matrix) const override {

            size_t pos_obs_count = pos_observations.size();

            if (pos_obs_count == 0) {
                return false;
            }

            // Inputs should have 3 rows and as many columns as examples
            input_matrix = Eigen::MatrixXf(3, pos_obs_count);
            for (size_t i = 0; i < pos_obs_count; i++) {
                input_matrix(0, i) = pos_observations[i].transl_.x();
                input_matrix(1, i) = pos_observations[i].transl_.y();
                input_matrix(2, i) = pos_observations[i].orientation_;
            }
            return true;
        }
    };

    class PoseGraph3dMovObjDistribution2d : public PoseGraphXdMovObjDistribution2d<3, Eigen::Quaterniond, 6> {
    public:
        PoseGraph3dMovObjDistribution2d(gp_kernel::Pose2dKernel &kernel) : PoseGraphXdMovObjDistribution2d<3, Eigen::Quaterniond, 6>(
                createQuaternionParameterization, kernel) {
        }

        static ceres::LocalParameterization* createQuaternionParameterization() {
            return new ceres::EigenQuaternionParameterization();
        }

        ceres::CostFunction *createMovableObjectCostFunctor(const std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> &movable_object_kde,
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

            return new ceres::AutoDiffCostFunction<pose_optimization::SampleBasedMovableObservationCostFunctor3D, 1, 3, 4>(
                    new pose_optimization::SampleBasedMovableObservationCostFunctor3D(
                            movable_object_kde,
                            observation_samples));
        }

        ceres::CostFunction *createGaussianBinaryCostFunctor(
                const GaussianBinaryFactor<3, Eigen::Quaterniond, 6> &factor) const override {
            return new ceres::AutoDiffCostFunction<pose_optimization::Odometry3dCostFunctor, 6, 3, 4, 3, 4>(
                    new pose_optimization::Odometry3dCostFunctor(
                            factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));
        };

        std::pair<double*, double*> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, 3, 1>>,
                        std::shared_ptr<Eigen::Quaterniond>> node_pose_pointers) const override {
            return {node_pose_pointers.first->data(), node_pose_pointers.second->coeffs().data()};
        };
    };

    class PoseGraph2dMovObjDistribution2d : public PoseGraphXdMovObjDistribution2d<2, double, 3> {
    public:
        PoseGraph2dMovObjDistribution2d(gp_kernel::Pose2dKernel &kernel) : PoseGraphXdMovObjDistribution2d<2, double, 3>(
                pose_optimization::AngleLocalParameterization::create, kernel) {
        }

        ceres::CostFunction *createMovableObjectCostFunctor(const std::shared_ptr<gp_regression::KernelDensityEstimator<3, gp_kernel::Pose2dKernel>> &movable_object_kde,
                                                            const MovableObservationFactor<2, double, 3> &factor,
                                                            const pose_optimization::CostFunctionParameters &cost_function_params) const override {
            std::vector<std::pair<Eigen::Vector2d, double>> observation_samples;
            util_random::Random rand_gen(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
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

            return new ceres::AutoDiffCostFunction<pose_optimization::SampleBasedMovableObservationCostFunctor2D, 1, 2, 1>(
                    new pose_optimization::SampleBasedMovableObservationCostFunctor2D(
                            movable_object_kde,
                            observation_samples));
        }

        ceres::CostFunction *createGaussianBinaryCostFunctor(
                const GaussianBinaryFactorType &factor) const override {
            return new ceres::AutoDiffCostFunction<pose_optimization::Odometry2dCostFunctor, 3, 2, 1, 2, 1>(
                    new pose_optimization::Odometry2dCostFunctor(
                            factor.translation_change_, factor.orientation_change_, factor.sqrt_information_));
        };

        std::pair<double*, double*> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, 2, 1>>,
                        std::shared_ptr<double>> node_pose_pointers) const override {
            return {node_pose_pointers.first->data(), node_pose_pointers.second.get()};
        }
    };
}

#endif //AUTODIFF_GP_POSE_3D_FACTOR_GRAPH_H
