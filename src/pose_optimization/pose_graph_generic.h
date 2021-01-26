//
// Created by amanda on 1/22/21.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_GENERIC_H
#define AUTODIFF_GP_POSE_GRAPH_GENERIC_H

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <eigen3/Eigen/Dense>

//#include <gaussian_process/gp_regression.h>
#include <gaussian_process/kernel_density_estimator.h>
#include <pose_optimization/pose_optimization_parameters.h>

namespace pose_graph {
    typedef uint64_t NodeId;

    /**
     * Node in the trajectory of the robot.
     */
    template<int TranslationDim, typename RotationType>
    struct Node {

        /**
         * Id of the node.
         */
        NodeId id_;

        /**
         * Estimated position of the node.
         *
         * TODO why did I make this a shared ptr again? So it could be updated by Ceres?
         */
        std::shared_ptr<Eigen::Matrix<double, TranslationDim, 1>> est_position_;

        /**
         * Estimated position of the node.
         *
         * TODO why did I make this a shared ptr again? So it could be updated by Ceres?
         */
        std::shared_ptr<RotationType> est_orientation_;
    };

    template<int TranslationDim, typename RotationType, int SqrtInformationDim>
    struct GaussianBinaryFactor {

        /**
         * Node that serves as the coordinate frame for the measurement.
         */
        NodeId from_node_;

        /**
         * Node that was estimated to be at the given position in the frame of the from_node.
         */
        NodeId to_node_;

        /**
         * Measured location of the to_node_ in the from_node_ frame.
         */
        Eigen::Matrix<double, TranslationDim, 1> translation_change_;

        /**
         * Provides measured orientation of the to_node in the from node's frame.
         */
        RotationType orientation_change_;

        /**
         * Square root information matrix providing uncertainty of the measurement.
         */
        Eigen::Matrix<double, SqrtInformationDim, SqrtInformationDim> sqrt_information_;
    };

    /**
     * Observation of an object in the map.
     */
    template<int TranslationDim, typename RotationType>
    struct MapObjectObservation {

        /**
         * Semantic class of the object.
         */
        std::string semantic_class_;

        /**
         * Location of the object in the map.
         */
        Eigen::Matrix<double, TranslationDim, 1> transl_;

        /**
         * Relative orientation of the object in the map.
         */
        RotationType orientation_;
    };

    /**
     * Negative movable object observation.
     *
     * No object of the given class was detected at the location.
     *
     * TODO Do we need this? Should this have an orientation?
     */
    template<int TranslationDim>
    struct NegativeMapObjectObservation {

        /**
         * Semantic class that was not present at the given location.
         */
        std::string semantic_class_;

        /**
         * Location in the map that did not have an object of the given type.
         */
        Eigen::Matrix<double, TranslationDim, 1> transl_;
    };

    /**
     * 3D Observation of a movable object.
     */
    template<int TranslationDim, typename RotationType, int CovDim>
    struct MovableObservation {

        /**
         * Semantic class of the object.
         */
        std::string semantic_class_;

        /**
         * Relative location of the object.
         */
        Eigen::Matrix<double, TranslationDim, 1> observation_transl_;

        /**
         * Relative orientation of the object.
         */
        RotationType observation_orientation_;

        // TODO
        Eigen::Matrix<double, CovDim, CovDim> observation_covariance_;
    };



    /**
     * Factor that encodes a sighting of a movable object.
     */
    template<int TranslationDim, typename RotationType, int CovDim>
    struct MovableObservationFactor {

        /**
         * Create the movable observation factor.
         *
         * @param node_id       Node that the object was observed at.
         * @param observation   Observation of the object, relative to the robot.
         */
        MovableObservationFactor(const NodeId &node_id,
                                 const MovableObservation<TranslationDim, RotationType, CovDim> &observation) :
                observed_at_node_(node_id), observation_(observation) {}

        // TODO should this contain one observation or all at the node?

        /**
         * Node that the object was observed at.
         */
        NodeId observed_at_node_;

        /**
         * Observation of the object, relative to the robot.
         */
        MovableObservation<TranslationDim, RotationType, CovDim> observation_;
    };

    /**
     * Pose Graph containing all factors that constrain the robot's pose and the nodes representing the robot's pose.
     *
     * This class does not have any specifics relating to dimension (2d vs 3d). Subclasses should supply logic specific
     * to a dimension.
     */
    template<typename MovObjKernelType, int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
            int MovableHeatMapTranslationDim, typename MovableHeatMapRotationType, int KernelDim>
    class PoseGraph {
    public:

        typedef Node<MeasurementTranslationDim, MeasurementRotationType> NodeType;
        typedef MovableObservation<MeasurementTranslationDim, MeasurementRotationType, CovDim> MovableObservationType;
        typedef MovableObservationFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> MovableObservationFactorType;
        typedef GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> GaussianBinaryFactorType;
        typedef MapObjectObservation<MovableHeatMapTranslationDim, MovableHeatMapRotationType> MapObjectObservationType;
        typedef NegativeMapObjectObservation<MovableHeatMapTranslationDim> NegativeMapObjectObservationType;
        typedef gp_regression::KernelDensityEstimator<KernelDim, MovObjKernelType> KdeType;

        /**
         * Create the pose graph.
         *
         * @param kernel Kernel for comparing 2d movable object poses. // TODO, this doesn't seem like it necessarily
         * belongs to this class intuitively, any way to restructure?
         */
        PoseGraph(const std::function<ceres::LocalParameterization*()> &rotation_local_parameterization_creator,
                  MovObjKernelType &kernel) : rotation_local_parameterization_creator_(rotation_local_parameterization_creator),
                  mov_obj_kernel_(kernel) {

        }

        virtual ~PoseGraph() {}

        /**
         * Add a node.
         *
         * @param node Node in the trajectory.
         */
        void addNode(const NodeType &node) {
            nodes_[node.id_] = node;
        }

        /**
         * Add factors from movable object detections.
         *
         * @param node_id               Node at which the object detections occurred.
         * @param observations_at_node  Observations that were made at the node.
         */
        void addMovableObservationFactors(const NodeId &node_id, const std::vector<MovableObservationType> &observations_at_node) {
            for (const MovableObservationType &observation : observations_at_node) {
                observation_factors_.emplace_back(node_id, observation);
            }
        }

        /**
         * Add a factor based on the estimated location of one node relative to another, assuming Gaussian noise.
         *
         * @param binary_factor Binary factor to add
         */
        void addGaussianBinaryFactor(const GaussianBinaryFactorType &binary_factor) {
            binary_factors_.push_back(binary_factor);
        }

        /**
         * Get the movable observation factors.
         *
         * @return movable observation factors.
         */
        std::vector<MovableObservationFactorType> getMovableObservationFactors() {
            return observation_factors_;
        }

        /**
         * Get the binary factors with gaussian noise.
         *
         * @return binary factors with gaussian noise.
         */
        std::vector<GaussianBinaryFactorType> getBinaryFactors() {
            return binary_factors_;
        }

        /**
         * Add 2d positive and negative observations to the GP regressors per class, or make new ones if this class has
         * not yet been observed.
         *
         * @param observations_by_class Observations by their class.
         */
//        void addMapFrameObservations(const std::unordered_map<std::string, std::pair<std::vector<NegativeMovableObservation2D>, std::vector<MapObservation2D>>> &observations_by_class) {
//            for (const auto &obs_by_class : observations_by_class) {
//                Eigen::MatrixXf inputs;
//                Eigen::MatrixXf outputs;
//                if (getMatrixRepresentationOfDetections(obs_by_class.second, inputs, outputs)) {
//                    auto gp_iter = movable_object_2d_gp_regressors_by_class_.find(obs_by_class.first);
//                    std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> gp_regressor;
//                    if (gp_iter != movable_object_2d_gp_regressors_by_class_.end()) {
//                        gp_regressor = gp_iter->second;
//                        gp_regressor->appendData(inputs, outputs);
//                    } else {
//                        gp_regressor = std::make_shared<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>>(inputs, outputs, &pose_2d_kernel_);
//                    }
//                    movable_object_2d_gp_regressors_by_class_[obs_by_class.first] = gp_regressor;
//                }
//            }
//        }

        void addMapFrameObservations(
                const std::unordered_map<std::string, std::pair<std::vector<NegativeMapObjectObservationType>,
                        std::vector<MapObjectObservationType>>> &observations_by_class) {
            for (const auto &obs_by_class : observations_by_class) {
                Eigen::MatrixXf inputs;
                if (getMatrixRepresentationOfDetections(obs_by_class.second.second, inputs)) {
                    auto kde_iter = movable_object_2d_kdes_by_class_.find(obs_by_class.first);
                    std::shared_ptr<KdeType> kde;
                    if (kde_iter != movable_object_2d_kdes_by_class_.end()) {
                        kde = kde_iter->second;
                        kde->appendData(inputs);
                    } else {
                        kde = std::make_shared<KdeType>(inputs, &mov_obj_kernel_);
                    }
                    movable_object_2d_kdes_by_class_[obs_by_class.first] = kde;
                }
            }
        }
//
//        /**
//         * Get the movable object gp regressor for the given semantic class.
//         *
//         * @param class_label Class label to get the regressor for.
//         *
//         * @return Regressor (or null pointer if it doesn't exist).
//         */
//        std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>> getMovableObjGpRegressor(const std::string &class_label) {
//            auto regressor_iter = movable_object_2d_gp_regressors_by_class_.find(class_label);
//            if (regressor_iter != movable_object_2d_gp_regressors_by_class_.end()) {
//                return regressor_iter->second;
//            }
//            return nullptr;
//        }

        std::shared_ptr<KdeType> getMovableObjKde(const std::string &class_label) {
            auto regressor_iter = movable_object_2d_kdes_by_class_.find(class_label);
            if (regressor_iter != movable_object_2d_kdes_by_class_.end()) {
                return regressor_iter->second;
            }
            return nullptr;
        }

        /**
         * Get the pointers to the position components of the given node.
         *
         * @param node_id[in]           Node id to get pointers to the position variables for.
         * @param pointer_results[out]  Pointers to the position and orientation variables for the node with the given
         *                              id.
         * @return True if the pointers were populated, false if not.
         */
        bool getNodePosePointers(const NodeId &node_id,
                                 std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                                         std::shared_ptr<MeasurementRotationType>> &pointer_results) {
            if (nodes_.find(node_id) != nodes_.end()) {
                NodeType node = nodes_.at(node_id);
                pointer_results = std::make_pair(node.est_position_, node.est_orientation_);
                return true;
            }
            return false;
        }

        void getNodePoses(std::unordered_map<NodeId,
                          std::pair<Eigen::Matrix<double, MeasurementTranslationDim, 1>, MeasurementRotationType>> &node_positions) {
            for (const auto &node : nodes_) {
                node_positions[node.first] = std::make_pair(Eigen::Matrix<double, MeasurementTranslationDim, 1>(*(node.second.est_position_)),
                                                            MeasurementRotationType(*(node.second.est_orientation_)));
//                LOG(INFO) << node.first;
//                LOG(INFO) << node_positions[node.first].first;
//                LOG(INFO) << node.second.est_position_->x() << ", " << node.second.est_position_->y() << ", " << node.second.est_position_->z();
            }
        }

        ceres::LocalParameterization* getRotationParameterization() const {
            return rotation_local_parameterization_creator_();
        }

        virtual ceres::CostFunction *createMovableObjectCostFunctor(
                const std::shared_ptr<KdeType> &movable_object_kde,
                const MovableObservationFactorType &factor,
                const pose_optimization::CostFunctionParameters &cost_function_params) const = 0;

        virtual ceres::CostFunction *createGaussianBinaryCostFunctor(
                const GaussianBinaryFactorType &factor) const = 0;

        virtual std::pair<double*, double*> getPointersToUnderlyingData(
                const std::pair<std::shared_ptr<Eigen::Matrix<double, MeasurementTranslationDim, 1>>,
                std::shared_ptr<MeasurementRotationType>> node_pose_pointers) const = 0;

    protected:

        // TODO there has to be a better way than just inserting samples at a variety of orientations
        const uint8_t kNumDiscreteOrientationsNegObservations = 5;

        std::function<ceres::LocalParameterization*()> rotation_local_parameterization_creator_;

        /**
         * Kernel for comparing 2d movable object pose similarity.
         */
        MovObjKernelType mov_obj_kernel_;

        /**
         * Map of node id to the nodes.
         */
        std::unordered_map<NodeId, NodeType> nodes_;

        /**
         * Movable observation factors.
         *
         * TODO should we store this in some more intelligent data structure? (Unordered map by viewing node id?).
         */
        std::vector<MovableObservationFactorType> observation_factors_;

        /**
         * Factors relating two nodes that assume gaussian noise.
         */
        std::vector<GaussianBinaryFactorType> binary_factors_;

        // TODO convert class labels to enum?
//        /**
//         * Movable object GP regressors, by their semantic class.
//         */
//        std::unordered_map<std::string, std::shared_ptr<gp_regression::GaussianProcessRegression<3, 1, gp_kernel::Pose2dKernel>>> movable_object_2d_gp_regressors_by_class_;

        /**
         * Movable object KDEs, by their semantic class.
         */
        std::unordered_map<std::string, std::shared_ptr<KdeType>> movable_object_2d_kdes_by_class_;

        // Commenting out because we're not currently using Negative detections - will probably be need to made into a
        // virtual method if we're using this
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
//                const std::pair<std::vector<NegativeMapObjectObservationType>,
//                        std::vector<MapObjectObservationType>> &pos_and_neg_observations,
//                Eigen::MatrixXf &input_matrix, Eigen::MatrixXf &output_matrix) const {
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

        virtual bool getMatrixRepresentationOfDetections(
                const std::vector<MapObjectObservationType> &pos_observations,
                Eigen::MatrixXf &input_matrix) const = 0;

//        bool getMatrixRepresentationOfDetections(
//                const std::vector<MapObjectObservationType> &pos_observations,
//                Eigen::MatrixXf &input_matrix) const {
//
//            size_t pos_obs_count = pos_observations.size();
//
//            if (pos_obs_count == 0) {
//                return false;
//            }
//
//            // Inputs should have 3 rows and as many columns as examples
//            input_matrix = Eigen::MatrixXf(3, pos_obs_count);
//            for (size_t i = 0; i < pos_obs_count; i++) {
//                input_matrix(0, i) = pos_observations[i].transl_.x();
//                input_matrix(1, i) = pos_observations[i].transl_.y();
//                input_matrix(2, i) = pos_observations[i].orientation_;
//            }
//            return true;
//        }
    };
}

#endif //AUTODIFF_GP_POSE_GRAPH_GENERIC_H
