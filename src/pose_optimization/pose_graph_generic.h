//
// Created by amanda on 1/22/21.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_GENERIC_H
#define AUTODIFF_GP_POSE_GRAPH_GENERIC_H

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <glog/logging.h>

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/autodiff_cost_function.h>
#include <eigen3/Eigen/Dense>

//#include <gaussian_process/gp_regression.h>
#include <gaussian_process/kernel_density_estimator.h>
#include <pose_optimization/pose_optimization_parameters.h>
#include <gaussian_process/gp_classifier.h>

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

        double obs_value_;

        // TODO consider adding which robot pose node this came from and the relative pose and variance relative to the
        //  robot
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
            int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType, int KernelDim>
    class PoseGraph {
    public:

        typedef Node<MeasurementTranslationDim, MeasurementRotationType> NodeType;
        typedef MovableObservation<MeasurementTranslationDim, MeasurementRotationType, CovDim> MovableObservationType;
        typedef MovableObservationFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> MovableObservationFactorType;
        typedef GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim> GaussianBinaryFactorType;
        typedef MapObjectObservation<MovObjDistributionTranslationDim, MovObjDistributionRotationType> MapObjectObservationType;
        typedef gp_regression::GaussianProcessClassifier<KernelDim, MovObjKernelType> GpcType;

        /**
         * Create the pose graph.
         *
         * @param kernel Kernel for comparing 2d movable object poses. // TODO, this doesn't seem like it necessarily
         * belongs to this class intuitively, any way to restructure?
         */
        PoseGraph(const std::function<ceres::LocalParameterization*()> &rotation_local_parameterization_creator,
                  const std::unordered_map<std::string, double> &obj_probability_prior_mean_by_class,
                  const double &default_obj_probability_prior_mean,
                  const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_mean,
                  const double &default_obj_probability_input_variance_for_mean,
                  const std::unordered_map<std::string, double> &obj_probability_input_variance_by_class_for_var,
                  const double &default_obj_probability_input_variance_for_var,
                  MovObjKernelType &mean_kernel,
                  MovObjKernelType &var_kernel) : rotation_local_parameterization_creator_(rotation_local_parameterization_creator),
                  obj_probability_prior_mean_by_class_(obj_probability_prior_mean_by_class),
                  default_obj_probability_prior_mean_(default_obj_probability_prior_mean),
                  obj_probability_input_variance_by_class_for_mean_(obj_probability_input_variance_by_class_for_mean),
                  default_obj_probability_input_variance_for_mean_(default_obj_probability_input_variance_for_mean),
                  obj_probability_input_variance_by_class_for_var_(obj_probability_input_variance_by_class_for_var),
                  default_obj_probability_input_variance_for_var_(default_obj_probability_input_variance_for_var),
                  mov_obj_mean_kernel_(mean_kernel),
                  mov_obj_var_kernel_(var_kernel){

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

        void addMovableObservationFactors(const std::vector<MovableObservationFactorType> &movable_observation_factors) {
            observation_factors_.insert(observation_factors_.end(), movable_observation_factors.begin(), movable_observation_factors.end());
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

        void addMapFrameObservations(
                const std::unordered_map<std::string, std::vector<MapObjectObservationType>> &observations_by_class) {
            for (const auto &obs_by_class : observations_by_class) {
                Eigen::MatrixXd inputs;
                Eigen::MatrixXd outputs;
                if (getMatrixRepresentationOfDetections(obs_by_class.second, inputs)
                        && getMatrixRepresentationOfDetectionSampleValue(obs_by_class.second, outputs)) {
                    auto gpc_iter = movable_object_gpcs_by_class_.find(obs_by_class.first);
                    std::shared_ptr<GpcType> gpc;
                    if (gpc_iter != movable_object_gpcs_by_class_.end()) {
                        gpc = gpc_iter->second;
                        gpc->appendData(inputs, outputs);
                    } else {
                        double prior_mean = default_obj_probability_prior_mean_;
                        if (obj_probability_prior_mean_by_class_.find(obs_by_class.first) != obj_probability_prior_mean_by_class_.end()) {
                            double mean_for_class = obj_probability_prior_mean_by_class_.at(obs_by_class.first);
                            if ((mean_for_class > 0) && (mean_for_class < 1)) {
                                prior_mean = mean_for_class;
                            }
                        }
                        double input_variance_for_mean = default_obj_probability_input_variance_for_mean_;
                        if (obj_probability_input_variance_by_class_for_mean_.find(obs_by_class.first) != obj_probability_input_variance_by_class_for_mean_.end()) {
                            double input_variance_for_class = obj_probability_input_variance_by_class_for_mean_.at(obs_by_class.first);
                            if (input_variance_for_class > 0) {
                                input_variance_for_mean = input_variance_for_class;
                            }
                        }
                        double input_variance_for_var = default_obj_probability_input_variance_for_var_;
                        if (obj_probability_input_variance_by_class_for_var_.find(obs_by_class.first) != obj_probability_input_variance_by_class_for_var_.end()) {
                            double input_variance_for_class = obj_probability_input_variance_by_class_for_var_.at(obs_by_class.first);
                            if (input_variance_for_class > 0) {
                                input_variance_for_var = input_variance_for_class;
                            }
                        }
                        LOG(INFO) << "Outputs size: " << outputs.size();
                        gpc = std::make_shared<GpcType>(inputs, outputs, prior_mean, input_variance_for_mean,
                                                        input_variance_for_var, &mov_obj_mean_kernel_,
                                                        &mov_obj_var_kernel_);
                    }
                    movable_object_gpcs_by_class_[obs_by_class.first] = gpc;
                }
            }
        }

        std::shared_ptr<GpcType> getMovableObjGpc(const std::string &class_label) {
            auto gpc_iter = movable_object_gpcs_by_class_.find(class_label);
            if (gpc_iter != movable_object_gpcs_by_class_.end()) {
                return gpc_iter->second;
            }
            LOG(WARNING) << "No Gaussian Process Classifier found for class " << class_label;
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
                const std::shared_ptr<GpcType> &movable_object_gpc,
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

        std::unordered_map<std::string, double> obj_probability_prior_mean_by_class_;

        double default_obj_probability_prior_mean_;

        std::unordered_map<std::string, double> obj_probability_input_variance_by_class_for_mean_;

        double default_obj_probability_input_variance_for_mean_;


        std::unordered_map<std::string, double> obj_probability_input_variance_by_class_for_var_;

        double default_obj_probability_input_variance_for_var_;

        /**
         * Kernel for comparing 2d movable object pose similarity.
         */
        MovObjKernelType mov_obj_mean_kernel_;

        /**
         * Kernel for comparing 2d movable object pose similarity.
         */
        MovObjKernelType mov_obj_var_kernel_;

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
        /**
         * Movable object Gaussian Process Classifiers, by their semantic class.
         */
        std::unordered_map<std::string, std::shared_ptr<GpcType>> movable_object_gpcs_by_class_;

        virtual bool getMatrixRepresentationOfDetections(
                const std::vector<MapObjectObservationType> &pos_observations,
                Eigen::MatrixXd &input_matrix) const = 0;

        bool getMatrixRepresentationOfDetectionSampleValue(const std::vector<MapObjectObservationType> &observations,
                                                           Eigen::MatrixXd &outputs_matrix) {
            size_t obs_count = observations.size();

            if (obs_count == 0) {
                return false;
            }

            outputs_matrix = Eigen::MatrixXd(1, obs_count);
            for (size_t i = 0; i < obs_count; i++) {
                outputs_matrix(0, i) = observations[i].obs_value_;
            }
            return true;
        }
    };
}

#endif //AUTODIFF_GP_POSE_GRAPH_GENERIC_H
