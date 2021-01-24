//
// Created by amanda on 1/22/21.
//

#ifndef AUTODIFF_GP_POSE_GRAPH_GENERIC_H
#define AUTODIFF_GP_POSE_GRAPH_GENERIC_H

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
}

#endif //AUTODIFF_GP_POSE_GRAPH_GENERIC_H
