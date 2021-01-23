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
    template<typename TranslationType, typename RotationType>
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
        std::shared_ptr<TranslationType> est_position_;

        /**
         * Estimated position of the node.
         *
         * TODO why did I make this a shared ptr again? So it could be updated by Ceres?
         */
        std::shared_ptr<RotationType> est_orientation_;
    };
}

#endif //AUTODIFF_GP_POSE_GRAPH_GENERIC_H
