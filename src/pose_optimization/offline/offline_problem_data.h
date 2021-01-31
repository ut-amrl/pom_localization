//
// Created by amanda on 1/28/21.
//

#ifndef AUTODIFF_GP_OFFLINE_PROBLEM_DATA_H
#define AUTODIFF_GP_OFFLINE_PROBLEM_DATA_H

#include <pose_optimization/pose_graph_generic.h>

namespace offline_optimization {


    template<int MeasurementTranslationDim, typename MeasurementRotationType, int CovDim,
            int MovObjDistributionTranslationDim, typename MovObjDistributionRotationType>
    struct OfflineProblemData {
        std::vector<pose_graph::GaussianBinaryFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim>> odometry_factors_;

        std::vector<pose_graph::MapObjectObservation<MovObjDistributionTranslationDim, MovObjDistributionRotationType>> map_object_observations_;

        std::vector<pose_graph::MovableObservationFactor<MeasurementTranslationDim, MeasurementRotationType, CovDim>> movable_observation_factors_;

        /**
         * Node ids and initial positions.
         *
         * Assumes that all nodes referenced in above factors are present here.
         */
        std::vector<pose_graph::Node<MeasurementTranslationDim, MeasurementRotationType>> initial_node_positions_;
    };

} // end offline optimization

#endif //AUTODIFF_GP_OFFLINE_PROBLEM_DATA_H
