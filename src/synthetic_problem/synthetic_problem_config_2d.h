//
// Created by amanda on 1/28/21.
//

#ifndef AUTODIFF_GP_SYNTHETIC_PROBLEM_CONFIG_2D_H
#define AUTODIFF_GP_SYNTHETIC_PROBLEM_CONFIG_2D_H

namespace synthetic_problem {

    struct SyntheticProblemNoiseConfig2d {
        double max_observable_moving_obj_distance_;

        double movable_observation_x_std_dev_;
        double movable_observation_y_std_dev_;
        double movable_observation_z_std_dev_;
        double movable_observation_yaw_std_dev_;

        double odometry_x_std_dev_;
        double odometry_y_std_dev_;
        double odometry_z_std_dev_;
        double odometry_yaw_std_dev_;

        bool add_additional_initial_noise_;
        float init_pose_gaussian_noise_x_ = 0.5;
        float init_pose_gaussian_noise_y_ = 0.5;
        float init_pose_gaussian_noise_z_ = 0.5;
        float init_pose_gaussian_noise_yaw_ = 0.5;

    };

}

#endif //AUTODIFF_GP_SYNTHETIC_PROBLEM_CONFIG_2D_H
