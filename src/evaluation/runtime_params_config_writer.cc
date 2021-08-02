#include <glog/logging.h>

#include <ros/ros.h>
#include <iostream>
#include <file_io/runtime_params_config_io.h>

DEFINE_string(cfg_file_name, "xxx", "Config file name");

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);
    ros::init(argc, argv,
              "runtime_params_config_writer");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string runtime_params_config_file = FLAGS_cfg_file_name;
    file_io::RuntimeParamsConfig runtime_params_config;

    runtime_params_config.max_gpc_samples_ = 500;

    runtime_params_config.num_nodes_in_optimization_window_ = 30;
    runtime_params_config.full_optimization_interval_ = 80;

    runtime_params_config.mean_position_kernel_len_ = 0.6;
    runtime_params_config.mean_orientation_kernel_len_ = 0.2;

    runtime_params_config.mean_position_kernel_var_ = 0.045;
    runtime_params_config.mean_orientation_kernel_var_ = 10000;

    runtime_params_config.default_obj_probability_input_variance_for_mean_ = 10;

    runtime_params_config.var_position_kernel_len_ = 7.5;
    runtime_params_config.var_orientation_kernel_len_ = 20;

    runtime_params_config.var_position_kernel_var_ = 30;
    runtime_params_config.var_orientation_kernel_var_ = 0.3;

    runtime_params_config.default_obj_probability_input_variance_for_var_ = 10;

    runtime_params_config.detection_variance_transl_x_ = 0.01;
    runtime_params_config.detection_variance_transl_y_ = 0.01;
    runtime_params_config.detection_variance_theta_ = 0.02;

//    runtime_params_config.odom_k1_ = 0.75;
//    runtime_params_config.odom_k2_ = 1;
//    runtime_params_config.odom_k3_ = 0.75;
//    runtime_params_config.odom_k4_ = 1;
//    runtime_params_config.odom_k5_ = 1;
//    runtime_params_config.odom_k6_ = 2;


    runtime_params_config.odom_k1_ = 0.75;
    runtime_params_config.odom_k2_ = 5;
    runtime_params_config.odom_k3_ = 0.75;
    runtime_params_config.odom_k4_ = 5;
    runtime_params_config.odom_k5_ = 2;
    runtime_params_config.odom_k6_ = 5;

//    runtime_params_config.odom_k1_ = 0.1;
//    runtime_params_config.odom_k2_ = 0.2;
//    runtime_params_config.odom_k3_ = 0.05;
//    runtime_params_config.odom_k4_ = 0.2;
//    runtime_params_config.odom_k5_ = 0.5;
//    runtime_params_config.odom_k6_ = 0.4;
    file_io::writeRuntimeParamsConfigToFile(runtime_params_config_file, runtime_params_config);

    return 0;
}