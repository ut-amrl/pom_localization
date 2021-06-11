//
// Created by amanda on 3/3/21.
//

#include <glog/logging.h>
#include <ros/ros.h>

#include <file_io/lidar_odom.h>
#include <h3d_dataset/h3d_file_operations.h>
#include <h3d_dataset/gps.h>
#include <h3d_dataset/obj_detections.h>
#include <pose_optimization/offline/offline_problem_runner.h>
#include <visualization/ros_visualization.h>
#include <pose_optimization/offline/ceres_visualization_callback_2d.h>

#include <h3d_dataset/dataset_odom.h>

int main(int argc, char** argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    ros::init(argc, argv,
              "output_gps_summary");

    ros::NodeHandle n;

    std::string h3d_dataset_directory = "/home/amanda/datasets/h3d/icra_benchmark_20200103_with_odom";
    std::string dataset_augmentation_dir = h3d_dataset_directory + "/augmentation";
    std::string output_filename = "gps_summary.csv";

    std::vector<std::string> all_scenarios = h3d::getAllScenarios(h3d_dataset_directory);

    for (const std::string &scenario_str : all_scenarios) {
        std::string csv_file_name = dataset_augmentation_dir + "/scenario_" + scenario_str + "/" + output_filename;

        std::vector<h3d::GPSData> first_gps_entries_per_file =
                h3d::getFirstGpsDataForEachFileNumInScenario(h3d_dataset_directory, scenario_str);

        std::ofstream csv_file(csv_file_name, std::ofstream::trunc);

        for (size_t i = 0; i < first_gps_entries_per_file.size(); i++) {
            h3d::GPSData gps_data = first_gps_entries_per_file[i];
            csv_file << std::fixed << std::setprecision(6) << gps_data.abs_lat_ << ", " << gps_data.abs_long_ << "\n";

        }
        csv_file.close();

    }

    return 0;
}