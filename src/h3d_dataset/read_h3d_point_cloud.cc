//
// Created by amanda on 2/13/21.
//

#ifndef AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC
#define AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC

#include <h3d_dataset/h3d_file_operations.h>
#include <h3d_dataset/point_cloud.h>
#include <string>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>
#include <h3d_dataset/gps.h>

namespace h3d {

}

int main(int argc, char** argv) {

    std::cout<<"Here!";
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    ros::init(argc, argv,
              "read_point_cloud");

    LOG(INFO) << "Init node";
    sleep(1);

    LOG(INFO) << "Creating node handle";
    ros::NodeHandle n;

    LOG(INFO) << "Creating pub";
    ros::Publisher point_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 10);
    LOG(INFO) << "Init pub";
    while (point_cloud_pub.getNumSubscribers() == 0) {
        LOG(INFO) << "No subscribers";
        sleep(1);
    }
    LOG(INFO) << "Found subscribers";
    ros::Duration(5).sleep();
    LOG(INFO) << "Done sleeping";

    std::string h3d_dataset_directory = "/home/amanda/datasets/h3d/icra_benchmark_20200103_with_odom";
    std::string scenario_number_str = "002";

    std::string scenario_dir_str = h3d::getScenarioDirectory(h3d_dataset_directory, scenario_number_str);

    int max_file_num_for_scenario = h3d::getMaxFileNumForScenario(scenario_dir_str, h3d::kPointCloudFilePrefix);

    LOG(INFO) << "Starting playback!";

    std::vector<pcl::PCLPointCloud2> point_clouds;


    for (int i = 0; i <= max_file_num_for_scenario; i++) {
        pcl::PCLPointCloud2 point_cloud; // TODO, what to do with this
        h3d::readPointCloudForScenario(scenario_dir_str, i, point_cloud);
        std::vector<h3d::GPSData> gps_data_for_file_num = h3d::readGpsDataForFileNum(scenario_dir_str, i);
        point_cloud.header.frame_id = "velodyne";
        point_cloud.header.stamp = 1e6 * gps_data_for_file_num[0].timestamp_;
//        ros::Time curr_time = ros::Time::now();
//        point_cloud.header.stamp = 1e6 * curr_time.sec + 1e-3 * curr_time.nsec;

        LOG(INFO) << "Is dense? " << (bool) point_cloud.is_dense;
        LOG(INFO) << "Fields";
        point_cloud.is_dense = (bool) true;


        for (size_t j = 0; j < point_cloud.fields.size(); j++) {
            if (point_cloud.fields[j].name == "radius") {
                LOG(INFO) << "Changing radius to intensity";
                point_cloud.fields[j].name = "intensity";
            }
            if (point_cloud.fields[j].name == "confidence") {
                LOG(INFO) << "Data type " << (int) point_cloud.fields[j].datatype;
                LOG(INFO) << "Changing confidence to ring";
                point_cloud.fields[j].name = "ring";
            }
        }


        pcl::PointCloud<PointXYZIRFloat> point_cloud_ring_as_float;
        pcl::fromPCLPointCloud2(point_cloud, point_cloud_ring_as_float);
        pcl::PointCloud<PointXYZIRInt> point_cloud_ring_as_int_and_filtered;

        point_cloud_ring_as_int_and_filtered.is_dense = point_cloud_ring_as_float.is_dense;
        point_cloud_ring_as_int_and_filtered.height = point_cloud_ring_as_float.height;
        point_cloud_ring_as_int_and_filtered.width = point_cloud_ring_as_float.width;
        point_cloud_ring_as_int_and_filtered.header = point_cloud_ring_as_float.header;
        LOG(INFO) << "Height, width " << point_cloud_ring_as_int_and_filtered.height << ", " << point_cloud_ring_as_int_and_filtered.width;
        for (const PointXYZIRFloat &point_float : point_cloud_ring_as_float.points) {
            PointXYZIRInt point_int;
            point_int.ring = point_float.ring;
            point_int.intensity = point_float.intensity;
            point_int.x = point_float.x;
            point_int.y = point_float.y;
            point_int.z = point_float.z;
            double point_dist_sq = std::pow(point_int.x, 2) + std::pow(point_int.y, 2) + std::pow(point_int.z, 2);
            if (point_dist_sq > 9) {
                point_cloud_ring_as_int_and_filtered.points.emplace_back(point_int);
            } else {
                point_cloud_ring_as_int_and_filtered.width--;
            }
        }
        pcl::toPCLPointCloud2(point_cloud_ring_as_int_and_filtered, point_cloud);

        point_clouds.emplace_back(point_cloud);
//        LOG(INFO) << point_cloud.fields;
//        for (uint32_t j = 0; j < point_cloud.height * point_cloud.width; j++) {
//            point_cloud.fields
//        }
//        point_cloud_pub.publish(point_cloud);

//        LOG(INFO) << point_cloud.header.stamp;
//        LOG(INFO) << point_cloud.header.frame_id;
//        LOG(INFO) << point_cloud.width;
//        LOG(INFO) << point_cloud.height;
//        rate.sleep();
    }

    ros::Rate rate(10);
    for (int i = 0; i <= max_file_num_for_scenario; i++) {
        LOG(INFO) << "Fields";
        for (size_t j = 0; j < point_clouds[i].fields.size(); j++) {
            LOG(INFO) << point_clouds[i].fields[j].name;
        }
        LOG(INFO) << "Publishing " << i;
        point_cloud_pub.publish(point_clouds[i]);
        rate.sleep();
    }

    return 0;
}

#endif //AUTODIFF_GP_READ_H3D_POINT_CLOUD_CC
