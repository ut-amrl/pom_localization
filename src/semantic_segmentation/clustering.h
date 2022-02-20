//
// Created by amanda on 2/19/22.
//

#ifndef AUTODIFF_GP_CLUSTERING_H
#define AUTODIFF_GP_CLUSTERING_H


//library include
#include <iostream>
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <velodyne_pointcloud/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <set>
#include <pcl/io/pcd_io.h>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <sensor_msgs/PointCloud2.h>


namespace semantic_segmentation {

    typedef pcl::PointXYZI Point2;

    typedef pcl::PointXYZI PointTypeIO;
    typedef pcl::PointXYZINormal PointTypeFull;

//    ros::Publisher pub1;
//    ros::Publisher pub2;
//    ros::Publisher pub3;
//    ros::Publisher pub4;
//    ros::Publisher pub5;

    const double kGroundPlaneHeight = -0.5;
    const double kGroundPlaneBandThickness = 0.35;
    const double kClusterTolerance = 0.3; // meters
    const int kMinClusterSize = 30;
    const int kMaxClusterSize = 25000;

    float theta_r = 180 * M_PI / 180; // The angle of rotation in radians

    double ROI_theta(double x, double y) {
        double r;
        double theta;

        r = sqrt((x * x) + (y * y));
        theta = acos(x / r) * 180 / M_PI;
        return theta;
    }

    class Clusterer {
    public:

        Clusterer(ros::NodeHandle &nh, const std::string &node_prefix = "") {
            orig_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(node_prefix + "velodyne_points", 5);
            clustered_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_clustered", 5);
        }


        std::vector<std::vector<Eigen::Vector3d>> clusterPoints(const sensor_msgs::PointCloud2 &input) {

            orig_pointcloud_pub_.publish(input);

            // TODO change return value to return something

            //1. Msg to pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(input, *cloud);

            for (auto p = cloud->points.begin(); p != cloud->points.end(); p++) {
                if (fabs(p->z - kGroundPlaneHeight) < kGroundPlaneBandThickness) {
                    cloud->points.erase(p);
                    p--;
                }
            }
//            std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                    new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(kClusterTolerance); // in meters.
            ec.setMinClusterSize(kMinClusterSize);
            ec.setMaxClusterSize(kMaxClusterSize);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            // The outer vector contains all clusters; the inner vector is all points for a given cluster
            std::vector<std::vector<Eigen::Vector3d>> clusters;

            int j = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<pcl::PointIndices>::const_iterator it =
                    cluster_indices.begin();
                 it != cluster_indices.end();
                 ++it) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                        new pcl::PointCloud<pcl::PointXYZ>);
                std::vector<Eigen::Vector3d> points_for_cluster;
                for (const auto &idx : it->indices) {
                    cloud_cluster->push_back((*cloud)[idx]); //*
                    // Created a colored point from (*cloud)[idx].
                    pcl::PointXYZRGB point;
                    point.x = (*cloud)[idx].x;
                    point.y = (*cloud)[idx].y;
                    point.z = (*cloud)[idx].z;
                    points_for_cluster.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
                    uint8_t r = (uint8_t) (j % 2) * 255;
                    uint8_t b = (uint8_t) (j % 4) * 63;
                    uint8_t g = 0;
                    uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
                    point.rgb = rgb;
                    colored_cloud->push_back(point);
                }
                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                //  std::cout << "PointCloud representing the Cluster: "
                //            << cloud_cluster->size ()
                //            << " data points." << std::endl;
                j++;
                clusters.emplace_back(points_for_cluster);
            }
//            std::cout << "Number of clusters: " << cluster_indices.size() << std::endl;
            // Create a point cloud with different colors for each cluster.

            // Publish the clustered cloud.
            sensor_msgs::PointCloud2 output;
            // pcl_conversions::fromPCL(colored_cloud, output);
            pcl::toROSMsg(*colored_cloud, output);
            output.header = input.header;
            clustered_pointcloud_pub_.publish(output);

            return clusters;
        }

    private:
        ros::Publisher orig_pointcloud_pub_;
        ros::Publisher clustered_pointcloud_pub_;

    };
}

#endif //AUTODIFF_GP_CLUSTERING_H
