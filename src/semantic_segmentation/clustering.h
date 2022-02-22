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

#include <file_io/clustering_config_io.h>


namespace semantic_segmentation {

    typedef pcl::PointXYZI Point2;

    class Clusterer {
    public:

        Clusterer(const file_io::ClusteringConfig &config, ros::NodeHandle &nh, const std::string &node_prefix = "")
                : config_(config) {
            orig_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(node_prefix + "velodyne_points", 5);
            clustered_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    node_prefix + "velodyne_points_clustered", 5);
            filtered_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(node_prefix + "velodyne_points_filtered",
                                                                              5);
            merged_z_pointcloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(node_prefix + "velodyne_points_z_merged",
                                                                              5);
        }

        std::vector<std::vector<Eigen::Vector3d>>
        clusterPoints(const sensor_msgs::PointCloud2 &input, const bool &filter_and_merge_clusters = false) {

            //1. Msg to pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(input, *cloud);
            std::vector<pcl::PointIndices> cluster_indices = clusterCloud(input.header, cloud, config_,
                                                                          filter_and_merge_clusters);

            orig_pointcloud_pub_.publish(input);

            // The outer vector contains all clusters; the inner vector is all points for a given cluster
            std::vector<std::vector<Eigen::Vector3d>> clusters;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGB>);
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                 it != cluster_indices.end(); ++it) {
                std::vector<Eigen::Vector3d> points_for_cluster;
                for (const auto &idx : it->indices) {
                    // Created a colored point from (*cloud)[idx].
                    pcl::PointXYZRGB point;
                    point.x = (*cloud)[idx].x;
                    point.y = (*cloud)[idx].y;
                    point.z = (*cloud)[idx].z;
                    points_for_cluster.emplace_back(Eigen::Vector3d(point.x, point.y, point.z));
                }

                clusters.emplace_back(points_for_cluster);
            }

            return clusters;
        }

    private:

        ros::Publisher orig_pointcloud_pub_;
        ros::Publisher clustered_pointcloud_pub_;
        ros::Publisher filtered_pointcloud_pub_;
        ros::Publisher merged_z_pointcloud_pub_;

        file_io::ClusteringConfig config_;

        void visualizeClusters(const std_msgs::Header &header, const std::vector<pcl::PointIndices> &cluster_indices,
                               ros::Publisher publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
            size_t j = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
                 it != cluster_indices.end(); ++it) {
                for (const auto &idx : it->indices) {
                    // Created a colored point from (*cloud)[idx].
                    pcl::PointXYZRGB point;
                    point.x = (*cloud)[idx].x;
                    point.y = (*cloud)[idx].y;
                    point.z = (*cloud)[idx].z;
                    uint8_t r = (uint8_t) (j % 2) * 255;
                    uint8_t b = (uint8_t) (j % 4) * 63;
                    uint8_t g = (uint8_t) (j % 6) * (255 / 6);
                    uint32_t rgb = ((uint32_t) r << 16 | (uint32_t) g << 8 | (uint32_t) b);
                    point.rgb = rgb;
                    colored_cloud->push_back(point);
                }
                j++;
            }

            // Publish the clustered cloud.
            sensor_msgs::PointCloud2 output;
            // pcl_conversions::fromPCL(colored_cloud, output);
            pcl::toROSMsg(*colored_cloud, output);
            output.header = header;
            publisher.publish(output);
        }

        std::vector<pcl::PointIndices> mergeClustersVertically(
                const std::vector<pcl::PointIndices> &orig_cluster_indices,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                const file_io::ClusteringConfig &config) {


            // TODO consider plotting
            std::unordered_set<size_t> skip_cluster_indices;

            std::vector<pcl::PointIndices> merged_clusters;

            bool any_merged = false;

            for (size_t orig_cluster_num = 0; orig_cluster_num < orig_cluster_indices.size(); orig_cluster_num++) {
                if (skip_cluster_indices.find(orig_cluster_num) != skip_cluster_indices.end()) {
                    continue;
                }
                std::unordered_set<size_t> merge_indices_with_orig;
                for (size_t candidate_merge_cluster = orig_cluster_num + 1;
                     candidate_merge_cluster < orig_cluster_indices.size(); candidate_merge_cluster++) {
                    if (skip_cluster_indices.find(candidate_merge_cluster) != skip_cluster_indices.end()) {
                        continue;
                    }
                    bool merge_with_orig = false;
                    for (const auto orig_cluster_index : orig_cluster_indices[orig_cluster_num].indices) {
                        Eigen::Vector2d orig_point((*cloud)[orig_cluster_index].x, (*cloud)[orig_cluster_index].y);
                        for (const auto candidate_merge_cluster_index : orig_cluster_indices[candidate_merge_cluster].indices) {
                            Eigen::Vector2d candidate_point((*cloud)[candidate_merge_cluster_index].x,
                                                            (*cloud)[candidate_merge_cluster_index].y);
                            if ((orig_point - candidate_point).norm() < config.z_merge_proximity) {
                                // TODO maybe we should require multiple points within the merge proximity?
                                merge_with_orig = true;
                                break;
                            }
                        }
                        if (merge_with_orig) {
                            break;
                        }
                    }
                    if (merge_with_orig) {
                        merge_indices_with_orig.insert(candidate_merge_cluster);
                        skip_cluster_indices.insert(candidate_merge_cluster);
                        any_merged = true;
                    }
                }
                if (merge_indices_with_orig.empty()) {
                    merged_clusters.emplace_back(orig_cluster_indices[orig_cluster_num]);
                } else {
                    pcl::PointIndices merged_indices = orig_cluster_indices[orig_cluster_num];
                    for (const size_t merge_cluster_index : merge_indices_with_orig) {
                        pcl::PointIndices to_merge_indices = orig_cluster_indices[merge_cluster_index];
                        merged_indices.indices.insert(merged_indices.indices.end(), to_merge_indices.indices.begin(),
                                                      to_merge_indices.indices.end());
                    }
                    merged_clusters.emplace_back(merged_indices);
                }
            }
            if (any_merged) {
                return mergeClustersVertically(merged_clusters, cloud, config);
            } else {
                return merged_clusters;
            }
        }


        std::vector<pcl::PointIndices> filterClusters(const std::vector<pcl::PointIndices> &orig_cluster_indices,
                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                      const file_io::ClusteringConfig &config) {

            int j = 0;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_filtered_cloud(
                    new pcl::PointCloud<pcl::PointXYZRGB>);

            std::vector<pcl::PointIndices> filtered_clusters;

            for (std::vector<pcl::PointIndices>::const_iterator it =
                    orig_cluster_indices.begin();
                 it != orig_cluster_indices.end();
                 ++it) {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                        new pcl::PointCloud<pcl::PointXYZ>);

                bool include_cluster = true;
                double max_horiz_span = 0.0;
                double max_z = -std::numeric_limits<double>::max();
                double max_vert_span = 0.0;
                for (const auto &idx : it->indices) {
                    // Created a colored point from (*cloud)[idx].
                    Eigen::Vector3d point_1((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);
                    if (point_1.z() > config.max_z) {
                        include_cluster = false;
                        break;
                    }
                    max_z = std::max(max_z, point_1.z());
                    for (const auto &compare_idx : it->indices) {
                        Eigen::Vector3d point_2((*cloud)[compare_idx].x, (*cloud)[compare_idx].y,
                                                (*cloud)[compare_idx].z);
                        if ((point_1 - point_2).norm() > config.max_cluster_span) {
                            include_cluster = false;
                            break;
                        }
                        max_vert_span = std::max(max_vert_span, abs(point_1.z() - point_2.z()));
                        Eigen::Vector2d horiz_diff(point_1.x() - point_2.x(), point_1.y() - point_2.y());
                        max_horiz_span = std::max(max_horiz_span, horiz_diff.norm());

                    }
                    double vert_to_horiz_span_ratio = max_vert_span / max_horiz_span;
                    if ((max_z < config.min_max_z) || (max_horiz_span < config.min_horiz_span) ||
                        (config.max_height_over_horiz_span <= vert_to_horiz_span_ratio)) {
                        include_cluster = false;
                    }
                    if (!include_cluster) {
                        break;
                    }
                }
                j++;
                if (include_cluster) {
                    filtered_clusters.emplace_back(*it);
                }
            }

            return filtered_clusters;
        }

        std::vector<pcl::PointIndices> clusterCloud(const std_msgs::Header &orig_cloud_header,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                    const file_io::ClusteringConfig &config,
                                                    const bool &filter_and_merge_clusters = false) {


            for (auto p = cloud->points.begin(); p != cloud->points.end(); p++) {
                if (fabs(p->z - config.ground_plane_height) < config.ground_plane_band_thickness) {
                    cloud->points.erase(p);
                    p--;
                }
            }

            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
                    new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(config.cluster_tolerance); // in meters.
            ec.setMinClusterSize(config.min_cluster_size);
            ec.setMaxClusterSize(config.max_cluster_size);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            visualizeClusters(orig_cloud_header, cluster_indices, clustered_pointcloud_pub_, cloud);


            if (!filter_and_merge_clusters) {
                return cluster_indices;
            }

            std::vector<pcl::PointIndices> merged_z_clusters = mergeClustersVertically(cluster_indices, cloud,
                                                                                       config);
            visualizeClusters(orig_cloud_header, merged_z_clusters, merged_z_pointcloud_pub_, cloud);

            std::vector<pcl::PointIndices> filtered_clusters = filterClusters(merged_z_clusters, cloud,
                                                                              config);
            visualizeClusters(orig_cloud_header, filtered_clusters, filtered_pointcloud_pub_, cloud);
            return filtered_clusters;
        }

    };
}

#endif //AUTODIFF_GP_CLUSTERING_H
