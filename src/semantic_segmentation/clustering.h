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
#include <visualization_msgs/Marker.h>

#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/project_inliers.h>


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
            line_fit_pub_ = nh.advertise<visualization_msgs::Marker>(node_prefix + "/line_fit", 100);
            line_filtered_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    node_prefix + "/velodyne_points_line_fit_filter", 10);

            ground_plane_marker_pub_ = nh.advertise<visualization_msgs::Marker>(node_prefix + "/ground_plane_viz", 2);
            line_fit_pub_ = nh.advertise<visualization_msgs::Marker>(node_prefix + "/line_fit", 100);
            ground_plane_removed_pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    node_prefix + "/velodyne_points_ground_plane_removed", 10);

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
        ros::Publisher line_fit_pub_;
        ros::Publisher line_filtered_pub_;

        ros::Publisher ground_plane_marker_pub_;
        ros::Publisher ground_plane_removed_pub_;

        file_io::ClusteringConfig config_;

        struct Rectangle {
            Eigen::Vector2d center;

            // Radians
            double angle;
        };

        void visualizeGroundPlane(const double &ground_plane_height, const double &ground_plane_band_thickness,
                                  const std_msgs::Header &input_header) {
            visualization_msgs::Marker marker;
            marker.header = input_header;

            marker.ns = "pcl_clustering";
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 200;
            marker.scale.y = 200;
            marker.scale.z = ground_plane_band_thickness * 2;

            marker.pose.position.z = ground_plane_height;

            marker.pose.orientation.w = 1;

            marker.type = visualization_msgs::Marker::CUBE;

            marker.color.a = 0.6;
//    marker.color.r = 1.0;
//    marker.color.b = 1.0;
//    marker.color.g = 1.0;
            marker.id = 1;

            ground_plane_marker_pub_.publish(marker);
        }

        void visualizeLines(const std::vector<Eigen::Vector2d> &points_projected_onto_line,
                            const std_msgs::Header &input_header,
                            const int32_t &id,
                            const bool &alt_color) {
            visualization_msgs::Marker marker;
            marker.header = input_header;

            marker.ns = "pcl_clustering";
            marker.action = visualization_msgs::Marker::ADD;
            marker.scale.x = 0.2; // TODO?
            for (const Eigen::Vector2d &point_on_line : points_projected_onto_line) {
                geometry_msgs::Point point;
                point.x = point_on_line.x();
                point.y = point_on_line.y();
                marker.points.emplace_back(point);
            }

            marker.pose.position.z = 1; // TODO?

            marker.pose.orientation.w = 1;

            marker.type = visualization_msgs::Marker::LINE_STRIP;

            marker.color.a = 1.0;
            if (alt_color) {
                marker.color.r = 1.0;
            }
            marker.color.b = 1.0;
            marker.color.g = 1.0;
            marker.id = id;

            line_fit_pub_.publish(marker);
        }

        void clearMarkers(const std_msgs::Header &input_header, ros::Publisher publisher) {
            visualization_msgs::Marker marker;
            marker.header = input_header;

            marker.ns = "pcl_clustering";
            marker.action = visualization_msgs::Marker::DELETEALL;

            line_fit_pub_.publish(marker);
        }

        std::pair<std::vector<int>, Eigen::VectorXf>
        fitLineRansac3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double &distance_threshold,
                        const int &max_ransac_iterations) {
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(
                    new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line_model);
            ransac.setDistanceThreshold(distance_threshold);
            ransac.setMaxIterations(max_ransac_iterations);
            ransac.computeModel();
            std::vector<int> inlier_indices;
            ransac.getInliers(inlier_indices);
            Eigen::VectorXf model_coeffs(6);
            ransac.getModelCoefficients(model_coeffs);
            return std::make_pair(inlier_indices, model_coeffs);
        }

        std::pair<std::vector<int>, Eigen::VectorXf>
        fitPerpendicularLineRansac3d(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const double &distance_threshold,
                                     const int &max_ransac_iterations, const Eigen::Vector3f &perp_to_axis,
                                     const double &angle_epsilon) {
            Eigen::Vector3f rotated_axis =
                    Eigen::Quaternionf(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ())) * perp_to_axis;

            pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>::Ptr line_model(
                    new pcl::SampleConsensusModelParallelLine<pcl::PointXYZ>(cloud));
            line_model->setAxis(rotated_axis);
            line_model->setEpsAngle(angle_epsilon);
            //LOG(INFO) << "Angle epsilon " << angle_epsilon;
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line_model);
            ransac.setDistanceThreshold(distance_threshold);
            ransac.setMaxIterations(max_ransac_iterations);
            ransac.computeModel();
            std::vector<int> inlier_indices;
            ransac.getInliers(inlier_indices);
            Eigen::VectorXf model_coeffs(6);
            ransac.getModelCoefficients(model_coeffs);
            Eigen::Vector3f model_axis = model_coeffs.bottomRows(3).normalized();
            double angle_between_axes = acos(model_axis.dot(perp_to_axis.normalized()));
            //LOG(INFO) << model_axis;
            //LOG(INFO) << "Angle between axes " << angle_between_axes;
            //LOG(INFO) << "Diff from perp " << abs(angle_between_axes - M_PI_2);
            double angle_between_target = acos(model_axis.dot(rotated_axis.normalized()));
            if (angle_between_target > M_PI_2) {
                angle_between_target -= M_PI;
                angle_between_target = abs(angle_between_target);
            }
            //LOG(INFO) << "2Angle between target axis " << angle_between_target;
            if (angle_between_target > angle_epsilon) {
                //LOG(INFO) << "Num inliers for bad angle " << inlier_indices.size();
            }
            return std::make_pair(inlier_indices, model_coeffs);
        }

        void visualizeClusters(const std_msgs::Header &header, const std::vector<pcl::PointIndices> &cluster_indices,
                               ros::Publisher publisher, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
            size_t j = 1;
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
            pcl::toROSMsg(*colored_cloud, output);
            output.header = header;
            publisher.publish(output);
        }

        std::vector<pcl::PointIndices>
        mergeClustersVertically(const std::vector<pcl::PointIndices> &orig_cluster_indices,
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
            std::vector<pcl::PointIndices> filtered_clusters;

            for (std::vector<pcl::PointIndices>::const_iterator it =
                    orig_cluster_indices.begin();
                 it != orig_cluster_indices.end();
                 ++it) {

                bool include_cluster = true;
                double max_horiz_span = 0.0;
                double max_z = std::numeric_limits<double>::lowest();
                double min_z = std::numeric_limits<double>::max();
                double max_vert_span = 0.0;
                size_t points_in_cluster = it->indices.size();
                for (size_t idx_num = 0; idx_num < points_in_cluster; idx_num++) {
                    int idx = it->indices[idx_num];
                    // Created a colored point from (*cloud)[idx].
                    Eigen::Vector3d point_1((*cloud)[idx].x, (*cloud)[idx].y, (*cloud)[idx].z);
                    if (point_1.z() > config.max_z) {
                        //LOG(INFO) << "Excluding cluster for max z " << point_1.z();
                        include_cluster = false;
                        break;
                    }
                    max_z = std::max(max_z, point_1.z());
                    min_z = std::min(min_z, point_1.z());
                    for (size_t compare_idx_num = idx_num + 1; compare_idx_num < points_in_cluster; compare_idx_num++) {
                        int compare_idx = it->indices[compare_idx_num];
                        Eigen::Vector3d point_2((*cloud)[compare_idx].x, (*cloud)[compare_idx].y,
                                                (*cloud)[compare_idx].z);
                        if ((point_1 - point_2).norm() > config.max_cluster_span) {
                            //LOG(INFO) << "Excluding cluster for having too large of a span << (point_1 - point_2).norm();
                            include_cluster = false;
                            break;
                        }
                        max_vert_span = std::max(max_vert_span, abs(point_1.z() - point_2.z()));
                        Eigen::Vector2d horiz_diff(point_1.x() - point_2.x(), point_1.y() - point_2.y());
                        max_horiz_span = std::max(max_horiz_span, horiz_diff.norm());
                    }

                    if (!include_cluster) {
                        break;
                    }
                }

                double vert_to_horiz_span_ratio = max_vert_span / max_horiz_span;
                if (include_cluster) {
                    if (config.max_height_over_horiz_span <= vert_to_horiz_span_ratio) {
                        //LOG(INFO) << "Excluding cluster for too high vert to horiz span " << vert_to_horiz_span_ratio;
                    }
                    if (max_z < config.min_max_z) {
                        //LOG(INFO) << "Excluding for too low max z " << max_z;
                    }
                    if (max_horiz_span < config.min_horiz_span) {
                        //LOG(INFO) << "Excluding for too low span " << max_horiz_span;
                    }
                    if (min_z > config.max_min_z) {
                        //LOG(INFO) << "Excluding for too high min z " << min_z;
                    }
                    double z_span = max_z - min_z;
                    if (z_span < config.min_z_span) {
                        //LOG(INFO) << "Excluding for too low z span " << z_span;
                    }

                    if ((max_z < config.min_max_z) || (max_horiz_span < config.min_horiz_span) ||
                        (config.max_height_over_horiz_span <= vert_to_horiz_span_ratio) || (min_z > config.max_min_z)
                        || (config.min_z_span > z_span)) {
                        include_cluster = false;
                    }
                }
                if (include_cluster) {
                    //LOG(INFO) << "Min/Max z of retained cluster " << min_z << ", " << max_z;
                    filtered_clusters.emplace_back(*it);
                }
            }

            return filtered_clusters;
        }


        int getNumDimensions(const Eigen::Matrix2Xd &points, const double &singular_value_threshold,
                             std::vector<double> &singular_value_ratios) {
            Eigen::Matrix2d scatter_mat;

            Eigen::Matrix2Xd mean_adjusted = points.colwise() - points.rowwise().mean();

            // TODO can we replace this with matrix math?
            double max_horiz_span = 0;
            for (long col_num = 0; col_num < mean_adjusted.cols(); col_num++) {
                scatter_mat += mean_adjusted.col(col_num) * mean_adjusted.col(col_num).transpose();
                for (long col_compare_num = col_num + 1; col_compare_num < mean_adjusted.cols(); col_compare_num++) {
                    max_horiz_span = std::max(max_horiz_span,
                                              (points.col(col_num) - points.col(col_compare_num)).norm());
                }
            }

            Eigen::JacobiSVD<Eigen::Matrix2d> svd(scatter_mat);
            Eigen::Vector2d singular_values = svd.singularValues();
            //LOG(INFO) << "Singular values " << singular_values(0) << ", " << singular_values(1);
            double singular_value_ratio = singular_values(0) / singular_values(1);
            singular_value_ratios.emplace_back(singular_value_ratio);
            //LOG(INFO) << "Singular value ratio " << singular_value_ratio;
            //LOG(INFO) << "Max horiz span " << max_horiz_span;
            if (isnan(singular_value_ratio)) {
                LOG(INFO) << "Scatter mat " << scatter_mat;
                exit(1);
            }

            if (isnan(singular_value_ratio) || (singular_value_ratio > singular_value_threshold)) {
                return 1;
            } else {
                return 2;
            }
        }

        double getLengthAlongLine(const Eigen::VectorXf &line_params, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster,
                                  const std_msgs::Header &header, const int32_t &marker_num, const bool &two_lines) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr line_1_projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> line_1_projecter;
            line_1_projecter.setModelType(pcl::SACMODEL_LINE);
            line_1_projecter.setInputCloud(cloud_cluster);
            pcl::ModelCoefficients::Ptr line_1_coeffs(new pcl::ModelCoefficients());
            line_1_coeffs->values.resize(6);
            std::vector<float> line_1_coeffs_vec(line_params.data(), line_params.data() + 6);
            line_1_coeffs->values = line_1_coeffs_vec;
            line_1_projecter.setModelCoefficients(line_1_coeffs);
            line_1_projecter.filter(*line_1_projected_cloud);

            pcl::PointXYZ first_point_pcl = line_1_projected_cloud->front();
            Eigen::Vector2d first_point(first_point_pcl.x, first_point_pcl.y);
            pcl::PointXYZ last_point = line_1_projected_cloud->back();
            Eigen::Vector2d direction_vec = first_point - Eigen::Vector2d(last_point.x, last_point.y);
            direction_vec = direction_vec / direction_vec.norm();

            double max_projection_val = std::numeric_limits<double>::lowest();
            double min_projection_val = std::numeric_limits<double>::max();

            std::vector<Eigen::Vector2d> projected_points;

            for (size_t i = 1; i < line_1_projected_cloud->size(); i++) {
                pcl::PointXYZ point = (*line_1_projected_cloud)[i];
                Eigen::Vector2d projected_point(point.x, point.y);
                projected_points.emplace_back(projected_point);
                Eigen::Vector2d adjusted_point = projected_point - first_point;
                double projection_val = adjusted_point.dot(direction_vec);
                max_projection_val = std::max(max_projection_val, projection_val);
                min_projection_val = std::min(min_projection_val, projection_val);
            }

            visualizeLines(projected_points, header, marker_num, two_lines);

            return abs(max_projection_val - min_projection_val);
        }

        std::pair<std::vector<Rectangle>, std::vector<pcl::PointIndices>>
        filterWithRectangleFit(const std::vector<pcl::PointIndices> &orig_cluster_indices,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               const file_io::ClusteringConfig &config,
                               const std_msgs::Header &header) {

            clearMarkers(header, line_fit_pub_);
            int marker_num = 0;
            std::vector<pcl::PointIndices> filtered_clusters;
            std::vector<Rectangle> rectangles_for_filtered_clusters; // TODO not doing this for now
            int dimension_1_count = 0;
            int dimension_2_count = 0;
            std::vector<double> singular_value_ratios;
            std::vector<double> inlier_percents;
            for (const pcl::PointIndices &cluster_indices : orig_cluster_indices) {

                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(
                        new pcl::PointCloud<pcl::PointXYZ>);
                size_t num_cluster_points = cluster_indices.indices.size();
                std::vector<Eigen::Vector2d> included_points;
                for (size_t index_num = 0; index_num < num_cluster_points; index_num++) {
                    int idx = cluster_indices.indices[index_num];
                    // Created a colored point from (*cloud)[idx].
                    pcl::PointXYZ point_2d;
                    point_2d.x = (*cloud)[idx].x;
                    point_2d.y = (*cloud)[idx].y;
                    point_2d.z = 0;
                    Eigen::Vector2d point(point_2d.x, point_2d.y);
                    bool any_within_tol = false;
                    for (const Eigen::Vector2d &included_point : included_points) {
                        if ((included_point - point).norm() < config.dedupe_point_for_line_fit_threshold) {
                            any_within_tol = true;
                            break;
                        }
                    }
                    if (!any_within_tol) {
                        cloud_cluster->push_back(point_2d);
                        included_points.emplace_back(point);
                    }
                }
                size_t included_count = included_points.size();
                //LOG(INFO) << "Cloud size before dedupe " << num_cluster_points << ", after dedupe " << included_count;
                Eigen::Matrix2Xd points_for_clust_mat(2, included_count);
                for (size_t point_num = 0; point_num < included_count; point_num++) {
                    points_for_clust_mat.col(point_num) = included_points[point_num];
                }

                cloud_cluster->width = cloud_cluster->size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                Rectangle rectangle;
                std::vector<int> rectangle_inliers_cloud_cluster;
                // Get if the blob is in 2 major directions or 1
                int major_dimensions = getNumDimensions(points_for_clust_mat, config.singular_value_threshold,
                                                        singular_value_ratios);

                //LOG(INFO) << "Major dimensions " << major_dimensions;
                if (major_dimensions == 1) {
                    dimension_1_count++;
                    std::pair<std::vector<int>, Eigen::VectorXf> ransac_fit = fitLineRansac3d(cloud_cluster,
                                                                                              config.ransac_line_distance_threshold,
                                                                                              config.ransac_max_iterations);
                    // If inliers is less than some percent, not a car
//            rectangle_inliers_cloud_cluster = ransac_fit.first;
                    //LOG(INFO) << "Line fit to " << ransac_fit.first.size() << " point with threshold " << config.ransac_line_distance_threshold;
                    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(
                            new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_cluster));
                    std::vector<int> points_within_looser_tol;
                    line_model->selectWithinDistance(ransac_fit.second, config.inlier_threshold_after_fit,
                                                     points_within_looser_tol);

                    //LOG(INFO) << points_within_looser_tol.size();
                    rectangle_inliers_cloud_cluster = points_within_looser_tol;
                    //LOG(INFO) << "Num points within looser dist " << config.inlier_threshold_after_fit << ": " << rectangle_inliers_cloud_cluster.size();
                    // TODO consider adding max deviation for the rest of the points as another threshold
                    double inlier_percent = ((double) rectangle_inliers_cloud_cluster.size()) / included_count;
                    inlier_percents.emplace_back(inlier_percent);
                    // TODO move these back to inside if
                    double length_along_l1 = getLengthAlongLine(ransac_fit.second, cloud_cluster, header, marker_num++,
                                                                false);
                    if (length_along_l1 > config.max_long_side_len) {
                        //LOG(INFO) << "Excluding because max side len was too long " << length_along_l1;
                        continue;
                    }

                    //LOG(INFO) << "Inlier percent " << inlier_percent;
                    if (inlier_percent >= config.min_rect_fit_inlier_percent_threshold) {
                        std::vector<double> distances_to_model;
                        line_model->getDistancesToModel(ransac_fit.second, distances_to_model);
                        double average_dist = 0;
                        for (const double &dist : distances_to_model) {
                            average_dist += dist;
                        }
                        average_dist = average_dist / distances_to_model.size();
                        //LOG(INFO) << "Average dist from straight line " << average_dist;
                        if (average_dist < config.min_avg_deviation_from_straight_line) {
                            //LOG(INFO) << "Average distance to straight line too small";
                            continue;
                        }
//                // TODO set the rectangle here
//                double max_inlier_dist = 0;
//                for (size_t inlier_idx_num_1 = 0; inlier_idx_num_1 < rectangle_inliers_cloud_cluster.size(); inlier_idx_num_1++) {
//                    int inlier_idx_1 = rectangle_inliers_cloud_cluster[inlier_idx_num_1];
//                    Eigen::Vector2d p1 = points_for_clust_mat.col(inlier_idx_1);
//                    for (size_t inlier_idx_num_2 = inlier_idx_num_1 + 1; inlier_idx_num_2 < rectangle_inliers_cloud_cluster.size(); inlier_idx_num_2++) {
//                        int inlier_idx_2 = rectangle_inliers_cloud_cluster[inlier_idx_num_2];
//                        Eigen::Vector2d p2 = points_for_clust_mat.col(inlier_idx_2);
//                        max_inlier_dist = std::max((p1 - p2).norm(), max_inlier_dist);
//                    }
//                }
//                // TODO fill in rectangle details later -- for now just want to get better clusters
//                if (max_inlier_dist > kMaxCarShortSide) {
//                    // This is the long side of the car
//                } else {
//
//                }
                    } else {
                        // Not a car -- keep going
                        //LOG(INFO) << "Excluding because inlier percent was too low " << inlier_percent;
                        continue;
                    }
                } else {
                    dimension_2_count++;
                    // Blob spans 2 dimensions
                    // Fit points to longer side
                    std::pair<std::vector<int>, Eigen::VectorXf> ransac_fit_first_line = fitLineRansac3d(cloud_cluster,
                                                                                                         config.ransac_line_distance_threshold,
                                                                                                         config.ransac_max_iterations);

                    // Fit points to a roughly perpendicular line
                    Eigen::Vector3f axis = ransac_fit_first_line.second.bottomRows(3);
                    std::pair<std::vector<int>, Eigen::VectorXf> ransac_fit_perp_line = fitPerpendicularLineRansac3d(
                            cloud_cluster, config.ransac_line_distance_threshold, config.ransac_max_iterations, axis,
                            config.perp_line_angle_epsilon);

                    bool valid_perp_line = !ransac_fit_perp_line.first.empty();

                    double inlier_percent;
                    if (valid_perp_line) {
                        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(
                                new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_cluster));
                        std::vector<int> points_within_looser_tol_first_line;
                        line_model->selectWithinDistance(ransac_fit_first_line.second,
                                                         config.inlier_threshold_after_fit,
                                                         points_within_looser_tol_first_line);
                        // TODO consider adding max deviation for the rest of the points as another threshold

                        std::vector<int> points_within_looser_tol_second_line;

                        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr perp_line_model(
                                new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_cluster));
                        line_model->selectWithinDistance(ransac_fit_perp_line.second, config.inlier_threshold_after_fit,
                                                         points_within_looser_tol_second_line);

                        std::unordered_set<int> indices_union;
                        std::copy(ransac_fit_first_line.first.begin(), ransac_fit_first_line.first.end(),
                                  std::inserter(indices_union, indices_union.begin()));
                        std::copy(ransac_fit_perp_line.first.begin(), ransac_fit_perp_line.first.end(),
                                  std::inserter(indices_union, indices_union.begin()));
                        //LOG(INFO) << "Perp lines fit to " << indices_union.size() << " point with threshold " << config.ransac_line_distance_threshold;

                        std::unordered_set<int> indices_union_looser_tol;
                        std::copy(points_within_looser_tol_first_line.begin(),
                                  points_within_looser_tol_first_line.end(),
                                  std::inserter(indices_union_looser_tol, indices_union_looser_tol.begin()));
                        std::copy(points_within_looser_tol_second_line.begin(),
                                  points_within_looser_tol_second_line.end(),
                                  std::inserter(indices_union_looser_tol, indices_union_looser_tol.begin()));

                        //LOG(INFO) << "Num points within looser dist " << config.inlier_threshold_after_fit << ": " << rectangle_inliers_cloud_cluster.size();

                        // TODO consider adding max deviation for the rest of the points as another threshold
//            double inlier_percent = ((double) indices_union.size()) / included_count;
                        inlier_percent = ((double) indices_union_looser_tol.size()) / included_count;
                        //LOG(INFO) << "Inlier percent " << inlier_percent;
                        inlier_percents.emplace_back(inlier_percent);

                        // TODO move these back to inside if
                        double length_along_l1 = getLengthAlongLine(ransac_fit_first_line.second, cloud_cluster, header,
                                                                    marker_num++, true);
                        if (length_along_l1 > config.max_long_side_len) {
                            //LOG(INFO) << "Excluding because max side len was too long " << length_along_l1;
                            continue;
                        }

                        double length_along_l2 = getLengthAlongLine(ransac_fit_perp_line.second, cloud_cluster, header,
                                                                    marker_num++, true);
                        if (length_along_l2 > config.max_long_side_len) {
                            //LOG(INFO) << "Excluding because max side len was too long " << length_along_l2;
                            continue;
                        }
                    } else {
                        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(
                                new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_cluster));
                        std::vector<int> points_within_looser_tol;
                        line_model->selectWithinDistance(ransac_fit_first_line.second,
                                                         config.inlier_threshold_after_fit,
                                                         points_within_looser_tol);
                        //LOG(INFO) << points_within_looser_tol.size();
                        rectangle_inliers_cloud_cluster = points_within_looser_tol;
                        //LOG(INFO) << "Num points within looser dist " << config.inlier_threshold_after_fit << ": " << rectangle_inliers_cloud_cluster.size();
                        // TODO consider adding max deviation for the rest of the points as another threshold
                        inlier_percent = ((double) rectangle_inliers_cloud_cluster.size()) / included_count;
                        // TODO move these back to inside if
                        double length_along_l1 = getLengthAlongLine(ransac_fit_first_line.second, cloud_cluster, header,
                                                                    marker_num++, false);
                        if (length_along_l1 > config.max_long_side_len) {
                            //LOG(INFO) << "Excluding because max side len was too long " << length_along_l1;
                            continue;
                        }
                    }

                    if (inlier_percent >= config.min_rect_fit_inlier_percent_threshold) {


                        // YAY a car!
                        // TODO fit rectangle
                    } else {
                        // Not a car

                        //LOG(INFO) << "Excluding because inlier percent was too low " << inlier_percent;
                        continue;
                    }

                }

                filtered_clusters.emplace_back(cluster_indices);
            }
            sort(inlier_percents.begin(), inlier_percents.end());
            std::string inlier_str;
            for (const double &perc : inlier_percents) {
                inlier_str += std::to_string(perc);
                inlier_str += ", ";
            }
            //LOG(INFO) << "Inlier percents " << inlier_str;
            sort(singular_value_ratios.begin(), singular_value_ratios.end());
            std::string svr_str;
            for (const double &ratio : singular_value_ratios) {
                svr_str += std::to_string(ratio);
                svr_str += ", ";
            }
            //LOG(INFO) << "Singular value ratios " << svr_str;
            //LOG(INFO) << "Dim 1 count " << dimension_1_count << ", dim 2 count " << dimension_2_count;
            //LOG(INFO) << "From " << orig_cluster_indices.size() << " kept " << filtered_clusters.size();

            return std::make_pair(rectangles_for_filtered_clusters, filtered_clusters);
        }

        std::vector<pcl::PointIndices> clusterCloud(const std_msgs::Header &orig_cloud_header,
                                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                    const file_io::ClusteringConfig &config,
                                                    const bool &filter_and_merge_clusters = false) {

            visualizeGroundPlane(config.ground_plane_height, config.ground_plane_band_thickness, orig_cloud_header);

            for (auto p = cloud->points.begin(); p != cloud->points.end(); p++) {
                double thickness_threshold = config.ground_plane_band_thickness;
                if (Eigen::Vector2d(p->x, p->y).norm() > config.more_conservative_ground_plane_radius) {
                    thickness_threshold = config.more_conservative_ground_plane_thickness;
                }
                if (fabs(p->z - config.ground_plane_height) < thickness_threshold) {
                    cloud->points.erase(p);
                    p--;
                }
            }
            std::cout << "PointCloud before filtering has: " << cloud->size() << " data points." << std::endl;
            pcl::PointIndices ground_plane_indices;
            ground_plane_indices.header = cloud->header;
            for (int i = 0; i < cloud->points.size(); i++) {
                ground_plane_indices.indices.emplace_back(i);
            }
            std::vector<pcl::PointIndices> ground_plane_indices_vec = {ground_plane_indices};

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

            if (!filter_and_merge_clusters) {
                visualizeClusters(orig_cloud_header, ground_plane_indices_vec, ground_plane_removed_pub_, cloud);
                visualizeClusters(orig_cloud_header, cluster_indices, clustered_pointcloud_pub_, cloud);

                return cluster_indices;
            }

            std::vector<pcl::PointIndices> merged_z_clusters = mergeClustersVertically(cluster_indices, cloud, config);
            std::vector<pcl::PointIndices> filtered_clusters = filterClusters(merged_z_clusters, cloud, config);
            std::pair<std::vector<Rectangle>, std::vector<pcl::PointIndices>> line_filtered =
                    filterWithRectangleFit(filtered_clusters, cloud, config, orig_cloud_header);

            visualizeClusters(orig_cloud_header, ground_plane_indices_vec, ground_plane_removed_pub_, cloud);
            visualizeClusters(orig_cloud_header, cluster_indices, clustered_pointcloud_pub_, cloud);
            visualizeClusters(orig_cloud_header, merged_z_clusters, merged_z_pointcloud_pub_, cloud);
            visualizeClusters(orig_cloud_header, filtered_clusters, filtered_pointcloud_pub_, cloud);
            visualizeClusters(orig_cloud_header, line_filtered.second, line_filtered_pub_, cloud);

            return line_filtered.second;
        }
    };
}

#endif //AUTODIFF_GP_CLUSTERING_H
