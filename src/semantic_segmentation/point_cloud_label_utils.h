//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_CAMERA_LIDAR_SEMANTIC_LABELER_H
#define AUTODIFF_GP_CAMERA_LIDAR_SEMANTIC_LABELER_H

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <eigen3/Eigen/Dense>

#include <base_lib/pose_reps.h>
#include <unordered_set>

#include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>

namespace semantic_segmentation {

    Eigen::Vector2d getPixelCoordForLidarPoint(const Eigen::Vector3d &point_rel_lidar,
                                               const pose::Pose3d &lidar_pose_rel_camera,
                                               const sensor_msgs::CameraInfoConstPtr &camera_info) {
        Eigen::Vector3d point_rel_camera = pose::transformPoint(lidar_pose_rel_camera, point_rel_lidar);

        Eigen::Matrix3d camera_mat = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(camera_info->K.data());
        Eigen::Vector3d projected_point = camera_mat * point_rel_camera;
        return Eigen::Vector2d(projected_point.x() / projected_point.z(), projected_point.y() / projected_point.z());
    }

    unsigned short getValueForPixel(const cv_bridge::CvImagePtr &image, const Eigen::Vector2d &pixel_coords) {
        return image->image.at<unsigned short>(pixel_coords.x(), pixel_coords.y());
    }

    template<typename PixelValueType>
    void getPixelValueAndNeighboringPixelValues(const cv_bridge::CvImagePtr &segmentation_image,
                                                const Eigen::Vector2d &unrounded_coord,
                                                const std::function<PixelValueType(const cv_bridge::CvImagePtr &,
                                                                                   const Eigen::Vector2d)> &pixel_val_retreiver,
                                                PixelValueType &pixel_value,
                                                std::vector<PixelValueType> &neighboring_values) {

        // Right now, just considering the immediate neighbors (the pixels that it would be if the unrounded pixel
        // value was rounded in another direction)
        Eigen::Vector2d rounded_coord = unrounded_coord.array().round().matrix();
        Eigen::Vector2d ceil_coord = unrounded_coord.array().ceil().matrix();
        Eigen::Vector2d floor_coord = unrounded_coord.array().ceil().matrix();
        Eigen::Vector2d floor_x_ceil_y(floor_coord.x(), ceil_coord.y());
        Eigen::Vector2d ceil_x_floor_y(ceil_coord.x(), floor_coord.y());

        std::vector<Eigen::Vector2d> coords = {ceil_coord, floor_coord, floor_x_ceil_y, ceil_x_floor_y};

        for (const Eigen::Vector2d &coord : coords) {
            if (coord == rounded_coord) {
                pixel_value = pixel_val_retreiver(segmentation_image, coord);
            } else {
                neighboring_values.emplace_back(pixel_val_retreiver(segmentation_image, coord));
            }
        }
    }

    template<typename PixelValueType>
    std::vector<std::pair<PixelValueType, Eigen::Vector3d>>
    getSemanticallyLabeledPointsRelLidar(const sensor_msgs::PointCloud2 &lidar_point_cloud,
                                         const sensor_msgs::ImageConstPtr &segmentation_image,
                                         const sensor_msgs::CameraInfoConstPtr &camera_info,
                                         const pose::Pose3d &camera_pose_rel_lidar,
                                         const std::function<PixelValueType(const cv_bridge::CvImagePtr &,
                                                                            const Eigen::Vector2d)> &pixel_val_retreiver,
                                         const std::unordered_set<PixelValueType> &pixel_labels_to_include) {

        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        pcl::fromROSMsg(lidar_point_cloud, point_cloud);

        cv_bridge::CvImagePtr cv_segmentation_image = cv_bridge::toCvCopy(segmentation_image);

        pose::Pose3d lidar_pose_rel_camera = pose::invertPose(camera_pose_rel_lidar);

        std::vector<std::pair<PixelValueType, Eigen::Vector3d>> semantically_labeled_points;

        for (const pcl::PointXYZI &lidar_point : point_cloud.points) {
            // Convert point to camera frame and project to image coordinates
            Eigen::Vector3d lidar_point_vec(lidar_point.x, lidar_point.y, lidar_point.z);
            Eigen::Vector2d point_in_cam_frame = getPixelCoordForLidarPoint(lidar_point_vec, lidar_pose_rel_camera,
                                                                            camera_info);

            // Check if it is within the image frame (if not, continue to next pixel)
            if ((point_in_cam_frame.x() < 0) || (point_in_cam_frame.x() >= camera_info->width) ||
                (point_in_cam_frame.y() < 0) || (point_in_cam_frame.y() >= camera_info->height)) {
                // Point isn't in the same camera frame
                continue;
            }

            PixelValueType pixel_val;
            std::vector<PixelValueType> neighboring_pixel_vals;
            getPixelValueAndNeighboringPixelValues(cv_segmentation_image, point_in_cam_frame, pixel_val_retreiver,
                                                   pixel_val, neighboring_pixel_vals);

            // Check if the pixel is a of the type to include (if so, continue to the next pixel)
            if (pixel_labels_to_include.find(pixel_val) == pixel_labels_to_include.end()) {
                // Pixel label wasn't one to consider
                continue;
            }

            // In order for a point to be included, we need the neighboring pixels to be the same type (don't want uncertainty due to boundary points)
            bool consistent_with_neighboring_pixels = true;
            for (const uint8_t &neighboring_pixel_val : neighboring_pixel_vals) {
                if (pixel_val != neighboring_pixel_val) {
                    consistent_with_neighboring_pixels = false;
                    break;
                }
            }

            if (consistent_with_neighboring_pixels) {
                // Add the lidar coordinate to the points to include
                semantically_labeled_points.emplace_back(std::make_pair(pixel_val, lidar_point_vec));
            }
        }

        return semantically_labeled_points;
    }
}

#endif //AUTODIFF_GP_CAMERA_LIDAR_SEMANTIC_LABELER_H
