//
// Created by amanda on 2/13/21.
//

#ifndef AUTODIFF_GP_POINT_CLOUD_H
#define AUTODIFF_GP_POINT_CLOUD_H

#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>


#include <h3d_dataset/h3d_file_operations.h>

namespace h3d {
    void readPointCloudFromPlyFile(const std::string plyfile, pcl::PCLPointCloud2 &cloud) {
        pcl::io::loadPLYFile(plyfile, cloud);
    }

    void readPointCloudForScenario(const std::string &scenario_dir, const int file_num, pcl::PCLPointCloud2 &cloud) {
        std::string ply_filename = scenario_dir;
        ply_filename += kPointCloudFilePrefix;
        ply_filename += convertFileNumToFileStrSuffix(file_num);
        ply_filename += kPointCloudFileExt;

        readPointCloudFromPlyFile(ply_filename, cloud);
    }
}
    /*
    * A point cloud type that has "ring" channel
    */
    struct PointXYZIRFloat
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;
        float ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRFloat,
                                       (float, x, x) (float, y, y)
                                               (float, z, z) (float, intensity, intensity)
                                               (float, ring, ring)
    )

    /*
     * A point cloud type that has "ring" channel
     */
    struct PointXYZIRInt
    {
        PCL_ADD_POINT4D
        PCL_ADD_INTENSITY;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRInt,
                                       (float, x, x) (float, y, y)
                                               (float, z, z) (float, intensity, intensity)
                                               (uint16_t, ring, ring)
    )


#endif //AUTODIFF_GP_POINT_CLOUD_H
