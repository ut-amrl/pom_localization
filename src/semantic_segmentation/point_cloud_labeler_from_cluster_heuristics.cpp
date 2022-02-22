//
// Created by amanda on 2/5/22.
//

#include <glog/logging.h>

#include <ros/ros.h>
#include <file_io/semantic_point_with_timestamp_io.h>
#include <semantic_segmentation/point_cloud_label_utils.h>

// Fix for conflicting LZ4 definitions in PCL vs rosbag from https://github.com/ethz-asl/lidar_align/issues/16
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated

#include <rosbag/bag.h>

#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue

#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <base_lib/pose_utils.h>
#include <semantic_segmentation/clustering.h>
#include <file_io/clustering_config_io.h>


DEFINE_string(param_prefix, "", "param_prefix");

DEFINE_string(semantic_points_file,
              "", "Semantic points output file");
DEFINE_string(lidar_point_cloud_topic,
              "/velodyne_points", "Lidar point cloud topic");
DEFINE_string(bag_file_name,
              "", "Name of the ROS bag file");
DEFINE_string(cluster_config_file, "", "Clustering config file");
DEFINE_string(odom_topic, "/jackal_velocity_controller/odom", "Odometry topic");

class PointCloudClusterProcessor {
public:

    // Need to change this to take in lidar relative to baselink and cameras relative to baselink
    PointCloudClusterProcessor(
            const file_io::ClusteringConfig &config,
            const std::string &bag_file_name,
            const std::string &odom_topic,
            const std::string &point_cloud_topic,
            const std::string &node_prefix,
            const unsigned short class_index,
            ros::NodeHandle &node_handle)
            : bag_file_name_(bag_file_name),
              min_time_between_frames_(config.time_between_frames),
              odom_topic_(odom_topic),
              point_cloud_topic_(point_cloud_topic),
              clusterer_(config, node_handle, node_prefix),
              class_index_(class_index) {}

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> getSemanticallyLabeledPoints() {

        rosbag::Bag bag;
        bag.open(bag_file_name_, rosbag::bagmode::Read);

        ros::Time last_included_lidar_stamp;

        std::vector<sensor_msgs::PointCloud2::ConstPtr> point_clouds_to_label;

        std::vector<std::string> topics = {point_cloud_topic_};

        rosbag::View bag_view(bag, rosbag::TopicQuery(topics));
        std::pair<ros::Time, ros::Time> min_and_max_odom_stamps = getMinAndMaxOdomStamps();

        sensor_msgs::PointCloud2::ConstPtr cloud_before_last_odom;
        bool found_first_cloud = false;
        ros::Time last_lidar_stamp;

        for (rosbag::MessageInstance const &m : bag_view) {
            sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
            ros::Time curr_stamp = msg->header.stamp;
            if ((curr_stamp < min_and_max_odom_stamps.first) || (curr_stamp > min_and_max_odom_stamps.second)) {
                continue;
            }
            if (!found_first_cloud) {
                point_clouds_to_label.emplace_back(msg);
                cloud_before_last_odom = msg;
                last_lidar_stamp = curr_stamp;
                last_included_lidar_stamp = curr_stamp;
                found_first_cloud = true;
            } else {
                if (last_lidar_stamp > curr_stamp) {
                    LOG(ERROR) << "Out of order point clouds";
                }
                if (cloud_before_last_odom->header.stamp < curr_stamp) {
                    cloud_before_last_odom = msg;
                }

                if ((curr_stamp - last_included_lidar_stamp) >= ros::Duration(min_time_between_frames_)) {
                    point_clouds_to_label.emplace_back(msg);
                    last_included_lidar_stamp = msg->header.stamp;
                }
                last_lidar_stamp = curr_stamp;
            }
        }
        bag.close();

        if (point_clouds_to_label.back()->header.stamp < cloud_before_last_odom->header.stamp) {
            point_clouds_to_label.emplace_back(cloud_before_last_odom);
        }

        std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> points_with_frame_info;
        for (size_t i = 0; i < point_clouds_to_label.size(); i++) {
            std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> points_for_frame =
                    getSemanticallyLabeledPointsForScan(*(point_clouds_to_label[i]));

            points_with_frame_info.insert(points_with_frame_info.end(), points_for_frame.begin(),
                                          points_for_frame.end());
        }

        return points_with_frame_info;
    }

    std::pair<ros::Time, ros::Time> getMinAndMaxOdomStamps() {

        rosbag::Bag bag;
        bag.open(bag_file_name_, rosbag::bagmode::Read);

        std::vector<std::string> topics = {odom_topic_};

        rosbag::View bag_view(bag, rosbag::TopicQuery(topics));
        bool found_first_odom_msg = false;

        ros::Time last_odom_stamp;
        ros::Time first_odom_stamp;


        for (rosbag::MessageInstance const &m : bag_view) {
            nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();

            ros::Time curr_stamp = msg->header.stamp;
            if (!found_first_odom_msg) {

                found_first_odom_msg = true;
                last_odom_stamp = curr_stamp;
                first_odom_stamp = curr_stamp;
                continue;
            }

            if (curr_stamp < first_odom_stamp) {
                first_odom_stamp = curr_stamp;
            }
            if (curr_stamp > last_odom_stamp) {
                last_odom_stamp = curr_stamp;
            }
        }
        bag.close();
        return std::make_pair(first_odom_stamp, last_odom_stamp);
    }

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo>
    getSemanticallyLabeledPointsForScan(const sensor_msgs::PointCloud2 &point_cloud) {

        std::vector<std::vector<Eigen::Vector3d>> clusters = clusterer_.clusterPoints(point_cloud, true);

        std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantically_labeled_points;

        for (size_t cluster_num = 0; cluster_num < clusters.size(); cluster_num++) {
            for (const Eigen::Vector3d &point : clusters[cluster_num]) {
                semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo point_with_info;
                point_with_info.point_x = point.x();
                point_with_info.point_y = point.y();
                point_with_info.point_z = point.z();
                point_with_info.semantic_label = class_index_;
                point_with_info.cluster_label = cluster_num;
                point_with_info.seconds = point_cloud.header.stamp.sec;
                point_with_info.nano_seconds = point_cloud.header.stamp.nsec;

                semantically_labeled_points.emplace_back(point_with_info);
            }
        }

        return semantically_labeled_points;
    }

private:

    std::string bag_file_name_;
    double min_time_between_frames_;
    std::string odom_topic_;
    std::string point_cloud_topic_;

    semantic_segmentation::Clusterer clusterer_;

    unsigned short class_index_;
};

std::vector<std::string> parseCommaSeparatedStrings(const std::string &comma_separated_string) {
    std::stringstream ss(comma_separated_string);
    std::vector<std::string> parsed_strings;

    while (ss.good()) {
        std::string substr;
        getline(ss, substr, ',');
        parsed_strings.push_back(substr);
    }

    return parsed_strings;
}

int main(int argc, char **argv) {

    google::ParseCommandLineFlags(&argc, &argv, false);

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string param_prefix = FLAGS_param_prefix;
    std::string node_prefix = FLAGS_param_prefix;
    if (!param_prefix.empty()) {
        param_prefix = "/" + param_prefix + "/";
        node_prefix += "_";
    }
    LOG(INFO) << "Prefix: " << param_prefix;

    ros::init(argc, argv,
              node_prefix + "point_cloud_labeler_from_cluster_heuristics");
    ros::NodeHandle n;

    if (FLAGS_bag_file_name.empty()) {
        LOG(ERROR) << "No bag file specified";
        exit(1);
    }

    if (FLAGS_cluster_config_file.empty()) {
        LOG(ERROR) << "No cluster config file specified";
        exit(1);
    }

    if (FLAGS_semantic_points_file.empty()) {
        LOG(ERROR) << "No ouptut file specified";
        exit(1);
    }

    std::string clustering_config_file_name = FLAGS_cluster_config_file;

    LOG(INFO) << "Reading clustering config from file " << clustering_config_file_name;
    file_io::ClusteringConfig clustering_config;
    file_io::readClusteringConfigFromFile(clustering_config_file_name, clustering_config);

    PointCloudClusterProcessor point_cloud_processor(
            clustering_config,
            FLAGS_bag_file_name,
            FLAGS_odom_topic,
            FLAGS_lidar_point_cloud_topic,
            param_prefix,
            10, n);

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantically_labeled_points =
            point_cloud_processor.getSemanticallyLabeledPoints();

    semantic_segmentation::writeSemanticallyLabeledPointWithTimestampInfosToFile(FLAGS_semantic_points_file,
                                                                                 semantically_labeled_points);
}