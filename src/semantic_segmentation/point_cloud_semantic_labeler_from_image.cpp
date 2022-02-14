//
// Created by amanda on 2/5/22.
//

#include <glog/logging.h>

#include <ros/ros.h>
#include <file_io/semantic_point_with_timestamp_io.h>
#include <unordered_set>
#include <semantic_segmentation/point_cloud_label_utils.h>
#include <file_io/pose_3d_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <base_lib/pose_utils.h>

#include <pcl_ros/transforms.h>

DEFINE_string(semantic_points_file,
              "xxx", "Semantic points output file");
DEFINE_string(camera_pose_rel_base_link_files,
              "xxx",
              "Comma separated names of the files that provides the camera pose relative to the base link frame");
DEFINE_double(min_time_between_frames,
              5.0, "Minimum time in seconds between frames to output semantic labels for");
DEFINE_string(camera_topics,
              "/left/color/image_raw", "Comma separated strings (no spaces) for camera topics");
DEFINE_string(camera_info_topics,
              "/left/color/camera_info",
              "Comma separated strings (no spaces) for the camera info topics");
DEFINE_string(odom_topic,
              "", "Odometry topic");
DEFINE_string(lidar_point_cloud_topic,
              "", "Lidar point cloud topic");
DEFINE_string(bag_file_name,
              "", "Name of the ROS bag file");
DEFINE_string(lidar_pose_rel_base_link_file,
              "xxx", "File providing the pose of the lidar in the base link frame");
DEFINE_double(max_time_between_lidar_and_cam,
              2, "Maximum time between a lidar point cloud and a segmentation frame");

struct CameraParams {
    std::string image_topic;

    std::string camera_info_topic;

    pose::Pose3d camera_pose_rel_base_link;
};

class PointCloudSemanticSegmentationProcessor {
public:

    typedef std::pair<size_t, sensor_msgs::Image::ConstPtr> ImageWithCamIndex;

    // Need to change this to take in lidar relative to baselink and cameras relative to baselink
    PointCloudSemanticSegmentationProcessor(const std::string &bag_file_name,
                                            const double &min_time_between_frames,
                                            const pose::Pose3d &lidar_pose_rel_baselink,
                                            const std::vector<CameraParams> &camera_params,
                                            const std::string &odom_topic,
                                            const std::string &point_cloud_topic,
                                            const std::unordered_set<unsigned short> &labels_of_interest,
                                            const double &max_time_between_lidar_and_cam)
            : bag_file_name_(bag_file_name),
              min_time_between_frames_(min_time_between_frames),
              lidar_pose_rel_baselink_(lidar_pose_rel_baselink),
              camera_params_(camera_params),
              odom_topic_(odom_topic),
              point_cloud_topic_(point_cloud_topic),
              labels_of_interest_(labels_of_interest),
              max_time_between_lidar_and_cam_(max_time_between_lidar_and_cam) {}

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> getSemanticallyLabeledPoints() {

        setPointCloudsToLabel();
        extractRelevantImageData();
        extractOdometry();

        std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> points_with_frame_info;
        for (size_t i = 0; i < point_clouds_to_label_.size(); i++) {
            std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> points_for_frame =
                    transformPointCloudAndGetSemanticallyLabeledPointsForScan(point_clouds_to_label_[i],
                                                                              image_for_point_clouds_[i],
                                                                              odometry_around_point_clouds_[i]);

            points_with_frame_info.insert(points_with_frame_info.end(), points_for_frame.begin(),
                                          points_for_frame.end());
        }

        return points_with_frame_info;
    }

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo>
    transformPointCloudAndGetSemanticallyLabeledPointsForScan(const sensor_msgs::PointCloud2ConstPtr &point_cloud,
                                                              const ImageWithCamIndex &image,
                                                              const std::vector<nav_msgs::OdometryConstPtr> &odom) {


        ros::Time image_time = image.second->header.stamp;
        ros::Time point_cloud_time = point_cloud->header.stamp;

        ros::Time first_time = (point_cloud_time < image_time) ? point_cloud_time : image_time;
        ros::Time last_time = (point_cloud_time < image_time) ? image_time : point_cloud_time;

        if ((last_time - first_time) >= ros::Duration(max_time_between_lidar_and_cam_)) {
            LOG(WARNING) << "No image frame near lidar at timestamp " << point_cloud_time;
            return {};
        }

        pose::Pose3d camera_pose_rel_base_link = camera_params_[image.first].camera_pose_rel_base_link;
        sensor_msgs::CameraInfoConstPtr camera_info = camera_infos_[image.first];

        // T^L_c = T^L_b * T^b_c;
        pose::Pose3d camera_pose_rel_lidar = pose::getPoseOfObj1RelToObj2(camera_pose_rel_base_link,
                                                                          lidar_pose_rel_baselink_);
        if (point_cloud_time == image_time) {
            return getSemanticallyLabeledPointsForScan(*point_cloud, camera_info, image.second, camera_pose_rel_lidar);
        }

        // I'm assuming the odometry only measures pose changes in 2D
        std::vector<pose::Pose2d> odom_poses;
        for (const nav_msgs::OdometryConstPtr &msg : odom) {
            pose::Pose3d odom_3d = std::make_pair(
                    Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z),
                    Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                       msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

            pose::Pose2d pose = pose::toPose2d(odom_3d);
            odom_poses.emplace_back(pose);
        }

        pose::Timestamp first_odom_time = std::make_pair(odom[0]->header.stamp.sec,
                                                         odom[0]->header.stamp.nsec);
        pose::Timestamp second_odom_time = std::make_pair(odom[1]->header.stamp.sec,
                                                          odom[1]->header.stamp.nsec);

        size_t odom_count = odom.size();
        pose::Timestamp last_odom_time = std::make_pair(odom[odom_count - 1]->header.stamp.sec,
                                                        odom[odom_count - 1]->header.stamp.nsec);
        pose::Timestamp second_to_last_odom_time = std::make_pair(odom[odom_count - 2]->header.stamp.sec,
                                                                  odom[odom_count -
                                                                       2]->header.stamp.nsec);

        std::pair<pose::Timestamp, pose::Pose2d> first_odom = std::make_pair(first_odom_time, odom_poses[0]);
        std::pair<pose::Timestamp, pose::Pose2d> second_odom = std::make_pair(second_odom_time, odom_poses[1]);

        std::pair<pose::Timestamp, pose::Pose2d> last_odom = std::make_pair(last_odom_time,
                                                                            odom_poses[odom_count - 1]);
        std::pair<pose::Timestamp, pose::Pose2d> second_to_last_odom = std::make_pair(second_to_last_odom_time,
                                                                                      odom_poses[odom_count - 2]);


        pose::Pose2d first_time_odom_pose = pose::interpolatePoses(first_odom, second_odom,
                                                                   std::make_pair(first_time.sec, first_time.nsec));


        pose::Pose2d last_time_odom_pose = pose::interpolatePoses(second_to_last_odom, last_odom,
                                                                  std::make_pair(first_time.sec, first_time.nsec));


        pose::Pose2d last_time_rel_first_pose = pose::getPoseOfObj1RelToObj2(last_time_odom_pose, first_time_odom_pose);

        // Have p_L_l (point in lidar frame in last frame), want p_L_f (point in lidar frame in first frame)
        // T^(L_f)_L_l = T^(lidar)_(base_link) * T^(base_link_first)_(base_link_last) * T^(base_link)_lidar
        Eigen::Affine3d last_time_lidar_rel_first_frame_lidar_eigen =
                pose::convertPoseToAffine(lidar_pose_rel_baselink_).inverse() *
                pose::convertPoseToAffine(pose::toPose3d(last_time_rel_first_pose)) *
                pose::convertPoseToAffine(lidar_pose_rel_baselink_);

        sensor_msgs::PointCloud2 transformed_point_cloud;
        // If last time is lidar, want to transform those points to be in first time frame
        Eigen::Matrix4f transform = last_time_lidar_rel_first_frame_lidar_eigen.matrix().cast<float>();
        if (last_time != point_cloud_time) {
            // If first time is lidar, reverse this
            transform = transform.inverse();
        }
        pcl_ros::transformPointCloud(transform, *point_cloud, transformed_point_cloud);

        return getSemanticallyLabeledPointsForScan(transformed_point_cloud, camera_info, image.second,
                                                   camera_pose_rel_lidar);
    }

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo>
    getSemanticallyLabeledPointsForScan(const sensor_msgs::PointCloud2 &point_cloud,
                                        const sensor_msgs::CameraInfoConstPtr &camera_info,
                                        const sensor_msgs::ImageConstPtr &segmentation_image,
                                        const pose::Pose3d &camera_pose_rel_lidar) { // Also need to include odometry of some sort

        std::vector<std::pair<unsigned short, Eigen::Vector3d>> labeled_points =
                semantic_segmentation::getSemanticallyLabeledPointsRelLidar<unsigned short>(
                        point_cloud, segmentation_image, camera_info, camera_pose_rel_lidar,
                        semantic_segmentation::getValueForPixel, labels_of_interest_);

        // Apply clustering algorithm
        std::vector<std::vector<std::pair<unsigned short, Eigen::Vector3d>>> points_by_cluster; // TODO

        std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantically_labeled_points;

        for (size_t cluster_num = 0; cluster_num < points_by_cluster.size(); cluster_num++) {
            std::vector<std::pair<unsigned short, Eigen::Vector3d>> points_for_cluster = points_by_cluster[cluster_num];
            for (const std::pair<unsigned short, Eigen::Vector3d> &point : points_for_cluster) {
                semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo point_with_info;
                point_with_info.point_x = point.second.x();
                point_with_info.point_y = point.second.y();
                point_with_info.point_z = point.second.z();
                point_with_info.semantic_label = point.first;
                point_with_info.cluster_label = cluster_num;
                point_with_info.seconds = segmentation_image->header.stamp.sec;
                point_with_info.nano_seconds = segmentation_image->header.stamp.nsec;

                semantically_labeled_points.emplace_back(point_with_info);
            }
        }

        return semantically_labeled_points;
    }

    void setPointCloudsToLabel() {

        rosbag::Bag bag;
        bag.open(bag_file_name_, rosbag::bagmode::Read);

        ros::Time last_lidar_stamp;

        std::vector<std::string> topics = {odom_topic_, point_cloud_topic_};
        for (const CameraParams &camera_param : camera_params_) {
            topics.emplace_back(camera_param.camera_info_topic);
        }

        rosbag::View bag_view(bag, rosbag::TopicQuery(topics));
        bool found_first_odom_msg = false;

        for (rosbag::MessageInstance const &m : bag_view) {
            if (m.getTopic() == odom_topic_) {
                if (!found_first_odom_msg) {
                    found_first_odom_msg = true;
                }
                continue;
            } else if (m.getTopic() == point_cloud_topic_) {
                if (found_first_odom_msg) {
                    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

                    if ((msg->header.stamp - last_lidar_stamp) < ros::Duration(min_time_between_frames_)) {
                        continue;
                    }
                    point_clouds_to_label_.emplace_back(msg);
                    last_lidar_stamp = msg->header.stamp;
                } else {

                    for (size_t i = 0; i < camera_params_.size(); i++) {
                        CameraParams camera_param = camera_params_[i];
                        if (m.getTopic() == camera_param.camera_info_topic) {
                            camera_infos_[i] = m.instantiate<sensor_msgs::CameraInfo>();
                            break;
                        }
                    }
                }
            }
        }
        bag.close();
    }

    void extractRelevantImageData() {
        rosbag::Bag bag;
        bag.open(bag_file_name_, rosbag::bagmode::Read);

        std::vector<std::string> topics = {odom_topic_};
        for (size_t i = 0; i < camera_params_.size(); i++) {
            for (const CameraParams &camera_param : camera_params_) {
                if (camera_infos_.find(i) != camera_infos_.end()) {
                    topics.emplace_back(camera_param.image_topic);
                }
            }
        }

        rosbag::View bag_view(bag, rosbag::TopicQuery(topics));
        bool found_first_odom_msg = false;

        ImageWithCamIndex last_image;
        ros::Time last_image_stamp;

        size_t next_point_cloud_to_get_images_for = 0;
        ros::Time next_point_cloud_timestamp = point_clouds_to_label_[next_point_cloud_to_get_images_for]->header.stamp;

        for (rosbag::MessageInstance const &m : bag_view) {
            if (m.getTopic() == odom_topic_) {
                if (!found_first_odom_msg) {
                    found_first_odom_msg = true;
                }
                continue;
            }
            for (size_t i = 0; i < camera_params_.size(); i++) {
                CameraParams camera_param = camera_params_[i];
                if (m.getTopic() == camera_param.image_topic) {
                    if (!found_first_odom_msg) {
                        continue;
                    }
                    sensor_msgs::Image::ConstPtr image_message = m.instantiate<sensor_msgs::Image>();
                    ImageWithCamIndex image_with_index = std::make_pair(i, image_message);
                    if (image_message->header.stamp == next_point_cloud_timestamp) {
                        image_for_point_clouds_.emplace_back(image_with_index);
                        next_point_cloud_to_get_images_for++;
                        next_point_cloud_timestamp = point_clouds_to_label_[next_point_cloud_to_get_images_for]->header.stamp;
                    } else if (image_message->header.stamp > next_point_cloud_timestamp) {
                        ros::Duration time_to_previous_image =
                                next_point_cloud_timestamp - last_image_stamp;
                        ros::Duration time_from_curr_image_to_point_cloud =
                                image_message->header.stamp - next_point_cloud_timestamp;

                        if (time_to_previous_image < time_from_curr_image_to_point_cloud) {
                            image_for_point_clouds_.emplace_back(last_image);
                        } else {
                            image_for_point_clouds_.emplace_back(image_with_index);
                        }
                        next_point_cloud_to_get_images_for++;
                        next_point_cloud_timestamp = point_clouds_to_label_[next_point_cloud_to_get_images_for]->header.stamp;
                    }
                    last_image = image_with_index;
                    last_image_stamp = image_with_index.second->header.stamp;
                    break;
                }
            }

            if (next_point_cloud_to_get_images_for >= point_clouds_to_label_.size()) {
                break;
            }
        }

        while (next_point_cloud_to_get_images_for < point_clouds_to_label_.size()) {
            // Add the last image for the remaining point clouds -- we'll filter out those where the image is too
            // temporally distant later
            image_for_point_clouds_.emplace_back(last_image);
            next_point_cloud_to_get_images_for++;
        }

        bag.close();
    }

    void extractOdometry() {
        rosbag::Bag bag;
        bag.open(bag_file_name_, rosbag::bagmode::Read);

        rosbag::View bag_view(bag, rosbag::TopicQuery({odom_topic_}));

        size_t next_point_cloud = 0;
        std::pair<ros::Time, ros::Time> next_times_to_get_odom_around;
        ros::Time lidar_time = point_clouds_to_label_[next_point_cloud]->header.stamp;
        ros::Time image_time = image_for_point_clouds_[next_point_cloud].second->header.stamp;
        std::vector<nav_msgs::OdometryConstPtr> accumulated;

        do {
            if (lidar_time == image_time) {
                odometry_around_point_clouds_.emplace_back((std::vector<nav_msgs::OdometryConstPtr>) {});
                next_point_cloud++;
                lidar_time = point_clouds_to_label_[next_point_cloud]->header.stamp;
                image_time = image_for_point_clouds_[next_point_cloud].second->header.stamp;
            } else if (lidar_time < image_time) {
                next_times_to_get_odom_around = std::make_pair(lidar_time, image_time);
            } else {
                next_times_to_get_odom_around = std::make_pair(image_time, lidar_time);
            }
        } while (lidar_time == image_time);

        nav_msgs::Odometry::ConstPtr last_odom;
        bool currently_accumulating = false;

        for (rosbag::MessageInstance const &m: bag_view) {
            nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();

            if (currently_accumulating) {
                accumulated.emplace_back(msg);
                if (msg->header.stamp >= next_times_to_get_odom_around.second) {
                    std::vector<nav_msgs::OdometryConstPtr> insert_vec;
                    std::copy(accumulated.begin(), accumulated.end(), std::back_inserter(insert_vec));
                    odometry_around_point_clouds_.emplace_back(insert_vec);
                    currently_accumulating = false;
                    accumulated.clear();
                    next_point_cloud++;

                    lidar_time = point_clouds_to_label_[next_point_cloud]->header.stamp;
                    image_time = image_for_point_clouds_[next_point_cloud].second->header.stamp;

                    do {
                        if (lidar_time == image_time) {
                            odometry_around_point_clouds_.emplace_back(
                                    (std::vector<nav_msgs::OdometryConstPtr>) {});
                            next_point_cloud++;
                            lidar_time = point_clouds_to_label_[next_point_cloud]->header.stamp;
                            image_time = image_for_point_clouds_[next_point_cloud].second->header.stamp;
                        } else if (lidar_time < image_time) {
                            next_times_to_get_odom_around = std::make_pair(lidar_time, image_time);
                        } else {
                            next_times_to_get_odom_around = std::make_pair(image_time, lidar_time);
                        }
                    } while (lidar_time == image_time);
                }
            } else {
                if (msg->header.stamp >= next_times_to_get_odom_around.first) {
                    if (msg->header.stamp > next_times_to_get_odom_around.first) {
                        accumulated.emplace_back(last_odom);
                    }
                    accumulated.emplace_back(msg);
                    currently_accumulating = true;
                }
            }

            last_odom = msg;

            if (next_point_cloud >= point_clouds_to_label_.size()) {
                break;
            }
        }

        while (next_point_cloud < point_clouds_to_label_.size()) {
            // Remove entries that we don't have odom coverage for
            point_clouds_to_label_.pop_back();
            image_for_point_clouds_.pop_back();
        }

        bag.close();
    }

private:

    std::string bag_file_name_;
    double min_time_between_frames_;
    pose::Pose3d lidar_pose_rel_baselink_;
    std::vector<CameraParams> camera_params_;
    std::string odom_topic_;
    std::string point_cloud_topic_;
    std::unordered_set<unsigned short> labels_of_interest_;
    double max_time_between_lidar_and_cam_;

    std::unordered_map<size_t, sensor_msgs::CameraInfo::ConstPtr> camera_infos_;

    std::vector<sensor_msgs::PointCloud2::ConstPtr> point_clouds_to_label_;

    // TODO Should we try to use multiple images to get more coverage/more robust labels?
    std::vector<ImageWithCamIndex> image_for_point_clouds_;

    std::vector<std::vector<nav_msgs::Odometry::ConstPtr>> odometry_around_point_clouds_;


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

    // The goal of this script is to
    //     Get the labels from a semantic segmentation image
    //     Find corresponding points in the lidar
    //     Apply the semantic labels to the lidar points
    //     Group the labeled points into clusters
    //     Output the labeled points to file. Each point should have a timestamp, timestamp-unique cluster id,
    //     3D coordinate, and semantic label

    ros::init(argc, argv,
              "point_cloud_semantic_image_segmentation");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::unordered_set<unsigned short> labels_of_interest = {7};

    if (FLAGS_min_time_between_frames < 0) {
        LOG(ERROR) << "Minimum time between frames needs to be positive";
        exit(1);
    }

    std::vector<std::string> camera_topics = parseCommaSeparatedStrings(FLAGS_camera_topics);
    std::vector<std::string> camera_info_topics = parseCommaSeparatedStrings(FLAGS_camera_info_topics);
    std::vector<std::string> camera_pose_files = parseCommaSeparatedStrings(FLAGS_camera_pose_rel_base_link_files);

    if ((camera_topics.size() != camera_info_topics.size()) || (camera_topics.size() != camera_pose_files.size())) {
        LOG(ERROR) << "Camera topics, camera info topics, and camera relative poses must all be the same size";
        exit(1);
    }

    if (camera_topics.size() < 1) {
        LOG(ERROR) << "There must be at least one camera";
        exit(1);
    }

    std::vector<CameraParams> camera_params;
    for (size_t i = 0; i < camera_topics.size(); i++) {
        CameraParams camera_param_entry;
        camera_param_entry.image_topic = camera_topics[i];
        camera_param_entry.camera_info_topic = camera_info_topics[i];
        std::vector<pose::Pose3d> pose_3ds;
        file_io::readPose3dsFromFile(camera_pose_files[i], pose_3ds);
        camera_param_entry.camera_pose_rel_base_link = pose_3ds.front();
    }

    std::vector<pose::Pose3d> lidar_pose_vec;
    file_io::readPose3dsFromFile(FLAGS_lidar_pose_rel_base_link_file, lidar_pose_vec);
    pose::Pose3d lidar_pose_rel_baselink = lidar_pose_vec.front();

    PointCloudSemanticSegmentationProcessor point_cloud_processor(FLAGS_bag_file_name,
                                                                  FLAGS_min_time_between_frames,
                                                                  lidar_pose_rel_baselink,
                                                                  camera_params,
                                                                  FLAGS_odom_topic,
                                                                  FLAGS_lidar_point_cloud_topic,
                                                                  labels_of_interest,
                                                                  FLAGS_max_time_between_lidar_and_cam);

    std::vector<semantic_segmentation::SemanticallyLabeledPointWithTimestampInfo> semantically_labeled_points =
            point_cloud_processor.getSemanticallyLabeledPoints();


    semantic_segmentation::writeSemanticallyLabeledPointWithTimestampInfosToFile(FLAGS_semantic_points_file,
                                                                                 semantically_labeled_points);
}