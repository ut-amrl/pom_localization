#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <ctime>

#include <file_io/parking_spot_io.h>
#include <file_io/object_positions_by_pose_io.h>

using namespace pose;

const std::string kParkingSpot3dFileParamName = "parking_spot_3d_file_name";
const std::string kParkingSpot2dFileParamName = "parking_spot_2d_file_name";

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "create_global_obj_locations_from_3d_parking_spots");
    ros::NodeHandle n;

    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string parking_spot_3d_file_name;
    std::string parking_spot_2d_file_name;

    if (!n.getParam(kParkingSpot3dFileParamName, parking_spot_3d_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kParkingSpot3dFileParamName;
        exit(1);
    }

    if (!n.getParam(kParkingSpot2dFileParamName, parking_spot_2d_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kParkingSpot3dFileParamName;
        exit(1);
    }

    std::vector<file_io::ParkingSpot3d> parking_spots_3d;
    file_io::readParkingSpot3dsFromFile(parking_spot_3d_file_name, parking_spots_3d);

    std::vector<file_io::ObjectPositionByPose> object_positions_by_pose;
    for (size_t i = 0; i < parking_spots_3d.size(); i++) {
        file_io::ParkingSpot3d parking_spot_3d = parking_spots_3d[i];
        file_io::ObjectPositionByPose obj_position_with_pose;
        obj_position_with_pose.pose_number_ = 0;
        obj_position_with_pose.identifier_ = i;
        pose::Pose3d pose_3d = std::make_pair(Eigen::Vector3d(parking_spot_3d.transl_x_, parking_spot_3d.transl_y_,
                                                              parking_spot_3d.transl_z_),
                                              Eigen::Quaterniond(parking_spot_3d.rot_w_, parking_spot_3d.rot_x_,
                                                                 parking_spot_3d.rot_y_, parking_spot_3d.rot_z_));
        pose::Pose2d pose_2d = toPose2d(pose_3d);
        obj_position_with_pose.transl_x_ = pose_2d.first.x();
        obj_position_with_pose.transl_y_ = pose_2d.first.y();
        obj_position_with_pose.theta_ = pose_2d.second;

        object_positions_by_pose.emplace_back(obj_position_with_pose);
    }

    file_io::writeObjectPositionsByPoseToFile(parking_spot_2d_file_name, object_positions_by_pose);

    return 0;
}