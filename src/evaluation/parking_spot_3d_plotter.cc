#include <glog/logging.h>

#include <visualization/ros_visualization.h>

#include <ros/ros.h>

#include <iostream>
#include <iomanip>
#include <ctime>

#include <file_io/parking_spot_io.h>

using namespace pose;

const std::string kParkingSpotFileParamName = "parking_spot_file_name";

const double kParkingSpotX = 4;
const double kParkingSpotY = 2;

int main(int argc, char **argv) {
    ros::init(argc, argv,
              "parking_spot_3d_plotter");
    ros::NodeHandle n;


    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    std::string parking_spot_file_name;

    LOG(INFO) << "HERE!!!";

    if (!n.getParam(kParkingSpotFileParamName, parking_spot_file_name)) {
        LOG(INFO) << "No parameter value set for parameter with name " << kParkingSpotFileParamName;
        exit(1);
    }


    std::shared_ptr<visualization::VisualizationManager> manager = std::make_shared<visualization::VisualizationManager>(
            n);


    LOG(INFO) << "HERE3!!!";

    std::vector<file_io::ParkingSpot3d> parking_spots;
    file_io::readParkingSpot3dsFromFile(parking_spot_file_name, parking_spots);


    LOG(INFO) << "HERE4!!!";
    std::vector<pose::Pose3d> parking_spot_poses;
    for (const file_io::ParkingSpot3d &parking_spot : parking_spots) {
        parking_spot_poses.emplace_back(std::make_pair(Eigen::Vector3d(parking_spot.transl_x_, parking_spot.transl_y_,
                                                                       parking_spot.transl_z_),
                                                       Eigen::Quaterniond(parking_spot.rot_w_, parking_spot.rot_x_,
                                                                          parking_spot.rot_y_, parking_spot.rot_z_)));
    }


    LOG(INFO) << "HERE5!!!";

    manager->displayParkingSpots(parking_spot_poses, kParkingSpotX, kParkingSpotY);

    for (int i = 0; i < 20; i++) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    LOG(INFO) << "HERE2!!!";

//    auto t = std::time(nullptr);
//    auto tm = *std::localtime(&t);
//    std::ostringstream oss;;
//    oss << std::put_time(&tm, "%d-%m-%Y_%H:%M:%S");
//    std::string time_str = oss.str();
//    std::string csv_file_name = "results/noise_eval_" + time_str + ".csv";

    return 0;
}