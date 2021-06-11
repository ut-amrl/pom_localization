//
// Created by amanda on 6/1/21.
//

#ifndef AUTODIFF_GP_PARKING_SPOT_IO_H
#define AUTODIFF_GP_PARKING_SPOT_IO_H

#include <cstdint>
#include <fstream>
#include <sstream>
#include <vector>

namespace file_io {
    struct ParkingSpot3d {
        double transl_x_;
        double transl_y_;
        double transl_z_;
        double rot_x_;
        double rot_y_;
        double rot_z_;
        double rot_w_;
    };

    struct ParkingSpot2d {
        double transl_x_;
        double transl_y_;
        double theta_;
    };

    void readParkingSpot3dLine(const std::string &line_in_file, ParkingSpot3d &parking_spot) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        parking_spot.transl_x_ = data[0];
        parking_spot.transl_y_ = data[1];
        parking_spot.transl_z_ = data[2];

        parking_spot.rot_x_ = data[3];
        parking_spot.rot_y_ = data[4];
        parking_spot.rot_z_ = data[5];
        parking_spot.rot_w_ = data[6];
    }

    void readParkingSpot3dsFromFile(const std::string &file_name, std::vector<ParkingSpot3d> &parking_spots) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            LOG(INFO) << "Line: " << line;

            ParkingSpot3d parking_spot;
            readParkingSpot3dLine(line, parking_spot);
            parking_spots.emplace_back(parking_spot);
        }
    }

    void readParkingSpot2dLine(const std::string &line_in_file, ParkingSpot2d &parking_spot) {
        std::stringstream ss(line_in_file);
        std::vector<double> data;
        while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            data.push_back(std::stod(substr));
        }
        parking_spot.transl_x_ = data[0];
        parking_spot.transl_y_ = data[1];
        parking_spot.theta_ = data[2];
    }

    void readParkingSpot2dsFromFile(const std::string &file_name, std::vector<ParkingSpot2d> &parking_spots) {
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            LOG(INFO) << "Line: " << line;

            ParkingSpot2d parking_spot;
            readParkingSpot2dLine(line, parking_spot);
            parking_spots.emplace_back(parking_spot);
        }
    }
}

#endif //AUTODIFF_GP_TRAJECTORY_2D_IO_H
