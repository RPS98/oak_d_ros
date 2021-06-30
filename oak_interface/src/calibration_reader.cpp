#include <cstdio>
#include <iostream>
#include <string>

#include <ros/ros.h>

// Inludes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std;

    // ROS
    ros::init(argc, argv, "calibration_reader");

    ros::Time::init();

    ros::NodeHandle nh;

    // Connect Device
    dai::Device device;

    dai::CalibrationHandler calibData = device.readCalibration();
    // calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;

    cout << "Intrinsics from defaultIntrinsics function" << endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Width -> " << width << endl;
    cout << "Height -> " << height << endl;

    cout << "Intrinsics from getCameraIntrinsics function full resolution ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 1280 x 720  ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 1280, 720);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 720 x 450 ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 720);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from getCameraIntrinsics function 300 x 300 ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 300, 300);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Intrinsics from RGB getCameraIntrinsics function 1920 x 1080 ->" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RGB, 1080, 1920);

    for(auto row : intrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    std::vector<std::vector<float>> extrinsics;

    cout << "Extrinsics from left->right test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from right->left test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::LEFT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from right->rgb test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RIGHT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from rgb->right test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::RGB, dai::CameraBoardSocket::RIGHT);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from left->rgb test ->" << endl;
    extrinsics = calibData.getCameraExtrinsics(dai::CameraBoardSocket::LEFT, dai::CameraBoardSocket::RGB);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    // std::vector<float> distorsion = calibData.getDistortionCoefficients(dai::CameraBoardSocket::RGB);
    // ROS_INFO_STREAM("Distorsion -> " )
    /*
    // cout << "Extrinsics from rgb->imu test ->" << endl;
    bool useSpecTranslation = true;
    extrinsics = calibData.getCameraToImuExtrinsics(dai::CameraBoardSocket::RGB, useSpecTranslation);
    
    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from rgb->imu test using useSpecTranslation->" << endl;
    useSpecTranslation = true;
    extrinsics = calibData.getCameraToImuExtrinsics(dai::CameraBoardSocket::RGB, useSpecTranslation);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }


    cout << "Extrinsics from imu->rgb test ->" << endl;
    useSpecTranslation = false;
    extrinsics = calibData.getImuToCameraExtrinsics(dai::CameraBoardSocket::RGB, useSpecTranslation);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }

    cout << "Extrinsics from imu->rgb test using useSpecTranslation->" << endl;
    useSpecTranslation = true;
    extrinsics = calibData.getImuToCameraExtrinsics(dai::CameraBoardSocket::RGB, useSpecTranslation);

    for(auto row : extrinsics) {
        for(auto val : row) cout << val << "  ";
        cout << endl;
    }
    */
    return 0;
}