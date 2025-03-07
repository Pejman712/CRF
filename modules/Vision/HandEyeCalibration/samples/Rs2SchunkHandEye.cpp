/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "CANSocket/CANSocket.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"
#include "ClosedLoopController/PIDController.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "Cameras/RealSenseCamera/RealSenseCamera.hpp"
#include "HandEyeCalibration/DataAcquisitionRobotArmRealSense.hpp"
#include "HandEyeCalibration/HandEye.hpp"

int main(int argc, char **argv) {
    crf::utility::logger::EventLogger logger("Rs2SchunkHandEye");

    if (argc < 6) {
        logger->info("Few arguments provide:");
        logger->info("  [1] Path to Schunk Arm configuration file");
        logger->info("  [2] Path to RGBD Camera config file");
        logger->info("  [3] Path to trajectory Json file");
        logger->info("  [4] Path to Hand Eye configuration file");
        logger->info("  [5] Can Socket interface name");
        return -1;
    }

    // Create Can socket
    auto can_socket = std::make_shared<CANSocket>(argv[5]);

    // Create Schunk Arm
    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::robots::schunkarm::SchunkArm>(
        can_socket, robotJSON);

    if (!arm->initialize()) {
        logger->error("Could not initialize the arm");
        return -1;
    }

    auto controller =
        std::make_shared<crf::robots::robotarmcontroller::RobotArmVelocityController>(arm);

    if (!controller->initialize()) {
        logger->error("Could not initialize the controller");
        return -1;
    }
    logger->info("Controller initialized");

    std::string configFileCamera(argv[2]);
    std::ifstream configCamera(configFileCamera);
    nlohmann::json configCameraJSON;
    configCamera >> configCameraJSON;
    auto camera = std::make_shared<crf::sensors::cameras::RealSenseCamera>(
        configCameraJSON.at("RealSense"));
    if (!camera->initialize()) {
        logger->info("There is a problem to initialize the RGBD Camera");
        return -1;
    }

    crf::applications::handeyecalibration::DataAcquisitionRobotArmRealsense dataAcquisitor(
        controller,
        camera,
        argv[3]);
    if (!dataAcquisitor.acquireCalibrationData()) {
        logger->error("Unable to acquire calibration data");
        return -1;
    }

    crf::applications::handeyecalibration::HandEye posecalc(argv[4]);
    if (!posecalc.estimateQRCodePose(0.1155, 0.1155)) {
        logger->info("Unable to compute QR Code pose");
        return -1;
    }

    if (!posecalc.estimateRobotCameraTransform()) {
        logger->info("Unable to estimate camera transform");
        return -1;
    }

    return 0;
}
