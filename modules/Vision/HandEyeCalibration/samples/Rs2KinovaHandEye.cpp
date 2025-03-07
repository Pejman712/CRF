/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales & Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <string>

#include "EventLogger/EventLogger.hpp"
#include "KinovaArm/KinovaJaco.hpp"
#include "KinovaArm/KinovaApiInterface.hpp"
#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "Cameras/RealSenseCamera/RealSenseCamera.hpp"
#include "HandEyeCalibration/DataAcquisitionRobotArmRealSense.hpp"
#include "HandEyeCalibration/HandEye.hpp"

int main(int argc, char **argv) {
    crf::utility::logger::EventLogger logger("Rs2KinovaHandEye");

    if (argc != 5) {
        logger->error("The number of arguments is not correct");
        return -1;
    }
    std::ifstream robotData(argv[1]);
    nlohmann::json robotJSON = nlohmann::json::parse(robotData);
    auto arm = std::make_shared<crf::robots::kinovaarm::KinovaJaco>(
        std::make_shared<crf::robots::kinovaarm::KinovaApiInterface>(),
        robotJSON);
    if (!arm->initialize()) {
        logger->error("Failed to initialize the robot arm ");
    }

    auto controller =
        std::make_shared<crf::robots::robotarmcontroller::RobotArmVelocityController>(arm);
    if (!controller->initialize()) {
        logger->error("Failed to initialize the controller ");
    }

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
    if (!posecalc.estimateQRCodePose(0.140, 0.140)) {
        logger->info("Unable to compute QR Code pose");
        return -1;
    }

    if (!posecalc.estimateRobotCameraTransform()) {
        logger->info("Unable to estimate camera transform");
        return -1;
    }

    return 0;
}
