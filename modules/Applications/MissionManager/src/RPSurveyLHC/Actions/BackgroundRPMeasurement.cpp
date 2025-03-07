/* © Copyright CERN 2022.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>
#include <chrono>

#include "MissionManager/RPSurveyLHC/Actions/BackgroundRPMeasurement.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

BackgroundRPMeasurement::BackgroundRPMeasurement(
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
    std::shared_ptr<crf::actuators::tim::ITIM> tim,
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployableDevice,
    const nlohmann::json& configFile):
        rpSensor_(rpSensor),
        tim_(tim),
        deployableDevice_(deployableDevice),
        stopThreads_(false),
        measuring_(false),
        logger_("BackgroundRPMeasurement") {
        logger_->debug("CTor");
        pathForResults_ = __FILE__;
        pathForResults_ = pathForResults_.substr(0, pathForResults_.find("cpproboticframework"));
        pathForResults_ += configFile["ResultsPath"].get<std::string>();
}

BackgroundRPMeasurement::~BackgroundRPMeasurement() {
    logger_->debug("DTor");
    deinitialize();
}

/*
 * This class runs on the premise that rpSensor, tim, and deployableArm are already initialized
 * (jplayang)
 */

bool BackgroundRPMeasurement::initialize() {
    logger_->debug("initialize");
    stopThreads_ = false;
    if (!measurementThread_.joinable()) {
        measurementThread_ = std::thread(&BackgroundRPMeasurement::execute, this);
    }
    return true;
}

bool BackgroundRPMeasurement::deinitialize() {
    logger_->debug("deinitialize");
    stopThreads_ = true;
    measuring_ = false;
    cv_.notify_one();
    if (measurementThread_.joinable()) {
        measurementThread_.join();
    }
    return true;
}

void BackgroundRPMeasurement::start() {
    logger_->debug("start");
    measuring_ = true;
    cv_.notify_one();
}

void BackgroundRPMeasurement::stop() {
    logger_->debug("stop");
    measuring_ = false;
}

// Private Methods

void BackgroundRPMeasurement::execute() {
    while (!stopThreads_) {
        std::mutex mtx;
        std::unique_lock<std::mutex> lck(mtx);
        cv_.wait(lck);

        std::string newFileName;
        char hostname[HOST_NAME_MAX];
        gethostname(hostname, HOST_NAME_MAX);

        char buffer[32];
        std::time_t now = std::time(NULL);
        std::strftime(buffer, 32, "_%d-%m-%Y_%H-%M-%S", std::localtime(&now));

        newFileName = pathForResults_ + hostname + "_RPSurvey" + buffer + ".csv";
        std::ofstream doseFile(newFileName);

        while (measuring_) {
            // At max speed (2m/s) for every 5 cm we need to measure every 25ms.
            // TODO(jplayang): Measure at a varianle frequency depending on TIMs velocity.
            std::this_thread::sleep_for(std::chrono::milliseconds(25));

            std::optional<float> doseOpt = rpSensor_->getDoseRate();
            std::optional<float> posOpt = tim_->getCurrentPosition();
            std::optional<float> vepOpt = tim_->getCurrentVelocity();
            if (!doseOpt || !posOpt || !vepOpt) continue;
            float dose = doseOpt.value();
            float pos = posOpt.value();
            float vel = vepOpt.value();

            int armPos = 0;
            if (deployableDevice_->isRetracted()) armPos = 1;
            else if (deployableDevice_->isMoving()) armPos = 2;
            else if (deployableDevice_->isDeployed()) armPos = 3;

            auto time = std::chrono::system_clock::now().time_since_epoch().count();

            doseFile << pos << "," << vel << "," << armPos << "," << dose << "," << time << "\n";
        }
        doseFile.close();
    }
}

}  // namespace crf::applications::missionmanager::rpsurveylhc
