/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *
 *  ==================================================================================================
*/

#include <memory>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>
#include <algorithm>
#include <array>
#include <future>
#include <unistd.h>

#include "MissionManager/RPSurveyLHC/Actions/Actions.hpp"

namespace crf::applications::missionmanager::rpsurveylhc {

Actions::Actions(
    std::shared_ptr<crf::actuators::tim::ITIM> tim,
    std::shared_ptr<crf::utility::missionutility::IDeployableDevice> deployable,
    std::shared_ptr<crf::sensors::rpsensor::IRPSensor> rpSensor,
    const nlohmann::json& configFile):
        tim_(tim),
        deployable_(deployable),
        rpSensor_(rpSensor),
        logger_("Actions"),
        timMovement_(tim_, configFile),
        measurements_(rpSensor_, tim_, deployable_, configFile),
        deviceMovementThread_(),
        stopThreads_(false),
        goingToStart_(false),
        goingToEnd_(false),
        finished_(false),
        initialized_(false),
        surveyParametersSet_(false),
        buffer_(),
        bufferMtx_(),
        startingDCUM_(),
        endingDCUM_(),
        parkingDCUM_(),
        timMovementThreshold_(configFile["Movement"]["TIMPositionThreshold"].get<float>()),
        maxBufferSize_(configFile["MaxBufferSize"].get<uint32_t>()) {
            logger_->debug("CTor");
            if (!initializeClients()) {
                logger_->warn("Failed to initialize clients");
            }
}

Actions::~Actions() {
    logger_->debug("DTor");
    deinitialize();
    deinitializeClients();
}

bool Actions::initialize() {
    logger_->info("initialize");
    if (!surveyParametersSet_) {
        errorLog("Missing survey data, mission can't start");
        return false;
    }
    if (initialized_) {
        errorLog("Already initialized");
        return true;
    }
    if (!measurements_.initialize()) {
        errorLog("Cannot initialize the RP Measures");
        return false;
    }
    clearMission();
    stopThreads_ = false;
    if (!deviceMovementThread_.joinable()) {
        deviceMovementThread_ = std::thread(&Actions::deviceMovement, this);
    }
    initialized_ = true;
    return true;
}

bool Actions::deinitialize() {
    logger_->info("deinitialize Action");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return true;
    }
    stopThreads_ = true;
    std::optional<bool> res = tim_->stop();
    if (!res) errorLog("Cannot stop the TIM");
    else if (!res.value()) errorLog("Cannot stop the TIM");
    if (deviceMovementThread_.joinable()) {
        deviceMovementThread_.join();
    }
    timMovement_.stopObstacleDetection();
    if (!measurements_.deinitialize()) {
        errorLog("Cannot deinitialize the RP Measures");
        return false;
    }
    initialized_ = false;
    clearMission();
    return true;
}

bool Actions::deploy() {
    logger_->info("deploy");
    std::optional<bool> res = tim_->devicesRetracted(false);
    if (!res) return false;
    if (!res.value()) return false;
    return deployable_->deploy();
}

bool Actions::retract() {
    logger_->info("retract");
    if (!deployable_->retract()) {
        return false;
    }
    std::optional<bool> res = tim_->devicesRetracted(true);
    if (!res) return false;
    if (!res.value()) return false;
    return true;
}

bool Actions::goToStart() {
    logger_->info("goToStart");
    if (!surveyParametersSet_) {
        errorLog("Survey parameters not set");
        return false;
    }
    std::optional<float> dcum = tim_->getCurrentPosition();
    if (!dcum) return false;

    goingToStart_ = true;
    if (!deployable_->isRetracted() && std::fabs(dcum.value()-startingDCUM_) > 0.2) {
        if (!retract()) {
            errorLog("Could not retract the devices to start train movement");
            goingToStart_ = false;
            return false;
        }
    }
    logger_->info("Going to start position of the survey (DCUM {})", startingDCUM_);
    if (!timMovement_.moveToDCUM(startingDCUM_)) {
        goingToStart_ = false;
        return false;
    }
    clearMission();
    goingToStart_ = false;
    return true;
}

bool Actions::goToEnd() {
    logger_->info("goToEnd");
    if (!surveyParametersSet_) {
        errorLog("Survey parameters not set");
        return false;
    }
    stopThreads_ = false;
    if (!deviceMovementThread_.joinable()) {
        deviceMovementThread_ = std::thread(&Actions::deviceMovement, this);
    }
    std::optional<float> dcum = tim_->getCurrentPosition();
    if (!dcum) return false;

    goingToEnd_ = true;

    crf::actuators::tim::LHCObstacle obstacle = tim_->getCurrentObstacleArea();
    auto closestObstacles = tim_->getClosestObstacleAreas();
    if (obstacle.isEmpty() &&
        closestObstacles[0].endPosition() + 30 < dcum.value() &&
        closestObstacles[1].startPosition() - 30 > dcum.value()) {
        if (!deploy()) {
            errorLog("Could not deploy the devices to start measurements");
            goingToEnd_ = false;
            return false;
        }
    } else {
        if (!retract()) {
            errorLog("Could not retract the devices to start measurements");
            goingToEnd_ = false;
            return false;
        }
    }

    measurements_.start();
    timMovement_.startObstacleDetection();

    logger_->info("Going to ending survey DCUM {}", endingDCUM_);
    if (!timMovement_.moveToDCUM(endingDCUM_)) {
        timMovement_.stopObstacleDetection();
        goingToEnd_ = false;
        return false;
    }

    measurements_.stop();
    timMovement_.stopObstacleDetection();
    stopThreads_ = true;
    if (deviceMovementThread_.joinable()) {
        deviceMovementThread_.join();
    }
    goingToEnd_ = false;
    finished_ = true;
    return true;
}

bool Actions::goHome() {
    logger_->info("goHome");
    if (isTIMParked()) {
        return true;
    }
    if (!deployable_->isRetracted()) {
        if (!retract()) {
            errorLog("Could not retract the devices to start train movement");
            return false;
        }
    }
    logger_->info("Going to home position in dcum {}", parkingDCUM_);
    if (!timMovement_.moveToDCUM(parkingDCUM_)) {
        return false;
    }
    return isTIMParked();
}

bool Actions::pause() {
    logger_->info("pause");
    std::optional<bool> res = tim_->stop();
    stopThreads_ = true;
    if (deviceMovementThread_.joinable()) {
        deviceMovementThread_.join();
    }
    if (!res) return false;
    if (!res.value()) return false;
    return true;
}

bool Actions::stop() {
    logger_->info("stop");

    std::optional<bool> res = tim_->stop();
    if (!res) return false;
    if (!res.value()) return false;

    measurements_.stop();
    timMovement_.stopObstacleDetection();
    stopThreads_ = true;
    if (deviceMovementThread_.joinable()) {
        deviceMovementThread_.join();
    }
    return true;
}

bool Actions::emergencyStop() {
    logger_->info("emergencyStop");
    if (!deployable_->stop()) {
        errorLog("Cannot assure the arm stopped");
    }
    if (!tim_->emergencyStop()) {
        errorLog("Cannot assure the train stopped");
    }
    return true;
}

nlohmann::json Actions::getStatus() {
    logger_->debug("getStatus");
    nlohmann::json json;
    json["initialized"] = initialized_;
    json["deployed"] = deployable_->isDeployed();
    json["retracted"] = deployable_->isRetracted();
    json["movingArm"] = deployable_->isMoving();
    json["movingTIM"] = timMovement_.isTIMMoving();
    json["enteringObstacle"] = timMovement_.isEnteringObstacle();
    json["exitingObstacle"] = timMovement_.isExitingObstacle();
    json["inObstacle"] = timMovement_.isInObstacle();
    json["finished"] = finished_.load();
    json["goingToStart"] = goingToStart_.load();
    json["goingToEnd"] = goingToEnd_.load();
    json["home"] = isTIMParked();
    json["startingDCUM"] = startingDCUM_;
    json["endingDCUM"] = endingDCUM_;
    json["parkingDCUM"] = parkingDCUM_;
    std::scoped_lock<std::mutex> lck(bufferMtx_);
    json["statusInfo"] = buffer_;
    return json;
}

void Actions::setSurveyParameters(const nlohmann::json& json) {
    startingDCUM_ = json.at("starting_dcum").get<float>();
    endingDCUM_ = json.at("ending_dcum").get<float>();
    parkingDCUM_ = json.at("parking_dcum").get<float>();
    if (startingDCUM_ < 0 || endingDCUM_ < 0 || parkingDCUM_ < 0) {
        return;
    }
    surveyParametersSet_ = true;
    return;
}

// Private Methods

bool Actions::initializeClients() {
    if (!rpSensor_->initialize()) {
        errorLog("Cannot initialize the RP sensor");
        return false;
    }
    if (!tim_->initialize()) {
        errorLog("Cannot initialize the TIM");
        return false;
    }
    if (!deployable_->initialize()) {
        errorLog("Cannot initialize the Deployable device");
        return false;
    }
    return true;
}

bool Actions::deinitializeClients() {
    if (!rpSensor_->deinitialize()) {
        errorLog("Cannot deinitialize the RP sensor");
        return false;
    }
    if (!tim_->deinitialize()) {
        errorLog("Cannot deinitialize the TIM");
        return false;
    }
    if (!deployable_->deinitialize()) {
        errorLog("Cannot deinitialize the Deployable device");
        return false;
    }
    return true;
}

void Actions::clearMission() {
    goingToStart_ = false;
    goingToEnd_ = false;
    finished_ = false;
}

void Actions::deviceMovement() {
    while (!stopThreads_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        if (timMovement_.isEnteringObstacle()) {
            if (timMovement_.needToRetractForNextObstacle()) {
                if (!retract()) {
                    tim_->emergencyStop();
                }
            }
        }
        if (timMovement_.isExitingObstacle()) {
            if (!deploy()) {
                tim_->emergencyStop();
            }
        }
    }
}

bool Actions::isTIMParked() {
    std::optional<float> posOpt = tim_->getCurrentPosition();
    if (!posOpt || !surveyParametersSet_) {
        return false;
    }
    if (std::fabs(parkingDCUM_ - posOpt.value()) < timMovementThreshold_) {
        return true;
    }
    return false;
}

void Actions::errorLog(const std::string& logg) {
    std::scoped_lock<std::mutex> lck(bufferMtx_);
    logger_->error(logg);
    buffer_.push_back(logg);
    if (buffer_.size() > maxBufferSize_) {
        buffer_.pop_front();
    }
    return;
}

}  // namespace crf::applications::missionmanager::rpsurveylhc
