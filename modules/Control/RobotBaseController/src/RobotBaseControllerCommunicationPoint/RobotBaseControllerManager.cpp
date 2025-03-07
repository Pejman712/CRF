/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *         Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>

#include "RobotBaseController/RobotBaseControllerCommunicationPoint/RobotBaseControllerManager.hpp"

namespace crf::control::robotbasecontroller {

RobotBaseControllerManager::RobotBaseControllerManager(
    std::shared_ptr<crf::actuators::robotbase::IRobotBase> robotbase,
    std::shared_ptr<crf::actuators::linearstage::ILinearStage> stage,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    robotbase_(robotbase),
    stage_(stage),
    simpleAccessControl_(controlAccessTimeout),
    controllerMode_(crf::control::robotbasecontroller::ControllerMode::NotDefined),
    initializationTimeout_(initializationTimeout),
    controllerInitialized_(false),
    logger_("RobotBaseControllerManager") {
        logger_->debug("CTor");
}

RobotBaseControllerManager::~RobotBaseControllerManager() {
    logger_->debug("DTor");
    controllerInitialized_ = false;
    initializationCV_.notify_one();
    if (checkLatestRequestThread_.joinable()) {
            checkLatestRequestThread_.join();
    }
    if (controllerMode_ != crf::control::robotbasecontroller::ControllerMode::NotDefined) {
        if (!controller_->deinitialize()) {
            logger_->warn("Failed to deinitialize Controller");
        }
    }
}

bool RobotBaseControllerManager::lockControl(const uint32_t &priority) {
    logger_->debug("lockControl");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("A higher priority communication point holds the access");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    if (controller_ != nullptr) {
        controller_->interruptTrajectory();
    }
    return true;
}

bool RobotBaseControllerManager::unlockControl(const uint32_t& priority) {
    logger_->debug("unlockControl");
    if (controllerMode_ != ControllerMode::NotDefined) {
        controller_->interruptTrajectory();
    }
    std::lock_guard<std::mutex> lock(accessControlMutex_);
    if (!simpleAccessControl_.releaseAccess(priority)) {
        logger_->warn("Failed to remove the priority number from the list");
        return false;
    }
    return true;
}

bool RobotBaseControllerManager::setControllerMode(const uint32_t &priority,
    const ControllerMode& mode) {
    logger_->debug("setControllerMode({})", mode);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (mode != controllerMode_) {
        if (controllerMode_ != ControllerMode::NotDefined) {
            if (!controller_->interruptTrajectory()) {
                logger_->warn("No trajectory or could not interrupt the trajectory in execution");
            }
        }
        if (mode == ControllerMode::Velocity) {
            controller_ = std::make_shared<RobotBaseVelocityController>(robotbase_);
        } else {
            // Add more modes here, if they need parameters overload this function
            logger_->warn("Mode not available");
            return false;
        }
        controllerInitialized_ = false;
        controllerMode_ = mode;
    }
    return true;
}

std::future<bool> RobotBaseControllerManager::setPosition(const int& priority,
    std::vector<crf::utility::types::TaskPose> position) {
    logger_->debug("setPosition");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return std::future<bool>();
    }
    return controller_->setPosition(position);
}

bool RobotBaseControllerManager::setVelocity(const int& priority,
    const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setVelocity(velocity);
}

bool RobotBaseControllerManager::interruptTrajectory(const int& priority) {
    logger_->debug("interruptTrajectory");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->interruptTrajectory();
}

bool RobotBaseControllerManager::setMaximumVelocity(const int& priority,
    const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setMaximumVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setMaximumVelocity(velocity);
}

bool RobotBaseControllerManager::setMaximumAcceleration(const int& priority,
    const crf::utility::types::TaskAcceleration& acceleration) {
    logger_->debug("setMaximumAcceleration");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setMaximumAcceleration(acceleration);
}

bool RobotBaseControllerManager::setStageVelocity(
    const int& priority, const float& velocity) {
    logger_->debug("setStageVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (stage_ == nullptr) {
        logger_->warn("The controller has not been configured with a lifting stage");
        return false;
    }
    return stage_->setTargetVelocity(velocity);
}

bool RobotBaseControllerManager::setStagePosition(
    const int& priority, const float& position) {
    logger_->debug("setStagePosition");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (stage_ == nullptr) {
        logger_->warn("The controller has not been configured with a lifting stage");
        return false;
    }
    return stage_->setTargetPosition(position);
}

nlohmann::json RobotBaseControllerManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);

    nlohmann::json statusJSON;
    int priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;

    lastRequestTime_ = std::chrono::high_resolution_clock::now();

    if (controllerMode_ == crf::control::robotbasecontroller::ControllerMode::NotDefined) {
        logger_->warn("No controller type was declared, by default we use Velocity");
        controller_ = std::make_shared<RobotBaseVelocityController>(robotbase_);
        controllerInitialized_ = false;
        controllerMode_ = crf::control::robotbasecontroller::ControllerMode::Velocity;
    }

    if (!controllerInitialized_) {
        if (!controller_->initialize()) {
            logger_->warn("Failed to initialize controller");
            statusJSON["status"] = "failedToInitialize";
            return statusJSON;
        }
        initializationCV_.notify_one();
        if (checkLatestRequestThread_.joinable()) {
            checkLatestRequestThread_.join();
        }
        controllerInitialized_ = true;
        checkLatestRequestThread_ = std::thread(
            &RobotBaseControllerManager::checkLatestRequestTime, this);
    }
    statusJSON["status"] = "initialized";
    statusJSON["mode"] = controllerMode_;
    statusJSON["position"] = controller_->getPosition();
    statusJSON["velocity"] = controller_->getVelocity();
    if (stage_ != nullptr) {
        boost::optional<float> velOpt = stage_->getActualVelocity();
        if (velOpt) {
            statusJSON["stageVelocity"] = velOpt.get();
        }
        boost::optional<float> posOpt = stage_->getActualPosition();
        if (posOpt) {
            statusJSON["stagePosition"] = posOpt.get();
        }
    }
    return statusJSON;
}

bool RobotBaseControllerManager::checkCommandPriority(const int &priority) {
    logger_->debug("checkCommandPriority");
    if (simpleAccessControl_.getHighestPriority() == 0) {
        logger_->warn("No control requested");
        return false;
    }
    if (static_cast<int>(simpleAccessControl_.getHighestPriority()) != priority) {
        logger_->warn("You don't have the control of the robot arm controller");
        return false;
    }
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("Failed to refresh priority access time");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    return true;
}

void RobotBaseControllerManager::checkLatestRequestTime() {
    while (controllerInitialized_) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto timeSinceRequest = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime-lastRequestTime_);
        if (timeSinceRequest > initializationTimeout_) {
            std::scoped_lock<std::mutex> lock(accessControlMutex_);
            if (!controller_->deinitialize()) {
                logger_->warn("Failed to deinitialize device - Keep trying");
            } else {
                controllerInitialized_ = false;
                logger_->info("Device deinitialized");
            }
        }
        /*
         * To avoid running this thread nonstop consuming resources we put a sleep ten times less
         * that the initializationTimeout_. Like this we ensure that if lastRequestTime_ does not
         * change there will be exactly 10 iterations of this loop before the call to deinitialize.
         * If during the sleep, lastRequestTime_ is updated we would de-synchronize and take longer
         * to deinitialize. That is why if timeSinceRequest_ is smaller than the standard sleep, we
         * instead sleep the time left for timeSinceRequest to reach timeinitializationTimeout_/10,
         * and in the following iterations we continue sleeping the standard time.
         */
        std::unique_lock<std::mutex> initializationLck(initializationMtx_);
        if (timeSinceRequest >= initializationTimeout_/10) {
            initializationCV_.wait_for(initializationLck, initializationTimeout_/10);
        } else {
            initializationCV_.wait_for(initializationLck, initializationTimeout_/10);
        }
    }
}

void RobotBaseControllerManager::checkController() {
    if (controllerMode_ == ControllerMode::NotDefined) {
        logger_->warn("No controller type was declared, by default we use Velocity");
        controller_ = std::make_shared<RobotBaseVelocityController>(robotbase_);
        controllerInitialized_ = false;
        controllerMode_ = ControllerMode::Velocity;
    }
    if (!controller_->initialize()) {
        logger_->warn("Failed to initialize controller");
        return;
    }
}

}  // namespace crf::control::robotbasecontroller
