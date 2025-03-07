/* © Copyright CERN 2020. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN EN/SMM/MRO
 *         Jorge Playán Garai CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <nlohmann/json.hpp>
#include <boost/optional.hpp>

#include "RobotArmController/RobotArmControllerCommunicationPoint/RobotArmControllerManager.hpp"

namespace crf::control::robotarmcontroller {

RobotArmControllerManager::RobotArmControllerManager(
    std::shared_ptr<crf::actuators::robotarm::IRobotArm> robotarm,
    std::shared_ptr<crf::actuators::gripper::IGripper> gripper,
    const std::chrono::milliseconds& initializationTimeout,
    const std::chrono::milliseconds& controlAccessTimeout) :
    robotarm_(robotarm),
    gripper_(gripper),
    controllerMode_(crf::control::robotarmcontroller::ControllerMode::NotDefined),
    simpleAccessControl_(controlAccessTimeout),
    initializationTimeout_(initializationTimeout),
    controllerInitialized_(false),
    logger_("RobotArmControllerManager") {
    logger_->debug("CTor");
}

RobotArmControllerManager::~RobotArmControllerManager() {
    logger_->debug("DTor");
    controllerInitialized_ = false;
    if (checkLatestRequestThread_.joinable()) {
            checkLatestRequestThread_.join();
    }
    if (controllerMode_ != crf::control::robotarmcontroller::ControllerMode::NotDefined) {
        if (!controller_->deinitialize()) {
            logger_->warn("Failed to deinitialize Controller");
        }
    }
}

bool RobotArmControllerManager::lockControl(const uint32_t &priority) {
    logger_->debug("lockControl");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("A higher priority communication point holds the access");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    if (controller_ != nullptr && priority != simpleAccessControl_.getHighestPriority()) {
        controller_->interruptTrajectory();
    }
    return true;
}

bool RobotArmControllerManager::unlockControl(const uint32_t &priority) {
    logger_->debug("unlockControl");
    std::lock_guard<std::mutex> lock(accessControlMutex_);
    if (!simpleAccessControl_.releaseAccess(priority)) {
        logger_->warn("Failed to remove the priority number from the list");
        return false;
    }
    if (controllerMode_ != ControllerMode::NotDefined) {
        controller_->interruptTrajectory();
    }
    return true;
}

bool RobotArmControllerManager::setControllerMode(const uint32_t &priority,
    const crf::control::robotarmcontroller::ControllerMode& mode) {
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
        if (mode == crf::control::robotarmcontroller::ControllerMode::Velocity) {
            controller_ = std::make_unique<RobotArmVelocityController>(robotarm_);
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

std::future<bool> RobotArmControllerManager::setPosition(const int priority,
    std::vector<crf::utility::types::JointPositions> position) {
    logger_->debug("setJointPositions");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return std::future<bool>();
    }
    return controller_->setPosition(position);
}

std::future<bool> RobotArmControllerManager::setPosition(const int priority,
    std::vector<crf::utility::types::TaskPose> position,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setTaskPose");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return std::future<bool>();
    }
    return controller_->setPosition(position, method, frame);
}

bool RobotArmControllerManager::setVelocity(const int priority,
    crf::utility::types::JointVelocities velocity) {
    logger_->debug("setJointVelocities");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setVelocity(velocity);
}

bool RobotArmControllerManager::setVelocity(const int priority,
    crf::utility::types::TaskVelocity velocity,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setTaskVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setVelocity(velocity, method, frame);
}

bool RobotArmControllerManager::setAcceleration(const int priority,
    crf::utility::types::JointAccelerations acceleration) {
    logger_->debug("setJointAccelerations");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setAcceleration(acceleration);
}

bool RobotArmControllerManager::setAcceleration(const int priority,
    crf::utility::types::TaskAcceleration acceleration,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setTaskAcceleration");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setAcceleration(acceleration, method, frame);
}

bool RobotArmControllerManager::interruptTrajectory(const int priority) {
    logger_->debug("interruptTrajectory");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->interruptTrajectory();
}

bool RobotArmControllerManager::setJointsMaximumVelocity(const int priority,
    crf::utility::types::JointVelocities velocity) {
    logger_->debug("setJointsMaximumVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setJointsMaximumVelocity(velocity);
}

bool RobotArmControllerManager::setJointsMaximumAcceleration(const int priority,
    crf::utility::types::JointAccelerations acceleration) {
    logger_->debug("setJointsMaximumAcceleration");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setJointsMaximumAcceleration(acceleration);
}

bool RobotArmControllerManager::setTaskMaximumVelocity(const int priority,
    crf::utility::types::TaskVelocity velocity) {
    logger_->debug("setTaskMaximumVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setTaskMaximumVelocity(velocity);
}

bool RobotArmControllerManager::setTaskMaximumAcceleration(const int priority,
    crf::utility::types::TaskAcceleration acceleration) {
    logger_->debug("setTaskMaximumAcceleration");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    checkController();
    if (!checkCommandPriority(priority)) {
        return false;
    }
    return controller_->setTaskMaximumAcceleration(acceleration);
}

nlohmann::json RobotArmControllerManager::getStatus() {
    logger_->debug("getStatus");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);

    nlohmann::json statusJSON;
    int priorityUnderControl = simpleAccessControl_.getHighestPriority();
    statusJSON["priorityUnderControl"] = priorityUnderControl;

    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    checkController();
    statusJSON["status"] = "initialized";
    statusJSON["jointPositions"] = controller_->getJointPositions();
    statusJSON["jointVelocities"] = controller_->getJointVelocities();
    statusJSON["jointAccelerations"] = controller_->getJointAccelerations();
    statusJSON["jointForceTorques"] = controller_->getJointForceTorques();
    statusJSON["taskPose"] = controller_->getTaskPose();
    statusJSON["taskVelocity"] = controller_->getTaskVelocity();
    statusJSON["taskAcceleration"] = controller_->getTaskAcceleration();
    if (gripper_ != nullptr) {
        boost::optional<float> position = gripper_->getPosition();
        if (position) statusJSON["gripperPosition"] = position.get();
    }
    statusJSON["mode"] = controllerMode_;
    return statusJSON;
}

bool RobotArmControllerManager::setGripperVelocity(const int priority, float velocity) {
    logger_->debug("setGripperVelocity");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (gripper_ == nullptr) {
        logger_->warn("The controller has not been configured with a gripper");
        return false;
    }
    return gripper_->setVelocity(velocity);
}

bool RobotArmControllerManager::setGripperPosition(const int priority, float position) {
    logger_->debug("setGripperPosition");
    std::scoped_lock<std::mutex> lock(accessControlMutex_);
    if (!checkCommandPriority(priority)) {
        return false;
    }
    if (gripper_ == nullptr) {
        logger_->warn("The controller has not been configured with a gripper");
        return false;
    }
    return gripper_->setPosition(position);
}

bool RobotArmControllerManager::checkCommandPriority(const int &priority) {
    logger_->debug("checkCommandPriority");
    if (simpleAccessControl_.getHighestPriority() == 0) {
        logger_->warn("No control requested");
        return false;
    }
    if (static_cast<int>(simpleAccessControl_.getHighestPriority()) < priority) {
        logger_->warn("You ({}) don't have the control of the robot arm controller, user is {}",
            priority,
            simpleAccessControl_.getHighestPriority());
        return false;
    }
    if (!simpleAccessControl_.requestAccess(priority)) {
        logger_->warn("Failed to refresh priority access time");
        return false;
    }
    lastRequestTime_ = std::chrono::high_resolution_clock::now();
    return true;
}

void RobotArmControllerManager::checkLatestRequestTime() {
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
         * instead sleep the time left for timeSinceRequest to reach initializationTimeout_/10,
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

void RobotArmControllerManager::checkController() {
    if (controllerMode_ == crf::control::robotarmcontroller::ControllerMode::NotDefined) {
        logger_->warn("No controller type was declared, by default we use Velocity");
        controller_ = std::make_unique<RobotArmVelocityController>(robotarm_);
        controllerInitialized_ = false;
        controllerMode_ = crf::control::robotarmcontroller::ControllerMode::Velocity;
    }
    if (!controllerInitialized_) {
        if (!controller_->initialize()) {
            logger_->warn("Failed to initialize controller");
            return;
        }
        controllerInitialized_ = true;
    }
}

}  // namespace crf::control::robotarmcontroller
