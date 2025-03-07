/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Julia Kabalar CERN EN/SMM/MRO 2019
 * Contributor: Jorge Playán Garai CERN BE/CEM/MRO 2021
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <future>
#include <optional>
#include <thread>

#include "RobotBaseController/RobotBaseVelocityController/RobotBaseVelocityController.hpp"
#include "TrajectoryPointGenerator/ReflexxesTrajectoryPointGenerator.hpp"

#define TASK_SPACE_SIZE 6

namespace crf::control::robotbasecontroller {

RobotBaseVelocityController::RobotBaseVelocityController(
  std::shared_ptr<crf::actuators::robotbase::IRobotBase> base) :
    base_(base),
    configuration_(base_->getConfiguration()),
    trajGenerator_(std::make_shared<
        crf::control::trajectorypointgenerator::ReflexxesTrajectoryPointGenerator>(
            crf::control::trajectorypointgenerator::ControlMode::VELOCITY,
            std::vector<bool>({true, true, false, false, false, true}),  // true on x,y,yaw (2D space and rotation on Z) NOLINT
            configuration_->getRTLoopTime()*1e-6)),
    currentState_(),
    targetState_(),
    taskLimit_(),
    stopControlLoop_(false),
    interruptTrajectory_(false),
    baseMoving_(false),
    initialized_(false),
    dimSelectionVec_(TASK_SPACE_SIZE),
    rtLoopTime_(std::chrono::microseconds(configuration_->getRTLoopTime())),
    trajGeneratorMode_(trajGenerator_->getControlMode()),
    currentStateMutex_(),
    targetStateMutex_(),
    logger_("RobotBaseVelocityController") {
    logger_->debug("CTor()");
    taskLimit_.velocity = configuration_->getTaskLimits().maximumVelocity;
    taskLimit_.acceleration = configuration_->getTaskLimits().maximumAcceleration;
}

RobotBaseVelocityController::~RobotBaseVelocityController() {
    logger_->debug("DTor()");
    if (initialized_) deinitialize();
}

bool RobotBaseVelocityController::initialize() {
    logger_->debug("initialize()");
    if (initialized_) return true;
    stopControlLoop_ = false;
    if (controlLoopThread_.joinable()) {
        logger_->warn("Control loop already running");
        return false;
    }
    if (!updateBaseStatus()) {
        logger_->warn("error in reading base status");
        return false;
    }
    if (!updateTrajGenerator(trajGeneratorMode_)) {
        logger_->warn("could not update trajectory generator");
        return false;
    }
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
    controlLoopThread_ = std::thread(&RobotBaseVelocityController::controlLoop, this);
    {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::milliseconds(500));
    }
    initialized_ = true;
    return true;
}

bool RobotBaseVelocityController::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) return true;
    interruptTrajectory_ = true;
    if (!controlLoopThread_.joinable()) {
        logger_->warn("Control loop was not running");
        return false;
    }
    interruptTrajectory();
    stopControlLoop_ = true;
    controlLoopThread_.join();
    initialized_ = false;
    if (!base_->stopBase()) {
        logger_->error("Failed to stop robot base");
        return false;
    }
    return true;
}

std::future<bool> RobotBaseVelocityController::setPosition(
  const crf::utility::types::TaskPose& targetPosition) {
    logger_->debug("setPosition()");
    std::vector<crf::utility::types::TaskPose> positionVector;
    positionVector.push_back(targetPosition);
    // Calling overloaded version with vector as input
    return setPosition(positionVector);
}

std::future<bool> RobotBaseVelocityController::setPosition(
  const std::vector<crf::utility::types::TaskPose>& targetPositions) {
    logger_->debug("setPosition(vector)");
    if (!initialized_) {
        logger_->debug("controller has never been initialized");
        return std::future<bool>();
    }
    if (targetPositions.empty()) {
        logger_->error("Positions vector is empty");
        return std::future<bool>();
    }
    if (!updateTrajGenerator(crf::control::trajectorypointgenerator::ControlMode::POSITION)) {
        logger_->warn("could not update trajectory generator");
        return std::future<bool>();
    }
    //  Here, robot stops between positions (v=0)
    std::vector<utility::types::TaskVelocity> velocities(targetPositions.size());
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
    return std::async(std::launch::async, [this, targetPositions, velocities]() {
        return taskTrajectoryExecution(targetPositions, velocities);
    });
}

bool RobotBaseVelocityController::setVelocity(
    const utility::types::TaskVelocity& targetVelocity) {
    logger_->debug("setVelocity()");
    if (!initialized_) {
        logger_->debug("Controller not initialized");
        return false;
    }
    if (!(targetVelocity[2] == 0 && targetVelocity[3] == 0 && targetVelocity[4] == 0)
        && !(std::isnan(targetVelocity[2]) &&
             std::isnan(targetVelocity[3]) &&
             std::isnan(targetVelocity[4]))) {
        logger_->error("Velocities in Z, Roll, or Yaw not allowed");
        return false;
    }
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
    if (!trajGenerator_->updateVelocityTarget(targetVelocity)) {
        logger_->debug("could not update velocity target");
        return false;
    }
    interruptTrajectory_ = false;
    std::lock_guard<std::mutex> lock(targetStateMutex_);
    targetState_.velocity = targetVelocity;
    return true;
}

bool RobotBaseVelocityController::setVelocity(
  const std::vector<crf::utility::types::TaskVelocity>& targetVelocity) {
    logger_->debug("setVelocity(vector)");
    logger_->error("setVelocity(vector) Not Implemented");
    return false;
}

bool RobotBaseVelocityController::interruptTrajectory() {
    logger_->debug("interruptTrajectory");
    if (!initialized_) {
        logger_->warn("Base controller not initialized");
        return false;
    }
    if (!baseMoving_) {
        logger_->info("Base is not moving");
        return true;
    } else {
        interruptTrajectory_ = true;
        logger_->info("Setting velocity to zero");
        setVelocity(crf::utility::types::TaskVelocity({.0, .0, .0, .0, .0, .0}));
        auto requestT = std::chrono::high_resolution_clock::now();
        while (baseMoving_) {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - requestT).count() > velocityCommandInterval_) {  // NOLINT
                logger_->error("Failed to stop the base in reasonable time: {}ms",
                    velocityCommandInterval_);
                return false;
            }
            std::this_thread::sleep_for(rtLoopTime_);
        }
    }
    return base_->stopBase();
}

crf::utility::types::TaskPose RobotBaseVelocityController::getPosition() {
    logger_->debug("getPosition()");
    std::lock_guard<std::mutex> lock(currentStateMutex_);
    return currentState_.pose;
}

crf::utility::types::TaskVelocity RobotBaseVelocityController::getVelocity() {
    logger_->debug("getVelocity()");
    std::lock_guard<std::mutex> lock(currentStateMutex_);
    return currentState_.velocity;
}

bool RobotBaseVelocityController::setMaximumVelocity(
    const utility::types::TaskVelocity& maximumVelocity) {
    logger_->debug("setMaximumVelocity");
    if (!initialized_) {
        logger_->warn("Base controller not initialized");
        return false;
    }
    for (uint8_t i = 0; i < 6; i++) {
        if (maximumVelocity[i] < 0) return false;
    }
    taskLimit_.velocity = maximumVelocity;
    if (!trajGenerator_->updateMotionConstraints(taskLimit_)) {
        logger_->warn("Could not update motion constraints");
        return false;
    }
    return true;
}

bool RobotBaseVelocityController::setMaximumAcceleration(
    const utility::types::TaskAcceleration& maximumAcceleration) {
    logger_->debug("setMaximumAcceleration");
    if (!initialized_) {
        logger_->warn("Base controller not initialized");
        return false;
    }
    for (uint8_t i = 0; i < 6; i++) {
        if (maximumAcceleration[i] < 0) return false;
    }
    taskLimit_.acceleration = maximumAcceleration;
    if (!trajGenerator_->updateMotionConstraints(taskLimit_)) {
        logger_->warn("Could not update motion constraints");
        return false;
    }
    return true;
}

// Private

bool RobotBaseVelocityController::updateBaseStatus() {
    bool localBaseMoving = false;
    auto updatedPositionValid = base_->getTaskPose();
    auto updatedVelocityValid = base_->getTaskVelocity();
    if (!updatedVelocityValid) {
        logger_->error("estimator cannot retrieve values from base");
        return false;
    }
    utility::types::TaskVelocity updatedVelocity = updatedVelocityValid.get();
    for (int i = 0; i < TASK_SPACE_SIZE; i++) {
        if (fabs(updatedVelocity[i]) > baseMovingVelocityThreshold_) {
            localBaseMoving = true;
        }
    }
    baseMoving_ = localBaseMoving;
    std::lock_guard<std::mutex> lock(currentStateMutex_);
    currentState_.velocity = updatedVelocity;
    return true;
}

void RobotBaseVelocityController::controlLoop() {
    logger_->debug("controlLoop()");
    crf::utility::types::TaskVelocity targetStateVelocity;
    while (!stopControlLoop_) {
        auto start = std::chrono::high_resolution_clock::now();
        bool errorInReading = !updateBaseStatus();

        if (!errorInReading) {
            currentStateMutex_.lock();
            auto localCurrentState = currentState_;
            currentStateMutex_.unlock();

            auto stateUpdated = trajGenerator_->updateCurrentState(localCurrentState);
            if (stateUpdated) {
                auto newTrajPoint = trajGenerator_->getTaskTrajectoryPoint();
                if (!newTrajPoint) {
                    logger_->debug("Could not get trajectory point");
                    errorInReading = true;
                } else {
                    targetStateVelocity = newTrajPoint.get().velocity;
                    //  For the moment we get the current values for position and acceleration from
                    //  trajectory point generator since we do not have respective sensor input
                    currentStateMutex_.lock();
                    currentState_.pose = newTrajPoint.get().pose;
                    currentState_.acceleration = newTrajPoint.get().acceleration;
                    currentStateMutex_.unlock();
                }
            }
        }
        if (errorInReading) {
            logger_->warn("Error in reading, setting velocity to zero");
            targetStateVelocity = {.0, .0, .0, .0, .0, .0};
        }
        if (start - lastCommandTime_ > std::chrono::milliseconds(500)) {
            targetStateVelocity = crf::utility::types::TaskVelocity({.0, .0, .0, .0, .0, .0});
        }
        if (interruptTrajectory_) {
            targetStateVelocity = crf::utility::types::TaskVelocity({.0, .0, .0, .0, .0, .0});
        }

        base_->setTaskVelocity(targetStateVelocity);

        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        } else {
            logger_->warn("controlLoop(): execution time longer than rtLoopTime; {} vs {}",
                rtLoopTime_.count(), elapsed.count());
        }
        cv_.notify_one();
    }
}

bool RobotBaseVelocityController::taskTrajectoryExecution(
    const std::vector<utility::types::TaskPose>& positions,
    const std::vector<utility::types::TaskVelocity>& velocities) {
    logger_->debug("taskTrajectoryExecution()");
    bool reached = false;
    if (positions.size() != velocities.size()) {
        logger_->warn("Position and velocity are not equal length");
        return reached;
    }
    interruptTrajectory_ = false;
    for (uint i = 0; i < positions.size(); i++) {
        if (!trajGenerator_->updatePositionTarget(positions[i])) {
            logger_->debug("could not update target position");
            reached = false;
            return false;
        }
        if (!trajGenerator_->updateVelocityTarget(velocities[i])) {
            logger_->warn("Could not update target velocity");
            return false;
        }
        targetStateMutex_.lock();
        targetState_.pose = positions[i];
        targetStateMutex_.unlock();
        reached = false;
        while (!reached) {
            std::this_thread::sleep_for(rtLoopTime_);
            lastCommandTime_ = std::chrono::high_resolution_clock::now();
            reached = inProximityOfTargetPosition(positions[i]);
            if (interruptTrajectory_) {
                logger_->warn("Trajectory interrupted manually");
                return false;
            }
        }
        logger_->info("inProximityOfTargetPosition");
    }
    return reached;
}

bool RobotBaseVelocityController::updateTrajGenerator(
    const crf::control::trajectorypointgenerator::ControlMode& mode) {
    logger_->debug("updateTrajGenerator()");

    if (trajGeneratorMode_ != mode) {
        trajGenerator_ =
            std::make_shared<control::trajectorypointgenerator::ReflexxesTrajectoryPointGenerator>(
                mode, std::vector<bool>({true, true, false, false, false, true}),
                configuration_->getRTLoopTime()*1e-6);
        trajGeneratorMode_ = mode;
    }
    if (!trajGenerator_->updateMotionConstraints(taskLimit_)) {
        logger_->warn("Could not update motion constraints");
        return false;
    }
    std::lock_guard<std::mutex> lock(currentStateMutex_);
    if (!trajGenerator_->updateCurrentState(currentState_)) {
        logger_->warn("Could not update initial state");
        return false;
    }
    if (!trajGenerator_->updateVelocityTarget(targetState_.velocity)) {
        logger_->warn("Could not update target velocity");
        return false;
    }
    return true;
}

bool RobotBaseVelocityController::inProximityOfTargetPosition(
    const utility::types::TaskPose& position) {
    logger_->debug("inProximityOfTargetPosition()");
    float difference = 0;
    std::lock_guard<std::mutex> lock(currentStateMutex_);
    difference += pow(position.getPosition()(0) - currentState_.pose.getPosition()(0), 2);
    difference += pow(position.getPosition()(1) - currentState_.pose.getPosition()(1), 2);
    difference += pow(position.getCardanXYZ()[2] - currentState_.pose.getCardanXYZ()[2], 2);
    return sqrt(difference) < proximityThreshold_;
}

}  // namespace crf::control::robotbasecontroller
