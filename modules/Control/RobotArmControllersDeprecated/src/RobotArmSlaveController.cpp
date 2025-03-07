/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "RobotArmControllersDeprecated/RobotArmSlaveController.hpp"

#include <vector>
#include <memory>
#include <cmath>
#include <chrono>
#include <thread>

namespace crf {
namespace applications {
namespace robotarmcontroller {

RobotArmSlaveController::RobotArmSlaveController(
        std::shared_ptr<robots::robotarm::IRobotArm> arm,
        std::shared_ptr<robots::robotarmkinematics::IRobotArmKinematics> kinematics,
        std::shared_ptr<algorithms::closedloopcontroller::IClosedLoopController> controller) :
        logger_("RobotoArmSlaveController"),
        arm_(arm),
        kinematics_(kinematics),
        positionController_(controller),
        configuration_(arm_->getConfiguration()),
        jointsCount_(configuration_->getNumberOfJoints()),
        rtLoopTime_(std::chrono::duration_cast<std::chrono::microseconds>(
                configuration_->getRTLoopTime())),
        stopControlLoop_(false),
        controlLoopThread_(),
        movementThreshold_(0.001),
        currentJointPositions_(jointsCount_),
        currentJointVelocities_(jointsCount_),
        targetJointPositions_(jointsCount_) {
                logger_->debug("cTor()");
}

RobotArmSlaveController::~RobotArmSlaveController() {
    deinitialize();
    logger_->debug("dTor()");
}

bool RobotArmSlaveController::initialize() {
    logger_->debug("initialize()");
    stopControlLoop_ = false;

    if (controlLoopThread_.joinable()) {
        logger_->warn("Control loop already running");
        return false;
    }

    auto currentJointPositions = arm_->getJointPositions();
    if (!currentJointPositions) {
        logger_->warn("Could not get current joints position");
        return false;
    }
    currentJointPositions_ = currentJointPositions.get();
    targetJointPositions_ = currentJointPositions_;

    controlLoopThread_ = std::thread(&RobotArmSlaveController::controlLoop, this);

    return true;
}

bool RobotArmSlaveController::deinitialize() {
    logger_->debug("deinitialize()");
    if (!controlLoopThread_.joinable()) {
        logger_->warn("Control loop was not running");
        return false;
    }

    stopControlLoop_ = true;
    controlLoopThread_.join();

    if (!arm_->stopArm()) {
        logger_->error("Failed to stop robot arm");
        return false;
    }

    return true;
}

bool RobotArmSlaveController::setJointPositions(const utility::types::JointPositions& jp) {
    if (!controlLoopThread_.joinable()) {
        logger_->error("Controller is not initialized");
        return false;
    }

    if (jp.size() != jointsCount_) {
        logger_->warn("Wrong vector size in setJointPositions parameter");
        return false;
    }

    for (int i=0; i < jointsCount_; i++) {
        float maxPos = configuration_->getJointsConfiguration()[i].maximumPosition;
        float minPos = configuration_->getJointsConfiguration()[i].minimumPosition;
        if ((jp(i) > maxPos) || (jp(i) < minPos)) {
            logger_->warn("Wrong jointPositions {} for joint {}", jp(i), i);
            return false;
        }
    }
    targetJointPositions_ = jp;
    return true;
}

bool RobotArmSlaveController::setTaskPose(const utility::types::TaskPose& cp) {
    std::vector<utility::types::JointPositions> solutions =
            kinematics_->getPositionInverseKinematic(cp, currentJointPositions_);

    if (solutions.empty()) {
        logger_->error("Inverse kinematics solver failed to find a solution!");
        return false;
    }

    utility::types::JointPositions solutionJP(jointsCount_);
    float minDistance = 10e6;
    for (size_t j = 0; j < solutions.size(); j++) {
        float distance = 0;
        crf::utility::types::JointPositions jp(6);
        for (int i = 0; i < jointsCount_; ++i) {
            jp(i) = solutions.at(j)(i);
            distance += pow(jp(i)- currentJointPositions_(i), 2);
        }

        if (distance < minDistance) {
            minDistance = distance;
            solutionJP = jp;
        }
    }

    targetJointPositions_ = solutionJP;
    return true;
}

utility::types::JointPositions RobotArmSlaveController::getJointPositions() {
    return currentJointPositions_;
}

utility::types::TaskPose RobotArmSlaveController::getTaskPose() {
    boost::optional<utility::types::TaskPose> cp =
            kinematics_->getPositionForwardKinematic(currentJointPositions_);
    return cp.get();
}

void RobotArmSlaveController::controlLoop() {
    logger_->debug("controlLoop() started");
    utility::types::JointVelocities nextJointVelocities(jointsCount_);
    while (!stopControlLoop_) {
        auto start = std::chrono::high_resolution_clock::now();
        auto readJointPos = arm_->getJointPositions();

        if (!readJointPos) {
            logger_->warn("controlLoop(): error reading from arm");
            nextJointVelocities = utility::types::JointVelocities(jointsCount_);
        } else {
            currentJointPositions_ = readJointPos.get();
            nextJointVelocities = calculateJointVelocities();
        }

        arm_->setJointVelocities(nextJointVelocities);
        currentJointVelocities_ = nextJointVelocities;

        std::chrono::microseconds elapsed
                = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        } else {
            logger_->warn("controlLoop(): execution time longer than rtLoopTime");
        }
    }
}

utility::types::JointVelocities RobotArmSlaveController::calculateJointVelocities() {
    std::vector<float> currentJP, targetJP;
    for (int i = 0; i < jointsCount_; ++i) {
        currentJP.push_back(currentJointPositions_(i));
        targetJP.push_back(targetJointPositions_(i));
    }

    boost::optional<std::vector<float>> pidOut =
            positionController_->calculate(currentJP, targetJP);
    if (!pidOut) {
        logger_->debug("Controller failed to calculate next velocity");
        return utility::types::JointVelocities(jointsCount_);
    }

    auto positionDifference = currentJointPositions_ - targetJointPositions_;
    auto calculatedJV = utility::types::JointVelocities(jointsCount_);
    for (int i = 0; i < jointsCount_; ++i) {
        if (std::fabs(pidOut.get()[i]) >
        configuration_->getJointsConfiguration()[i].maximumVelocity) {
            calculatedJV(i) =
                -std::copysign(1.0, pidOut.get()[i]) *
                configuration_->getJointsConfiguration()[i].maximumVelocity;
        } else if (std::fabs(positionDifference(i)) < movementThreshold_) {
            calculatedJV(i) = 0;
        } else {
            calculatedJV(i) = -pidOut.get()[i];
        }
    }

    auto velocityDifference = calculatedJV - currentJointVelocities_;
    float timeStep = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>
            (rtLoopTime_).count()/1000.0); // NOLINT
    for (int i=0; i < jointsCount_; i++) {
        float jointMaxAcc = configuration_->getJointsConfiguration()[i].maximumAcceleration;
        float maxPos = configuration_->getJointsConfiguration()[i].maximumPosition;
        float minPos = configuration_->getJointsConfiguration()[i].minimumPosition;

        if (std::fabs(velocityDifference(i)) > jointMaxAcc * timeStep) {
            if (velocityDifference(i) > 0) {
                calculatedJV(i) = currentJointVelocities_(i) + jointMaxAcc  * timeStep;
            } else {
                calculatedJV(i) = currentJointVelocities_(i) - jointMaxAcc  * timeStep;
            }
        }

        if ((calculatedJV(i) > 0) && (currentJointPositions_(i) >= maxPos)) {
            calculatedJV(i) = 0;
        } else if  ((calculatedJV(i) < 0) && (currentJointPositions_(i) <= minPos)) {
            calculatedJV(i) = 0;
        }
    }

    return calculatedJV;
}

////////////////////////////
// NOT IMPLEMENTED FUNCTIONS
std::future<bool> RobotArmSlaveController::executeJointsTrajectory(
        const std::vector<utility::types::JointPositions>&) {
    logger_->error("executeJointsTrajectory() not implemented");
    return std::future<bool>();
}

std::future<bool> RobotArmSlaveController::executeJointsTrajectory(
        const std::shared_ptr<algorithms::trajectorygenerator::IJointsTrajectoryGenerator>&) {
    logger_->error("executeTaskTrajectoryLinear() not implemented");
    return std::future<bool>();
}

std::future<bool> RobotArmSlaveController::executeTaskTrajectory(
        const std::vector<utility::types::TaskPose>&) {
    logger_->error("executeTaskTrajectoryLinear() not implemented");
    return std::future<bool>();
}

std::future<bool> RobotArmSlaveController::executeTaskTrajectoryLinear(
        const std::vector<utility::types::TaskPose>&) {
    logger_->error("executeTaskTrajectoryLinear() not implemented");
    return std::future<bool>();
}

std::future<bool> RobotArmSlaveController::executeTaskTrajectory(
        const std::shared_ptr<algorithms::trajectorygenerator::ITaskTrajectoryGenerator>&) {
    logger_->error("interruptTrajectory() not implemented");
    return std::future<bool>();
}

bool RobotArmSlaveController::interruptTrajectory() {
    logger_->error("interruptTrajectory() not implemented");
    return false;
}

bool RobotArmSlaveController::setJointsMaximumVelocity(
        const utility::types::JointVelocities&) {
    logger_->error("setJointsMaximumVelocity() not implemented");
    return false;
}

bool RobotArmSlaveController::setJointsMaximumAcceleration(
        const utility::types::JointAccelerations&) {
    logger_->error("setJointsMaximumAcceleration() not implemented");
    return false;
}

bool RobotArmSlaveController::setTaskMaximumVelocity(
        const utility::types::TaskVelocity&) {
    logger_->error("setTaskMaximumVelocity() not implemented");
    return false;
}

bool RobotArmSlaveController::setTaskMaximumAcceleration(
        const utility::types::TaskAcceleration&) {
    logger_->error("setTaskMaximumAcceleration() not implemented");
    return false;
}

bool RobotArmSlaveController::setJointVelocities(
        const utility::types::JointVelocities&) {
    logger_->error("setJointVelocities() not implemented");
    return false;
}

bool RobotArmSlaveController::setTaskVelocity(
        const utility::types::TaskVelocity&, bool) {
    logger_->error("setTaskVelocity() not implemented");
    return false;
}

utility::types::JointVelocities RobotArmSlaveController::getJointVelocities() {
    logger_->error("getJointVelocities() not implemented");
    return utility::types::JointVelocities(6);
}

crf::utility::types::JointForceTorques RobotArmSlaveController::getJointForceTorques() {
    logger_->error("getJointForceTorques() not implemented");
    return crf::utility::types::JointForceTorques(6);
}

utility::types::TaskVelocity RobotArmSlaveController::getTaskVelocity() {
    logger_->error("getTaskVelocity() not implemented");
    return utility::types::TaskVelocity();
}

}  // namespace robotarmcontroller
}  // namespace applications
}  // namespace crf
