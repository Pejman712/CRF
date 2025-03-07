/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Authors: Jorge Playán Garai CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

#include "MotionController/AbstractMotionController.hpp"

namespace crf::control::motioncontroller {

AbstractMotionController::AbstractMotionController(
    std::shared_ptr<IRobot> robot,
    std::shared_ptr<IController> controller,
    const std::chrono::milliseconds& commandTimeInterval):
    loopTime_(std::chrono::duration_cast<std::chrono::microseconds>(
        robot->getConfiguration()->getRobotControllerLoopTime())),
    trajectoryTimeCounter_(),
    smoothStop_(false),
    robotJointSignals_(),
    robotTaskSignals_(),
    codes_(),
    robot_(robot),
    controller_(controller),
    commandTimeInterval_(commandTimeInterval),
    robotConfig_(robot_->getConfiguration()),
    controlLoopStopped_(true),
    strictRealTime_(false),
    jointControl_(true),
    logger_("AbstractMotionController") {
        logger_->debug("CTor");

        // For every joint calculate the worst case with Max Acc and Max Vel
        double maxFirstDerviative = 0;
        crf::actuators::robot::JointLimits limits = robotConfig_->getJointLimits();
        for (uint32_t i = 0; i < robotConfig_->getJointSpaceDoF(); i++) {
            // This formula was previously calculated
            double value = 0.75*limits.maxAcceleration[i]/limits.maxVelocity[i];
            if (value > maxFirstDerviative) {
                maxFirstDerviative = value;
            }
        }
        sinusoid_ = std::make_unique<Sinusoid>(1.0, 0.0, maxFirstDerviative,
            ComputationMethod::Limit1stDerivative);

        std::optional<double> rangeOpt = sinusoid_->getRange();
        softStopRange_ = rangeOpt.value();

        // Environmental variable to activate and deactivate strict real-time
        const char* env_ptr = std::getenv("REAL_TIME_STRICT");
        if (env_ptr != NULL) {
            strictRealTime_ = true;
        }
}

AbstractMotionController::~AbstractMotionController() {
    logger_->info("DTor");
    deinitialize();
}

bool AbstractMotionController::initialize() {
    if (!robot_->initialize()) {
        return false;
    }
    startControlLoop();
    return true;
}

bool AbstractMotionController::deinitialize() {
    softStop();
    return robot_->deinitialize();
}

void AbstractMotionController::startJointsControlLoop() {
    switchToJointControl();
    startControlLoop();
    return;
}

void AbstractMotionController::startTaskControlLoop() {
    switchToTaskControl();
    startControlLoop();
    return;
}

void AbstractMotionController::softStop() {
    logger_->debug("softStop");
    if (controlLoopStopped_) return;
    smoothStop_ = true;
    std::unique_lock<std::mutex> lck(softStopMtx_);
    if (stoppedLoop_.wait_for(lck, std::chrono::milliseconds(
        static_cast<int>(softStopRange_ * 1.5))) == std::cv_status::timeout) {  // ms +50%
        logger_->critical("Controller stopped signal not received, triggering hard stop");
        hardStop();
    }
    robot_->softStop();
    stopControlLoop();
    return;
}

void AbstractMotionController::hardStop() {
    stopControlLoop();
    robot_->hardStop();
}

std::set<crf::Code> AbstractMotionController::getStatus() {
    codes_.merge(robot_->robotStatus());
    return codes_;
}

Signals AbstractMotionController::getSignals() {
    Signals result;
    std::unique_lock<std::mutex> lck(robotMtx_);
    if (controlLoopStopped_) {
        result.joints = getRobotJointValues();
        result.task = calculateRobotTaskValues(result.joints);
        return result;
    }
    statusAccess_.wait_for(lck, loopTime_ * 2);
    result.joints = robotJointSignals_;
    result.task = robotTaskSignals_;
    return result;
}

std::shared_ptr<RobotConfiguration> AbstractMotionController::getConfiguration() {
    return robot_->getConfiguration();
}

// Protected

void AbstractMotionController::switchToJointControl() {
    if (jointControl_) return;
    softStop();
    jointControl_ = true;
}

void AbstractMotionController::switchToTaskControl() {
    if (!jointControl_) return;
    softStop();
    jointControl_ = false;
}

bool AbstractMotionController::isControlLoopActive() {
    return !controlLoopStopped_;
}

void AbstractMotionController::startControlLoop() {
    if (!controlLoopStopped_) {
        logger_->warn("The control loop is already running");
        return;
    }
    if (controlLoopThread_.joinable()) {
        controlLoopThread_.join();
    }
    controlLoopStopped_ = false;
    controlLoopThread_ = std::thread(&AbstractMotionController::controlLoop, this);
}

void AbstractMotionController::stopControlLoop() {
    controlLoopStopped_ = true;
    if (!controlLoopThread_.joinable()) return;
    controlLoopThread_.join();
}

void AbstractMotionController::updateLastCommandTime() {
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
}

void AbstractMotionController::commandTimeout() {
    logger_->warn("Last command timeout");
}

// Private

void AbstractMotionController::controlLoop() {
    logger_->debug("controlLoop");

    trajectoryTimeCounter_ = 0;
    smoothStopCounter_ = 0;

    Signals action;

    codes_.clear();
    codes_.insert(crf::Code::OK);

    while (!controlLoopStopped_) {
        std::chrono::high_resolution_clock::time_point loopStart =
            std::chrono::high_resolution_clock::now();

        robotJointSignals_ = getRobotJointValues();
        robotTaskSignals_ = calculateRobotTaskValues(robotJointSignals_);
        statusAccess_.notify_all();

        if (jointControl_) {
            action = jointControl();
        } else {
            action = taskControl();
        }

        if (controller_->checkStopCondition()) {
            smoothStop_ = true;
        }

        crf::expected<bool> result = sendSignalToRobot(action);
        if (!result) {
            codes_.insert(result.get_response().code());
            smoothStop_ = true;
        }

        std::chrono::high_resolution_clock::time_point loopEnd =
            std::chrono::high_resolution_clock::now();

        if (commandTimeInterval_ != std::chrono::milliseconds(0) &&
            std::chrono::duration_cast<std::chrono::milliseconds>(
            loopEnd - lastCommandTime_).count() > commandTimeInterval_.count() &&
            commandSet_) {
            logger_->warn("No command received in less than {}ms", commandTimeInterval_.count());
            commandTimeout();
            commandSet_ = false;
        }

        std::chrono::microseconds loopDuration =
            std::chrono::duration_cast<std::chrono::microseconds>(loopEnd - loopStart);

        if (loopDuration > loopTime_) {
            logger_->critical("Real Time Violation - Loop Duration = {} μs", loopDuration.count());
            codes_.insert(crf::Code::RealTimeViolation);
            if (strictRealTime_) smoothStop_ = true;
        }
        std::this_thread::sleep_until(loopStart + loopTime_);
    }
    stoppedLoop_.notify_all();
    smoothStop_ = false;
}

JointSignals AbstractMotionController::getRobotJointValues() {
    logger_->debug("getRobotJointValues");
    JointSignals robotSignals;
    robotSignals.positions = robot_->getJointPositions();
    robotSignals.velocities = robot_->getJointVelocities();
    robotSignals.accelerations = robot_->getJointAccelerations();
    robotSignals.forceTorques = robot_->getJointForceTorques();
    return robotSignals;
}

TaskSignals AbstractMotionController::calculateRobotTaskValues(const JointSignals& jointValues) {
    logger_->debug("calculateRobotTaskValues");
    TaskSignals robotSignals;
    auto fKinematics = robot_->getConfiguration()->getForwardKinematics();

    // TODO(adiazros): Although the robot's internal kinematic definition may differ from ours, we
    // currently lack a method to independently calculate forces and torques in the task space.
    // Therefore, we rely on the robot's data, despite the potential mismatch.
    robotSignals.forceTorque = robot_->getTaskForceTorque();

    // If the forward kinematics are not available we try to get the values from the robot.
    if (fKinematics == nullptr) {
        robotSignals.pose = robot_->getTaskPose();
        robotSignals.velocity = robot_->getTaskVelocity();
        robotSignals.acceleration = robot_->getTaskAcceleration();
        return robotSignals;
    }

    if (!jointValues.positions) {
        robotSignals.pose = crf::Code::MethodNotAllowed;
        robotSignals.velocity = crf::Code::MethodNotAllowed;
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    std::optional<TaskPose> poseOpt = fKinematics->getPose(
        jointValues.positions.value());

    if (!poseOpt) {
        logger_->error("Forward kinematics failed to return the robot task pose");
        robotSignals.pose = crf::Code::InternalServerError;
        robotSignals.velocity = crf::Code::MethodNotAllowed;
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    robotSignals.pose = poseOpt.value();

    if (!jointValues.velocities) {
        robotSignals.velocity = crf::Code::MethodNotAllowed;
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    std::optional<TaskVelocity> velOpt = fKinematics->getVelocity(
        jointValues.positions.value(),
        jointValues.velocities.value());

    if (!velOpt) {
        logger_->error("Forward kinematics failed to return the robot task velocity");
        robotSignals.velocity = crf::Code::MethodNotAllowed;
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    robotSignals.velocity = velOpt.value();

    if (!jointValues.accelerations) {
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    std::optional<TaskAcceleration> accOpt = fKinematics->getAcceleration(
        jointValues.positions.value(),
        jointValues.velocities.value(),
        jointValues.accelerations.value());

    if (!accOpt) {
        logger_->error("Forward kinematics failed to return the robot task acceleration");
        robotSignals.acceleration = crf::Code::MethodNotAllowed;
        return robotSignals;
    }

    robotSignals.acceleration = accOpt.value();
    return robotSignals;
}

TaskSignals AbstractMotionController::smoothStopScaling(const TaskSignals& reference) {
    logger_->debug("smoothStopScaling(joint)");

    if (!smoothStop_) {
        trajectoryTimeCounter_++;
        return reference;
    }

    SmoothStopSigmas sigmas = calculateSigma();
    trajectoryTimeCounter_ += sigmas.sigma0thDer;

    if (!reference.velocity)  {
        logger_->warn("The soft stop needs at least the velocity derivative");
        return reference;
    }
    TaskSignals smoothStopRef = reference;
    smoothStopRef.velocity = smoothStopRef.velocity.value().raw()*sigmas.sigma0thDer;
    if (!smoothStopRef.acceleration) return smoothStopRef;
    for (uint32_t i = 0; i < robotConfig_->getTaskSpaceDoF(); i++) {
        smoothStopRef.acceleration.value()[i] =
            smoothStopRef.acceleration.value()[i]*sigmas.sigma0thDer*sigmas.sigma0thDer;
        smoothStopRef.acceleration.value()[i] +=
            smoothStopRef.velocity.value()[i]*sigmas.sigma1stDer;
    }
    return smoothStopRef;
}

JointSignals AbstractMotionController::smoothStopScaling(const JointSignals& reference) {
    logger_->debug("smoothStopScaling(task)");
    if (!smoothStop_) {
        trajectoryTimeCounter_++;
        return reference;
    }

    SmoothStopSigmas sigmas = calculateSigma();
    trajectoryTimeCounter_ += sigmas.sigma0thDer;

    if (!reference.velocities)  {
        logger_->warn("The soft stop needs at least the velocity derivative");
        return reference;
    }
    JointSignals smoothStopRef = reference;
    smoothStopRef.velocities = smoothStopRef.velocities.value().raw()*sigmas.sigma0thDer;
    if (!smoothStopRef.accelerations) return smoothStopRef;
    for (uint32_t i = 0; i < robotConfig_->getJointSpaceDoF(); i++) {
        smoothStopRef.accelerations.value()[i] =
            smoothStopRef.accelerations.value()[i]*sigmas.sigma0thDer*sigmas.sigma0thDer;
        smoothStopRef.accelerations.value()[i] +=
            smoothStopRef.velocities.value()[i]*sigmas.sigma1stDer;
    }
    return smoothStopRef;
}

AbstractMotionController::SmoothStopSigmas AbstractMotionController::calculateSigma() {
    SmoothStopSigmas result;
    double smoothStopEvaluationPoint = smoothStopCounter_*loopTime_.count()/1e6;
    if (smoothStopEvaluationPoint > sinusoid_->getRange().value()) {
        controlLoopStopped_ = true;
        logger_->info("Soft Stop finished, control loop stopped");
        result.sigma0thDer = sinusoid_->evaluate(sinusoid_->getRange().value(), 0).value();
        result.sigma1stDer = sinusoid_->evaluate(sinusoid_->getRange().value(), 1).value();
        return result;
    }
    result.sigma0thDer = sinusoid_->evaluate(smoothStopEvaluationPoint, 0).value();
    result.sigma1stDer = sinusoid_->evaluate(smoothStopEvaluationPoint, 1).value();
    smoothStopCounter_++;
    return result;
}

crf::expected<bool> AbstractMotionController::sendSignalToRobot(
    const Signals& input) {
    logger_->debug("sendJointSignalToRobot");
    bool smooth = true;
    crf::expected<bool> result = true;
    if (input.joints.positions) {
        auto aux = robot_->setJointPositions(smooth, input.joints.positions.value());
        if (!aux) result = aux;
    }
    if (input.joints.velocities) {
        auto aux = robot_->setJointVelocities(smooth, input.joints.velocities.value());
        if (!aux) result = aux;
    }
    if (input.joints.forceTorques) {
        auto aux = robot_->setJointForceTorques(smooth, input.joints.forceTorques.value());
        if (!aux) result = aux;
    }
    if (input.task.pose) {
        auto aux = robot_->setTaskPose(smooth, input.task.pose.value());
        if (!aux) result = aux;
    }
    if (input.task.velocity) {
        auto aux = robot_->setTaskVelocity(smooth, input.task.velocity.value());
        if (!aux) result = aux;
    }
    if (input.task.forceTorque) {
        auto aux = robot_->setTaskForceTorque(smooth, input.task.forceTorque.value());
        if (!aux) result = aux;
    }
    return result;
}

}  // namespace crf::control::motioncontroller
