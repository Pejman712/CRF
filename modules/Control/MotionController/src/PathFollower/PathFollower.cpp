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

#include "MotionController/PathFollower/PathFollower.hpp"

namespace crf::control::motioncontroller {

PathFollower::PathFollower(
    std::shared_ptr<IRobot> robot,
    std::shared_ptr<IController> controller,
    std::shared_ptr<IJointTrajectoryGenerator> jointTrajGenerator,
    std::shared_ptr<ITaskTrajectoryGenerator> taskTrajGenerator):
    AbstractMotionController(robot, controller),
    robot_(robot),
    controller_(controller),
    jointTrajGenerator_(jointTrajGenerator),
    taskTrajGenerator_(taskTrajGenerator),
    robotConfig_(robot_->getConfiguration()),
    logger_("PathFollower") {
    logger_->debug("CTor");
}

PathFollower::~PathFollower() {
    logger_->debug("DTor");
    AbstractMotionController::deinitialize();
}

bool PathFollower::initialize() {
    logger_->debug("initialize");
    if (!robot_->initialize()) {
        return false;
    }
    crf::expected<crf::utility::types::JointPositions> currentPos = getSignals().joints.positions;
    crf::expected<crf::utility::types::TaskPose> currentPose = getSignals().task.pose;
    if (!currentPos || !currentPose) {
        logger_->error("Could not retrieve current position of the robot");
        return false;
    }
    taskTrajGenerator_->reset();
    jointTrajGenerator_->reset();
    controller_->reset();
    jointTrajGenerator_->setInitialPosition(currentPos.value());
    nlohmann::json initialPositionIK;
    initialPositionIK["UpdateJointsInitialPosition"] = currentPos.value();
    controller_->setParameters(initialPositionIK);
    taskTrajGenerator_->setInitialPose(currentPose.value());
    switchToTaskControl();
    startControlLoop();
    return true;
}

crf::expected<bool> PathFollower::appendPath(const std::vector<JointPositions>& path) {
    logger_->debug("appendJoints");
    crf::actuators::robot::JointLimits limits = robotConfig_->getJointLimits();
    for (auto& pathPoint : path) {
        if (pathPoint.size() != robotConfig_->getJointSpaceDoF()) {
            logger_->error("Joint dimensions are not correct ({} vs {})",
                pathPoint.size(), robotConfig_->getJointSpaceDoF());
            return crf::Code::BadRequest;
        }
        if (!isBetween(limits.minPosition, limits.maxPosition, pathPoint)) {
            logger_->error("Path out of the robot limits");
            return crf::Code::BadRequest;
        }
    }
    switchToJointControl();
    if (isControlLoopActive()) {
        // Already in joints, append path, and release memory if needed
        jointTrajGenerator_->append(path);
        jointTrajGenerator_->clearMemory();
        return true;
    }
    stopControlLoop();

    // Set start point, reset traj and controller, start control loop in joints
    crf::utility::types::JointPositions currentPos = getSignals().joints.positions.value();
    jointTrajGenerator_->reset();
    controller_->reset();
    jointTrajGenerator_->setInitialPosition(currentPos);
    jointTrajGenerator_->append(path);
    startControlLoop();
    return true;
}

crf::expected<bool> PathFollower::appendPath(
    const std::vector<TaskPose>& path,
    const TrajectoryExecutionMethod& method,
    const PointReferenceFrame& frame) {
    logger_->debug("appendTask");

    // If we are in the reference frame of the TCP, we transform all the points.
    std::vector<TaskPose> fullPath(path);
    crf::utility::types::TaskPose currentPose = getSignals().task.pose.value();
    crf::utility::types::JointPositions currentPosition = getSignals().joints.positions.value();
    if (frame == PointReferenceFrame::TCP) {
        for (uint64_t i = 0; i < path.size(); i++) {
            fullPath[i] = multiply(currentPose, path[i]);
        }
    } else if (frame != PointReferenceFrame::Global) {
        return crf::Code::BadRequest;
    }
    if (method == TrajectoryExecutionMethod::JointSpace) {
        // TODO(any): Probably not possible with the new IK
        return crf::Code::NotImplemented;
    }
    switchToTaskControl();
    if (isControlLoopActive()) {
        // Already in task, append path, and release memory if needed
        taskTrajGenerator_->append(fullPath);
        taskTrajGenerator_->clearMemory();
        return true;
    }
    stopControlLoop();

    // Set start point, reset traj and controller, start control loop in task
    taskTrajGenerator_->reset();
    controller_->reset();
    taskTrajGenerator_->setInitialPose(currentPose);
    taskTrajGenerator_->append(fullPath);
    nlohmann::json initialPositionIK;
    initialPositionIK["UpdateJointsInitialPosition"] = currentPosition;
    controller_->setParameters(initialPositionIK);
    startControlLoop();
    return true;
}

crf::expected<bool> PathFollower::setVelocity(const JointVelocities& velocity) {
    logger_->error("Method setVelocity(Joints) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> PathFollower::setVelocity(
    const TaskVelocity& velocity,
    const PointReferenceFrame& frame) {
    logger_->error("Method setVelocity(Task) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> PathFollower::setTorque(const JointForceTorques& torque) {
    logger_->error("Method setTorque(Joints) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> PathFollower::setTorque(
    const TaskForceTorque& torque,
    const PointReferenceFrame& frame) {
    logger_->error("Method setTorque(Task) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> PathFollower::setProfileVelocity(const JointVelocities& velocity) {
    logger_->debug("setProfileVelocity(joint)");
    if (robotConfig_->getJointSpaceDoF() != velocity.size()) return crf::Code::BadRequest;
    jointTrajGenerator_->setProfileVelocity(velocity);
    return true;
}

crf::expected<bool> PathFollower::setProfileVelocity(const TaskVelocity& velocity) {
    logger_->debug("setProfileVelocity(task)");
    taskTrajGenerator_->setProfileVelocity(velocity);
    return crf::Code::NotImplemented;
}

crf::expected<bool> PathFollower::setProfileAcceleration(const JointAccelerations& acceleration) {
    logger_->debug("setProfileAcceleration(joint)");
    if (robotConfig_->getJointSpaceDoF() != acceleration.size()) return crf::Code::BadRequest;
    jointTrajGenerator_->setProfileAcceleration(acceleration);
    return true;
}

crf::expected<bool> PathFollower::setProfileAcceleration(const TaskAcceleration& acceleration) {
    logger_->debug("setProfileAcceleration(task)");
    taskTrajGenerator_->setProfileAcceleration(acceleration);
    return crf::Code::NotImplemented;
}

crf::expected<bool> PathFollower::setParameters(const nlohmann::json& params) {
    return controller_->setParameters(params["Controller"]);
}

nlohmann::json PathFollower::getCurrentParameters() {
    nlohmann::json json;
    json["Controller"] = controller_->getParameters();
    return json;
}

nlohmann::json PathFollower::getParametersDefinition() const {
    nlohmann::json json;
    return json;
}

crf::expected<bool> PathFollower::isTrajectoryRunning() {
    if (taskTrajGenerator_ == nullptr) {
        return jointTrajGenerator_->isTrajectoryRunning();
    }
    return jointTrajGenerator_->isTrajectoryRunning() || taskTrajGenerator_->isTrajectoryRunning();
}

// Private

Signals PathFollower::jointControl() {
    JointSignals ref;
    ref = jointTrajGenerator_->getTrajectoryPoint(trajectoryTimeCounter_*loopTime_.count()/1e6);
    ref = smoothStopScaling(ref);
    return controller_->calculate(ref, robotJointSignals_, robotTaskSignals_);
}

Signals PathFollower::taskControl() {
    TaskSignals ref;
    ref = taskTrajGenerator_->getTrajectoryPoint(trajectoryTimeCounter_*loopTime_.count()/1e6);
    ref = smoothStopScaling(ref);
    return controller_->calculate(ref, robotJointSignals_, robotTaskSignals_);
}

}  // namespace crf::control::motioncontroller
