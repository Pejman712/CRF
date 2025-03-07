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

#include "MotionController/Teleoperation/Teleoperation.hpp"

namespace crf::control::motioncontroller {

Teleoperation::Teleoperation(
    std::shared_ptr<IRobot> robot,
    std::shared_ptr<IController> controller,
    const std::chrono::milliseconds& velocityTimeInterval):
    AbstractMotionController(robot, controller, velocityTimeInterval),
    robot_(robot),
    controller_(controller),
    robotConfig_(robot_->getConfiguration()),
    logger_("Teleoperation") {
    logger_->debug("CTor");

    crf::actuators::robot::JointLimits limitsJoint = robotConfig_->getJointLimits();
    crf::actuators::robot::TaskLimits limitsTask = robotConfig_->getTaskLimits();
    shaperJoints_ = std::make_shared<InputShaperJointsController>(
        JointVelocities(robotConfig_->getJointSpaceDoF()),
        limitsJoint.maxAcceleration);
    shaperTask_ = std::make_shared<InputShaperTaskController>(
        TaskVelocity(),  // This has to change for custom ones
        limitsTask.maxAcceleration);
}

Teleoperation::~Teleoperation() {
    logger_->debug("DTor");
    AbstractMotionController::deinitialize();
}

crf::expected<bool> Teleoperation::appendPath(const std::vector<JointPositions>& path) {
    logger_->error("Method appendPath(Joint) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::appendPath(
    const std::vector<TaskPose>& path,
    const TrajectoryExecutionMethod& method,
    const PointReferenceFrame& frame) {
    logger_->error("Method appendPath(Task) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::setVelocity(const JointVelocities& velocity) {
    logger_->debug("setVelocity(joints)");
    crf::actuators::robot::JointLimits limits = robotConfig_->getJointLimits();
    if (!isBetween(
            crf::utility::types::JointVelocities(limits.maxVelocity.raw()*-1),
            limits.maxVelocity,
            velocity)) {
        logger_->error("Joint Velocity out of the robot limits");
        return crf::Code::BadRequest;
    }
    switchToJointControl();
    if (!isControlLoopActive()) {
        controller_->reset();
        startControlLoop();
    }
    updateLastCommandTime();
    shaperJoints_->setReference(velocity);
    return true;
}

crf::expected<bool> Teleoperation::setVelocity(
    const TaskVelocity& velocity,
    const PointReferenceFrame& frame) {
    logger_->debug("setVelocity(task)");
    crf::actuators::robot::TaskLimits limits = robotConfig_->getTaskLimits();
    if (!isBetween(
            crf::utility::types::TaskVelocity(limits.maxVelocity.raw()*-1),
            limits.maxVelocity,
            velocity)) {
        logger_->error("Task velocity out of the robot limits");
        return crf::Code::BadRequest;
    }
    if (controller_->checkStopCondition()) {
        logger_->error("Controller has a stop condition activated");
        return crf::Code::LowManipulability;
    }
    crf::utility::types::TaskVelocity finalTaskVelocity = velocity;
    Signals sig = getSignals();
    if (frame == crf::control::motioncontroller::PointReferenceFrame::TCP) {
        if (!sig.task.pose) {
            logger_->error("Task position of the robot not known");
            return crf::Code::NotFound;  // Change it when we have a forward kinematics error
        }
        Eigen::Matrix3d rotationMatrix = sig.task.pose.value().getRotationMatrix();
        Eigen::Vector3d tvel(finalTaskVelocity[0], finalTaskVelocity[1], finalTaskVelocity[2]);
        Eigen::Vector3d rvel(finalTaskVelocity[3], finalTaskVelocity[4], finalTaskVelocity[5]);
        Eigen::Vector3d tvelrotated = rotationMatrix*tvel;
        Eigen::Vector3d rvelrotated = rotationMatrix*rvel;
        finalTaskVelocity = crf::utility::types::TaskVelocity(
            {tvelrotated(0), tvelrotated(1), tvelrotated(2),
            rvelrotated(0), rvelrotated(1), rvelrotated(2)});
    }
    switchToTaskControl();
    if (!isControlLoopActive()) {
        controller_->reset();
        startControlLoop();
    }
    updateLastCommandTime();
    shaperTask_->setReference(finalTaskVelocity);
    return true;
}

crf::expected<bool> Teleoperation::setTorque(const JointForceTorques& torque) {
    logger_->error("Method setTorque(Joints) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::setTorque(const TaskForceTorque& torque,
    const PointReferenceFrame& frame) {
    logger_->error("Method setTorque(Task) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::setProfileVelocity(const JointVelocities& velocity) {
    logger_->error("Method setProfileVelocity(Joint) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::setProfileVelocity(const TaskVelocity& velocity) {
    logger_->error("Method setProfileVelocity(Task) not implemented");
    return crf::Code::MethodNotAllowed;
}

crf::expected<bool> Teleoperation::setProfileAcceleration(const JointAccelerations& acceleration) {
    logger_->debug("setProfileAcceleration(joint)");
    crf::actuators::robot::JointLimits limits = robotConfig_->getJointLimits();
    if (!isLesser(acceleration, limits.maxAcceleration)) {
        logger_->error("Joint acceleration out of the robot limits");
        return crf::Code::BadRequest;
    }
    shaperJoints_->setAcceleration(acceleration);
    return true;
}

crf::expected<bool> Teleoperation::setProfileAcceleration(const TaskAcceleration& acceleration) {
    logger_->debug("setProfileAcceleration(task)");
    crf::actuators::robot::TaskLimits limits = robotConfig_->getTaskLimits();
    if (!(isLesser(acceleration, limits.maxAcceleration))) {
        logger_->error("Task acceleration out of the robot limits");
        return crf::Code::BadRequest;
    }
    shaperTask_->setAcceleration(acceleration);
    return true;
}

void Teleoperation::softStop() {
    logger_->debug("softStop");
    AbstractMotionController::softStop();
    shaperJoints_->reset();
    shaperTask_->reset();
}

void Teleoperation::hardStop() {
    logger_->debug("hardStop");
    AbstractMotionController::hardStop();
    shaperJoints_->reset();
    shaperTask_->reset();
}

crf::expected<bool> Teleoperation::setParameters(const nlohmann::json& params) {
    return controller_->setParameters(params["Controller"]);
}

nlohmann::json Teleoperation::getCurrentParameters() {
    nlohmann::json json;
    json["Controller"] = controller_->getParameters();
    return json;
}

nlohmann::json Teleoperation::getParametersDefinition() const {
    nlohmann::json json;
    return json;
}

crf::expected<bool> Teleoperation::isTrajectoryRunning() {
    return crf::Code::MethodNotAllowed;
}

// Private

Signals Teleoperation::jointControl() {
    JointSignals ref;
    ref.velocities = shaperJoints_->getVelocity(trajectoryTimeCounter_*loopTime_.count()/1e6);
    ref = smoothStopScaling(ref);
    return controller_->calculate(ref, robotJointSignals_, robotTaskSignals_);
}

Signals Teleoperation::taskControl() {
    TaskSignals ref;
    ref.velocity = shaperTask_->getVelocity(trajectoryTimeCounter_*loopTime_.count()/1e6);
    ref = smoothStopScaling(ref);
    return controller_->calculate(ref, robotJointSignals_, robotTaskSignals_);
}

void Teleoperation::commandTimeout() {
    // If no command received in some time we set the ref to 0
    shaperJoints_->setReference(JointVelocities(robotConfig_->getJointSpaceDoF()));
    shaperTask_->setReference(TaskVelocity());  // This has to change for custom
}

}  // namespace crf::control::motioncontroller
