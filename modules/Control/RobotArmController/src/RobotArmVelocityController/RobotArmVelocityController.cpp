/* © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Jorge Playán Garai CERN BE/CEM/MRO
 *         Alejandro Diaz Rosales CERN EN/SMM/MRO
 *         Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <boost/optional.hpp>  // Change for std::optional after we change it in Robot Arm

#include "RobotArmController/RobotArmVelocityController/RobotArmVelocityController.hpp"
#include "RobotArmKinematics/RobotArmKDLKinematics/RobotArmKDLKinematics.hpp"

#define MARGIN 0.005
#define TASK_SPACE_SIZE 6

namespace crf::control::robotarmcontroller {

RobotArmVelocityController::RobotArmVelocityController(
    std::shared_ptr<crf::actuators::robotarm::IRobotArm> arm):
    arm_(arm),
    kinematics_(std::make_shared<crf::control::robotarmkinematics::RobotArmKDLKinematics>(
        arm_->getConfiguration())),
    jointPositionsPIDController_(),
    taskPosePIDController_(),
    logging_(false),
    configuration_(arm_->getConfiguration()),
    jointsConfiguration_(configuration_->getJointsConfiguration()),
    numberJoints_(configuration_->getNumberOfJoints()),
    targetJointVelocities_(numberJoints_),
    rtLoopTime_(std::chrono::duration_cast<std::chrono::microseconds>(
        configuration_->getRTLoopTime())),
    currentTaskStatusMutex_(),
    currentJointsStatusMutex_(),
    targetVelocityMutex_(),
    controlLoopThread_(),
    jointsMaximumVelocity_(numberJoints_),
    jointsMaximumAcceleration_(numberJoints_),
    taskMaximumVelocity_(),
    taskMaximumAcceleration_(),
    currentJointPositions_(numberJoints_),
    currentJointVelocities_(numberJoints_),
    currentJointForceTorques_(numberJoints_),
    currentTaskPose_(),
    currentTaskVelocity_(),
    taskVelocityToJoints_(),
    lastCommandTime_(),
    lastUpdateTime_(),
    stopControlLoop_(false),
    interruptTrajectory_(false),
    armMoving_(false),
    trajectoryMethod_(TrajectoryExecutionMethod::JointSpace),
    velocityCommandInterval_(500),
    initialized_(false),
    targetPosition_(numberJoints_),
    targetPositionTask_(),
    targetVelocity_(numberJoints_),
    targetVelocityTask_(),
    recordTargetJointPos_(),
    recordTargetJointVel_(),
    recordTargetTaskPos_(),
    recordTargetTaskVel_(),
    recordJointPos_(),
    recordJointVel_(),
    recordTaskPos_(),
    recordTaskVel_(),
    logger_("RobotArmVelocityController") {
        logger_->debug("CTor");

        if (std::getenv(CONTROLLER_ENABLE_LOGGING)) {
            logging_ = true;
        }

        for (unsigned int i = 0; i < numberJoints_; i++) {
            jointsMaximumAcceleration_[i] = jointsConfiguration_[i].maximumAcceleration;
            jointsMaximumVelocity_[i] = jointsConfiguration_[i].maximumVelocity;
        }
        taskMaximumAcceleration_ = configuration_->getTaskLimits().maximumAcceleration;
        taskMaximumVelocity_ = configuration_->getTaskLimits().maximumVelocity;

        std::vector<double> JointKp = configuration_->getParametersPIDs().KpJoint;
        std::vector<double> JointKi = configuration_->getParametersPIDs().KiJoint;
        std::vector<double> JointKd = configuration_->getParametersPIDs().KdJoint;

        std::vector<double> TaskKp = configuration_->getParametersPIDs().KpTask;
        std::vector<double> TaskKi = configuration_->getParametersPIDs().KiTask;
        std::vector<double> TaskKd = configuration_->getParametersPIDs().KdTask;

        jointPositionsPIDController_ =
            std::make_shared<crf::control::closedloopcontroller::PIDController>(
                JointKp,
                JointKi,
                JointKd);
        taskPosePIDController_ =
            std::make_shared<crf::control::closedloopcontroller::PIDController>(
                TaskKp,
                TaskKi,
                TaskKd);
}

RobotArmVelocityController::~RobotArmVelocityController() {
    logger_->debug("DTor");
    if (initialized_) {
        RobotArmVelocityController::deinitialize();
    }
    if (logging_) {
        std::vector<std::vector<crf::utility::types::JointPositions>> valuesJointPos;
        valuesJointPos.push_back(recordJointPos_);
        valuesJointPos.push_back(recordTargetJointPos_);
        std::vector<std::vector<crf::utility::types::JointVelocities>> valuesJointVel;
        valuesJointVel.push_back(recordJointVel_);
        valuesJointVel.push_back(recordTargetJointVel_);
        crf::utility::graphplot::PlotJointPositionsOverTime(valuesJointPos, valuesJointVel);

        std::vector<std::vector<crf::utility::types::TaskPose>> valuesTask;
        valuesTask.push_back(recordTaskPos_);
        valuesTask.push_back(recordTargetTaskPos_);
        crf::utility::graphplot::PlotTaskPoseOverTime(valuesTask);
    }
}

bool RobotArmVelocityController::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return true;
    }
    // This is needed to enable the operation in the EtherCAT until the disableBrakes is done
    arm_->initialize();
    arm_->disableBrakes();
    stopControlLoop_ = false;
    trajectoryMethod_ = TrajectoryExecutionMethod::JointSpace;
    targetJointVelocities_ = utility::types::JointVelocities(numberJoints_);
    taskVelocityToJoints_ = utility::types::TaskVelocity();
    lastCommandTime_ = std::chrono::high_resolution_clock::now();

    if (!updateArmStatus()) {
        logger_->warn("Could not get information from the robot arm");
        return false;
    }
    if (controlLoopThread_.joinable()) {
        logger_->warn("Control loop already running");
        return false;
    }
    controlLoopThread_ = std::thread(&RobotArmVelocityController::controlLoop, this);

    initialized_ = true;
    return true;
}

bool RobotArmVelocityController::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Already deinitialized");
        return true;
    }

    if (!controlLoopThread_.joinable()) {
        logger_->warn("Control loop was not running");
        return false;
    }

    stopControlLoop_ = true;
    controlLoopThread_.join();
    initialized_ = false;

    if (!arm_->stopArm()) {
        logger_->error("Failed to stop robot arm");
        return false;
    }
    // This is needed to disable the operation in the EtherCAT until the enableBrakes is done
    arm_->deinitialize();
    arm_->enableBrakes();
    return true;
}

std::future<bool> RobotArmVelocityController::setPosition(
    const crf::utility::types::JointPositions& position) {
    logger_->debug("setPosition(JointPositions)");

    // We call setPosition(const std::vector<crf::utility::types::JointPositions>& positions)
    std::vector<crf::utility::types::JointPositions> path;
    path.push_back(position);
    return setPosition(path);
}


std::future<bool> RobotArmVelocityController::setPosition(
    const std::vector<crf::utility::types::JointPositions>& positions) {
    logger_->debug("setPosition(std::vector<JointPositions>)");
    if (!initialized_) {
        logger_->error("Controller is not initialized");
        return std::future<bool>();
    }
    if (positions.empty()) {
        logger_->error("Positions vector is empty");
        return std::future<bool>();
    }

    if (positions.size() == 1) {
        if (positions[0].size() != numberJoints_) {
            logger_->error("The {}-th joint position has wrong size, {}", 0, positions[0].size());
            return std::future<bool>();
        }
        if (crf::utility::types::areAlmostEqual(getJointPositions(), positions[0], MARGIN)) {
            return std::async(std::launch::async, []() { return true; });
        }
    }

    // Add the current position of the robot to the path
    std::vector<crf::utility::types::JointPositions> path;
    path.push_back(getJointPositions());

    // Check all the positions and add them in order to internal path
    for (unsigned int i = 0; i < positions.size(); i++) {
        if (positions[i].size() != numberJoints_) {
            logger_->error("The {}-th joint position has wrong size, {}", i, positions[i].size());
            return std::future<bool>();
        }
        for (unsigned int j = 0; j < numberJoints_; j++) {
            if ((positions[i][j] > jointsConfiguration_[j].maximumPosition + MARGIN) ||
                (positions[i][j] < jointsConfiguration_[j].minimumPosition - MARGIN)) {
                logger_->error("The {}-th joint position elem {} is out of range: {} vs [{}, {}]",
                    i, j, positions[i][j],
                    jointsConfiguration_[j].maximumPosition,
                    jointsConfiguration_[j].minimumPosition);
                return std::future<bool>();
            }
        }
        path.push_back(positions[i]);
    }

    // Creation of the trajectory using the path calling JointsTimeOptimalTrajectory
    float maxDeviation = 0.01;
    float timeStep = std::chrono::duration<float>(rtLoopTime_).count();
    auto trajectory = std::make_shared<trajectorygeneratordeprecated::JointsTimeOptimalTrajectory>(
        jointsMaximumVelocity_, jointsMaximumAcceleration_, timeStep, maxDeviation);
    if (!trajectory->computeTrajectory(path)) {
        logger_->error("Could not compute the trajectory");
        return std::future<bool>();
    }

    return std::async(std::launch::async, [this, trajectory]() {
        return executeJointsTrajectory(trajectory);
    });
}

std::future<bool> RobotArmVelocityController::setPosition(
    const crf::utility::types::TaskPose& position,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setPosition(TaskPose)");

    // We call setPosition(const std::vector<crf::utility::types::TaskPose>& positions)
    std::vector<crf::utility::types::TaskPose> path;
    path.push_back(position);
    return setPosition(path, method, frame);
}

std::future<bool> RobotArmVelocityController::setPosition(
    const std::vector<crf::utility::types::TaskPose>& positions,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setPosition(std::vector<TaskPose>)");
    if (!initialized_) {
        logger_->error("Controller is not initialized");
        return std::future<bool>();
    }
    if (positions.empty()) {
        logger_->warn("Empty input vector");
        return std::future<bool>();
    }

    // If we are in the reference frame of the TCP, we transform all the points.
    std::vector<crf::utility::types::TaskPose> positionsReferenced;
    if (frame == PointReferenceFrame::TCP) {
        crf::utility::types::TaskPose currentPos = getTaskPose();
        for (uint64_t i = 0; i < positions.size(); i++) {
            positionsReferenced.push_back(multiply(currentPos, positions[i]));
        }
    } else if (frame == PointReferenceFrame::Global) {
        for (uint64_t i = 0; i < positions.size(); i++) {
            positionsReferenced.push_back(positions[i]);
        }
    } else {
        logger_->error("Not valid point reference frame");
        return std::future<bool>();
    }

    if (method == TrajectoryExecutionMethod::JointSpace) {
        // Add the current position of the robot to use it as a seed for the Inverse Kinematics
        std::vector<crf::utility::types::JointPositions> jointPath;
        jointPath.push_back(getJointPositions());
        for (unsigned int i = 0; i < positionsReferenced.size(); i++) {
            // Convert all the positions and add them in order to internal path
            std::vector<crf::utility::types::JointPositions> nextJointPositionsVec =
                kinematics_->getPositionInverseKinematic(positionsReferenced[i], jointPath[i]);
            if (nextJointPositionsVec.empty()) {
                logger_->warn("Inverse kinematics error for target position: {}",
                    positionsReferenced[i]);
                return std::future<bool>();
            }
            jointPath.push_back(nextJointPositionsVec.at(0));
        }
        // We call setPosition(const std::vector<crf::utility::types::JointPositions>& positions)
        return setPosition(jointPath);
    }

    // Add the current position of the robot to the path
    std::vector<utility::types::TaskPose> taskPath;
    taskPath.push_back(getTaskPose());

    // Add positions to the internal path in order
    for (unsigned int i = 0; i < positionsReferenced.size(); i++) {
        taskPath.push_back(positionsReferenced[i]);
    }

    // Creation of the trajectory using the path calling TaskLinearTrajectory
    float timeStep = std::chrono::duration<float>(rtLoopTime_).count();
    std::shared_ptr<crf::control::trajectorygeneratordeprecated::TaskLinearTrajectory> trajectory =
        std::make_shared<crf::control::trajectorygeneratordeprecated::TaskLinearTrajectory>(
            taskMaximumVelocity_,
            taskMaximumAcceleration_,
            timeStep);
    if (!trajectory->computeTrajectory(taskPath)) {
        logger_->error("Could not compute the trajectory");
        return std::future<bool>();
    }

    if (method == TrajectoryExecutionMethod::TaskSpaceJointLoop) {
        return std::async(std::launch::async, [this, trajectory]() {
            return executeJointsTrajectory(trajectory);
        });
    } else if (method == TrajectoryExecutionMethod::TaskSpaceTaskLoop) {
        return std::async(std::launch::async, [this, trajectory]() {
            return executeTaskTrajectory(trajectory);
        });
    }
    logger_->error("Not valid trajectory execution method");
    return std::future<bool>();
}

bool RobotArmVelocityController::setVelocity(
    const crf::utility::types::JointVelocities& velocity) {
    logger_->debug("setVelocity(JointVelocities)");
    if (!initialized_) {
        logger_->error("Controller is not initialized");
        return false;
    }

    return RobotArmVelocityController::setJointVelocities(
        velocity,
        crf::control::robotarmcontroller::TrajectoryExecutionMethod::JointSpace);
}

bool RobotArmVelocityController::setVelocity(
    const std::vector<crf::utility::types::JointVelocities>& velocities) {
    if (velocities.size() == 1) {
        return RobotArmVelocityController::setVelocity(velocities.at(0));
    }
    logger_->debug("setVelocity(std::vector<JointVelocities>)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::setVelocity(
    const crf::utility::types::TaskVelocity& velocity,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(TaskVelocity)");
    if (!initialized_) {
        logger_->error("Controller is not initialized");
        return false;
    }
    // For now we only use this method but on the future we might use more
    if (method != TrajectoryExecutionMethod::TaskSpaceTaskLoop) {
        logger_->error("Wrong execution method selected");
        return false;
    }
    return RobotArmVelocityController::setTaskVelocity(velocity, method, frame);
}

bool RobotArmVelocityController::setVelocity(
    const std::vector<crf::utility::types::TaskVelocity>& velocities,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    if (velocities.size() == 1) {
        return RobotArmVelocityController::setVelocity(velocities.at(0), method, frame);
    }
    // For now we only use this method but on the future we might use more
    if (method != TrajectoryExecutionMethod::TaskSpaceTaskLoop) {
        logger_->error("Wrong execution method selected");
        return false;
    }
    logger_->debug("setVelocity(std::vector<TaskVelocity>)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::setAcceleration(
    const crf::utility::types::JointAccelerations& acceleration) {
    logger_->debug("setVelocity(JointAccelerations)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::setAcceleration(
    const std::vector<crf::utility::types::JointAccelerations>& accelerations) {
    logger_->debug("setVelocity(std::vector<JointAccelerations>)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::setAcceleration(
    const crf::utility::types::TaskAcceleration& acceleration,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(TaskAcceleration)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::setAcceleration(
    const std::vector<crf::utility::types::TaskAcceleration>& accelerations,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setVelocity(std::vector<TaskAcceleration>)");
    logger_->error("Functionality not available in this controller");
    return false;
}

bool RobotArmVelocityController::interruptTrajectory() {
    logger_->debug("interruptTrajectory");
    if (!initialized_) {
        logger_->error("Controller is not initialized");
        return false;
    }

    interruptTrajectory_ = true;

    // If it takes longer than the maximum time and the arm didn't stop moving then it failed
    std::chrono::high_resolution_clock::time_point requestTime =
        std::chrono::high_resolution_clock::now();
    while (armMoving_) {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - requestTime) > velocityCommandInterval_) {
            logger_->error("Could not interrupt the movement");
            return false;
        }
        std::this_thread::sleep_for(rtLoopTime_);
    }
    return true;
}

crf::utility::types::JointPositions RobotArmVelocityController::getJointPositions() {
    logger_->debug("getJointPositions");
    std::lock_guard<std::mutex> lock(currentJointsStatusMutex_);
    return currentJointPositions_;
}

crf::utility::types::JointVelocities RobotArmVelocityController::getJointVelocities() {
    logger_->debug("getJointVelocities");
    std::lock_guard<std::mutex> lock(currentJointsStatusMutex_);
    return currentJointVelocities_;
}

crf::utility::types::JointAccelerations RobotArmVelocityController::getJointAccelerations() {
    logger_->debug("getJointAccelerations");
    return crf::utility::types::JointAccelerations(numberJoints_);
}

crf::utility::types::JointForceTorques RobotArmVelocityController::getJointForceTorques() {
    logger_->debug("getJointForceTorques");
    return currentJointForceTorques_;
}

crf::utility::types::TaskPose RobotArmVelocityController::getTaskPose() {
    logger_->debug("getTaskPose");
    std::lock_guard<std::mutex> lock(currentTaskStatusMutex_);
    return currentTaskPose_;
}

crf::utility::types::TaskVelocity RobotArmVelocityController::getTaskVelocity() {
    logger_->debug("getTaskVelocity");
    std::lock_guard<std::mutex> lock(currentTaskStatusMutex_);
    return currentTaskVelocity_;
}

crf::utility::types::TaskAcceleration RobotArmVelocityController::getTaskAcceleration() {
    logger_->debug("getTaskAcceleration");
    return crf::utility::types::TaskAcceleration();
}

bool RobotArmVelocityController::setJointsMaximumVelocity(
    const crf::utility::types::JointVelocities& velocity) {
    logger_->debug("setJointsMaximumVelocity");
    if (velocity.size() != numberJoints_) {
        logger_->warn("Wrong JointVelocities input size");
        return false;
    }
    // Check that it doesn't pass the configuration limits
    for (unsigned int i = 0; i < numberJoints_; i++) {
        if ((velocity[i] <= 0) || (velocity[i] > jointsConfiguration_[i].maximumVelocity)) {
            logger_->warn("Incorrect value for joint {}", i);
            return false;
        }
    }
    jointsMaximumVelocity_ = velocity;
    return true;
}

bool RobotArmVelocityController::setJointsMaximumAcceleration(
    const crf::utility::types::JointAccelerations& acceleration) {
    logger_->debug("setJointsMaximumAcceleration");
    if (acceleration.size() != numberJoints_) {
        logger_->warn("Wrong JointAccelerations input size");
        return false;
    }
    // Check that it doesn't pass the configuration limits
    for (unsigned int i = 0; i < numberJoints_; i++) {
        if ((acceleration[i] <= 0) ||
            (acceleration[i] > jointsConfiguration_[i].maximumAcceleration)) {
            logger_->warn("JointAccelerations must be within 0 and {0} for joint {1}",
                jointsConfiguration_[i].maximumAcceleration, i);
            return false;
        }
    }
    jointsMaximumAcceleration_ = acceleration;
    return true;
}

bool RobotArmVelocityController::setTaskMaximumVelocity(
  const crf::utility::types::TaskVelocity& velocity) {
    logger_->debug("setTaskMaximumVelocity");
    // Check that it doesn't pass the configuration limits
    for (int i=0; i < 6; i++) {
        if ((velocity[i] <= 0) ||
            (velocity[i] > configuration_->getTaskLimits().maximumVelocity[i])) {
            logger_->warn("TaskVelocity must be within 0 and {0} for joint {1}",
                configuration_->getTaskLimits().maximumVelocity[i], i);
            return false;
        }
    }
    taskMaximumVelocity_ = velocity;
    return true;
}

bool RobotArmVelocityController::setTaskMaximumAcceleration(
  const crf::utility::types::TaskAcceleration& acceleration) {
    logger_->debug("setTaskMaximumAcceleration");
    // Check that it doesn't pass the configuration limits
    for (int i=0; i < 6; i++) {
        if ((acceleration[i] <= 0) ||
            (acceleration[i] > configuration_->getTaskLimits().maximumAcceleration[i])) {
            logger_->warn("TaskAcceleration must be within 0 and {0} for joint {1}",
                configuration_->getTaskLimits().maximumAcceleration[i], i);
            return false;
        }
    }
    taskMaximumAcceleration_ = acceleration;
    return true;
}

/////////////////////
// Private Methods //
/////////////////////

bool RobotArmVelocityController::setJointVelocities(
    const crf::utility::types::JointVelocities& velocity,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method) {
    logger_->debug("setJointVelocities");
    if (velocity.size() != numberJoints_) {
        logger_->warn("Wrong vector size in velocity parameter");
        return false;
    }
    // If the velocity is outside the limits we set it to the maximum possible
    crf::utility::types::JointVelocities velocityInput = velocity;
    for (unsigned int i = 0; i < numberJoints_; i++) {
        if (fabs(velocityInput[i]) > jointsMaximumVelocity_[i]) {
            if (velocityInput[i] > 0) {
                velocityInput[i] = jointsMaximumVelocity_[i];
            } else {
                velocityInput[i] = -jointsMaximumVelocity_[i];
            }
        }
    }
    targetVelocityMutex_.lock();
    trajectoryMethod_ = method;
    targetJointVelocities_ = velocityInput;
    targetVelocityMutex_.unlock();
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
    return true;
}

bool RobotArmVelocityController::setTaskVelocity(
    const crf::utility::types::TaskVelocity& velocity,
    crf::control::robotarmcontroller::TrajectoryExecutionMethod method,
    crf::control::robotarmcontroller::PointReferenceFrame frame) {
    logger_->debug("setTaskVelocity");
    // If we are in the reference frame of the TCP, we rotate the velocity.
    // This assumes that the velocity you are giving is the absolute velocity.
    crf::utility::types::TaskVelocity finalTaskVelocity = velocity;
    if (frame == crf::control::robotarmcontroller::PointReferenceFrame::TCP) {
        Eigen::Matrix3d rotationMatrix = getTaskPose().getRotationMatrix();
        Eigen::Vector3d tvel(finalTaskVelocity[0], finalTaskVelocity[1], finalTaskVelocity[2]);
        Eigen::Vector3d rvel(finalTaskVelocity[3], finalTaskVelocity[4], finalTaskVelocity[5]);
        Eigen::Vector3d tvelrotated = rotationMatrix*tvel;
        Eigen::Vector3d rvelrotated = rotationMatrix*rvel;
        finalTaskVelocity = crf::utility::types::TaskVelocity(
            {tvelrotated(0), tvelrotated(1), tvelrotated(2),
            rvelrotated(0), rvelrotated(1), rvelrotated(2)});
    }
    targetVelocityMutex_.lock();
    taskVelocityToJoints_ = finalTaskVelocity;
    trajectoryMethod_ = method;
    targetVelocityMutex_.unlock();
    lastCommandTime_ = std::chrono::high_resolution_clock::now();
    return true;
}

bool RobotArmVelocityController::executeJointsTrajectory(
    const std::shared_ptr<crf::control::trajectorygeneratordeprecated::IJointsTrajectoryGenerator>& trajectory) {  // NOLINT
    logger_->debug("executeJointsTrajectory");

    interruptTrajectory_ = false;
    std::vector<double> targetPosVec(numberJoints_);
    std::vector<double> currentPosVec(numberJoints_);
    jointPositionsPIDController_->reset();

    boost::optional<float> durationOpt = trajectory->getDuration();
    if (!durationOpt) {
        logger_->error("Couldn't get the duration of trajectory");
        return false;
    }
    float duration = durationOpt.get();
    logger_->info("Trajectory duration: {}", duration);

    std::chrono::system_clock::time_point startTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time(0);
    std::chrono::duration<float> updateTime(0);

    // Start the loop that will follow the trajectory.
    while (duration > time.count() && (!interruptTrajectory_)) {
        // Loop time
        std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();
        updateTime = std::chrono::duration_cast<std::chrono::microseconds>(
            lastUpdateTime_ - startTime);
        if (updateTime.count() < 0) {
            updateTime = std::chrono::duration<float>(0);
        }
        time = updateTime + rtLoopTime_;

        // Get references of position and velocity
        boost::optional<utility::types::JointPositions> targetPositionOpt =
            trajectory->getJointPositions(updateTime.count());
        boost::optional<utility::types::JointVelocities> targetVelocityOpt =
            trajectory->getJointVelocities(time.count());

        if ((!targetPositionOpt) || (!targetVelocityOpt)) {
            logger_->warn("Error retrieving velocity or position set point in joints trajectory");
            break;
        }
        crf::utility::types::JointPositions targetPosition = targetPositionOpt.get();
        crf::utility::types::JointVelocities targetVelocity = targetVelocityOpt.get();

        std::unique_lock<std::mutex> jointStatusLock(currentJointsStatusMutex_);
        targetPosition_ = targetPosition;
        targetVelocity_ = targetVelocity;
        jointStatusLock.unlock();

        targetPosVec.clear();
        currentPosVec.clear();

        // Get current joints position values
        crf::utility::types::JointPositions jointPositions = getJointPositions();

        for (unsigned int i=0; i < numberJoints_; i++) {
            targetPosVec.push_back(targetPosition[i]);
            currentPosVec.push_back(jointPositions[i]);
        }

        // PID
        boost::optional<std::vector<double> > pidVelocityOpt =
            jointPositionsPIDController_->calculate(targetPosVec, currentPosVec);
        if (!pidVelocityOpt) {
            logger_->warn("Error retrieving PID output in joints trajectory");
            break;
        }
        std::vector<double> pidVelocity = pidVelocityOpt.get();

        // Feed-forward velocity
        for (unsigned int i=0; i < numberJoints_; i++) {
            targetVelocity[i] += pidVelocity[i];
        }
/*
        // See difference between real and objective
        float diffMax = 0.0;
        int indexDiffMax = 0;
        for (unsigned int i=1; i < numberJoints_; i++) {
            float difference = fabs(currentPosVec[i] - targetPosVec[i]);
            float diffMaxNext = difference > diffMax ? difference : diffMax;
            indexDiffMax =  difference > diffMax ? i : indexDiffMax;
            diffMax = diffMaxNext;
        }

        // If there is a big difference between both then
        if (diffMax > 0.05) {
            logger_->error("Big error displacement ({0}) between target trajectory and current position. Trajectory is interrupted. (Current {1} , Expected {2})",  // NOLINT
                diffMax, currentPosVec[indexDiffMax], targetPosition(indexDiffMax));
            setVelocity(utility::types::JointVelocities(numberJoints_));
            return false;
        }
*/
        // We send the velocity to the control
        setVelocity(targetVelocity);

        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        } else {
            logger_->warn("Execution time longer than rtLoopTime");
        }
    }

    setVelocity(utility::types::JointVelocities(numberJoints_));

    if (interruptTrajectory_) {
        logger_->warn("Trajectory was interrupted manually");
        return false;
    }
    return true;
}

bool RobotArmVelocityController::executeJointsTrajectory(
  const std::shared_ptr<crf::control::trajectorygeneratordeprecated::ITaskTrajectoryGenerator>& trajectory) {  // NOLINT
    logger_->debug("executeJointsTrajectory");

    interruptTrajectory_ = false;
    std::vector<double> targetPosVec(numberJoints_);
    std::vector<double> currentPosVec(numberJoints_);
    jointPositionsPIDController_->reset();

    boost::optional<float> durationOpt = trajectory->getDuration();
    if (!durationOpt) {
        logger_->error("Couldn't get the duration of trajectory");
        return false;
    }
    float duration = durationOpt.get();

    std::chrono::system_clock::time_point startTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> time(0);
    std::chrono::duration<float> updateTime(0);

    logger_->info("Trajectory duration: {}", duration);

    // Follow trajectory
    while (duration > time.count() && (!interruptTrajectory_)) {
        // Loop time
        std::chrono::system_clock::time_point start = std::chrono::high_resolution_clock::now();

        updateTime = std::chrono::duration_cast<std::chrono::microseconds>(
        lastUpdateTime_ - startTime);
        if (updateTime.count() < 0) {
            updateTime = std::chrono::duration<float>(0);
        }
        time = updateTime + rtLoopTime_;

        // Get actual values
        utility::types::JointPositions jointPositions = getJointPositions();

        // Get references of position and velocity
        boost::optional<utility::types::TaskPose> targetPositionOpt =
            trajectory->getTaskPose(updateTime.count());
        boost::optional<utility::types::TaskVelocity> targetVelocityOpt =
            trajectory->getTaskVelocity(time.count());

        if ((!targetPositionOpt) || (!targetVelocityOpt)) {
            logger_->warn("jointTrajectoryExecution(Task): error retrieving velocity or position setpoint");  // NOLINT
            break;
        }
        crf::utility::types::TaskPose targetPositionTask = targetPositionOpt.get();
        crf::utility::types::TaskVelocity targetVelocityTask = targetVelocityOpt.get();

        // Change task to joints
        std::vector<crf::utility::types::JointPositions> targetPositionVec =
            kinematics_->getPositionInverseKinematic(targetPositionTask, jointPositions);

        if (targetPositionVec.empty()) {
            logger_->warn("Inverse kinematics error for target position: {}", targetPositionTask);
            return false;
        }

        boost::optional<crf::utility::types::JointVelocities> targetVelocityOptJoi =
            kinematics_->getVelocityInverseKinematic(targetVelocityTask, jointPositions);

        if (!targetVelocityOptJoi) {
            logger_->warn("Inverse kinematics error for target velocity: {}", targetVelocityTask);
            return false;
        }

        crf::utility::types::JointPositions targetPosition = targetPositionVec.at(0);
        crf::utility::types::JointVelocities targetVelocity = targetVelocityOptJoi.get();

        std::unique_lock<std::mutex> jointStatusLock(currentJointsStatusMutex_);
        targetPosition_ = targetPosition;
        targetVelocity_ = targetVelocity;
        jointStatusLock.unlock();

        targetPosVec.clear();
        currentPosVec.clear();

        for (unsigned int i=0; i < numberJoints_; i++) {
            targetPosVec.push_back(targetPosition[i]);
            currentPosVec.push_back(jointPositions[i]);
        }

        // PID
        boost::optional<std::vector<double> > pidVelocityOpt =
            jointPositionsPIDController_->calculate(targetPosVec, currentPosVec);
        if (!pidVelocityOpt) {
            logger_->warn("jointTrajectoryExecution(Task): error retrieving pid output");
            break;
        }

        std::vector<double> pidVelocity = pidVelocityOpt.get();

        // Feed-forward velocity
        for (unsigned int i=0; i < numberJoints_; i++) {
            targetVelocity[i] += pidVelocity[i];
        }
        // See difference between real and objective
        float diffMax = 0.0;
        int indexDiffMax = 0;
        for (unsigned int i=1; i < numberJoints_; i++) {
            float difference = fabs(currentPosVec[i] - targetPosVec[i]);
            float diffMaxNext = difference > diffMax ? difference : diffMax;
            indexDiffMax =  difference > diffMax ? i : indexDiffMax;
            diffMax = diffMaxNext;
        }

        // If there is a big difference between both then we think we collided so we stop
        if (diffMax > 0.3) {
            logger_->error("jointTrajectoryExecution(Task): big error displacement ({0}) between target trajectory and current position. Trajectory is interrupted. (Current {1} , Expected {2})",  // NOLINT
                diffMax, currentPosVec[indexDiffMax], targetPosition[indexDiffMax]);
            setVelocity(crf::utility::types::JointVelocities(numberJoints_));
            return false;
        }

        // We send the velocity to the control
        setVelocity(targetVelocity);

        // Update loop times
        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        } else {
            logger_->warn("executeJointsTrajectory(): execution time longer than rtLoopTime");
        }
    }

    setVelocity(crf::utility::types::JointVelocities(numberJoints_));

    if (interruptTrajectory_) {
        logger_->warn("Trajectory was interrupted manually");
        return false;
    }
    return true;
}

bool RobotArmVelocityController::executeTaskTrajectory(
  const std::shared_ptr<crf::control::trajectorygeneratordeprecated::ITaskTrajectoryGenerator>&
    trajectory) {
        logger_->debug("executeTaskTrajectory");

        if (!initialized_) {
            logger_->error("Controller is not initialized");
            return false;
        }

        interruptTrajectory_ = false;
        std::vector<double> targetPosVec(numberJoints_);
        std::vector<double> currentPosVec(numberJoints_);
        taskPosePIDController_->reset();

        boost::optional<float> durationOpt = trajectory->getDuration();
        if (!durationOpt) {
            logger_->error("Couldn't get the duration of trajectory");
            return false;
        }
        float duration = durationOpt.get();

        std::chrono::system_clock::time_point startTime =
            std::chrono::high_resolution_clock::now();
        std::chrono::duration<float> time(0);
        std::chrono::duration<float> updateTime(0);

        logger_->debug("Trajectory duration: {}", duration);

        // Follow trajectory
        while ((duration > time.count()) && (!interruptTrajectory_)) {
            // Loop updates
            // Loop time
            std::chrono::system_clock::time_point start =
                std::chrono::high_resolution_clock::now();

            updateTime = std::chrono::duration_cast<std::chrono::microseconds>(
            lastUpdateTime_ - startTime);
            if (updateTime.count() < 0) {
                updateTime = std::chrono::duration<float>(0);
            }
            time = updateTime + rtLoopTime_;

            crf::utility::types::JointPositions currJoiPos =
                RobotArmVelocityController::getJointPositions();

            // Get references of position and velocity
            boost::optional<utility::types::TaskPose> targetPositionOpt =
                trajectory->getTaskPose(updateTime.count());
            boost::optional<utility::types::TaskVelocity> targetVelocityOpt =
                trajectory->getTaskVelocity(time.count());

            if ((!targetPositionOpt) || (!targetVelocityOpt)) {
                logger_->warn(
                    "jointTrajectoryExecution: error retrieving velocity or position setpoint");
                return false;
            }

            crf::utility::types::TaskPose targetPosition = targetPositionOpt.get();
            crf::utility::types::TaskVelocity targetVelocity = targetVelocityOpt.get();

            std::vector<crf::utility::types::JointPositions> currentJointsPosOpt1 =
                kinematics_->getPositionInverseKinematic(targetPosition, currJoiPos);

            if (currentJointsPosOpt1.size() == 0) {
                logger_->error("Error in inverse kinematic calculation");
                return false;
            }
            crf::utility::types::JointPositions targetPositionJoi =
                currentJointsPosOpt1.at(0);

            std::unique_lock<std::mutex> jointStatusLock(currentJointsStatusMutex_);
            targetPositionTask_ = targetPosition;
            targetVelocityTask_ = targetVelocity;
            targetPosition_ = targetPositionJoi;
            jointStatusLock.unlock();

            targetPosVec.clear();
            currentPosVec.clear();

            boost::optional<utility::types::TaskPose> currentTaskPosOpt =
                kinematics_->getPositionForwardKinematic(currJoiPos);

            if (!currentTaskPosOpt) {
                logger_->warn("executeTaskTrajectory(): error in position forward kinematic calculation"); // NOLINT
                return false;
            }
            crf::utility::types::TaskPose currentTaskPos =
                currentTaskPosOpt.get();

            for (unsigned int i = 0; i < 3; i++) {
                targetPosVec.push_back(targetPosition.getPosition()(i));
                currentPosVec.push_back(currentTaskPos.getPosition()(i));
            }
            for (unsigned int i = 3; i < 6; i++) {
                targetPosVec.push_back(targetPosition.getCardanXYZ()[i]);
                currentPosVec.push_back(currentTaskPos.getCardanXYZ()[i]);
            }

            // PID with velocities for the position
            boost::optional<std::vector<double>> pidVelocityXYZOpt =
                taskPosePIDController_->calculate(targetPosVec, currentPosVec);
            if (!pidVelocityXYZOpt) {
                logger_->warn("jointTrajectoryExecution: error retrieving pid output");
                break;
            }
            std::vector<double> pidVelocity = pidVelocityXYZOpt.get();

            // Velocity feed-forward
            for (unsigned int i = 0; i < 3; i++) {
               targetVelocity[i] += pidVelocity[i];
            }

            float differenceXYZ = 0;
            crf::utility::types::TaskPose taskPose =
                RobotArmVelocityController::getTaskPose();

            // Check error in the position XYZ
            for (unsigned int i = 0; i < 3; i++) {
                differenceXYZ += pow(
                    targetPosition.getPosition()(i) - taskPose.getPosition()(i), 2);
            }
            differenceXYZ = sqrt(differenceXYZ);

            if (differenceXYZ > 0.05) {
                logger_->error(
                    "taskTrajectoryExecution: big error displacement ({0}) between target trajectory and current position.", // NOLINT
                    differenceXYZ);
                    RobotArmVelocityController::setVelocity(
                        utility::types::TaskVelocity(),
                        TrajectoryExecutionMethod::TaskSpaceTaskLoop,
                        crf::control::robotarmcontroller::PointReferenceFrame::Global);
                return false;
            }

            /*
             * Check error in the angle RPY (jlayang)
             *
             * We should add a way to check the difference in the angle but in RPY there is no easy
             * way of doing it. We cannot apply the PID because of unstability in the movmement. We
             * can try later if we move to quaternions, for now we only do the feed-forward of the
             * angle without PID and we don't check it. For this reason, we do not recommend using
             * this loop, better with joints.
             *
             * float differenceRPY = 0.0;
             * if (differenceRPY > 0.1) {
             *     logger_->error(
             *         "Big error displacement ({0}) between target trajectory and current angle.",
             *         differenceRPY);
             *     RobotArmVelocityController::setVelocity(utility::types::TaskVelocity(),
             *         TrajectoryExecutionMethod::TaskSpaceTaskLoop,
             *         crf::control::robotarmcontroller::PointReferenceFrame::Global);
             *     return false;
             * }
             *
             */

            // Send velocity to control
            RobotArmVelocityController::setVelocity(
                targetVelocity,
                TrajectoryExecutionMethod::TaskSpaceTaskLoop,
                crf::control::robotarmcontroller::PointReferenceFrame::Global);

            // Loop time checking and waiing if necessary
            std::chrono::microseconds elapsed =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::high_resolution_clock::now() - start);
            if ((rtLoopTime_ - elapsed).count() > 0) {
                std::this_thread::sleep_for(rtLoopTime_ - elapsed);
            } else {
                logger_->warn(
                    "executeTaskTrajectory(): execution time longer than rtLoopTime");
            }
        }

        RobotArmVelocityController::setVelocity(
            utility::types::TaskVelocity(),
            TrajectoryExecutionMethod::TaskSpaceTaskLoop,
            crf::control::robotarmcontroller::PointReferenceFrame::Global);

        if (interruptTrajectory_) {
            logger_->warn("Trajectory was interrupted manually");
            return false;
        }
        return true;
}

void RobotArmVelocityController::controlLoop() {
    logger_->debug("controlLoop()");
    utility::types::JointVelocities nextJointVelocities(numberJoints_);
    while (!stopControlLoop_) {
        std::chrono::high_resolution_clock::time_point start =
            std::chrono::high_resolution_clock::now();
        bool errorInReading = !RobotArmVelocityController::updateArmStatus();
        // Send previous iteration velocities
        if (!arm_->setJointVelocities(nextJointVelocities)) {
            logger_->error("Not able to set joint arm velocities");
        }

        if ((std::chrono::duration_cast<std::chrono::milliseconds>(start - lastCommandTime_) < velocityCommandInterval_)  // NOLINT
         && (!errorInReading)) {
            if (trajectoryMethod_ == TrajectoryExecutionMethod::TaskSpaceJointLoop ||
                trajectoryMethod_ == TrajectoryExecutionMethod::JointSpace) {
                    logger_->debug("JointSpace");
                    nextJointVelocities = RobotArmVelocityController::jointVelocitiesControl();
            } else if (trajectoryMethod_ ==
                TrajectoryExecutionMethod::TaskSpaceTaskLoop) {
                    logger_->debug("TaskSpace");
                    nextJointVelocities = RobotArmVelocityController::taskVelocityControl();
            } else {
                logger_->error("Trajectory execution method not set, setting velocity empty");
                nextJointVelocities = crf::utility::types::JointVelocities(numberJoints_);
            }
        } else {
            if (errorInReading) {
                logger_->warn("Stopping, error in reading");
            } else {
                // logger_->warn("Stopping, command interval exceeded");
            }
            nextJointVelocities = utility::types::JointVelocities(numberJoints_);
        }
        // Get time and wait for the time loop to finish.
        std::chrono::microseconds elapsed =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - start);
        if ((rtLoopTime_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(rtLoopTime_ - elapsed);
        } else {
            logger_->warn("controlLoop(): execution time longer than rtLoopTime");
        }
    }
}

bool RobotArmVelocityController::updateArmStatus() {
    lastUpdateTime_ = std::chrono::high_resolution_clock::now();
    bool localArmMoving = false;
    // Get previous update velocity
    crf::utility::types::JointVelocities jointVelocities = getJointVelocities();
    // If velocity superior to threshold then it's moving
    for (unsigned int i = 0; i < numberJoints_; i++) {
        if (fabs(jointVelocities[i]) > 0.02) {  // Obtained empirically from the static values
            localArmMoving = true;
        }
    }
    armMoving_ = localArmMoving;
    // Get current position and velocity in joints
    boost::optional<crf::utility::types::JointPositions> currentJointPositions =
        arm_->getJointPositions();
    boost::optional<crf::utility::types::JointVelocities> currentJointVelocities =
        arm_->getJointVelocities();
    boost::optional<crf::utility::types::JointForceTorques> currentJointForceTorques =
        arm_->getJointForceTorques();

    if (!currentJointForceTorques) {
        // logger_->warn("Error reading torque from arm");
        currentJointForceTorques_ = crf::utility::types::JointForceTorques(numberJoints_);
    } else {
        currentJointForceTorques_ = currentJointForceTorques.get();
    }

    if ((!currentJointVelocities) || (!currentJointPositions)) {
        logger_->warn("Error reading from arm");
        return false;
    }
    // Update with new joint values
    std::unique_lock<std::mutex> jointStatusLock(currentJointsStatusMutex_);
    currentJointPositions_ = currentJointPositions.get();
    currentJointVelocities_ = currentJointVelocities.get();
    if (logging_) {
        recordTargetJointPos_.push_back(targetPosition_);
        recordTargetJointVel_.push_back(targetVelocity_);
        recordJointPos_.push_back(currentJointPositions_);
        recordJointVel_.push_back(currentJointVelocities_);
    }
    jointStatusLock.unlock();

    // Get task position and velocity
    boost::optional<utility::types::TaskPose> currentTaskPoseOpt =
        kinematics_->getPositionForwardKinematic(getJointPositions());
    boost::optional<utility::types::TaskVelocity> currentTaskVelocityOpt =
        kinematics_->getVelocityForwardKinematic(getJointPositions(), getJointVelocities());
    if ((!currentTaskPoseOpt) || (!currentTaskVelocityOpt)) {
        logger_->warn("controlLoop(): error in position forward kinematic calculation");
        return false;
    }
    // Update with new task values
    std::unique_lock<std::mutex> taskStatusLock(currentTaskStatusMutex_);
    currentTaskPose_ = currentTaskPoseOpt.get();
    currentTaskVelocity_ = currentTaskVelocityOpt.get();
    if (logging_) {
        recordTargetTaskPos_.push_back(targetPositionTask_);
        recordTargetTaskVel_.push_back(targetVelocityTask_);
        recordTaskPos_.push_back(currentTaskPose_);
        recordTaskVel_.push_back(currentTaskVelocity_);
    }
    taskStatusLock.unlock();
    return true;
}

utility::types::JointVelocities RobotArmVelocityController::jointVelocitiesControl() {
    logger_->debug("jointVelocitiesControl");

    targetVelocityMutex_.lock();
    utility::types::JointVelocities targetJointVelocities = targetJointVelocities_;
    targetVelocityMutex_.unlock();
    utility::types::JointPositions currentJointPositions =
        RobotArmVelocityController::getJointPositions();
    utility::types::JointVelocities currentJointVelocities =
        RobotArmVelocityController::getJointVelocities();
    utility::types::JointVelocities nextJointVelocities(numberJoints_);

    // float timeStep = (float)std::chrono::duration_cast<std::chrono::milliseconds>(rtLoopTime_).count()/1000; // NOLINT
    bool velocityNotValid = false;
    for (unsigned int i=0; i < nextJointVelocities.size(); i++) {
        nextJointVelocities[i] = targetJointVelocities[i];

        // If we are in task trajectories, when we reach a limit we stop everything
        if (trajectoryMethod_ == TrajectoryExecutionMethod::TaskSpaceJointLoop ||
            trajectoryMethod_ == TrajectoryExecutionMethod::TaskSpaceTaskLoop) {
            if ((nextJointVelocities[i] > 0) &&
                (currentJointPositions[i] >= jointsConfiguration_[i].maximumPosition)) {
                    velocityNotValid = true;
                    break;
            } else if  ((nextJointVelocities[i] < 0) &&
                (currentJointPositions[i] <= jointsConfiguration_[i].minimumPosition)) {
                    velocityNotValid = true;
                    break;
            }
        }
        // If the arm is close to the position limit in joints, we brake it by a factor
        if (nextJointVelocities[i] > 0) {
            float distanceToLimit = fabs(
                jointsConfiguration_[i].maximumPosition - currentJointPositions[i]);
            float brakingVelocity = sqrt(2*distanceToLimit*jointsMaximumAcceleration_[i]);

            if (brakingVelocity < nextJointVelocities[i]) {
                nextJointVelocities[i] = brakingVelocity;
            }
        } else if (nextJointVelocities[i] < 0) {
            float distanceToLimit = fabs(
                jointsConfiguration_[i].minimumPosition - currentJointPositions[i]);
            float brakingVelocity = -sqrt(2*distanceToLimit*jointsMaximumAcceleration_[i]);
            if (brakingVelocity > nextJointVelocities[i]) {
                nextJointVelocities[i] = brakingVelocity;
            }
        }
        // Check with maximum velocities
        if ((nextJointVelocities[i] > 0) &&
            (currentJointPositions[i] >= jointsConfiguration_[i].maximumPosition)) {
                nextJointVelocities[i] = 0;
        } else if  ((nextJointVelocities[i] < 0) &&
                    (currentJointPositions[i] <= jointsConfiguration_[i].minimumPosition)) {
                nextJointVelocities[i] = 0;
        }
    }
    if (velocityNotValid) {
        logger_->error("Velocity not valid, stopping trajectory");
        interruptTrajectory_ = true;  // We should use another way of stopping the trajectory
        nextJointVelocities = crf::utility::types::JointVelocities(numberJoints_);
    }
    return nextJointVelocities;
}

utility::types::JointVelocities RobotArmVelocityController::taskVelocityControl() {
    logger_->debug("taskVelocityControl");

    utility::types::JointVelocities nextJointVelocities(numberJoints_);
    utility::types::JointPositions currentJointPositions =
        RobotArmVelocityController::getJointPositions();

    // Calculate joint velocities necessary for the task vel
    boost::optional<utility::types::JointVelocities> computedJointVelocitiesOpt =
        kinematics_->getVelocityInverseKinematic(taskVelocityToJoints_, currentJointPositions);
    if (!computedJointVelocitiesOpt) {
        return nextJointVelocities;
    }
    utility::types::JointVelocities computedJointVelocities = computedJointVelocitiesOpt.get();

    // Restrict to maximum velocity
    float maxDivisionFactor = 1;
    for (unsigned int i = 0; i < numberJoints_; i++) {
        if (fabs(computedJointVelocities[i]) > jointsMaximumVelocity_[i]) {
            float divisionFactor =
                jointsMaximumVelocity_[i]/fabs(computedJointVelocities[i]);
            if (divisionFactor < maxDivisionFactor) {
                maxDivisionFactor = divisionFactor;
            }
        }
    }
    computedJointVelocities =
        crf::utility::types::JointVelocities(computedJointVelocities.raw() * maxDivisionFactor);

    // Add the calculated velocities to the join velocity cotnrol
    targetVelocityMutex_.lock();
    targetJointVelocities_ = computedJointVelocities;
    targetVelocityMutex_.unlock();
    nextJointVelocities = RobotArmVelocityController::jointVelocitiesControl();
    return nextJointVelocities;
}

}  // namespace crf::control::robotarmcontroller
