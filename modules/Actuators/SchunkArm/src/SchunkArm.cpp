/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/optional.hpp>

#include "CANSocket/ICANSocket.hpp"
#include "SchunkArm/SchunkArm.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include "SchunkArm/SchunkDevice.hpp"
#include "SchunkArm/SchunkGripper.hpp"

namespace crf::actuators::schunkarm {

SchunkArm::SchunkArm(std::shared_ptr<ICANSocket> socket,
    const nlohmann::json& robotConfigFile,
    std::shared_ptr<SchunkGripper> gripper):
    millidegreeToRad_(M_PI / 180 /1000),
    numberOfJoints_(6),
    mainLoopPeriodMilliseconds_(3),
    logger_("SchunkArm"),
    socket_(socket),
    robotConfigFile_(robotConfigFile),
    gripper_(gripper),
    jointPositions_(numberOfJoints_),
    defaultVelocityPercentage_(15),
    timeOutForSafeWriteInUSec_(500),
    jointVelocities_(numberOfJoints_),
    configuration_(new robotarm::RobotArmConfiguration),
    arm_(),
    controlLoopThread_(),
    runControlLoop_(false),
    initialized_(false) {
    logger_->debug("CTor");
    auto joint1 = std::make_shared<SchunkDevice>(socket_, JOINT1);
    auto joint2 = std::make_shared<SchunkDevice>(socket_, JOINT2);
    auto joint3 = std::make_shared<SchunkDevice>(socket_, JOINT3);
    auto joint4 = std::make_shared<SchunkDevice>(socket_, JOINT4);
    auto joint5 = std::make_shared<SchunkDevice>(socket_, JOINT5);
    auto joint6 = std::make_shared<SchunkDevice>(socket_, JOINT6);
    arm_.push_back(joint1);
    arm_.push_back(joint2);
    arm_.push_back(joint3);
    arm_.push_back(joint4);
    arm_.push_back(joint5);
    arm_.push_back(joint6);
}

SchunkArm::~SchunkArm() {
    logger_->debug("DTor");
    deinitialize();
}

bool SchunkArm::initialize() {
    logger_->debug("initialize()");
    if (initialized_) {
        logger_->debug("SchunkArm already initialized");
        return false;
    }
    if (!socket_->initialize()) {
        logger_->error("Initialization of CAN socket failed");
        return false;
    }
    if (!configuration_->parse(robotConfigFile_)) {
        logger_->critical("Could not parse the configuration file");
        return false;
    }
    // you need to get clean the can otherwise the control messages might get lost in the noise
    if (!SchunkDevice::cleanCanBus(socket_)) {
        logger_->critical("Canbus could not be cleaned");
        return false;
    }
    for (int i=0; i < numberOfJoints_; i++) {
        if (!arm_.at(i)->initialize()) {
            logger_->critical("Initialization of motor" + std::to_string(i+3) + " failed!");
            return false;
        }
    }
    if (gripper_ != nullptr) {
        if (!gripper_->initialize()) {
            logger_->error("Could not initialize the gripper");
            return false;
        }
    }
    // it will make the joint broaddcast its position every X millisecond
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->getStatePeriodic(mainLoopPeriodMilliseconds_);
    }
    this->startControlLoop();
    // Give enough time to the controlLoop to start and receive the first packets
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // This procedure allows the schunk arm to start properly after a powercut
    // without the need of starting the program twice
    for (int i=0; i < numberOfJoints_; i++) {
        int jointDirection = std::copysign(1.0, jointPositions_[i]);
        arm_.at(i)->setVelocity(-1 * jointDirection * 0.0001);
    }
    bool allBrakesOff = false;
    while (!allBrakesOff) {
        allBrakesOff = true;
        for (int i=0; i < numberOfJoints_; i++) {
            allBrakesOff = arm_.at(i)->isBrakeActive() ? false : allBrakesOff;
        }
    }
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->setVelocity(0.0);
    }
    logger_->debug("Initialized");
    initialized_ = true;
    return true;
}

bool SchunkArm::deinitialize() {
    logger_->debug("deinitialize()");
    if (!initialized_) {
        logger_->debug("SchunkArm already deinitialized");
        return false;
    }
    // cancels the state broadcasting of the motor, makes it easier to reinitialize later
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->getStatePeriodic(0);
    }
    stopControlLoop();
    for (int i=0; i < numberOfJoints_; i++) {
        if (!arm_.at(i)->deinitialize()) {
            logger_->critical("Deinitialization of motor" + std::to_string(i+3) + " failed!");
            return false;
        }
    }
    if (!socket_->deinitialize()) {
        logger_->error("Denitialization of CAN socket failed");
        return false;
    }
    logger_->debug("Deinitialized");
    initialized_ = false;
    return true;
}

boost::optional<utility::types::JointPositions> SchunkArm::getJointPositions() {
    if (!initialized_) {
        logger_->debug("Unable to perform getJointPositions(), arm was not initialized");
        return boost::none;
    }
    return jointPositions_;
}

boost::optional<utility::types::JointVelocities> SchunkArm::getJointVelocities() {
    if (!initialized_) {
        logger_->debug("Unable to perform getJointVelocities(), arm was not initialized");
        return boost::none;
    }
    return jointVelocities_;
}

boost::optional<crf::utility::types::JointForceTorques> SchunkArm::getJointForceTorques() {
    logger_->warn("getJointForceTorques() not supported");
    return boost::none;
}

boost::optional<utility::types::TaskPose> SchunkArm::getTaskPose() {
    logger_->warn("getTaskPose() not supported");
    return boost::none;
}

boost::optional<utility::types::TaskVelocity> SchunkArm::getTaskVelocity() {
    logger_->warn("getTaskVelocity() not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> SchunkArm::getTaskForceTorque() {
    logger_->warn("setTaskForceTorque() not supported");
    return boost::none;
}

bool SchunkArm::setJointPositions(const utility::types::JointPositions& jointPositions) {
    logger_->debug("setJointPositions()");
    if (!initialized_) {
        logger_->debug("Unable to setJointPositions(), arm was not initialized");
        return false;
    }
    auto jointsConfiguration = configuration_->getJointsConfiguration();
    for (int i=0; i < numberOfJoints_; i++) {
        if ((jointPositions[i] > jointsConfiguration[i].maximumPosition) ||
            (jointPositions[i] < jointsConfiguration[i].minimumPosition)) {
                logger_->warn("Requested position out of range.");
                return false;
            }
        if (arm_.at(i)->getErrorCode() != boost::none) {
            logger_->warn("Motor {} is in error, can't set position.", i);
            return false;
        }
    }

    // The target velocity needs to be set, otherwise it will use previous velocity
    // if the previous velocity was 0, the arm will not move
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->setTargetVelocityToPercentage(defaultVelocityPercentage_);
        std::this_thread::sleep_for(std::chrono::microseconds(timeOutForSafeWriteInUSec_));
    }

    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->setPosition(jointPositions[i]);
        std::this_thread::sleep_for(std::chrono::microseconds(timeOutForSafeWriteInUSec_));
    }
    return true;
}

bool SchunkArm::setJointVelocities(const utility::types::JointVelocities& jointVelocities) {
    if (!initialized_) {
        logger_->debug("Unable to setJointVelocities(), arm was not initialized");
        return false;
    }
    auto jointsConfiguration = configuration_->getJointsConfiguration();
    for (int i=0; i < numberOfJoints_; i++) {
        if (fabs(jointVelocities[i]) > jointsConfiguration[i].maximumVelocity) {
            logger_->warn("Requested position out of range.");
            return false;
        }
        if (arm_.at(i)->getErrorCode() != boost::none) {
            logger_->warn("Motor {} is in error, can't set velocity.", i);
            return false;
        }
    }
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->setVelocity(jointVelocities[i]);
    }
    return true;
}

bool SchunkArm::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques() not supported");
    return false;
}

bool SchunkArm::setTaskPose(const utility::types::TaskPose& goalPosition) {
    logger_->warn("setTaskPose() not supported");
    return false;
}

bool SchunkArm::setTaskVelocity(const utility::types::TaskVelocity& velocity, bool TCP) {
    logger_->warn("setTaskVelocity() not supported");
    return false;
}

bool SchunkArm::stopArm() {
    logger_->debug("stopArm()");
    if (!initialized_) {
        logger_->debug("Unable to stopArm(), arm was not initialized");
        return false;
    }
    return hardStop();
}

bool SchunkArm::enableBrakes() {
    logger_->warn("enableBrakes() not supported");
    return false;
}

bool SchunkArm::disableBrakes() {
    logger_->warn("disableBrakes() not supported");
    return false;
}

std::shared_ptr<crf::actuators::robotarm::RobotArmConfiguration> SchunkArm::getConfiguration() {
    logger_->debug("getConfiguration()");
    return configuration_;
}

bool SchunkArm::startControlLoop() {
    logger_->debug("startControlLoop()");
    if (controlLoopThread_.joinable()) {
        logger_->warn("mainControlLoop already started");
        return false;
    }
    runControlLoop_ = true;
    controlLoopThread_ = std::thread([this]() {
        mainControlLoop();
    });
    return true;
}

bool SchunkArm::stopControlLoop() {
    logger_->debug("stopControlLoop()");
    if (!controlLoopThread_.joinable()) {
        logger_->warn("mainControlLoop was not started");
        return false;
    }
    runControlLoop_ = false;
    controlLoopThread_.join();
    return true;
}

bool SchunkArm::updateRobotState() {
    auto endTime = std::chrono::high_resolution_clock::now() +
        std::chrono::milliseconds(mainLoopPeriodMilliseconds_);
    int numberOfMsg = 0;
    // Reads out the CAN messages in a cycle
    while (true) {
        auto now = std::chrono::high_resolution_clock::now();
        if (now > endTime || numberOfMsg == 12) {
            break;
        }
        can_frame recv;
        if (socket_->read(&recv) == -1) {
            logger_->debug("No messsage in CAN, it is probably down");
            return false;
        }
        int motorID = (recv.can_id & GET_DEVICE_CODE);
        // if the message is from a joint
        if ((motorID >= JOINT1) && (motorID <= JOINT6)) {
            motorID -= JOINT1;
            // if state message write it
            if ((recv.data[1] == FRAG_START) && (recv.data[2] == GET_STATE)) {
                arm_.at(motorID)->parseStatePeriodic(recv);
                numberOfMsg++;
            } else if (recv.data[1] == FRAG_END) {
                arm_.at(motorID)->parseStatePeriodic(recv);
                numberOfMsg++;
                boost::optional<uint8_t> error = arm_.at(motorID)->getErrorCode();
                if (error) {
                    std::string errorMsg = SchunkDevice::translateErrorToString(error.get());
                    arm_.at(motorID)->handleError(errorMsg);
                }
            } else if ((recv.data[2] == ASCII_O) && (recv.data[3] == ASCII_K)) {  // hex code for OK
                continue;  // the message is an acknowledgement for a previous command
            } else if (recv.data[2] == INFO_FAILED) {
                logger_->debug("Message " + SchunkDevice::translateMessageCodeToString(recv.data[1])
                    + " failed on motor " + std::to_string(motorID));
            } else if (recv.data[1] == CMD_ERROR) {
                std::string error = SchunkDevice::translateErrorToString(recv.data[2]);
                arm_.at(motorID)->handleError(error);
            } else {
                logger_->debug("Recieved unexpected message on motor " + std::to_string(motorID+3));
                SchunkDevice::printCanPacket(recv);
            }
        } else if (motorID == GRIPPER) {
            if (gripper_ != nullptr) {
                gripper_->updateState(recv);
                continue;
            }
        } else {
            logger_->debug("Recieved a message with a wong motorID : " + std::to_string(motorID));
            SchunkDevice::printCanPacket(recv);
        }
    }
    return true;
}

bool SchunkArm::softStop() {
    logger_->debug("softStop()");
    for (int i=0; i < numberOfJoints_; i++) {
        arm_.at(i)->setVelocity(0);
    }
    logger_->debug("SoftBreaks applied to robot arm, because no new command");
    return true;
}

bool SchunkArm::hardStop() {
    logger_->debug("hardStop()");
    for (int i=0; i < numberOfJoints_; i++) {
        if (!arm_.at(i)->applyBreak()) {
            logger_->critical("Breaks could not be applied");
            return false;
        }
    }
    logger_->debug("Breaks applied to robot arm, because no new command");
    return true;
}

void SchunkArm::mainControlLoop() {
    logger_->debug("mainControlLoop started");

    while (runControlLoop_) {
        // Update robot state
        if (updateRobotState()) {
            for (int i=0; i < numberOfJoints_; i++) {
                this->jointPositions_[i] = arm_.at(i)->getMotorPosition().get();
                this->jointVelocities_[i] = arm_.at(i)->getMotorVelocity().get();
            }
        }
    }
}

}  // namespace crf::actuators::schunkarm
