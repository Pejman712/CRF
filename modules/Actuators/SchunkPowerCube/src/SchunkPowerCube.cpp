/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Giacomo Lunghi CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include <string>
#include <vector>
#include <memory>
#include <linux/can.h>
#include <boost/optional.hpp>

#include "SchunkPowerCube/SchunkPowerCube.hpp"

namespace crf {
namespace robots {
namespace schunkpowercube {

SchunkPowerCube::SchunkPowerCube(std::shared_ptr<communication::cansocket::ICANSocket> socket,
    const nlohmann::json& robotConfigFileName) :
    logger_("SchunkPowerCube"),
    socket_(socket),
    robotConfigFileName_(robotConfigFileName),
    configuration_(new SchunkPowerCubeConfiguration),
    framesHandlers_(),
    initialized_(false),
    brakesStatus_(),
    movingStatus_(),
    errorStatus_(),
    hardwareOkStatus_(),
    jointPositions_(1),
    jointVelocities_(1),
    jointForceTorques_(1),
    jointsOffset_(),
    stopThread_(false),
    canReceiverThread_(),
    canSenderThread_(),
    jointCanIDCorrespondance_(),
    motors_(),
    brakesEnabled_(false),
    updateFrequency_(std::chrono::milliseconds(10)) {
    logger_->debug("CTor");
    framesHandlers_.insert({ 0x27, &SchunkPowerCube::msgStatusHandler });
    framesHandlers_.insert({ 0x3C, &SchunkPowerCube::msgPositionHandler });
    framesHandlers_.insert({ 0x41, &SchunkPowerCube::msgVelocityHandler });
    framesHandlers_.insert({ 0x4D, &SchunkPowerCube::msgCurrentHandler });
    framesHandlers_.insert({ 0x41, &SchunkPowerCube::msgSetAckHandler });
}

SchunkPowerCube::~SchunkPowerCube() {
    logger_->debug("DTor");

    if (initialized_)
        deinitialize();
}

bool SchunkPowerCube::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Already initialized");
        return false;
    }
    if (!socket_->initialize()) {
        logger_->error("Initialization of CAN socket failed");
        return false;
    }
    if (!configuration_->parse(robotConfigFileName_)) {
        logger_->error("Failed to read the configuration file");
        return false;
    }
    brakesStatus_.resize(configuration_->getNumberOfJoints());
    movingStatus_.resize(configuration_->getNumberOfJoints());
    errorStatus_.resize(configuration_->getNumberOfJoints());
    hardwareOkStatus_.resize(configuration_->getNumberOfJoints());
    jointPositions_ = utility::types::JointPositions(configuration_->getNumberOfJoints());
    jointVelocities_ = utility::types::JointVelocities(configuration_->getNumberOfJoints());
    jointForceTorques_ = crf::utility::types::JointForceTorques(
        configuration_->getNumberOfJoints());
    jointsOffset_ = configuration_->getJointsOffset();

    stopThread_ = false;
    canReceiverThread_ = std::thread(&SchunkPowerCube::canReceiver, this);
    canSenderThread_ = std::thread(&SchunkPowerCube::canSender, this);

    std::vector<int> jointsCanID = configuration_-> getJointsCanID();
    for (int jointID = 0; jointID < configuration_->getNumberOfJoints(); jointID++) {
        jointCanIDCorrespondance_.insert({jointsCanID[jointID], jointID});
        std::shared_ptr<SchunkPowerCubeDevice> motor;
        motor.reset(new SchunkPowerCubeDevice(jointsCanID[jointID], socket_));
        motor->initialize();
        motor->reset();
        motors_.push_back(motor);
    }

    initialized_ = true;
    return true;
}

bool SchunkPowerCube::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    enableBrakes();

    stopThread_ = true;
    canReceiverThread_.join();
    canSenderThread_.join();
    if (!socket_->deinitialize()) {
        logger_->error("Denitialization of CAN socket failed");
        return false;
    }

    initialized_ = false;
    return true;
}

boost::optional<crf::utility::types::JointPositions> SchunkPowerCube::getJointPositions() {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    utility::types::JointPositions adjustedJointPositions(configuration_->getNumberOfJoints());
    for (int jointID = 0; jointID < configuration_->getNumberOfJoints(); jointID++) {
        adjustedJointPositions(jointID) = jointPositions_(jointID) + jointsOffset_[jointID];
    }
    return adjustedJointPositions;
}

boost::optional<crf::utility::types::JointVelocities> SchunkPowerCube::getJointVelocities()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    return jointVelocities_;
}

boost::optional<crf::utility::types::JointForceTorques> SchunkPowerCube::getJointForceTorques()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return boost::none;
    }
    return jointForceTorques_;
}

boost::optional<crf::utility::types::TaskPose> SchunkPowerCube::getTaskPose()  {
    logger_->warn("getTaskPose: operation not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskVelocity> SchunkPowerCube::getTaskVelocity()  {
    logger_->warn("getTaskVelocity: operation not supported");
    return boost::none;
}

boost::optional<crf::utility::types::TaskForceTorque> SchunkPowerCube::getTaskForceTorque() {
    logger_->warn("getTaskForceTorque: operation not supported");
    return boost::none;
}

bool SchunkPowerCube::setJointPositions(
    const crf::utility::types::JointPositions& jointPositions) {
    logger_->warn("setJointPositions: operation not supported");
    return true;
}

bool SchunkPowerCube::setJointVelocities(
    const crf::utility::types::JointVelocities& jointVelocities) {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    if (brakesEnabled_) {
        logger_->warn("Brakes are enabled, disable the brakes first");
        return false;
    }

    for (int jointID = 0; jointID < configuration_->getNumberOfJoints(); jointID++) {
        if (std::fabs(jointVelocities(jointID)) >
            configuration_->getJointsConfiguration()[jointID].maximumVelocity) {
            logger_->error(
                "The requested velocity ({}) for the joint {} is bigger than the maximum limit",
                jointVelocities(jointID), jointID);
            return false;
        }
    }

    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->setVelocity(jointVelocities(i));
    }

    return true;
}

bool SchunkPowerCube::setJointForceTorques(
    const crf::utility::types::JointForceTorques& jointForceTorques) {
    logger_->warn("setJointForceTorques: operation not supported");
    return false;
}

bool SchunkPowerCube::setTaskPose(
    const crf::utility::types::TaskPose& position) {
    logger_->warn("setTaskPose: operation not supported");
    return false;
}

bool SchunkPowerCube::setTaskVelocity(
    const crf::utility::types::TaskVelocity& velocity, bool TCP)  {
    logger_->warn("setTaskVelocity: operation not supported");
    return false;
}

bool SchunkPowerCube::stopArm()  {
    if (!initialized_) {
        logger_->warn("Not initialized");
        return false;
    }

    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->setVelocity(0);
    }

    return true;
}

bool SchunkPowerCube::enableBrakes() {
    brakesEnabled_ = true;
    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->fastStop();
    }

    return true;
}

bool SchunkPowerCube::disableBrakes()  {
    brakesEnabled_ = false;
    for (size_t i=0; i < motors_.size(); i++) {
        motors_[i]->setVelocity(0);
    }

    return true;
}

std::shared_ptr<robotarm::RobotArmConfiguration> SchunkPowerCube::getConfiguration()  {
    return configuration_;
}

void SchunkPowerCube::canReceiver() {
    while (!stopThread_) {
        can_frame frame = {};
        if (socket_->read(&frame) != 16) {
            continue;
        }

        char msgCode = frame.data[1];

        auto iterator = framesHandlers_.find(msgCode);
        if (iterator == framesHandlers_.end()) {
            /*printf("id: %x, dlc: %x, data: %x %x %x %x %x %x %x %x\n",
                frame.can_id, frame.can_dlc, frame.data[0],
                frame.data[1], frame.data[2], frame.data[3],
                frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            logger_->warn("Not known message: {0}", msgCode);*/
            continue;
        }

        int canId = frame.can_id & 0x0F;
        //  There us risk accessing non-existing elements via operator []. Better use find
        int jointNum = jointCanIDCorrespondance_[canId];
        iterator->second(this, frame, jointNum);
    }
}

void SchunkPowerCube::canSender() {
    while (!stopThread_) {
        auto start = std::chrono::high_resolution_clock::now();
        for (size_t i=0; i < motors_.size(); i++) {
            motors_[i]->getPosition();
            motors_[i]->getVelocity();
            motors_[i]->getCurrent();
            motors_[i]->getStatus();
        }

        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((updateFrequency_ - elapsed).count() > 0) {
            std::this_thread::sleep_for(updateFrequency_ - elapsed);
        } else {
            logger_->warn("controlLoop(): execution time longer than updateFrequency_");
        }
    }
}

bool SchunkPowerCube::msgStatusHandler(const can_frame& frame, int jointNumber) {
    brakesStatus_[jointNumber] = (frame.data[3] >> 1) & 0x01;
    movingStatus_[jointNumber] = (frame.data[3] >> 3) & 0x01;
    errorStatus_[jointNumber] = frame.data[5] != 0x00;
    hardwareOkStatus_[jointNumber] = (frame.data[2] >> 1) & 0x01;
    return true;
}

bool SchunkPowerCube::msgPositionHandler(const can_frame& frame, int jointNumber) {
    float radians;
    std::memcpy(&radians, frame.data+2, 4);
    jointPositions_(jointNumber) = radians;
    return true;
}

bool SchunkPowerCube::msgVelocityHandler(const can_frame& frame, int jointNumber) {
    float radians;
    std::memcpy(&radians, frame.data+2, 4);
    jointVelocities_(jointNumber) = radians;
    return true;
}

bool SchunkPowerCube::msgCurrentHandler(const can_frame& frame, int jointNumber) {
    float amperes;
    std::memcpy(&amperes, frame.data+2, 4);
    jointForceTorques_(jointNumber) = amperes;
    return true;
}

bool SchunkPowerCube::msgSetAckHandler(const can_frame& frame, int jointNumber) {
    return true;
}

}  // namespace schunkpowercube
}  // namespace robots
}  // namespace crf
