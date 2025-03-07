/* © Copyright CERN 2021.  All rights reserved. This software is released under a CERN proprietary software licence.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Thomas Breant CERN EN/SMM/MRO
 *
 *  ==================================================================================================
*/

#include "Gripper/SchunkGripperCANOpen/SchunkGripperCANOpen.hpp"

#include <boost/optional.hpp>
#include <memory>
#include <string>

namespace crf {
namespace actuators {
namespace gripper {

SchunkGripperCANOpen::SchunkGripperCANOpen(
    std::shared_ptr<communication::cansocket::ICANSocket> sock,
    std::shared_ptr<crf::devices::canopendevices::CANOpenContext> ctx):
        socket_(sock),
        ctx_(ctx),
        initialized_(false),
        defaultGripperVelocity_(0),
        gripperDirection_(0),
        logger_("SchunkGripperCANOpen") {
    logger_->debug("CTor");
    gripperDevice_ = std::make_shared<crf::devices::canopendevices::ERB>(
        0xC, socket_, crf::devices::canopendevices::ModesOfOperation::ProfilePositionMode);
    // 0xC is the CAN ID stablished by Schunk for the gripper -> check Schunk Commands
}

SchunkGripperCANOpen::~SchunkGripperCANOpen() {
    logger_->debug("DTor");
    deinitialize();
}

bool SchunkGripperCANOpen::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->debug("Unable to initialize(), gripper already initialized");
        return false;
    }
    ctx_->addDevice(gripperDevice_);
    if (!gripperDevice_->initialize()) {
        logger_->error("Could not initialize the gripper");
        return false;
    }
    if (!gripperDevice_->enableOperation()) {
        logger_->error("Could not enable the operation on the gripper");
        return false;
    }
    gripperMaxPosition_ = gripperDevice_->getPositionLimits().second;
    gripperMaxVelocity_ = gripperDevice_->getMaximumVelocity();
    defaultGripperVelocity_ = gripperMaxVelocity_/2;

    // this is the default value of the gripper
    if (gripperDevice_->setProfileVelocity(defaultGripperVelocity_)) {
        targetVelocity_ = defaultGripperVelocity_;
    } else {
        return false;
    }
    initialized_ = true;
    return true;
}

bool SchunkGripperCANOpen::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->debug("Unable to deinitialize(), gripper is not initialized");
        return false;
    }
    if (!gripperDevice_->deinitialize()) {
        return false;
    }
    initialized_ = false;
    return true;
}

boost::optional<float> SchunkGripperCANOpen::getPosition() {
    logger_->debug("getPosition");
    if (!initialized_) {
        logger_->debug("Unable to perform getPosition(), gripper was not initialized");
        return boost::none;
    }
    float gripperPosPercentage = gripperDevice_->getPosition().value()/gripperMaxPosition_*100;

    // The gripper minimum position can be a bit less than zero, about -1 degree
    // but this is a "bug" in the SchunkMotion protocol
    if (gripperPosPercentage < 0) {
        gripperPosPercentage = 0;
    }

    // We need to flip it, for the device 0 is closed, for interface is 100
    float gripperPosInverted = 100 - gripperPosPercentage;
    return gripperPosInverted;
}

bool SchunkGripperCANOpen::isGrasping() {
    logger_->debug("isGrasping");
    if (!initialized_) {
        logger_->debug("Unable to isGrasping(), gripper was not initialized");
        return false;
    }
    // When the arm is closed or open, sometimes it gives back motion blocked
    float currentPos = getPosition().value();
    if ((currentPos > gripperClosedThreshold_) ||
        (currentPos < gripperOpenThreshold_)) {
        return false;
    }
    return (!gripperDevice_->positionReached() && gripperDevice_->getVelocity().value() == 0);
}

bool SchunkGripperCANOpen::setPosition(float percentage) {
    logger_->debug("setPosition(pos)");
    if (!initialized_) {
        logger_->debug("Unable to setPosition(), gripper was not initialized");
        return false;
    }
    if ((percentage > 100) || (percentage < 0)) {
        logger_->debug("Wrong input for setPosition(), percentage should be between 0 and 100");
        return false;
    }

    float invertedPercentage = 100 - percentage;
    float posInMdeg = gripperMaxPosition_ * (invertedPercentage / 100);
    return gripperDevice_->setPosition(posInMdeg, defaultGripperVelocity_, false);
}

bool SchunkGripperCANOpen::setPosition(float position, float velocity) {
    logger_->debug("setPosition(pos, vel)");
    if (!initialized_) {
        logger_->debug("Unable to setPosition(), gripper was not initialized");
        return false;
    }
    if ((position > 100) || (position < 0)) {
        logger_->debug("Wrong input for setPosition(), position percentage should be between 0 and 100");  // NOLINT
        return false;
    }
    if ((velocity > 100) || (velocity < 0)) {
        logger_->debug("Wrong input for setPosition(), velocity percentage should be between 0 and 100");  // NOLINT
        return false;
    }

    float invertedPercentage = 100 - position;
    float pos = gripperMaxPosition_ * (invertedPercentage / 100);
    float vel = gripperMaxVelocity_ * (velocity / 100);
    return gripperDevice_->setPosition(pos, vel, false);
}


bool SchunkGripperCANOpen::setPosition(GripperState state) {
    if (state == Gripper_Open) {
        return setPosition(0);
    } else if (state == Gripper_Closed) {
        return setPosition(100);
    }
    logger_->debug("Wrong input for setPosition(GripperState state)");
    return false;
}

bool SchunkGripperCANOpen::setGraspingForce(float percentage) {
    logger_->debug("setGraspingForce");
    if (!initialized_) {
        logger_->debug("Unable to setGraspingForce(), gripper was not initialized");
        return false;
    }
    logger_->warn("setGraspingForce() was not implemented");
    return false;
}

bool SchunkGripperCANOpen::setVelocity(float percentage) {
    logger_->debug("setVelocity");
    if (!initialized_) {
        logger_->debug("Unable to setVelocity(), gripper was not initialized");
        return false;
    }
    if (std::fabs(percentage) > 100) {
        logger_->debug("Wrong input for setVelocity(), percentage should be between -100 and 100");
        return false;
    }
    if (percentage == 0) {
        gripperDirection_ = 0;
        return stopGripper();
    }

    // You set target velocity instead of actually movement velocity
    // If you set directly the movement the gripper will reach zero and get stuck,
    // this is a bug in Schunk motion API

    if (percentage < 0) {
        if (gripperDirection_ != -1) {
            gripperDirection_ = -1;
            return setPosition(gripperOpenThreshold_, std::fabs(percentage));
        }
    } else {
        if (gripperDirection_ != 1) {
            gripperDirection_ = 1;
            return setPosition(gripperClosedThreshold_, std::fabs(percentage));
        }
    }
    return true;
}

bool SchunkGripperCANOpen::stopGripper() {
    logger_->debug("stopGripper");
    return gripperDevice_->stop();
}

}  // namespace gripper
}  // namespace actuators
}  // namespace crf
