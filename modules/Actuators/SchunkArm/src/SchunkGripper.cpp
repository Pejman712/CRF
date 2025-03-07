/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

#include "SchunkArm/SchunkGripper.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include "CANSocket/ICANSocket.hpp"

#include <boost/optional.hpp>
#include <memory>
#include <string>

namespace crf::actuators::schunkarm {

SchunkGripper::SchunkGripper(std::shared_ptr<ICANSocket> sock):
        logger_("SchunkGripper"),
        socket_(sock),
        gripperDevice_(sock, GRIPPER),
        initialized_(false),
        getStateTimePeriodMilisec_(40),
        defaultGripperVelocity_(20),
        gripperOpenThreshold_(2),
        gripperClosedThreshold_(100 - gripperOpenThreshold_),
        gripperDirection_(0) {
}

bool SchunkGripper::initialize() {
    if (initialized_) {
        logger_->debug("Unable to initialize(), gripper already initialized");
        return false;
    }

    // if you connect to can, the devices might be making too much noise to use it
    if (!SchunkDevice::cleanCanBus(socket_)) {
        logger_->critical("Canbus could not be cleaned");
        return false;
    }

    if (!gripperDevice_.initialize()) {
        return false;
    }

    // we have to reduce the current so the gripper does not overheat
    if (!gripperDevice_.setTargetCurrentToPercentage(25)) {
        return false;
    }

    // this is the default value of the gripper
    if (gripperDevice_.setTargetVelocityToPercentage(defaultGripperVelocity_)) {
        targetVelocityPercentage_ = defaultGripperVelocity_;
    } else {
        return false;
    }

    gripperMaxPosition_ = gripperDevice_.getMaxPosition().get();
    // makes the gripper broadcast its state every x millisec
    gripperDevice_.getStatePeriodic(getStateTimePeriodMilisec_);
    initialized_ = true;
    logger_->debug("SchunkGripper initialized");
    return true;
}

bool SchunkGripper::deinitialize() {
    if (!initialized_) {
        logger_->debug("Unable to deinitialize(), gripper is not initialized");
        return false;
    }

    gripperDevice_.getStatePeriodic(0);  // makes the gripper shut up
    if (!gripperDevice_.deinitialize()) {
        return false;
    }
    initialized_ = false;
    logger_->debug("SchunkGripper deinitialized");
    return true;
}

boost::optional<float> SchunkGripper::getPosition() {
    if (!initialized_) {
        logger_->debug("Unable to perform getPosition(), gripper was not initialized");
        return boost::none;
    }
    float gripperPosPercentage =
            gripperDevice_.getMotorPosition().get() / gripperMaxPosition_ * 100;

    // The gripper minimum position can be a bit less than zero, about -1 degree
    // but this is a "bug" in the SchunkMotion protocol
    if (gripperPosPercentage < 0) {
        gripperPosPercentage = 0;
    }

    // we need to flip it, for the device 0 is closed, for interface is 100
    float gripperPosInverted = 100 - gripperPosPercentage;
    return gripperPosInverted;
}

bool SchunkGripper::isGrasping() {
    if (!initialized_) {
        logger_->debug("Unable to isGrasping(), gripper was not initialized");
        return false;
    }
    // when the arm is closed or open, sometimes it gives back motion blocked
    float currentPos = getPosition().get();
    if ((currentPos > gripperClosedThreshold_) ||
        (currentPos < gripperOpenThreshold_)) {
        return false;
    }

    return gripperDevice_.isMoveBlocked();
}

bool SchunkGripper::setPosition(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setPosition(), gripper was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage < 0)) {
        logger_->debug("Wrong input for setPosition(), percentage should be between 0 and 100");
        return false;
    }

    if (targetVelocityPercentage_ == 0) {
        if (gripperDevice_.setTargetVelocityToPercentage(defaultGripperVelocity_)) {
            targetVelocityPercentage_ = defaultGripperVelocity_;
        } else {
            logger_->debug("Unable to setPosition(), unable to change gripper velocity from 0");
            return false;
        }
    }

    float invertedPercentage = 100 - percentage;
    float posInRadians = gripperMaxPosition_ * (invertedPercentage / 100);

    return gripperDevice_.setPosition(posInRadians);
}

bool SchunkGripper::setPosition(GripperState state) {
    if (state == Gripper_Open) {
        return setPosition(0);
    } else if (state == Gripper_Closed) {
        return setPosition(100);
    } else {
        logger_->debug("Wrong input for setPosition(GripperState state)");
        return false;
    }
}

bool SchunkGripper::setGraspingForce(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setGraspingForce(), gripper was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage <0)) {
        logger_->debug("Wrong input for setGraspingForce(),"
                       " percentage should be between 0 and 100");
        return false;
    }

    return gripperDevice_.setTargetCurrentToPercentage(percentage);
}

bool SchunkGripper::setVelocity(float percentage) {
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

    // you set target velocity instead of actually movement velocity
    // if you set directly the movement the gripper will reach zero and get stuck
    // this is a bug in Schunk motion API
    if (gripperDevice_.setTargetVelocityToPercentage(std::fabs(percentage))) {
        targetVelocityPercentage_ = std::fabs(percentage);
    } else {
        return false;
    }

    if (percentage < 0) {
        if (gripperDirection_ != -1) {
            gripperDirection_ = -1;
            return setPosition(Gripper_Open);
        }
    } else {
        if (gripperDirection_ != 1) {
            gripperDirection_ = 1;
            return setPosition(Gripper_Closed);
        }
    }

    return true;
}

bool SchunkGripper::updateState(can_frame messageToGripper) {
    if (!initialized_) {
        logger_->debug("Unable to updateState(), gripper was not initialized");
        return false;
    }

    int packetMotorID = messageToGripper.can_id & GET_DEVICE_CODE;
    if (packetMotorID != GRIPPER) {
        logger_->debug("Unable to perform updateState() wrong motor code : "
                       + std::to_string(packetMotorID) + " != "
                       + std::to_string(GRIPPER));
        return false;
    }

    if ((messageToGripper.data[1] == FRAG_START) &&
        (messageToGripper.data[2] == GET_STATE)) {
        gripperDevice_.parseStatePeriodic(messageToGripper);
        return true;
    } else if (messageToGripper.data[1] == FRAG_END) {
        gripperDevice_.parseStatePeriodic(messageToGripper);
        boost::optional<uint8_t> error = gripperDevice_.getErrorCode();
        if (error) {
            std::string errorMsg = SchunkDevice::translateErrorToString(error.get());
            return gripperDevice_.handleError(errorMsg);
        }
        return true;
    } else if ((messageToGripper.data[2] == ASCII_O) &&
               (messageToGripper.data[3] == ASCII_K)) {  // hex code for OK
        return true;  // the message is an acknowledgement for a previous command
    } else if (messageToGripper.data[2] == INFO_FAILED) {
        logger_->debug("Message " +
                       SchunkDevice::translateMessageCodeToString(messageToGripper.data[1]) +
                       " failed on gripper " + std::to_string(packetMotorID));
        return true;
    } else if (messageToGripper.data[1] == MOVE_POS) {
        // this message contains a 4 byte int
        // it is the time needed to perform the position movement in milliseconds
        return true;
    } else if (messageToGripper.data[1] == CMD_POS_REACHED) {
        // this message contains a 4 byte int
        // current position in millidegs
        return true;
    } else if (messageToGripper.data[1] == CMD_MOVE_BLOCKED) {
        // this message contains a 4 byte int
        // it is the position where the movement was blocked
        return true;
    } else if (messageToGripper.data[1] == CMD_ERROR) {
        std::string error = SchunkDevice::translateErrorToString(messageToGripper.data[2]);
        return gripperDevice_.handleError(error);
    } else {
        logger_->debug("Recieved unexpected message on gripper ");
        SchunkDevice::printCanPacket(messageToGripper);
        return false;
    }
}

bool SchunkGripper::stopGripper() {
    return gripperDevice_.applyBreak();
}

}  // namespace crf::actuators::schunkarm
