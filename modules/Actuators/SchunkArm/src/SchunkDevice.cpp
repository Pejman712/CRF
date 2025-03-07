/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
* Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
*
* Author: Zsolt Pasztori CERN EN/SMM/MRO
*
*  ==================================================================================================
*/

#include "SchunkArm/SchunkDevice.hpp"
#include "SchunkArm/SchunkCommands.hpp"
#include <sstream>
#include <thread>
#include <string>
#include <memory>

namespace crf::actuators::schunkarm {

SchunkDevice::SchunkDevice(std::shared_ptr<ICANSocket> sock, int id):
        millidegreeToRad_(M_PI / 180 /1000),
        radToMilliDeg_(180.0 / M_PI *1000),
        socket_(sock),
        canID_(id),
        initialized_(false),
        logger_("SchunkMotor"+ std::to_string(id)) {
}

SchunkDevice::~SchunkDevice() {
}

bool SchunkDevice::initialize() {
    if (initialized_) {
        logger_->debug("SchunkDevice already initialized");
        return false;
    }

    if (!getState()) {
        return false;
    }

    if (!referenced_) {
        logger_->critical("Motor is not referenced!");
        return false;
    }
    if (error_) {
        std::string errorString = translateErrorToString(errorCode_);
        if (!handleError(errorString)) {
            logger_->critical("Critical error, Motor could not be initialized!");
            return false;
        }
    }
    if (moving_) {
        applyBreak();
    }

    if (!getConfigurationParameters()) {
        return false;
    }
    initialized_ = true;

    // The original values are set to 10%, which makes the robot accelerate slowly
    setTargetVelocityToPercentage(80);
    setTargetAccelerationToPercentage(80);
    setTargetCurrentToPercentage(40);

    return true;
}

bool SchunkDevice::deinitialize() {
    if (!initialized_) {
        logger_->debug("SchunkDevice already deinitialized");
        return false;
    }

    if (!brakeOn_) {
        applyBreak();
    }
    initialized_ = false;
    return true;
}

bool SchunkDevice::getConfigurationParameters() {
    // before each write you should sleep a millisec, otherwise the device will be kicked of CAN
    std::chrono::milliseconds duration(1);

    int packetsRecieved = 0;

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x03;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = GET_CONFIG;

    frame.data[2] = MAX_POS;
    std::this_thread::sleep_for(duration);
    if (socket_->write(&frame) != 16) {
        return false;
    }

    frame.data[2] = MIN_POS;
    std::this_thread::sleep_for(duration);
    if (socket_->write(&frame) != 16) {
        return false;
    }
    frame.data[2] = MAX_SPEED;
    std::this_thread::sleep_for(duration);
    if (socket_->write(&frame) != 16) {
        return false;
    }
    frame.data[2] = MAX_ACCELERATION;
    std::this_thread::sleep_for(duration);
    if (socket_->write(&frame) != 16) {
        return false;
    }
    frame.data[2] = MAX_CURRENT;
    std::this_thread::sleep_for(duration);
    if (socket_->write(&frame) != 16) {
        return false;
    }

    while (packetsRecieved != 5) {
        can_frame recv_frame{};
        int returnVal = socket_->read(&recv_frame);
        if (returnVal == -1) {
            logger_->critical("Could not recieve message trough CAN, getConfigurationParameters()");
            return false;
        }

        if (recv_frame.can_id == MESSAGE_FROM_DEVICE + canID_ && recv_frame.data[1] == GET_CONFIG) {
            int value = 0;
            std::memcpy(&value, &recv_frame.data[3], sizeof(int));

            if (recv_frame.data[2] == MAX_POS) {
                maxPosition_ = value;
                packetsRecieved++;
            } else if (recv_frame.data[2] == MIN_POS) {
                minPosition_ = value;
                packetsRecieved++;
            } else if (recv_frame.data[2] == MAX_SPEED) {
                maxVelocity_ = value;
                packetsRecieved++;
            } else if (recv_frame.data[2] == MAX_ACCELERATION) {
                maxAcceleration_ = value;
                packetsRecieved++;
            } else if (recv_frame.data[2] == MAX_CURRENT) {
                maxCurrent_ = value;
                packetsRecieved++;
            } else {
                logger_->debug("Unexpected message during getConfigurationParameters()");
                printCanPacket(recv_frame);
            }
        } else {
            logger_->debug("Unexpected message during getConfigurationParameters()");
            printCanPacket(recv_frame);
        }
    }
    return true;
}

bool SchunkDevice::printCanPacket(const can_frame& frame) {
    crf::utility::logger::EventLogger logger_("SchunkDeviceStaticFunction");
    int motorID = frame.can_id;
    std::stringstream sstream;
    sstream << "0x" << std::hex << motorID;
    std::string msg = "CAN ID:" + sstream.str() + ";";
    std::string msgCode = translateMessageCodeToString(frame.data[1]);
    msg += msgCode +";";
    if (msgCode == "CMD_INFO") {
        msg += translateInfoToString(frame.data[2]);
    } else if ((msgCode == "CMD_ERROR") ||
               (msgCode == "CMD_WARNING")) {
        msg += translateErrorToString(frame.data[2]);
    } else {
        for (int i = 2; i < frame.can_dlc; i++) {
            msg += "data[" + std::to_string(i) + "] : ";
            msg += std::to_string((unsigned int) frame.data[i]) + ";";
        }
    }
    logger_->debug(msg);
    return true;
}

std::string SchunkDevice::translateMessageCodeToString(uint8_t msg) {
    std::string message;
    switch (msg) {
        case GET_CONFIG:
            message = "GET_CONFIG";
            break;
        case SET_CONFIG:
            message = "SET_CONFIG";
            break;
        case GET_CONFIG_EXT:
            message = "GET_CONFIG_EXT";
            break;
        case SET_CONFIG_EXT:
            message = "SET_CONFIG_EXT";
            break;
        case FRAG_START:
            message = "FRAG_START";
            break;
        case FRAG_MIDDLE:
            message = "FRAG_MIDDLE";
            break;
        case FRAG_END:
            message = "FRAG_END";
            break;
        case FRAG_ACK:
            message = "FRAG_ACK";
            break;
        case CMD_ERROR:
            message = "CMD_ERROR";
            break;
        case CMD_WARNING:
            message = "CMD_WARNING";
            break;
        case CMD_INFO:
            message = "CMD_INFO";
            break;
        case CMD_ACK:
            message = "CMD_ACK";
            break;
        case CMD_FAST_STOP:
            message = "CMD_FAST_STOP";
            break;
        case CMD_STOP:
            message = "CMD_STOP";
            break;
        case CMD_REFERENCE:
            message = "CMD_REFERENCE";
            break;
        case CMD_MOVE_BLOCKED:
            message = "CMD_MOVE_BLOCKED";
            break;
        case CMD_POS_REACHED:
            message = "CMD_POS_REACHED";
            break;
        case GET_STATE:
            message = "GET_STATE";
            break;
        case GET_DETAILED_ERROR_INFO:
            message = "GET_DETAILED_ERROR_INFO";
            break;
        case CMD_REFERENCE_HAND:
            message = "CMD_REFERENCE_HAND";
            break;
        case GET_STATE_AXIS:
            message = "GET_STATE_AXIS";
            break;
        case SET_TARGET_VEL:
            message = "SET_TARGET_VEL";
            break;
        case SET_TARGET_ACC:
            message = "SET_TARGET_ACC";
            break;
        case SET_TARGET_JERK:
            message = "SET_TARGET_JERK";
            break;
        case SET_TARGET_CUR:
            message = "SET_TARGET_CUR";
            break;
        case SET_TARGET_TIME:
            message = "SET_TARGET_TIME";
            break;
        case SET_TARGET_POS:
            message = "SET_TARGET_POS";
            break;
        case SET_TARGET_POS_REL:
            message = "SET_TARGET_POS_REL";
            break;
        case MOVE_POS:
            message = "MOVE_POS";
            break;
        case MOVE_POS_TIME:
            message = "MOVE_POS_TIME";
            break;
        case MOVE_CURR:
            message = "MOVE_CURR";
            break;
        case MOVE_VEL:
            message = "MOVE_VEL";
            break;
        case MOVE_GRIP:
            message = "MOVE_GRIP";
            break;
        case MOVE_POS_REL:
            message = "MOVE_POS_REL";
            break;
        case MOVE_POS_TIME_REL:
            message = "MOVE_POS_TIME_REL";
            break;
        case CMD_REBOOT:
            message = "CMD_REBOOT";
            break;
        default:
            message = "Unknown command " + std::to_string(msg);
    }
    return message;
}

std::string SchunkDevice::translateErrorToString(uint8_t errorCode) {
    std::string error;
    switch (errorCode) {
        case NOT_REFERENCED:
            error = "NOT_REFERENCED";
            break;
        case ERROR_TEMP_LOW:
            error = "ERROR_TEMP_LOW";
            break;
        case ERROR_TEMP_HIGH:
            error = "ERROR_TEMP_HIGH";
            break;
        case ERROR_LOGIC_LOW:
            error = "ERROR_LOGIC_LOW";
            break;
        case ERROR_LOGIC_HIGH:
            error = "ERROR_LOGIC_HIGH";
            break;
        case ERROR_MOTOR_VOLTAGE_LOW:
            error = "ERROR_MOTOR_VOLTAGE_LOW";
            break;
        case ERROR_MOTOR_VOLTAGE_HIGH:
            error = "ERROR_MOTOR_VOLTAGE_HIGH";
            break;
        case ERROR_CABLE_BREAK:
            error = "ERROR_CABLE_BREAK";
            break;
        case ERROR_OVERSHOOT:
            error = "ERROR_OVERSHOOT";
            break;
        case ERROR_WRONG_RAMP_TYPE:
            error = "ERROR_WRONG_RAMP_TYPE";
            break;
        case ERROR_CONFIG_MEMORY:
            error = "ERROR_CONFIG_MEMORY";
            break;
        case ERROR_PROGRAM_MEMORY:
            error = "ERROR_PROGRAM_MEMORY";
            break;
        case ERROR_INVALID_PHRASE:
            error = "ERROR_INVALID_PHRASE";
            break;
        case ERROR_SOFT_LOW:
            error = "ERROR_SOFT_LOW";
            break;
        case ERROR_SOFT_HIGH:
            error = "ERROR_SOFT_HIGH";
            break;
        case ERROR_SERVICE:
            error = "ERROR_SERVICE";
            break;
        case ERROR_FAST_STOP:
            error = "ERROR_FAST_STOP";
            break;
        case ERROR_TOW:
            error = "ERROR_TOW";
            break;
        case ERROR_VPC3:
            error = "ERROR_VPC3";
            break;
        case ERROR_FRAGMENTATION:
            error = "ERROR_FRAGMENTATION";
            break;
        case ERROR_COMMUTATION:
            error = "ERROR_COMMUTATION";
            break;
        case ERROR_CURRENT:
            error = "ERROR_CURRENT";
            break;
        case ERROR_I2T:
            error = "ERROR_I2T";
            break;
        case ERROR_INITIALIZE:
            error = "ERROR_INITIALIZE";
            break;
        case ERROR_INTERNAL:
            error = "ERROR_INTERNAL";
            break;
        case ERROR_TOO_FAST:
            error = "ERROR_TOO_FAST";
            break;
        case ERROR_RESOLVER_CHECK_FAILED:
            error = "ERROR_RESOLVER_CHECK_FAILED";
            break;
        case ERROR_MATH:
            error = "ERROR_MATH";
            break;
        default:
            error = "Unknown error " + std::to_string(errorCode);
    }
    return error;
}

std::string SchunkDevice::translateInfoToString(uint8_t infoCode) {
    std::string info;
    switch (infoCode) {
        case INFO_BOOT:
            info = "INFO_BOOT";
            break;
        case INFO_UNKNOWN_COMMAND:
            info = "INFO_UNKNOWN_COMMAND";
            break;
        case INFO_FAILED:
            info = "INFO_FAILED";
            break;
        case NOT_REFERENCED:
            info = "NOT_REFERENCED";
            break;
        case INFO_SEARCH_SINE_VECTOR:
            info = "INFO_SEARCH_SINE_VECTOR";
            break;
        case INFO_NO_ERROR:
            info = "INFO_NO_ERROR";
            break;
        case INFO_COMMUNICATION_ERROR:
            info = "INFO_COMMUNICATION_ERROR";
            break;
        case INFO_TIMOUT:
            info = "INFO_TIMOUT";
            break;
        case INFO_CHECKSUM:
            info = "INFO_CHECKSUM";
            break;
        case INFO_MESSAGE_LENGTH:
            info = "INFO_MESSAGE_LENGTH";
            break;
        case INFO_WRONG_PARAMETER:
            info = "INFO_WRONG_PARAMETER";
            break;
        default:
            info = "Unknown info " + std::to_string(infoCode);
    }
    return info;
}

bool SchunkDevice::cleanCanBus(std::shared_ptr<ICANSocket> socket) {
    crf::utility::logger::EventLogger logger("SchunkDeviceStaticFunction");

    // just to be sure send error acknowledge to each joint
    for (int i = 3; i < 9; i++) {
        can_frame resp{};
        resp.can_id = MESSAGE_TO_DEVICE+i;
        resp.can_dlc = 0x02;
        resp.data[0] = resp.can_dlc - 1;
        resp.data[1] = CMD_ACK;
        if (socket->write(&resp) != 16) {
            return false;
        }
        std::chrono::milliseconds duration(1);
        std::this_thread::sleep_for(duration);
    }
    // just to be sure stop state broadcasting to each joint
    for (int i = 3; i < 9; i++) {
        can_frame frame{};
        frame.can_id = MESSAGE_TO_DEVICE+i;
        frame.can_dlc = 0x07;
        frame.data[0] = frame.can_dlc - 1;
        frame.data[1] = GET_STATE;
        int timePeriod = 0;
        std::memcpy(&frame.data[2], &timePeriod, sizeof(int));
        frame.data[6] = GET_ONLY_STATE;
        if (socket->write(&frame) != 16) {
            return false;
        }
        std::chrono::milliseconds duration(1);
        std::this_thread::sleep_for(duration);
    }

    bool busSilent = false;
    auto lastCommandSet = std::chrono::system_clock::now();
    auto readLoopStart = std::chrono::system_clock::now();

    while (!busSilent) {
        auto timeNow = std::chrono::system_clock::now();
        std::chrono::duration<double> timeSinceLastCommand = timeNow - lastCommandSet;
        if (timeSinceLastCommand.count() > 1) {
            logger->debug("Socket cleared");
            return true;
        }

        std::chrono::duration<double> timeSinceLoopStart = timeNow - readLoopStart;
        if (timeSinceLoopStart.count() > 2) {
            logger->critical("Socket could not be cleared, restart robot manually!");
            return false;
        }
        can_frame frame{};
        int i = socket->read(&frame);
        if (i == -1) {
            logger->debug("Socket cleared");
            return true;
        }

        // this is needed so the arm and gripper can be initialized separately
        int packetMotorID = frame.can_id & GET_DEVICE_CODE;
        if (packetMotorID == GRIPPER) {
            if (frame.data[1] == CMD_ERROR) {
                can_frame resp{};
                resp.can_id = MESSAGE_TO_DEVICE+GRIPPER;
                resp.can_dlc = 0x02;
                resp.data[0] = resp.can_dlc - 1;
                resp.data[1] = CMD_ACK;
                if (socket->write(&resp) != 16) {
                    return false;
                }
            } else if ((frame.data[1] == FRAG_START) ||
                       (frame.data[1] == FRAG_END)) {
                continue;
            } else {
                lastCommandSet = std::chrono::system_clock::now();
            }
            std::chrono::milliseconds duration(10);
            std::this_thread::sleep_for(duration);
        } else {
            lastCommandSet = std::chrono::system_clock::now();
        }
    }
    return true;
}

bool SchunkDevice::parseDeviceState(uint8_t state) {
    referenced_ = static_cast<bool >(state & 0x01);
    moving_ = static_cast<bool >((state >> 1) & 0x01);
    warning_ = static_cast<bool >((state >> 3) & 0x01);
    error_ = static_cast<bool >((state >> 4) & 0x01);
    brakeOn_ = static_cast<bool >((state >> 5) & 0x01);
    moveBlocked_ = static_cast<bool >((state >> 6) & 0x01);
    positionReached_ = static_cast<bool >((state >> 7) & 0x01);

    return true;
}

bool SchunkDevice::quitError() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x02;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_ACK;
    if (socket_->write(&frame) != 16) {
        return false;
    }
    return true;
}

bool SchunkDevice::handleError(std::string error) {
    if ((error == "ERROR_FAST_STOP") || (error == "ERROR_OVERSHOOT") ||
        (error == "ERROR_SOFT_LOW") || (error == "ERROR_SOFT_HIGH") ||
        (error == "ERROR_TOO_FAST") || (error == "ERROR_TOW")
        //  above happens when current is too small for the torque
         || (error == "ERROR_I2T")  // I2T is thrown by the gripper
        ) {
        if (!quitError()) {
            return false;
        }
        std::chrono::milliseconds duration(1);
        std::this_thread::sleep_for(duration);
        logger_->debug("Motor error " + error + " was handled!");
        return true;
    } else {
        applyBreak();
        logger_->critical("Motor has a critical error : " + error);
        return false;
    }
}

bool SchunkDevice::applyBreak() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x02;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_STOP;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    return true;
}

bool SchunkDevice::fastStop() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x02;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_FAST_STOP;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    return true;
}

bool SchunkDevice::setPosition(float rad) {
    if (!initialized_) {
        logger_->debug("Unable to setPosition(), motor was not initialized");
        return false;
    }

    int mposition = rad * radToMilliDeg_;

    if (mposition > maxPosition_) {
        std::string positionComparison = std::to_string(mposition) + " > ";
        positionComparison += std::to_string(maxPosition_);
        logger_->debug("Unable to setPosition(), the maximum position bound exceeded, "
        + positionComparison + " !");
        return false;
    }
    if (mposition < minPosition_) {
        logger_->debug("Unable to setPosition(), the minimum position bound exceeded!");
        return false;
    }

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x06;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = MOVE_POS;
    std::memcpy(&frame.data[2], &mposition, sizeof(int));

    if (socket_->write(&frame) != 16) {
        return false;
    }
    targetPosition_ = mposition;
    return true;
}

bool SchunkDevice::setVelocity(float radPerSec) {
    if (!initialized_) {
        logger_->debug("Unable to setVelocity(), motor was not initialized");
        return false;
    }

    if ((radPerSec < 0) && ((motorPosition_ - minPosition_ * millidegreeToRad_) < 0.02)) {
        logger_->debug("Unable to setVelocity(), minimum position reached");
        return false;
    }

    if ((radPerSec > 0) && ((motorPosition_ - maxPosition_ * millidegreeToRad_) > -0.02)) {
        logger_->debug("Unable to setVelocity(), maximum position  reached");
        return false;
    }

    int mvelocity = radPerSec * radToMilliDeg_;
    if (std::abs(mvelocity) > maxVelocity_) {
        logger_->debug("Unable to setVelocity(), the maximum velocity exceeded!");
        return false;
    }

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x06;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = MOVE_VEL;
    std::memcpy(&frame.data[2], &mvelocity, sizeof(int));
    if (socket_->write(&frame) != 16) {
        return false;
    }

    targetVelocity_ = mvelocity;
    return true;
}

bool SchunkDevice::setTargetVelocityToPercentage(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setTargetVelocityToPercentage(), motor was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage <0)) {
        logger_->debug("Wrong input for setTargetVelocityToPercentage()");
        return false;
    }

    int mvelocity = maxVelocity_ * (percentage / 100);

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x06;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = SET_TARGET_VEL;
    std::memcpy(&frame.data[2], &mvelocity, sizeof(int));
    if (socket_->write(&frame) != 16) {
        return false;
    }

    targetVelocity_ = mvelocity;
    return true;
}

bool SchunkDevice::setTargetAccelerationToPercentage(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setTargetAccelerationToPercentage(), motor was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage <0)) {
        logger_->debug("Wrong input for setTargetAccelerationToPercentage()");
        return false;
    }

    int macceleration = maxAcceleration_ * (percentage / 100);

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x06;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = SET_TARGET_ACC;
    std::memcpy(&frame.data[2], &macceleration, sizeof(int));
    if (socket_->write(&frame) != 16) {
        return false;
    }

    targetAcceleration_ = macceleration;
    return true;
}

bool SchunkDevice::setTargetCurrentToPercentage(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setTargetCurrentToPercentage(), motor was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage <0)) {
        logger_->debug("Wrong input for setTargetCurrentToPercentage()");
        return false;
    }

    int mcurrent = maxCurrent_ * (percentage / 100);

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x06;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = SET_TARGET_CUR;
    std::memcpy(&frame.data[2], &mcurrent, sizeof(int));
    if (socket_->write(&frame) != 16) {
        return false;
    }

    targetCurrent_ = mcurrent;
    return true;
}

bool SchunkDevice::getState() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = GET_STATE;
    int timePeriod = 0;
    std::memcpy(&frame.data[2], &timePeriod, sizeof(int));
    frame.data[6] = GET_POS_VEL;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    bool messageRecieved = false;
    char byteBuffer = 0x00;  // It is for storing the first byte of the velocity
    while (!messageRecieved) {
        can_frame recv_frame{};
        int returnVal = socket_->read(&recv_frame);
        if (returnVal == -1) {
            logger_->critical("Could not recieve message trough CAN, getState()");
            return false;
        }

        int intBuffer = 0;
        if ((recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_) &&
            (recv_frame.data[1] == GET_STATE)) {
            // sometimes the robot sends back 4f4b, which means OK, we need to ask again
            if ((recv_frame.data[2] == ASCII_O) && (recv_frame.data[3] == ASCII_K)) {
                if (socket_->write(&frame) != 16) {
                    return false;
                }
            }
        } else if (((recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_)||
                    (recv_frame.can_id == ERROR_FROM_DEVICE+canID_)) &&
                   (recv_frame.data[1] == FRAG_START)) {
            std::memcpy(&intBuffer, &recv_frame.data[3], sizeof(int));
            motorPosition_ = intBuffer * millidegreeToRad_;
            std::memcpy(&byteBuffer, &recv_frame.data[7], sizeof(char));
        } else if (((recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_)||
                    (recv_frame.can_id == ERROR_FROM_DEVICE+canID_)) &&
                   (recv_frame.data[1] == FRAG_END)) {
            messageRecieved = true;
            std::memcpy(&recv_frame.data[1], &byteBuffer, sizeof(char));
            std::memcpy(&intBuffer, &recv_frame.data[1], sizeof(int));
            motorVelocity_ = intBuffer * millidegreeToRad_;
            parseDeviceState(recv_frame.data[5]);
            errorCode_ = recv_frame.data[6];
            if (error_) {
                logger_->debug("Error occured " + translateErrorToString(errorCode_));
            }
        } else if ((recv_frame.data[2] == ASCII_O) && (recv_frame.data[3] == ASCII_K)) {
            continue;
        } else {
            logger_->debug("Unexpected message during getState()");
            printCanPacket(recv_frame);
        }
    }
    return true;
}

bool SchunkDevice::getStatePeriodic(int milisecond) {
    if (!initialized_) {
        logger_->debug("Unable to perform getStatePeriodic(), motor was not initialized");
        return false;
    }

    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x07;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = GET_STATE;
    std::memcpy(&frame.data[2], &milisecond, sizeof(int));
    frame.data[6] = GET_POS_VEL;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    return true;
}

bool SchunkDevice::parseStatePeriodic(can_frame frame) {
    if (!initialized_) {
        logger_->debug("Unable to perform parseStatePeriodic(), motor was not initialized");
        return false;
    }

    int packetMotorID = frame.can_id & GET_DEVICE_CODE;
    if (packetMotorID != canID_) {
        logger_->debug("Unable to perform parseStatePeriodic() wrong motor code : "
                       + std::to_string(packetMotorID));
        return false;
    }

    int intBuffer = 0;
    if (frame.data[1] == FRAG_START) {
        // Parsing position frame
        std::memcpy(&intBuffer, &frame.data[3], sizeof(int));
        motorPosition_ = intBuffer * millidegreeToRad_;
        std::memcpy(&lastByteFromPositionMessage_, &frame.data[7], sizeof(char));
        return true;
    } else if (frame.data[1] == FRAG_END) {
        // Parsing velocity frame
        std::memcpy(&frame.data[1], &lastByteFromPositionMessage_, sizeof(char));
        std::memcpy(&intBuffer, &frame.data[1], sizeof(int));
        motorVelocity_ = intBuffer * millidegreeToRad_;
        parseDeviceState(frame.data[5]);
        errorCode_ = frame.data[6];
        if (error_) {
            logger_->debug("Error occured " + translateErrorToString(errorCode_));
        }
        return true;
    } else {
        logger_->debug("Unable to perform parseStatePeriodic() wrong message : "
                       + translateMessageCodeToString(frame.data[1]));
        return false;
    }
}

bool SchunkDevice::reboot() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x02;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_REBOOT;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    bool messageRecieved = false;
    while (!messageRecieved) {
        can_frame recv_frame{};
        int returnVal = socket_->read(&recv_frame);
        if (returnVal == -1) {
            logger_->critical("Could not recieve message trough CAN");
            return false;
        }
        if ((recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_) &&
            (recv_frame.data[1] == CMD_INFO) &&
            (recv_frame.data[2] == INFO_BOOT)) {
            logger_->debug("Motor was succesfully rebooted!");
            return true;
        } else if ((recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_) &&
                   (recv_frame.data[1] == CMD_REBOOT) &&
                   (recv_frame.data[2] == ASCII_O) && (recv_frame.data[3] == ASCII_K)) {
            continue;
        } else if (((recv_frame.can_id == ERROR_FROM_DEVICE+canID_) ||
                    (recv_frame.can_id == MESSAGE_FROM_DEVICE+canID_)) &&
                   (recv_frame.data[1] == CMD_ERROR) ) {
            if (handleError(translateErrorToString(recv_frame.data[1]))) {
                if (socket_->write(&frame) != 16) {
                    return false;
                }
            }
            return false;
        } else {
            logger_->debug("Unexpected message during reboot()");
            printCanPacket(recv_frame);
        }
    }
    return true;
}

bool SchunkDevice::reference() {
    can_frame frame{};
    frame.can_id = MESSAGE_TO_DEVICE+canID_;
    frame.can_dlc = 0x02;
    frame.data[0] = frame.can_dlc - 1;
    frame.data[1] = CMD_REFERENCE;
    if (socket_->write(&frame) != 16) {
        return false;
    }

    // TODO(zpasztor) : check out the behaviour
    return true;
}

boost::optional<float> SchunkDevice::getMotorPosition() {
    if (!initialized_) {
        logger_->debug("Unable to perform getMotorPosition(), motor was not initialized");
        return boost::none;
    }
    return motorPosition_;
}

boost::optional<float> SchunkDevice::getMotorVelocity() {
    if (!initialized_) {
        logger_->debug("Unable to perform getMotorVelocity(), motor was not initialized");
        return boost::none;
    }
    return motorVelocity_;
}

boost::optional<float> SchunkDevice::getMaxPosition() {
    if (!initialized_) {
        logger_->debug("Unable to perform getMaxPosition(), motor was not initialized");
        return boost::none;
    }
    return maxPosition_ * millidegreeToRad_;
}

boost::optional<float> SchunkDevice::getMinPosition() {
    if (!initialized_) {
        logger_->debug("Unable to perform getMinPosition(), motor was not initialized");
        return boost::none;
    }
    return minPosition_ * millidegreeToRad_;
}

boost::optional<float> SchunkDevice::getMaxVelocity() {
    if (!initialized_) {
        logger_->debug("Unable to perform getMaxVelocity(), motor was not initialized");
        return boost::none;
    }
    return maxVelocity_ * millidegreeToRad_;
}

bool SchunkDevice::isBrakeActive() {
    if (!initialized_) {
        return true;
    }

    return brakeOn_;
}

bool SchunkDevice::isMoveBlocked() {
    if (!initialized_) {
        logger_->debug("Unable to perform isMoveBlocked(), motor was not initialized");
        return false;
    }
    return moveBlocked_;
}

boost::optional<uint8_t> SchunkDevice::getErrorCode() {
    if (!initialized_ || !error_) {
        return boost::none;
    }
    return errorCode_;
}

}  // namespace crf::actuators::schunkarm
