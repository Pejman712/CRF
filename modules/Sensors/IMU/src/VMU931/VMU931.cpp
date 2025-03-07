/*
 * © Copyright CERN 2021. All rights reserved. This software is released under a CERN proprietary software licence. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 * 
 * Author: Giacomo Lunghi CERN EN/SMM/MRO 2017
 * Contributor: Jose Vicente Marti Aviles CERN BE/CEM/MRO 2021
 * 
 *  ==================================================================================================
 */

#include "IMU/VMU931/VMU931.hpp"

#include <dirent.h>

#include <memory>
#include <string>
#include <vector>
#include <thread>

#define MESSAGE_START_FLAG 0x01
#define STRING_START_FLAG 0x02
#define STRING_END_FLAG 0x03
#define MESSAGE_END_FLAG 0x04
#define TIMOUT_BETWEEN_CHARACTERS_MS 5
#define TIMOUT_BETWEEN_MESSAGES_MS 50
#define END_OF_TIMESTAMP_FIELD 4

namespace crf::sensors::imu {

VMU931::VMU931(std::shared_ptr<communication::serialcommunication::ISerialCommunication> serial):
    serial_(serial),
    logger_("VMU931"),
    initialized_(false),
    imuData_{} {
    logger_->debug("CTor");
}

VMU931::~VMU931() {
    deinitialize();
    logger_->debug("DTor");
}

bool VMU931::initialize()  {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->error("Device has already been initialized");
        return false;
    }
    if (receiverThread_.joinable()) {
        stopReceiverThread_ = true;
        receiverThread_.join();
    }
    if (!serial_->initialize()) {
        logger_->error("Failed to initialize serial communications");
        return false;
    }
    if (!startDataBroadcast()) {
        logger_->error("Failed to start the data broadcast");
        return false;
    }
    stopReceiverThread_ = false;
    receiverThread_ = std::thread(&VMU931::receiver, this);
    initialized_ = true;
    logger_->info("Initialized");
    return true;
}

bool VMU931::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        return false;
    }
    if (receiverThread_.joinable()) {
        stopReceiverThread_ = true;
        receiverThread_.join();
    } else {
        logger_->warn("Thread is not joinable");
        return false;
    }
    if (!serial_->deinitialize()) {
        logger_->debug("Serial communication was not deinitialized");
        return false;
    }
    logger_->debug("Deinitialized");
    return true;
}

IMUSignals VMU931::getSignal() {
    logger_->debug("getSignals");
    if (!initialized_) {
        logger_->error("Device not initialized");
        IMUSignals output;
        output.position = crf::Code::NotInitialized;
        output.orientation = crf::Code::NotInitialized;
        output.linearVelocity = crf::Code::NotInitialized;
        output.angularVelocity = crf::Code::NotInitialized;
        output.linearAcceleration = crf::Code::NotInitialized;
        output.angularAcceleration = crf::Code::NotInitialized;
        output.magneticField = crf::Code::NotInitialized;
        return output;
    }
    return imuData_;
}

crf::expected<bool> VMU931::calibrate() {
    logger_->debug("calibrate");
    if (!initialized_) {
        logger_->error("Device not initialized");
        return crf::Code::NotInitialized;
    }
    if (!writeMsgToDevice("varl")) {
        logger_->error("Calibration unsuccessful");
        return false;
    }
    return true;
}

void VMU931::receiver() {
    while (!stopReceiverThread_) {
        std::string startByte(1, 0);
        serial_->read(&startByte, 1);
        if (startByte[0] != MESSAGE_START_FLAG && startByte[0] != STRING_START_FLAG) {
            continue;
        }

        std::string messageSize(1, 0);
        serial_->read(&messageSize, 1);
        int size = static_cast<int>(messageSize[0]) - 2;
        // This happens when the data bytes contain the message and string start byte, this way we
        // avoid std::length_error
        if (size <= 0) {
            logger_->debug("Got empty message");
            continue;
        }
        std::string messageBuffer(size, 0);
        serial_->read(&messageBuffer, size);

        if (startByte[0] == MESSAGE_START_FLAG) {
            if (messageBuffer.back() != MESSAGE_END_FLAG) {
                logger_->debug("Got malformed message");
                continue;
            }
            parseMessage(messageBuffer, size);
        }
        if (startByte[0] == STRING_START_FLAG) {
            if (messageBuffer.back() != STRING_END_FLAG) {
                logger_->debug("Got malformed string");
                continue;
            }
            // We need to cut the first byte and the last two bytes, because it is the length.
            std::string cleanString = messageBuffer.substr(1, size-3);
            logger_->debug(cleanString);
        }
    }
}

bool VMU931::parseMessage(const std::string& message, int bufferSize) {
    const char* messageBuffer = message.c_str();
    char messageType = messageBuffer[0];

    if (messageType == 's') {
        logger_->debug("Received unexpected status message");
        return false;
    }
    char localBuffer[20];
    // Need to flip the float byte order, because sensor sends big endians, but x86 uses little
    // endian.
    //  -  Data starts at byte address 5.
    //  -  Address 0 contains the data byte.
    //  -  Address 1-4 contains the timestamp.
    //  -  Inverts order of all bytes, including timestamp.
    for (int i = 0; i < 5; i++) {
        localBuffer[i] = messageBuffer[4-i];
        localBuffer[i+4] = messageBuffer[8-i];
        localBuffer[i+8] = messageBuffer[12-i];
        localBuffer[i+12] = messageBuffer[16-i];
    }

    // Copies timestamp on imuData structure
    // std::memcpy(&imuData_.timestamp, localBuffer, 4);
    std::array<double, 3> linearAcceleration;
    std::array<double, 3> angularVelocity;
    std::array<double, 3> magneticField;
    std::array<double, 4> orientation;

    switch (messageType) {
        case 'a':  // Accelerometer data
            for (int i = 0; i < 3; i++) {
                std::memcpy(&linearAcceleration[i], localBuffer+i*4+4, 4);
            }
            break;
        case 'g':  // Gyroscope data
            for (int i = 0; i < 3; i++) {
                std::memcpy(&angularVelocity[i], localBuffer+i*4+4, 4);
            }
            break;
        case 'c':  // Magnetometer data
            for (int i = 0; i < 3; i++) {
                std::memcpy(&magneticField[i], localBuffer+i*4+4, 4);
            }
            break;
        case 'e':  // Euler Angles
                // std::memcpy(&imuData_.roll, localBuffer+4, 4);
                // std::memcpy(&imuData_.pitch, localBuffer+8, 4);
                // std::memcpy(&imuData_.yaw, localBuffer+12, 4);
            break;
        case 'q':  // Quaternion data
            for (int i = 0; i < 4; i++) {
                std::memcpy(&orientation[i], localBuffer+i*4+4, 4);
            }
            break;
        case 'h':  // Heading data (basically yaw angle)
                // std::memcpy(&imuData_.heading, localBuffer+4, 4);
            break;
        default:
            break;
    }
    imuData_.linearAcceleration = linearAcceleration;
    imuData_.angularVelocity = angularVelocity;
    imuData_.magneticField = magneticField;
    imuData_.orientation = orientation;
    return true;
}

bool VMU931::startDataBroadcast() {
    logger_->debug("startDataBroadcast");
    // Status message
    writeMsgToDevice("vars");

    bool statusRead = false;
    std::string status_messageBuffer;
    while (!statusRead) {
        std::string startByte(1, 0);
        do {
            serial_->read(&startByte, 1);
        } while (startByte[0] != MESSAGE_START_FLAG);
        std::string messageSize(1, 0);
        serial_->read(&messageSize, 1);
        int size = static_cast<int>(messageSize[0] - 2);
        // Some error happened during reading
        if (size <= 0) {
            continue;
        }
        status_messageBuffer.resize(size);
        serial_->read(&status_messageBuffer, size);
        char messageType = status_messageBuffer.c_str()[0];
        if (messageType == 's') {
            statusRead = true;
        }
    }

    // Activates all the sensors on the IMU
    if ((status_messageBuffer[7] & 0b01000000) == 0) {
        logger_->debug("Activated heading");
        writeMsgToDevice("varh");
    }
    if ((status_messageBuffer[7] & 0b00010000) == 0) {
        logger_->debug("Activated Euler Angles");
        writeMsgToDevice("vare");
    }
    if ((status_messageBuffer[7] & 0b00001000) == 0) {
        logger_->debug("Activated Magnetometer");
        writeMsgToDevice("varc");
    }
    if ((status_messageBuffer[7] & 0b00000100) == 0) {
        logger_->debug("Activated Quaternions");
        writeMsgToDevice("varq");
    }
    if ((status_messageBuffer[7] & 0b00000010) == 0) {
        logger_->debug("Activated Gyroscope");
        writeMsgToDevice("varg");
    }
    if ((status_messageBuffer[7] & 0b00000001) == 0) {
        logger_->debug("Activated Accelerometer");
        writeMsgToDevice("vara");
    }
    logger_->debug("All sensors are streaming");

    // Sets accelerometers resolution to 16g (maximum)
    writeMsgToDevice("var7");

    return true;
}

bool VMU931::writeMsgToDevice(const std::string& msg) {
    logger_->debug("writeMsgToDevice {}", msg);
    // We have to wait between writing each character, otherwise the device overloads
    for (std::string::size_type i = 0; i < msg.size(); i++) {
        if (serial_->write(msg.substr(i, 1)) == -1) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMOUT_BETWEEN_CHARACTERS_MS));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(TIMOUT_BETWEEN_MESSAGES_MS));
    return true;
}

}  // namespace crf::sensors::imu
