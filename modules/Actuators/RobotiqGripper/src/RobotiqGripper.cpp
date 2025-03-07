/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Zsolt Pasztori CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */

// The code was written based on 2F-85_2F-140_Instruction_Manual_PDF_20181126.pdf

#include "RobotiqGripper/RobotiqGripper.hpp"

#include <string>
#include <thread>

namespace crf {
namespace robots {
namespace robotiqgripper {

RobotiqGripper::RobotiqGripper(const std::string& devAddress):
        logger_("RobotiqGripper"),
        deviceAddress_(devAddress),
        initialized_(false),
        bus_(nullptr),
        gripperInputRegister(0x03E8),
        gripperOutputRegister(0x07D0),
        gripperActivated_(false),
        objectDetected_(false),
        posRequestEcho_(0),
        actualPosition_(0),
        current_(0),
        defaultVelocity(100),  // value is between [0 - min, 255 - max]
        targetForce_(100) {  // value is between [0 - min, 255 - max]
        logger_->debug("CTor");
}

RobotiqGripper::~RobotiqGripper() {
    logger_->debug("DTor");
    deinitialize();
}

bool RobotiqGripper::initialize() {
    if (initialized_) {
        logger_->debug("Unable to perform initialize(), device already initialized");
        return false;
    }

    // Init modbus structure
    int baudRate = 115200;
    char parity = 'N';  // None
    int dataBit = 8;
    int stopBit = 1;
    bus_.reset(modbus_new_rtu(deviceAddress_.c_str(), baudRate, parity, dataBit, stopBit),
        &modbus_free);
    if (!bus_) {
        logger_->critical("Unable to create the libmodbus context");
        return false;
    }

    int retval = modbus_connect(bus_.get());
    if (retval == -1) {
        std::string error = modbus_strerror(errno);
        logger_->critical("Connection failed with error : " + error);
        return false;
    }

    int slaveID = 9;
    int result = modbus_set_slave(bus_.get(), slaveID);
    if (result == -1) {
        logger_->critical("Unable to set slave ID");
        return false;
    }

    readGripperState();
    if (!gripperActivated_) {
        // Deactivate gripper
        // after power loss the activation bit stays on in the gripper
        // you need to set it manually to 0, so the robot can begin calibration
        uint16_t msg[3];
        msg[0] = 0x0;
        msg[1] = 0x0;
        msg[2] = 0x0;

        if (modbus_write_registers(bus_.get(), gripperInputRegister, 3, msg) == -1) {
            std::string error = modbus_strerror(errno);
            logger_->critical("Deactivation failed with error : " + error);
            return false;
        }
        // Activate gripper
        msg[0] = 0x0100;
        msg[1]= 0x0;
        msg[2] = 0x0;

        if (modbus_write_registers(bus_.get(), gripperInputRegister, 3, msg) == -1) {
            std::string error = modbus_strerror(errno);
            logger_->critical("Connection failed with error : " + error);
            return false;
        }
    }

    // Wait for gripper to do the activation movement
    while (!gripperActivated_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        readGripperState();
    }

    initialized_ = true;
    logger_->debug("Gripper initialized!");
    return true;
}

bool RobotiqGripper::deinitialize() {
    if (!initialized_) {
        logger_->debug("Unable to perform deinitialize(), device is not initialized");
        return false;
    }

    modbus_close(bus_.get());

    initialized_ = false;
    logger_->debug("Gripper deinitialized!");
    return true;
}

boost::optional<float> RobotiqGripper::getPosition() {
    if (!initialized_) {
        logger_->debug("Unable to perform getPosition(), gripper was not initialized");
        return boost::none;
    }
    readGripperState();
    // 255 is the max position
    float positionPercentage = (actualPosition_ / 255.0) * 100;
    return positionPercentage;
}

bool RobotiqGripper::isGrasping() {
    if (!initialized_) {
        logger_->debug("Unable to perform isGrasping(), gripper was not initialized");
        return false;
    }
    readGripperState();
    return objectDetected_;
}

bool RobotiqGripper::setPosition(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to perform setPosition(), gripper was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage < 0)) {
        logger_->debug("Wrong input for setPosition(), percentage should be between 0 and 100");
        return false;
    }

    uint8_t targetPosition = percentage / 100 * 255;
    return writeGripperState(targetPosition, defaultVelocity);
}

bool RobotiqGripper::setPosition(GripperState state) {
    if (state == Gripper_Open) {
        return setPosition(0);
    } else if (state == Gripper_Closed) {
        return setPosition(100);
    } else {
        logger_->debug("Wrong input for setPosition(GripperState state)");
        return false;
    }
}

bool RobotiqGripper::setGraspingForce(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setGraspingForce(), gripper was not initialized");
        return false;
    }

    if ((percentage > 100) || (percentage <0)) {
        logger_->debug("Wrong input for setGraspingForce(),"
                       " percentage should be between 0 and 100");
        return false;
    }
    targetForce_ = percentage / 100 * 255;
    return true;
}

bool RobotiqGripper::stopGripper() {
    if (!initialized_) {
        logger_->debug("Unable to setGraspingForce(), gripper was not initialized");
        return false;
    }

    uint16_t msg[3];
    msg[0] = 0x0100;  // sets rGTO and rACT registers
    // rGTO is set to zero
    // rACT needs to be set otherwise the gripper loses calibration

    msg[1]=  0;
    msg[2] = 0;
    if (modbus_write_registers(bus_.get(), gripperInputRegister, 3, msg) == -1) {
        std::string error = modbus_strerror(errno);
        logger_->critical("Connection failed with error : " + error);
        return false;
    }
    return true;
}

bool RobotiqGripper::setVelocity(float percentage) {
    if (!initialized_) {
        logger_->debug("Unable to setVelocity(), gripper was not initialized");
        return false;
    }

    if (std::fabs(percentage) > 100) {
        logger_->debug("Wrong input for setVelocity(), percentage should be between -100 and 100");
        return false;
    }

    if (percentage == 0) {
        return stopGripper();
    }
    // you set target velocity instead of actually movement velocity
    uint8_t targetSpeed = std::fabs(percentage) / 100 *255;

    if (percentage <= 0) {
        uint8_t gripperOpen = 0;
        return writeGripperState(gripperOpen, targetSpeed);
    } else {
        uint8_t gripperClose = 255;
        return writeGripperState(gripperClose, targetSpeed);
    }
}

bool RobotiqGripper::readGripperState() {
    uint16_t msg[3];
    if (modbus_read_registers(bus_.get(), gripperOutputRegister, 3, msg) == -1) {
        std::string error = modbus_strerror(errno);
        logger_->critical("Connection failed with error : " + error);
        return false;
    }
    // before copying you need to swap the byte order of uint16_t
    for (int i = 0; i < 3; i++) {
        msg[i] = (msg[i] >> 8) | (msg[i] << 8);
    }

    uint8_t registers[6];
    std::memcpy(registers, msg, sizeof(char) * 6);

    uint8_t gSTA = (registers[0] & 0b00110000) >> 4;
    gripperActivated_ = (gSTA == 0x03);
    uint8_t gOBJ = (registers[0] & 0b11000000) >> 6;
    if (gOBJ == 0x01 || gOBJ == 0x02) {
        objectDetected_ = true;
    } else {
        objectDetected_ = false;
    }

    posRequestEcho_ = registers[3];
    actualPosition_ = registers[4];
    current_ = registers[5];
    return true;
}

bool RobotiqGripper::writeGripperState(uint8_t targetPosition, uint8_t targetSpeed) {
    uint16_t msg[3];
    msg[0] = 0x0900;  // sets rGTO and rACT registers
    // rGTO must be set so the gripper starts positioning
    // rACT needs to be set otherwise the gripper loses calibration

    msg[1]=  (0xFF <<8) + (targetPosition);
    msg[2] = (targetSpeed << 8) + targetForce_;
    // targetForce_ comes from class private field
    if (modbus_write_registers(bus_.get(), gripperInputRegister, 3, msg) == -1) {
        std::string error = modbus_strerror(errno);
        logger_->critical("Connection failed with error : " + error);
        return false;
    }

    return true;
}

}  // namespace robotiqgripper
}  // namespace robots
}  // namespace crf
