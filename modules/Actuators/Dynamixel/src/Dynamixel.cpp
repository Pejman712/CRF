/* © Copyright CERN 2019. All rights reserved. This software is released under a CERN proprietary software license. 
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Carlos Veiga Almagro CERN EN/SMM/MRO
 *
 *  ==================================================================================================
 */
#include <string>
#include <utility>
#include <memory>

#include "Dynamixel/Dynamixel.hpp"
#include "Dynamixel/DynamixelSDK.hpp"

namespace crf {
namespace devices {
namespace dynamixelstepper {

Dynamixel::Dynamixel(const std::string & port, const nlohmann::json & configFile,
    std::shared_ptr<IDynamixelSDK> dxl) : logger_("Dynamixel"), port_(port),
    dxl_ConfigFile_(configFile), dxl_comm_result_(COMM_TX_FAIL), portHandler_(NULL),
    packetHandler_(NULL), initialized_(false), dynamixelSdk_(dxl) {
        logger_->debug("Ctor");
        if (dynamixelSdk_ == nullptr) {
            dynamixelSdk_.reset(new DynamixelSDK);
        }
}

bool Dynamixel::initialize() {
    logger_->debug("initialize");
    if (initialized_) {
        logger_->warn("Dynamixel set already initialized");
        return false;
    }
    if (!configuration_.parse(dxl_ConfigFile_)) {
        logger_->error("Could not parse the configuration file");
        return false;
    }
    if (!createHandlers()) {
        logger_->error("Could not create the connection with Dynamixel");
        return false;
    }
    initialized_ = true;
    return true;
}

bool Dynamixel::deinitialize() {
    logger_->debug("deinitialize");
    if (!initialized_) {
        logger_->warn("Dynamixel already deinitialized");
        return false;
    }
    if (!closeConnection()) {
        logger_->error("Could not close the connection properly");
        return false;
    }
    initialized_ = false;
    return true;
}

bool Dynamixel::createHandlers() {
    logger_->debug("createHandlers");
    portHandler_   = dynamixelSdk_->getPortHandler(port_.c_str());
    packetHandler_ = dynamixelSdk_->getPacketHandler(
        configuration_.getMotorsChain()[0].protocolVersion);
    if (!connectToDynamixel()) {
        logger_->error("Error connecting to Dynamixel");
        return false;
    }
    for (int i = 0; i < configuration_.getNumberOfMotors(); i++) {
        zero_position_.push_back(configuration_.getMotorsChain()[i].limits.DXLStartValue);
    }
    return true;
}

bool Dynamixel::connectToDynamixel() {
    logger_->debug("connectToDynamixel");
    if (!dynamixelSdk_->openPort(portHandler_)) {
        logger_->error("Problem to open the port");
        return false;
    }
    if (!dynamixelSdk_->setBaudRate(portHandler_, configuration_.getMotorsChain()[0].baudRate)) {
        logger_->error("Problem to set the port baudrate");
        return false;
    }
    // Enable Dynamixel Torque
    uint8_t dxl_error = 0;
    for (const auto& motor : configuration_.getMotorsChain()) {
        dynamixelSdk_->write1ByteTxRx(packetHandler_, portHandler_, motor.dxl_id,
            motor.addrMx.torqueEnable, 1, &dxl_error);
        if (dxl_error != 0) {
            logger_->error(R"(Problem to write in motor with dxl_id: {0}
                The error is: {1})", motor.dxl_id, dynamixelSdk_->getRxPacketError(packetHandler_,
                dxl_error));
            return false;
        }
    }
    return true;
}

bool Dynamixel::writeDynamixel(uint8_t index, uint16_t goal_position) {
    logger_->debug("writeDynamixel");
    if (!initialized_) {
        logger_->warn("The Dynamixel has to be initialized");
        return false;
    }
    uint8_t dxl_error = 0;
    auto motor = configuration_.getMotorsChain()[index];
    dynamixelSdk_->write2ByteTxRx(packetHandler_, portHandler_, motor.dxl_id,
        motor.addrMx.goalPosition, goal_position, &dxl_error);

    // It is commented due to a dynamixel_sdk's bug at the moment of asking for the current
    // motor situation that creates a delay on the performance
    /*do {  // waitinf to finish the movement
        auto dxl_comm_result = dynamixelSdk_->read2ByteTxRx(packetHandler_, portHandler_,
        configuration_.getMotorsChain()[index].dxl_id, ADDR_MX_MOVING, &dxl_moving, &dxl_error);
    } while ( dxl_moving != 0);*/
    if (dxl_error != 0) {
        logger_->error(R"(Problem to write in motor with dxl_id: {0}
            The error is: {1})", motor.dxl_id, dynamixelSdk_->getRxPacketError(packetHandler_,
            dxl_error));
        return false;
    }
    return true;
}

int16_t Dynamixel::readCurrentPosition(uint8_t index) {
    logger_->debug("readCurrentPosition");
    if (!initialized_) {
        logger_->warn("The Dynamixel has to be initialized");
        return -1;
    }
    uint8_t dxl_error = 0;
    uint16_t dxl_current_position = 0;
    auto motor = configuration_.getMotorsChain()[index];
    dynamixelSdk_->read2ByteTxRx(packetHandler_, portHandler_, motor.dxl_id,
        motor.addrMx.presentPosition, &dxl_current_position, &dxl_error);
    if (dxl_error != 0) {
        logger_->error(R"(Problem to read in motor with dxl_id: {0}
            The error is: {1})", motor.dxl_id, dynamixelSdk_->getRxPacketError(packetHandler_,
            dxl_error));
        return -1;
    }
    return dxl_current_position;
}

bool Dynamixel::closeConnection() {
    logger_->debug("closeConnection");
    // Disable Dynamixel Torque
    uint8_t dxl_error = 0;
    for (const auto& motor : configuration_.getMotorsChain()) {
        dynamixelSdk_->write1ByteTxRx(packetHandler_, portHandler_, motor.dxl_id,
            motor.addrMx.torqueEnable, 0, &dxl_error);
        if (dxl_error != 0) {
            logger_->error(R"(Problem to write in motor with dxl_id: {0}
                The error is: {1})", motor.dxl_id, dynamixelSdk_->getRxPacketError(packetHandler_,
                dxl_error));
            return false;
        }
    }
    dynamixelSdk_->closePort(portHandler_);
    return true;
}

Dynamixel::~Dynamixel() {
    logger_->debug("DTor");
    if (initialized_) deinitialize();
}

}  // namespace dynamixelstepper
}  // namespace devices
}  // namespace crf
