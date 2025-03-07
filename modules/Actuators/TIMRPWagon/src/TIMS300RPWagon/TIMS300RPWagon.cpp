/* © Copyright CERN 2022. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Alejandro Diaz Rosales CERN BE/CEM/MRO 2022
 *
 *  ==================================================================================================
 */

#include <optional>
#include <map>
#include <tuple>
#include <memory>
#include <chrono>
#include <iostream>

#include <nlohmann/json.hpp>

#include "TIMRPWagon/TIMS300RPWagon/TIMS300RPWagon.hpp"
#include "SiemensPLC/ISiemensPLC.hpp"
#include "SiemensPLC/SiemensPLCS7.hpp"
#include "TIMRPWagon/TIMRPWagonConfiguration.hpp"

namespace crf::actuators::timrpwagon {

TIMS300RPWagon::TIMS300RPWagon(const nlohmann::json& configFile,
    std::shared_ptr<crf::devices::siemensplc::ISiemensPLC> plc) :
    configFile_(configFile),
    plc_(plc),
    logger_("TIMS300RPWagon"),
    isInitialized_(false),
    configuration_(new crf::actuators::timrpwagon::TIMRPWagonConfiguration),
    stopUpdatePCLValuesThread_(true) {
    logger_->debug("CTor");
}

TIMS300RPWagon::~TIMS300RPWagon() {
    logger_->debug("DTor");
    deinitialize();
}

bool TIMS300RPWagon::initialize() {
    logger_->info("initialize");
    if (isInitialized_) {
        logger_->error("There is already a connection established with the PLC");
        return true;
    }
    if (!configuration_->parse(configFile_)) {
        logger_->error("Failed to read the configuration file");
        return false;
    }
    commandTimeout_ = configuration_->getCommandTimeout();
    if (plc_ == nullptr) {
        plc_ = std::make_shared<crf::devices::siemensplc::SiemensPLCS7>(
            configuration_->getIPAddress(),
            configuration_->getRack(),
            configuration_->getSlot());
    }
    if (!plc_->initialize()) {
        logger_->error("Could not initialize the connection to the PLC");
        return false;
    }
    if (!updatePLCValues()) {
        logger_->error("Could not read datablocks from PLC");
        return false;
    }

    // We launch the thread and wait for at least twice the time of the first loop, to retrive the
    // values.
    stopUpdatePCLValuesThread_ = false;
    updatePLCValuesThread_ = std::thread(&TIMS300RPWagon::grabber, this);
    std::unique_lock<std::mutex> lock(firstGrabberLoopMutex_);
    if (firstGrabberLoop_.wait_for(lock, 2*configuration_->getUpdateInterval()) ==
        std::cv_status::timeout) {
        logger_->error("The update of the PLC values took longer than expected");
        return false;
    }
    isInitialized_ = true;
    return true;
}

bool TIMS300RPWagon::deinitialize() {
    logger_->info("deinitialize");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return false;
    }
    stopUpdatePCLValuesThread_ = true;
    if (updatePLCValuesThread_.joinable()) {
        updatePLCValuesThread_.join();
    }
    if (!plc_->deinitialize()) {
        logger_->error("Could not deinitialize connection to PLC");
        return false;
    }
    isInitialized_ = false;
    return true;
}

bool TIMS300RPWagon::isConnected() {
    if (!isInitialized_) {
        logger_->error("The TIMS300RPWagon is not initialized");
        return false;
    }
    if (plc_ == nullptr) {
        logger_->error("Missing PLC object for connection");
        return false;
    }
    if (!plc_->isConnected()) {
        logger_->error("Not connected to PLC");
        return false;
    }
    return true;
}

RPArmPosition TIMS300RPWagon::getRPArmPosition() {
    logger_->debug("isRPArmInError");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return RPArmPosition::NotDefined;
    }
    if (statusDB_.isEmpty()) {
        return RPArmPosition::NotDefined;
    }
    if (statusDB_.rpArmRetracted()) {
        return RPArmPosition::Retracted;
    } else if (statusDB_.rpArmInTheMiddle()) {
        return RPArmPosition::InTheMiddle;
    } else if (statusDB_.rpArmDeployed()) {
        return RPArmPosition::Deployed;
    }
    return RPArmPosition::NotDefined;
}

std::optional<bool> TIMS300RPWagon::retractRPArm() {
    logger_->debug("retractRPArm");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, false,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].second)) {
            logger_->error("Could not write enable RP arm manual control command");
            return false;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["RetractRPArm"].first,
        configuration_->getCommandsVariablesDBLocation()["RetractRPArm"].second)) {
            logger_->error("Could not write retract RP arm command");
            return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        RPArmPosition position = getRPArmPosition();
        if (position == RPArmPosition::Retracted) {
            return true;
        }
        std::this_thread::sleep_for(configuration_->getUpdateInterval());
    }
    logger_->error("The RP arm didn't retract after {} microseconds", commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300RPWagon::deployRPArm() {
    logger_->debug("deployRPArm");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, false,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].second)) {
            logger_->error("Could not write enable RP arm manual control command");
            return false;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["DeployRPArm"].first,
        configuration_->getCommandsVariablesDBLocation()["DeployRPArm"].second)) {
            logger_->error("Could not write deploy RP arm command");
            return false;
    }
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    while (std::chrono::duration_cast<std::chrono::microseconds>(end - start) < commandTimeout_) {
        end = std::chrono::high_resolution_clock::now();
        RPArmPosition position = getRPArmPosition();
        if (position == RPArmPosition::Deployed) {
            return true;
        }
        std::this_thread::sleep_for(configuration_->getUpdateInterval());
    }
    logger_->error("The RP arm didn't deploy after {} microseconds", commandTimeout_.count());
    return false;
}

std::optional<bool> TIMS300RPWagon::moveRPArmUp() {
    logger_->debug("moveRPArmUp");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].second)) {
            logger_->error("Could not write enable RP arm manual control command");
            return false;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["MoveRPArmUp"].first,
        configuration_->getCommandsVariablesDBLocation()["MoveRPArmUp"].second)) {
            logger_->error("Could not write move RP arm up command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::moveRPArmDown() {
    logger_->debug("moveRPArmDown");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].first,
        configuration_->getCommandsVariablesDBLocation()["EnableRPArmManualControl"].second)) {
            logger_->error("Could not write enable RP arm manual control command");
            return false;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["MoveRPArmDown"].first,
        configuration_->getCommandsVariablesDBLocation()["MoveRPArmDown"].second)) {
            logger_->error("Could not write move RP arm down command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::stopRPArm() {
    logger_->debug("stopRPArm");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["StopRPArm"].first,
        configuration_->getCommandsVariablesDBLocation()["StopRPArm"].second)) {
            logger_->error("Could not write stop RP arm command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::lockRPArm() {
    logger_->debug("lockRPArm");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, false,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["ReleaseLockRPArm"].first,
        configuration_->getCommandsVariablesDBLocation()["ReleaseLockRPArm"].second)) {
            logger_->error("Could not write lock RP arm command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::unlockRPArm() {
    logger_->debug("unlockRPArm");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["ReleaseLockRPArm"].first,
        configuration_->getCommandsVariablesDBLocation()["ReleaseLockRPArm"].second)) {
            logger_->error("Could not write unlock RP arm command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::isRPArmInError() {
    logger_->debug("isRPArmInError");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (statusDB_.isEmpty()) {
        return std::nullopt;
    }
    return statusDB_.rpArmInError();
}

std::optional<bool> TIMS300RPWagon::resetRPArmDriver() {
    logger_->debug("resetRPArmDriver");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["ResetRPArmDriver"].first,
        configuration_->getCommandsVariablesDBLocation()["ResetRPArmDriver"].second)) {
            logger_->error("Could not write stop RP arm command");
            return false;
    }
    return true;
}

std::optional<bool> TIMS300RPWagon::acknowledgeErrors() {
    logger_->debug("acknowledgeErrors");
    if (!isConnected()) {
        logger_->error("There is no connection with the PLC");
        return std::nullopt;;
    }
    if (!plc_->writeRegister(devices::siemensplc::RegisterType::R_BOOL, true,
        configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsVariablesDBLocation()["AcknowledgeErrors"].first,
        configuration_->getCommandsVariablesDBLocation()["AcknowledgeErrors"].second)) {
            logger_->error("Could not write acknowledge error command");
            return false;
    }
    return true;
}

std::shared_ptr<TIMRPWagonConfiguration> TIMS300RPWagon::getConfiguration() {
    logger_->debug("getConfiguration");
    return configuration_;
}

bool TIMS300RPWagon::updatePLCValues() {
    logger_->debug("updatePLCValues");
    bool result = true;
    std::string buffer = plc_->readDB(configuration_->getCommandsDatablockLocation().first,
        configuration_->getCommandsDatablockLocation().second, 0);
    if (buffer.length() == configuration_->getCommandsDatablockLocation().second) {
        if (!commandsDB_.parseSiemensPLCBuffer(buffer,
            configuration_->getCommandsVariablesDBLocation())) {
            logger_->warn("Failed to parse Inputs Datablock");
        }
    } else {
        logger_->error("Failed to read Inputs Datablock");
        result = false;
    }
    buffer = plc_->readDB(configuration_->getStatusDatablockLocation().first,
        configuration_->getStatusDatablockLocation().second, 0);
    if (buffer.length() == configuration_->getStatusDatablockLocation().second) {
        if (!statusDB_.parseSiemensPLCBuffer(buffer,
            configuration_->getStatusVariablesDBLocation())) {
            logger_->warn("Failed to parse Status Datablock");
        }
    } else {
        logger_->error("Failed to read Status Datablock {} {}", buffer.length(),
            configuration_->getStatusDatablockLocation().second);
        result = false;
    }
    return result;
}

void TIMS300RPWagon::grabber() {
    logger_->debug("grabber");
    while (!stopUpdatePCLValuesThread_) {
        std::chrono::time_point start = std::chrono::high_resolution_clock::now();

        updatePLCValues();

        std::chrono::microseconds elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::high_resolution_clock::now() - start);
        if ((configuration_->getUpdateInterval() - elapsed).count() > 0) {
            std::this_thread::sleep_for(configuration_->getUpdateInterval() - elapsed);
        } else {
            logger_->warn("updatePLCValues(): execution time longer than the update interval");
        }
        firstGrabberLoop_.notify_one();
    }
}

}  // namespace crf::actuators::timrpwagon
